/*
 * Gree Fan Remote - stateful IR remote for Flipper Zero  (model: YAN1F6)
 *
 * Rebuilds the FULL Gree state packet on every send and recomputes the
 * checksum, so the fan snaps to whatever the fields say. No capture-replay,
 * no desync. Verified against real captures AND the IRremoteESP8266 spec.
 *
 * Frame: 8 bytes, two 32-bit blocks LSB-first, 0b010 footer between, ~19ms gap.
 *
 * Byte 0: bits0-2 Mode | bit3 Power | bits4-5 Fan | bit6 SwingAuto | bit7 Sleep
 * Byte 1: bits0-3 Temp (C-16) | bits4-7 timer
 * Byte 2: bits0-3 TimerHours | bit4 Turbo | bit5 Light | bit6 ModelA | bit7 Xfan
 * Byte 3: 0x5 marker in high nibble
 * Byte 4: bits0-3 SwingV | bits4-6 SwingH
 * Byte 5: bits0-1 DisplayTemp | bit2 IFeel | bit6 WiFi
 * Byte 6: unused
 * Byte 7: bit2 Econo | bits4-7 Checksum
 *
 * Checksum = (10 + low nibbles of bytes0-3 + high nibbles of bytes4-6) & 0x0F
 */

#include <furi.h>
#include <gui/gui.h>
#include <gui/view_dispatcher.h>
#include <gui/modules/submenu.h>
#include <gui/modules/variable_item_list.h>
#include <notification/notification.h>
#include <notification/notification_messages.h>
#include <infrared.h>
#include <string.h>

// ---- Gree IR timing constants (microseconds) ----
#define GREE_HDR_MARK   9000
#define GREE_HDR_SPACE  4500
#define GREE_BIT_MARK   620
#define GREE_ONE_SPACE  1600
#define GREE_ZERO_SPACE 540
#define GREE_MSG_SPACE  19000
#define GREE_FREQUENCY  38000
#define GREE_DUTY       0.33f

// Known-good template captured from the real "power on" press.
static const uint8_t GREE_TEMPLATE[8] = {0x29, 0x08, 0x60, 0x50, 0x02, 0x41, 0x00, 0xF0};

#define TEMP_MIN 16
#define TEMP_MAX 30

typedef struct {
    uint8_t power; // 0/1
    uint8_t mode;  // 0 auto,1 cool,2 dry,3 fan,4 heat
    uint8_t temp;  // 16..30
    uint8_t fan;   // 0 auto,1..3
    uint8_t swing; // SwingV: 0 off,1 auto,2..6 fixed positions
    uint8_t turbo; // 0/1
    uint8_t sleep; // 0/1
    uint8_t light; // 0/1
    uint8_t econo; // 0/1
} GreeState;

typedef enum {
    GreeViewBrand,
    GreeViewRemote,
} GreeViewId;

// Remote row order (must match add order in build_remote_view)
typedef enum {
    ItemPower,
    ItemMode,
    ItemTemp,
    ItemFan,
    ItemSwing,
    ItemTurbo,
    ItemSleep,
    ItemLight,
    ItemEcono,
    ItemSend,
} GreeItem;

typedef struct {
    Gui* gui;
    ViewDispatcher* view_dispatcher;
    Submenu* submenu;
    VariableItemList* var_list;
    NotificationApp* notifications;
    GreeState state;
} GreeApp;

static const char* const mode_names[] = {"Auto", "Cool", "Dry", "Fan", "Heat"};
static const char* const fan_names[] = {"Auto", "1", "2", "3"};
// SwingV index maps 1:1 to the protocol value (0..6)
static const char* const swing_names[] =
    {"Off", "Auto", "Up", "Mid-Up", "Middle", "Mid-Down", "Down"};

#define MODE_COUNT  (sizeof(mode_names) / sizeof(mode_names[0]))
#define FAN_COUNT   (sizeof(fan_names) / sizeof(fan_names[0]))
#define SWING_COUNT (sizeof(swing_names) / sizeof(swing_names[0]))

// ---- Encoder: state -> 8 bytes with valid checksum ----
static void gree_build_frame(const GreeState* s, uint8_t out[8]) {
    memcpy(out, GREE_TEMPLATE, 8);

    // Byte 0: mode | power | fan | (swingAuto=0) | sleep
    out[0] = (s->mode & 0x07) | ((s->power & 0x01) << 3) | ((s->fan & 0x03) << 4) |
             ((s->sleep & 0x01) << 7);

    // Byte 1: temp in low nibble, keep timer bits from template
    out[1] = (out[1] & 0xF0) | ((uint8_t)(s->temp - TEMP_MIN) & 0x0F);

    // Byte 2: turbo(bit4) + light(bit5); keep TimerHours, ModelA, Xfan
    out[2] = (out[2] & ~0x30) | ((s->turbo & 0x01) << 4) | ((s->light & 0x01) << 5);

    // Byte 4: SwingV in low nibble, keep SwingH
    out[4] = (out[4] & 0xF0) | (s->swing & 0x0F);

    // Byte 7: Econo at bit2, checksum in high nibble
    out[7] = ((s->econo & 0x01) << 2);
    uint8_t sum = 10;
    for(int i = 0; i < 4; i++) sum += (out[i] & 0x0F);
    for(int i = 4; i < 7; i++) sum += (out[i] >> 4);
    out[7] |= (sum & 0x0F) << 4;
}

// ---- Encoder: 8 bytes -> raw IR timings. Returns count. ----
static size_t gree_build_timings(const uint8_t frame[8], uint32_t* t) {
    size_t n = 0;
    t[n++] = GREE_HDR_MARK;
    t[n++] = GREE_HDR_SPACE;

    for(int b = 0; b < 4; b++) { // block 1: bytes 0-3
        for(int bit = 0; bit < 8; bit++) {
            t[n++] = GREE_BIT_MARK;
            t[n++] = ((frame[b] >> bit) & 1) ? GREE_ONE_SPACE : GREE_ZERO_SPACE;
        }
    }

    const uint8_t footer = 0b010; // 3-bit footer, LSB first -> 0,1,0
    for(int bit = 0; bit < 3; bit++) {
        t[n++] = GREE_BIT_MARK;
        t[n++] = ((footer >> bit) & 1) ? GREE_ONE_SPACE : GREE_ZERO_SPACE;
    }

    t[n++] = GREE_BIT_MARK; // connector
    t[n++] = GREE_MSG_SPACE;

    for(int b = 4; b < 8; b++) { // block 2: bytes 4-7
        for(int bit = 0; bit < 8; bit++) {
            t[n++] = GREE_BIT_MARK;
            t[n++] = ((frame[b] >> bit) & 1) ? GREE_ONE_SPACE : GREE_ZERO_SPACE;
        }
    }

    t[n++] = GREE_BIT_MARK; // closing mark
    return n; // 139
}

static void gree_transmit(GreeApp* app) {
    uint8_t frame[8];
    uint32_t timings[140];
    gree_build_frame(&app->state, frame);
    size_t cnt = gree_build_timings(frame, timings);
    infrared_send_raw_ext(timings, (uint32_t)cnt, true, GREE_FREQUENCY, GREE_DUTY);
    notification_message(app->notifications, &sequence_blink_green_100);
}

// ---- Field change callbacks ----
static void power_changed(VariableItem* item) {
    GreeApp* app = variable_item_get_context(item);
    uint8_t i = variable_item_get_current_value_index(item);
    app->state.power = i;
    variable_item_set_current_value_text(item, i ? "On" : "Off");
}

static void mode_changed(VariableItem* item) {
    GreeApp* app = variable_item_get_context(item);
    uint8_t i = variable_item_get_current_value_index(item);
    app->state.mode = i;
    variable_item_set_current_value_text(item, mode_names[i]);
}

static void temp_changed(VariableItem* item) {
    GreeApp* app = variable_item_get_context(item);
    uint8_t i = variable_item_get_current_value_index(item);
    app->state.temp = TEMP_MIN + i;
    char buf[8];
    snprintf(buf, sizeof(buf), "%d C", app->state.temp);
    variable_item_set_current_value_text(item, buf);
}

static void fan_changed(VariableItem* item) {
    GreeApp* app = variable_item_get_context(item);
    uint8_t i = variable_item_get_current_value_index(item);
    app->state.fan = i;
    variable_item_set_current_value_text(item, fan_names[i]);
}

static void swing_changed(VariableItem* item) {
    GreeApp* app = variable_item_get_context(item);
    uint8_t i = variable_item_get_current_value_index(item);
    app->state.swing = i; // index == SwingV value (0..6)
    variable_item_set_current_value_text(item, swing_names[i]);
}

static void turbo_changed(VariableItem* item) {
    GreeApp* app = variable_item_get_context(item);
    uint8_t i = variable_item_get_current_value_index(item);
    app->state.turbo = i;
    variable_item_set_current_value_text(item, i ? "On" : "Off");
}

static void sleep_changed(VariableItem* item) {
    GreeApp* app = variable_item_get_context(item);
    uint8_t i = variable_item_get_current_value_index(item);
    app->state.sleep = i;
    variable_item_set_current_value_text(item, i ? "On" : "Off");
}

static void light_changed(VariableItem* item) {
    GreeApp* app = variable_item_get_context(item);
    uint8_t i = variable_item_get_current_value_index(item);
    app->state.light = i;
    variable_item_set_current_value_text(item, i ? "On" : "Off");
}

static void econo_changed(VariableItem* item) {
    GreeApp* app = variable_item_get_context(item);
    uint8_t i = variable_item_get_current_value_index(item);
    app->state.econo = i;
    variable_item_set_current_value_text(item, i ? "On" : "Off");
}

// OK pressed on a remote row
static void remote_enter_callback(void* context, uint32_t index) {
    GreeApp* app = context;
    if(index == ItemSend) {
        gree_transmit(app);
    }
}

// Brand picked -> go to remote
static void brand_callback(void* context, uint32_t index) {
    UNUSED(index);
    GreeApp* app = context;
    view_dispatcher_switch_to_view(app->view_dispatcher, GreeViewRemote);
}

// ---- Navigation (Back button) ----
static uint32_t nav_exit_callback(void* context) {
    UNUSED(context);
    return VIEW_NONE;
}

static uint32_t nav_to_brand_callback(void* context) {
    UNUSED(context);
    return GreeViewBrand;
}

// helper for simple On/Off rows
static VariableItem* add_toggle(
    GreeApp* app, const char* label, uint8_t value, VariableItemChangeCallback cb) {
    VariableItem* item = variable_item_list_add(app->var_list, label, 2, cb, app);
    variable_item_set_current_value_index(item, value);
    variable_item_set_current_value_text(item, value ? "On" : "Off");
    return item;
}

static void build_remote_view(GreeApp* app) {
    VariableItem* item;
    char buf[8];

    add_toggle(app, "Power", app->state.power, power_changed);

    item = variable_item_list_add(app->var_list, "Mode", MODE_COUNT, mode_changed, app);
    variable_item_set_current_value_index(item, app->state.mode);
    variable_item_set_current_value_text(item, mode_names[app->state.mode]);

    item = variable_item_list_add(
        app->var_list, "Temp", (TEMP_MAX - TEMP_MIN + 1), temp_changed, app);
    variable_item_set_current_value_index(item, app->state.temp - TEMP_MIN);
    snprintf(buf, sizeof(buf), "%d C", app->state.temp);
    variable_item_set_current_value_text(item, buf);

    item = variable_item_list_add(app->var_list, "Fan", FAN_COUNT, fan_changed, app);
    variable_item_set_current_value_index(item, app->state.fan);
    variable_item_set_current_value_text(item, fan_names[app->state.fan]);

    item = variable_item_list_add(app->var_list, "Swing", SWING_COUNT, swing_changed, app);
    variable_item_set_current_value_index(item, app->state.swing);
    variable_item_set_current_value_text(item, swing_names[app->state.swing]);

    add_toggle(app, "Turbo", app->state.turbo, turbo_changed);
    add_toggle(app, "Sleep", app->state.sleep, sleep_changed);
    add_toggle(app, "Light", app->state.light, light_changed);
    add_toggle(app, "Econo", app->state.econo, econo_changed);

    // Action row (no value) - sends on OK
    variable_item_list_add(app->var_list, ">> SEND <<", 0, NULL, NULL);

    variable_item_list_set_enter_callback(app->var_list, remote_enter_callback, app);
}

static GreeApp* gree_app_alloc(void) {
    GreeApp* app = malloc(sizeof(GreeApp));

    // Defaults: off, cool, 24C, auto fan, swing off, light on (matches remote)
    app->state.power = 0;
    app->state.mode = 1;
    app->state.temp = 24;
    app->state.fan = 0;
    app->state.swing = 0;
    app->state.turbo = 0;
    app->state.sleep = 0;
    app->state.light = 1;
    app->state.econo = 0;

    app->gui = furi_record_open(RECORD_GUI);
    app->notifications = furi_record_open(RECORD_NOTIFICATION);

    app->view_dispatcher = view_dispatcher_alloc();
    view_dispatcher_attach_to_gui(app->view_dispatcher, app->gui, ViewDispatcherTypeFullscreen);

    app->submenu = submenu_alloc();
    submenu_add_item(app->submenu, "Gree", 0, brand_callback, app);
    view_set_previous_callback(submenu_get_view(app->submenu), nav_exit_callback);
    view_dispatcher_add_view(app->view_dispatcher, GreeViewBrand, submenu_get_view(app->submenu));

    app->var_list = variable_item_list_alloc();
    build_remote_view(app);
    view_set_previous_callback(
        variable_item_list_get_view(app->var_list), nav_to_brand_callback);
    view_dispatcher_add_view(
        app->view_dispatcher, GreeViewRemote, variable_item_list_get_view(app->var_list));

    view_dispatcher_switch_to_view(app->view_dispatcher, GreeViewBrand);
    return app;
}

static void gree_app_free(GreeApp* app) {
    view_dispatcher_remove_view(app->view_dispatcher, GreeViewBrand);
    view_dispatcher_remove_view(app->view_dispatcher, GreeViewRemote);
    submenu_free(app->submenu);
    variable_item_list_free(app->var_list);
    view_dispatcher_free(app->view_dispatcher);
    furi_record_close(RECORD_NOTIFICATION);
    furi_record_close(RECORD_GUI);
    free(app);
}

int32_t gree_remote_app(void* p) {
    UNUSED(p);
    GreeApp* app = gree_app_alloc();
    view_dispatcher_run(app->view_dispatcher);
    gree_app_free(app);
    return 0;
}

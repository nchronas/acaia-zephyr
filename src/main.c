// /*
//  * Copyright (c) 2018 Jan Van Winkel <jan.van_winkel@dxplore.eu>
//  *
//  * SPDX-License-Identifier: Apache-2.0
//  */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>

// #define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
// LOG_MODULE_REGISTER(app);
LOG_MODULE_REGISTER(acaia_zephyr_full, LOG_LEVEL_DBG);


// // static uint32_t count;

// // #ifdef CONFIG_RESET_COUNTER_SW0
// // static struct gpio_dt_spec button_gpio = GPIO_DT_SPEC_GET_OR(
// // 		DT_ALIAS(sw0), gpios, {0});
// // static struct gpio_callback button_callback;

// // static void button_isr_callback(const struct device *port,
// // 				struct gpio_callback *cb,
// // 				uint32_t pins)
// // {
// // 	ARG_UNUSED(port);
// // 	ARG_UNUSED(cb);
// // 	ARG_UNUSED(pins);

// // 	count = 0;
// // }
// // #endif /* CONFIG_RESET_COUNTER_SW0 */

// // #ifdef CONFIG_LV_Z_ENCODER_INPUT
// // static const struct device *lvgl_encoder =
// // 	DEVICE_DT_GET(DT_COMPAT_GET_ANY_STATUS_OKAY(zephyr_lvgl_encoder_input));
// // #endif /* CONFIG_LV_Z_ENCODER_INPUT */

// // #ifdef CONFIG_LV_Z_KEYPAD_INPUT
// // static const struct device *lvgl_keypad =
// // 	DEVICE_DT_GET(DT_COMPAT_GET_ANY_STATUS_OKAY(zephyr_lvgl_keypad_input));
// // #endif /* CONFIG_LV_Z_KEYPAD_INPUT */

// // static void lv_btn_click_callback(lv_event_t *e)
// // {
// // 	ARG_UNUSED(e);

// // 	count = 0;
// // }

//----------------------------------------------------------
/*
 * Zephyr port of the "AcaiaArduinoBLE.cpp" functionality.
 * Full code translation for scanning, connecting, determining scale type,
 * subscribing to weight notifications, and sending commands (tare, heartbeat, etc.).
 *
 * By default, this looks for scales whose names start with:
 *   "CINCO", "ACAIA", "PYXIS", "LUNAR", "PEARL", "PROCH", "BOOKO"
 * 
 * Potential modifications required:
 *   - Adjust the characteristic/service UUIDs to match your actual scale.
 *   - Confirm data lengths and parsing for notifications.
 *   - Tweak scanning intervals/timeouts to your needs.
 *   - Insert robust error handling, reconnection, etc., for production usage.
 */

#include <zephyr/kernel.h>
// #include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/sys/byteorder.h>
#include <math.h>
#include <string.h>
#include <stdio.h>


// -----------------------------------------------------------------------------
// Definitions and Data

#define HEARTBEAT_PERIOD_MS 3000

// Scale types
typedef enum {
    SCALE_OLD,    // Lunar (pre-2021)
    SCALE_NEW,    // Lunar (2021), Pyxis
    SCALE_GENERIC // Felicita Arc, etc
} scale_type_t;

// We'll keep the same sets of commands (from the original code)
static uint8_t IDENTIFY[20]             = { 0xef, 0xdd, 0x0b, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x9a, 0x6d };
static uint8_t HEARTBEAT[7]             = { 0xef, 0xdd, 0x00, 0x02, 0x00, 0x02, 0x00 }; // Original
static uint8_t NOTIFICATION_REQUEST[14] = { 0xef, 0xdd, 0x0c, 0x09, 0x00, 0x01, 0x01, 0x02, 0x02, 0x05, 0x03, 0x04, 0x15, 0x06 };
static uint8_t START_TIMER[7]           = { 0xef, 0xdd, 0x0d, 0x00, 0x00, 0x00, 0x00 };
static uint8_t STOP_TIMER[7]            = { 0xef, 0xdd, 0x0d, 0x00, 0x02, 0x00, 0x02 };
static uint8_t RESET_TIMER[7]           = { 0xef, 0xdd, 0x0d, 0x00, 0x01, 0x00, 0x01 };
static uint8_t TARE_ACAIA[6]            = { 0xef, 0xdd, 0x04, 0x00, 0x00, 0x00 };

// Generic scale commands
static uint8_t TARE_GENERIC[6]          = { 0x03, 0x0a, 0x01, 0x00, 0x00, 0x08 };
static uint8_t START_TIMER_GENERIC[6]   = { 0x03, 0x0a, 0x04, 0x00, 0x00, 0x0a };
static uint8_t STOP_TIMER_GENERIC[6]    = { 0x03, 0x0a, 0x05, 0x00, 0x00, 0x0d };
static uint8_t RESET_TIMER_GENERIC[6]   = { 0x03, 0x0a, 0x06, 0x00, 0x00, 0x0c };

// -----------------------------------------------------------------------------
// Characteristic UUIDs from your definitions:

// OLD version = 16-bit = 0x2a80 (both read and write)
static const struct bt_uuid_16 UUID_OLD_READ  = BT_UUID_INIT_16(0x2a80);
static const struct bt_uuid_16 UUID_OLD_WRITE = BT_UUID_INIT_16(0x2a80);

// NEW version = 128-bit
static const struct bt_uuid_128 UUID_NEW_WRITE = BT_UUID_INIT_128(
    0xb3, 0x9b, 0x72, 0x34, 0xbe, 0xec, 0xd4, 0xa8,
    0xf4, 0x43, 0x41, 0x88, 0x43, 0x53, 0x53, 0x49
);
// ^ "49535343-8841-43f4-a8d4-ecbe34729bb3", reversed for Zephyr's endianness

static const struct bt_uuid_128 UUID_NEW_READ = BT_UUID_INIT_128(
    0x16, 0x96, 0x24, 0x47, 0xc6, 0x23, 0x61, 0xba,
    0xd9, 0x4b, 0x4d, 0x1e, 0x43, 0x53, 0x53, 0x49
);
// ^ "49535343-1e4d-4bd9-ba61-23c647249616"

// GENERIC version = 16-bit
static const struct bt_uuid_16 UUID_GENERIC_READ  = BT_UUID_INIT_16(0xff11);
static const struct bt_uuid_16 UUID_GENERIC_WRITE = BT_UUID_INIT_16(0xff12);

// Known scale name prefixes
static const char *scale_prefixes[] = {
    "CINCO",
    "ACAIA",
    "PYXIS",
    "LUNAR",
    "PEARL",
    "PROCH",
    "BOOKO",
    NULL
};

// -----------------------------------------------------------------------------
// Global Variables to mimic the internal state of the Arduino class

static bool       _debug          = true;  // Toggle debug prints
static float      _currentWeight  = 0.0f;
static bool       _connected      = false;
static scale_type_t _type         = SCALE_OLD;
static uint32_t   _lastHeartBeat  = 0;     // For heartbeat tracking (ms)
static struct bt_conn *default_conn = NULL;

static struct bt_gatt_discover_params discover_params;

// We store references (handles) for read/write characteristics once discovered
static uint16_t _read_handle  = 0x55; // or 0x54
static uint16_t _write_handle = 0x52; // or 0x51
// static uint16_t _read_handle  = 0x54; // or 0x54
// static uint16_t _write_handle = 0x51; // or 0x51

// We'll set this once we confirm we can subscribe to notifications
static bool _subscribed = false;

// Zephyr GATT discovery & subscribe parameters
static struct bt_gatt_discover_params disc_params;
static struct bt_gatt_subscribe_params subscribe_params;
static struct bt_gatt_write_params write_params;

// "new data" tracking to mimic Arduino's newWeightAvailable logic
static bool _newDataAvailable = false;

// Timeout for scanning if user calls init() with no MAC (10s like the Arduino code)
#define SCAN_TIMEOUT_MS 10000

// We won't do MAC-based scanning in this example (like the Arduino code does with scanForAddress),
// but you can adapt as needed. We'll just scan for name prefixes.

static void swap_endianness(uint8_t *dst, const uint8_t *src, size_t length)
{
    for (size_t i = 0; i < length; i++) {
        dst[i] = src[length - 1 - i];
    }
}

// -----------------------------------------------------------------------------
// Utility: Check if a BLE device name matches known scale prefixes
static bool isScaleName(const char* name)
{
    if (!name || !strlen(name)) {
        return false;
    }
    for (int i = 0; scale_prefixes[i] != NULL; i++) {
        if (strncmp(name, scale_prefixes[i], strlen(scale_prefixes[i])) == 0) {
            return true;
        }
    }
    return false;
}

// -----------------------------------------------------------------------------
// Callback for scanning (device_found)

static bool data_cb(struct bt_data *data, void *user_data)
{

	char *name_buf = (char *)user_data;
	if (data->type == BT_DATA_NAME_COMPLETE ||
		data->type == BT_DATA_NAME_SHORTENED) {
		size_t len = MIN(data->data_len, 31);
		memcpy(name_buf, data->data, len);
		name_buf[len] = '\0';
	}
	return true; // continue parsing
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi,
                         uint8_t adv_type, struct net_buf_simple *ad)
{
    char name_found[32];
    memset(name_found, 0, sizeof(name_found));

    // // Parse advertisement data
    // bt_data_parse(ad, [](struct bt_data *data, void *user_data) {

    //     char *name_buf = (char *)user_data;
    //     if (data->type == BT_DATA_NAME_COMPLETE ||
    //         data->type == BT_DATA_NAME_SHORTENED) {
    //         size_t len = MIN(data->data_len, 31);
    //         memcpy(name_buf, data->data, len);
    //         name_buf[len] = '\0';
    //     }
    //     return true; // continue parsing
    // }, name_found);

    bt_data_parse(ad, data_cb, name_found);

    if (isScaleName(name_found)) {
        LOG_INF("Found scale '%s' RSSI %d, connecting...", name_found, rssi);

        // Stop scanning and connect
        bt_le_scan_stop();
        struct bt_le_conn_param *param = BT_LE_CONN_PARAM_DEFAULT;
        int rc = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, param, &default_conn);
        if (rc) {
            LOG_ERR("Failed to create conn (err %d)", rc);
        }
    }
}

/* A convenience function to print all GATT properties for a characteristic. */
static void print_char_properties(const struct bt_gatt_chrc *chrc)
{
    if (!chrc) {
        LOG_INF("No characteristic data");
        return;
    }

    LOG_INF("Properties for this characteristic (handle: 0x%04x):", chrc->value_handle);

    if (chrc->properties & BT_GATT_CHRC_BROADCAST) {
        LOG_INF(" - BROADCAST");
    }
    if (chrc->properties & BT_GATT_CHRC_READ) {
        LOG_INF(" - READ");
    }
    if (chrc->properties & BT_GATT_CHRC_WRITE_WITHOUT_RESP) {
        LOG_INF(" - WRITE WITHOUT RESPONSE");
    }
    if (chrc->properties & BT_GATT_CHRC_WRITE) {
        LOG_INF(" - WRITE");
    }
    if (chrc->properties & BT_GATT_CHRC_NOTIFY) {
        LOG_INF(" - NOTIFY");
    }
    if (chrc->properties & BT_GATT_CHRC_INDICATE) {
        LOG_INF(" - INDICATE");
    }
    if (chrc->properties & BT_GATT_CHRC_AUTH) {
        LOG_INF(" - AUTH SIGNED WRITES");
    }
    if (chrc->properties & BT_GATT_CHRC_EXT_PROP) {
        LOG_INF(" - EXTENDED PROPERTIES");
    }
}


// -----------------------------------------------------------------------------
// When discovery is complete, we can check if we've found the read/write characteristics
static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params)
{
    if (!attr) {
        LOG_INF("Discovery complete");
        memset(params, 0, sizeof(*params));

        // We have discovered all characteristics. Now figure out the scale type.
        // Check which read characteristic is found:
        // if (_read_handle) {
        //     // We'll read the descriptor to see which UUID matched
        //     // However, in Zephyr, we only get the handle, so we must store partial info in the user_data or a separate approach.
        //     // For now, let's see if we "guess" which type was found by also comparing the discovered UUID in real-time:
        // }

        // If we found the OLD read char handle, see if it matches the OLD write char handle
        // We'll do a separate pass for each discovered attribute so let's do a simpler approach:
        // We'll rely on finding the exact characteristic in the callback below. Let’s just end discovery here.
        return BT_GATT_ITER_STOP;
    }

    // GATT characteristic discovered
    struct bt_gatt_chrc *gchrc = (struct bt_gatt_chrc *)attr->user_data;
    if (!gchrc) {
        return BT_GATT_ITER_CONTINUE;
    }

    // The characteristic's UUID:
    struct bt_uuid *uuid = gchrc->uuid;
    if (!uuid) {
        return BT_GATT_ITER_CONTINUE;
    }

    char uuid_str[BT_UUID_STR_LEN];
    bt_uuid_to_str(uuid, uuid_str, sizeof(uuid_str));

    if (_debug) {
        LOG_INF("Discovered char: handle 0x%04x, UUID %s, properties 0x%02x",
                gchrc->value_handle, uuid_str, gchrc->properties);
    }

    if (params->type == BT_GATT_DISCOVER_DESCRIPTOR) {
        /* Check if this is the CCCD */
        // if (!bt_uuid_cmp(attr->uuid, BT_UUID_DESCRIPTOR_CCC)) {
            // ccc_handle = attr->handle;
            LOG_INF("Found CCCD handle: 0x%04x", attr->handle);
        // }
    }

    if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
        const struct bt_gatt_chrc *chr = attr->user_data;
        LOG_INF("Characteristic handle: 0x%04x, value handle: 0x%04x, uuid: ",
               attr->handle, chr->value_handle);
        /* Print the UUID. For 16-bit, easy to cast; for 128-bit, need more logic. */
        if (chr->uuid->type == BT_UUID_TYPE_16) {
            const struct bt_uuid_16 *u16 = BT_UUID_16(chr->uuid);
            LOG_INF("CH: 0x%04x", u16->val);
        } else {
            char uuid_str[37];
            bt_uuid_to_str(chr->uuid, uuid_str, sizeof(uuid_str));
            LOG_INF("CH: %s", uuid_str);
        }
		print_char_properties(chr);
    }

    // Compare with known read/write UUIDs
    // For example, compare memory of our static struct bt_uuid_128:
    // For old scale (16-bit 0x2a80) read/write
    // if (!bt_uuid_cmp(gchrc->uuid, BT_UUID_DECLARE_16(0x2a80))) {
    //     // Old scale
    //     _read_handle  = gchrc->value_handle;  // We might assign the same handle, as your defines are identical
    //     _write_handle = gchrc->value_handle;
    //     _type = SCALE_OLD;
    // }
    // For new scale (128-bit)
    if (!bt_uuid_cmp(gchrc->uuid, &UUID_NEW_READ.uuid)) {
        _read_handle = gchrc->value_handle;
        // _write_handle = gchrc->value_handle;
		// _read_handle = bt_gatt_attr_get_handle	(	attr->handle);	

        // _type = SCALE_NEW;
    } else if (!bt_uuid_cmp(gchrc->uuid, &UUID_NEW_WRITE.uuid)) {
        _write_handle = gchrc->value_handle;
        // _read_handle = gchrc->value_handle;

        // /* Initialize discovery parameters for CCCD */
        // memset(&discover_params, 0, sizeof(discover_params));
        // discover_params.uuid = &gchrc->uuid;
        // discover_params.func = discover_func; // Reuse the same discovery callback
        // discover_params.start_handle = attr->handle + 1;
        // discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
        // discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
        // // discover_params.value_handle = gchrc->value_handle;

        // int err = bt_gatt_discover(conn, &discover_params);
        // if (err) {
        //     LOG_ERR("Descriptor discovery failed (err %d)", err);
        // } else {
        //     LOG_INF("Started CCCD discovery for handle 0x%04x",  gchrc->value_handle);
        // }
        // _write_handle = attr->handle;
		// _write_handle = bt_gatt_attr_get_handle	(	attr->handle);	

        // if (_type != SCALE_NEW) {
        //     _type = SCALE_NEW;
        // }
    }
    // // For generic scale (16-bit ff11 read, ff12 write)
    // else if (!bt_uuid_cmp(gchrc->uuid, BT_UUID_DECLARE_16(0xff11))) {
    //     _read_handle = gchrc->value_handle;
    //     _type = SCALE_GENERIC;
    // } else if (!bt_uuid_cmp(gchrc->uuid, BT_UUID_DECLARE_16(0xff12))) {
    //     _write_handle = gchrc->value_handle;
    //     if (_type != SCALE_GENERIC) {
    //         _type = SCALE_GENERIC;
    //     }
    // }

    return BT_GATT_ITER_CONTINUE;
}

/* Function to discover CCCD for a given characteristic */
static void discover_cccd(struct bt_conn *conn, uint16_t char_handle) {
    /* Initialize discovery parameters for CCCD */
    memset(&discover_params, 0, sizeof(discover_params));
    discover_params.uuid = &UUID_NEW_WRITE.uuid;
    discover_params.func = discover_func; // Reuse the same discovery callback
    discover_params.start_handle = char_handle + 1;
    discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;

    int err = bt_gatt_discover(conn, &discover_params);
    if (err) {
        LOG_ERR("Descriptor discovery failed (err %d)", err);
    } else {
        LOG_INF("Started CCCD discovery for handle 0x%04x", char_handle);
    }
}


// -----------------------------------------------------------------------------
// Notification callback (similar to "newWeightAvailable" logic)
static uint8_t notify_cb(struct bt_conn *conn,
                         struct bt_gatt_subscribe_params *params,
                         const void *data, uint16_t length)
{
    LOG_INF("Notify");
	
    if (!data) {
        LOG_INF("Notification disabled");
        // return BT_GATT_ITER_STOP;
        return BT_GATT_ITER_CONTINUE;

    }

    const uint8_t *pData = data;

    if (_debug) {
        printk("%u bytes: 0x", length);
        for (int i = 0; i < length; i++) {
            printk("%02x", pData[i]);
        }
        printk("\n");
    }

    // Parse weight based on scale type
    // if (_type == SCALE_NEW) {
        // 13 or 17 bytes, old snippet:
        // if ( (13 == length || 17 == length) && pData[4] == 0x05 )
        if ((length == 13 || length == 17) && pData[4] == 0x05) {
            // Weight at bytes [5],[6], scale factor in [9], sign in [10]
            // int16_t raw = ((pData[6] & 0xff) << 8) | (pData[5] & 0xff);
            // float scale  = pow(10, pData[9]);
            // float sign   = (pData[10] & 0x02) ? -1.0f : 1.0f;
            // _currentWeight = (raw / scale) * sign;

                        _currentWeight = (((pData[6] & 0xff) << 8) + (pData[5] & 0xff)) / pow(10,pData[9]);
            _newDataAvailable = true;
            LOG_INF("New weight: %.2f g %u.%u.%u.", _currentWeight, pData[5], pData[6], pData[9]);
        }
    // } else if (_type == SCALE_OLD) {
    //     // 10 bytes, weight in [2],[3], scale factor in [6], sign in [7]
    //     if (length == 10) {
    //         int16_t raw  = ((pData[3] & 0xff) << 8) | (pData[2] & 0xff);
    //         float scale  = pow(10, pData[6]);
    //         float sign   = (pData[7] & 0x02) ? -1.0f : 1.0f;
    //         _currentWeight = (raw / scale) * sign;
    //         _newDataAvailable = true;
    //     }
    // } else if (_type == SCALE_GENERIC) {
    //     // 20 bytes. Weight bytes [7],[8],[9], sign at [6] if ASCII '-'
    //     if (length == 20) {
    //         int32_t raw = ( (pData[7] & 0xff) << 16 ) 
    //                     | ( (pData[8] & 0xff) << 8 ) 
    //                     | ( (pData[9] & 0xff) );
    //         if (pData[6] == 0x2D) { // ASCII '-'
    //             raw = -raw;
    //         }
    //         _currentWeight = raw / 100.0f;
    //         _newDataAvailable = true;
    //     }
    // }

    return BT_GATT_ITER_CONTINUE;
}

// -----------------------------------------------------------------------------
// Callback: connected
static void connected_cb(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err 0x%02x)", err);
        return;
    }

    LOG_INF("Connected!");
    default_conn = bt_conn_ref(conn);
    _connected = true;

    // Start discovery of characteristics
    memset(&disc_params, 0, sizeof(disc_params));
    disc_params.uuid = NULL;  // discover all
    disc_params.func = discover_func;
    disc_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    disc_params.end_handle   = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    // disc_params.type         =  BT_GATT_DISCOVER_PRIMARY;
    disc_params.type         = BT_GATT_DISCOVER_CHARACTERISTIC;


    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));
	LOG_INF("Connected to %s\n", addr_str);

    int rc = bt_gatt_discover(default_conn, &disc_params);
    if (rc) {
        LOG_ERR("bt_gatt_discover failed (err %d)", rc);
    }
}

// -----------------------------------------------------------------------------
// Callback: disconnected
static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason 0x%02x)", reason);
    if (default_conn) {
        bt_conn_unref(default_conn);
        default_conn = NULL;
    }
    _connected      = false;
    _subscribed     = false;
    _read_handle    = 0;
    _write_handle   = 0;
    _type           = SCALE_OLD;
}

// Register connection callbacks
// static struct bt_conn_cb conn_callbacks = {
//     .connected = connected_cb,
//     .disconnected = disconnected_cb
// };

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected_cb,
    .disconnected = disconnected_cb,
};

// -----------------------------------------------------------------------------
// Start scanning
static void start_scanning(void)
{
    struct bt_le_scan_param scan_params = {
        .type       = BT_HCI_LE_SCAN_ACTIVE,
        .options    = BT_LE_SCAN_OPT_NONE,
        .interval   = 0x0030,
        .window     = 0x0030,
        .timeout    = SCAN_TIMEOUT_MS / 10,  // Zephyr scan timeout is in 0.1s units
    };

    LOG_INF("Starting scan for up to %d ms...", SCAN_TIMEOUT_MS);
    int rc = bt_le_scan_start(&scan_params, device_found);
    if (rc) {
        LOG_ERR("bt_le_scan_start failed (err %d)", rc);
    }
}

void temp_cb (struct bt_conn *conn, uint8_t err, struct bt_gatt_write_params *params) {
	LOG_INF("Called");
}
static void identity_req(bool swap);
static void notification_req(bool swap);

// -----------------------------------------------------------------------------
// "init()" equivalent
static bool init_scale(void)
{
    // We mimic the Arduino code's scanning approach (no MAC).
    // If you want a MAC-based approach, you'd adapt accordingly.

    // If we are already connected from a previous attempt, disconnect first
    if (default_conn) {
        bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    }

    // Start scanning
    start_scanning();
    // We do not block in Zephyr; scanning is async. We can either wait or check later.
    // Arduino code used a 10s do-while approach. We can simulate that with a k_sleep or wait for a callback.

    // For simplicity, let's just wait up to 10 seconds until connected or not
    uint32_t start = k_uptime_get_32();
    while (k_uptime_get_32() - start < SCAN_TIMEOUT_MS) {
        if (_connected) {
            break;
        }
        k_sleep(K_MSEC(10));
    }

    if (!_connected) {
        LOG_INF("Failed to find scale within %d ms", SCAN_TIMEOUT_MS);
        return false;
    }

    // Wait for discovery to finish
    k_sleep(K_MSEC(500)); // rough wait so "discover_func" finishes

	bool err = false;
    if (!_write_handle) {
        LOG_ERR("Failed to discover write handles. Disconnecting...");
        bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		err = true;
    }

    if (!_read_handle) {
        LOG_ERR("Failed to discover read handles. Disconnecting...");
        bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		err = true;
    }
	if(err) {
		return false;
	}

    // Subscribe to the read characteristic
    if (!_subscribed) {
        memset(&subscribe_params, 0, sizeof(subscribe_params));
        subscribe_params.notify        = notify_cb;
        // subscribe_params.value         = BT_GATT_CHRC_NOTIFY;
        subscribe_params.value         = BT_GATT_CCC_NOTIFY;
        	 
        subscribe_params.value_handle  = _read_handle;
        subscribe_params.ccc_handle    = _read_handle + 1; 
        // subscribe_params.ccc_handle    = NULL; 
        // Might not always be handle+1 for CCC, but often is. Check actual device or do descriptor discovery.

        int rc = bt_gatt_subscribe(default_conn, &subscribe_params);
        if (rc && rc != -EALREADY) {
            LOG_ERR("Subscribe failed (err %d)", rc);
            return false;
        }
        _subscribed = true;
        LOG_INF("Subscribed to read handle 0x%04x", _read_handle);
    }

	// uint8_t tmp_identify[20];
	// swap_endianness(tmp_identify,IDENTIFY, sizeof(IDENTIFY));

	// uint8_t tmp_not[14];
	// swap_endianness(tmp_not,NOTIFICATION_REQUEST, sizeof(NOTIFICATION_REQUEST));

    // // Write IDENTIFY
    // memset(&write_params, 0, sizeof(write_params));
    // write_params.handle = _write_handle;
    // write_params.data   = tmp_identify;
    // write_params.length = sizeof(IDENTIFY);
    // write_params.func = temp_cb;
    // // int rc = bt_gatt_write_without_response(default_conn, &write_params);
	// // int rc = bt_gatt_write_without_response	(default_conn, _write_handle, IDENTIFY, sizeof(IDENTIFY), false);
    // int rc = bt_gatt_write(default_conn, &write_params);
    // if (rc) {
    //     LOG_ERR("identify write failed (err %d)", rc);
    //     return false;
    // } else {
    //     LOG_INF("identify write successful");
    // }

    // k_sleep(K_MSEC(100));


    // Write NOTIFICATION_REQUEST
    // memset(&write_params, 0, sizeof(write_params));
    // write_params.handle = _write_handle;
    // write_params.data   = tmp_not;
    // write_params.length = sizeof(NOTIFICATION_REQUEST);
    // write_params.func = temp_cb;
    // rc = bt_gatt_write(default_conn, &write_params);
	// // rc = bt_gatt_write_without_response	(default_conn, _write_handle, NOTIFICATION_REQUEST, sizeof(NOTIFICATION_REQUEST), false);
    // if (rc) {
    //     LOG_ERR("notification request write failed (err %d)", rc);
    //     return false;
    // } else {
    //     LOG_INF("notification request write successful");
    // }

	// 	k_sleep(K_MSEC(100));


			identity_req(false);
     		k_sleep(K_MSEC(100));
			notification_req(false);
     		k_sleep(K_MSEC(100));

    return true;
}



// -----------------------------------------------------------------------------
// Helper to do a simple write
static bool write_command(const uint8_t* data, size_t len)
{
    if (!_connected || !_write_handle) {
        LOG_ERR("Not connected or no write_handle!");
        return false;
    }
    memset(&write_params, 0, sizeof(write_params));
    write_params.handle = _write_handle;
    write_params.data   = data;
    write_params.length = len;
	write_params.func = temp_cb;

    int rc = bt_gatt_write(default_conn, &write_params);

		k_sleep(K_MSEC(100));
	// int rc = bt_gatt_write_without_response	(default_conn, _write_handle, data, len, false);
    if (rc) {
        LOG_ERR("Write failed (err %d)", rc);
        _connected = false;
        return false;
    }
    return true;
}

// Tare
static bool tare(void)
{
    bool ok;
    if (_type == SCALE_GENERIC) {
        ok = write_command(TARE_GENERIC, sizeof(TARE_GENERIC));
    } else {
        ok = write_command(TARE_ACAIA, sizeof(TARE_ACAIA));
    }
    LOG_INF("tare write successful %u", ok);
    return ok;
}

// Start Timer
static bool startTimer(void)
{
    bool ok;
    if (_type == SCALE_GENERIC) {
        ok = write_command(START_TIMER_GENERIC, sizeof(START_TIMER_GENERIC));
    } else {
        ok = write_command(START_TIMER, sizeof(START_TIMER));
    }
    LOG_INF("start timer write successful %u", ok);
    return ok;
}

// Stop Timer
static bool stopTimer(void)
{
    bool ok;
    if (_type == SCALE_GENERIC) {
        ok = write_command(STOP_TIMER_GENERIC, sizeof(STOP_TIMER_GENERIC));
    } else {
        ok = write_command(STOP_TIMER, sizeof(STOP_TIMER));
    }
    LOG_INF("stop timer write successful %u", ok);
    return ok;
}

// Reset Timer
static bool resetTimer(void)
{
    bool ok;
    if (_type == SCALE_GENERIC) {
        ok = write_command(RESET_TIMER_GENERIC, sizeof(RESET_TIMER_GENERIC));
    } else {
        ok = write_command(RESET_TIMER, sizeof(RESET_TIMER));
    }
    LOG_INF("reset timer write successful %u", ok);
    return ok;
}

static void identity_req(bool swap) {
    if (!_connected) {
        return false;
    }
	LOG_INF("Identity");
	
	uint8_t tmp[20];
	swap_endianness(tmp, IDENTIFY, sizeof(IDENTIFY));

	bool ok = false;
	if(swap) {
      ok = write_command(tmp, sizeof(IDENTIFY));
	} else {
       ok = write_command(IDENTIFY, sizeof(IDENTIFY));
	}
}

static void notification_req(bool swap) {
    if (!_connected) {
        return false;
    }
	LOG_INF("Notifications");
	
	uint8_t tmp[14];
	swap_endianness(tmp,NOTIFICATION_REQUEST, sizeof(NOTIFICATION_REQUEST));

	bool ok = false;
	if(swap) {
      ok = write_command(tmp, sizeof(NOTIFICATION_REQUEST));
	} else {
       ok = write_command(NOTIFICATION_REQUEST, sizeof(NOTIFICATION_REQUEST));
	}
}

// Heartbeat
static bool heartbeat(bool swap)
{
    if (!_connected || (_type != SCALE_OLD && _type != SCALE_NEW)) {
        // Generic doesn't require heartbeat
        return false;
    }
	LOG_INF("Heartbeat");
	uint8_t tmp[7];
	swap_endianness(tmp,HEARTBEAT, sizeof(HEARTBEAT));

	bool ok = false;
	if(swap) {
      ok = write_command(tmp, sizeof(HEARTBEAT));
	} else {
       ok = write_command(HEARTBEAT, sizeof(HEARTBEAT));
	}

    if (ok) {
		LOG_INF("HR ok");
        _lastHeartBeat = k_uptime_get_32();
    } else {
		LOG_INF("HR not ok");

        _connected = false;
    }
    return ok;
}

// getWeight
static float getWeight(void)
{
    return _currentWeight;
}

// isConnected
static bool isConnected(void)
{
    return _connected;
}

// heartbeatRequired
static bool heartbeatRequired(void)
{
    // if (_type == SCALE_OLD || _type == SCALE_NEW) {
        uint32_t now = k_uptime_get_32();
        return (now - _lastHeartBeat) > HEARTBEAT_PERIOD_MS;
    // } else {
    //     return false;
    // }
}

// newWeightAvailable
static bool newWeightAvailable(void)
{
    bool tmp = _newDataAvailable;
    _newDataAvailable = false;  // reset
    return tmp;
}

// -----------------------------------------------------------------------------
// main()

// void main(void)
// {
//     LOG_INF("Zephyr full code translation of AcaiaArduinoBLE example starting...");

//     int err = bt_enable(NULL);
//     if (err) {
//         LOG_ERR("Bluetooth init failed (err %d)", err);
//         return;
//     }
//     LOG_INF("Bluetooth initialized");

//     // Register connection callbacks
//     bt_conn_cb_register(&conn_callbacks);

//     // Initialize scale (scan, connect, discover, subscribe, identify, etc.)
//     if (!init_scale()) {
//         LOG_INF("init_scale() failed. Exiting main.");
//         return;
//     }

//     LOG_INF("Scale is connected, type = %d", _type);

//     // Main loop
//     while (1) {
//         k_sleep(K_MSEC(1000));

//         // If scale requires heartbeat, do it
//         if (heartbeatRequired()) {
//             heartbeat();
//         }

//         // Check for new weight
//         if (newWeightAvailable()) {
//             float w = getWeight();
//             LOG_INF("New weight: %.2f g", w);
//         }

//         // Demo: call various commands (uncomment if you want to test).
//         // For instance:
//         // tare();
//         // startTimer();
//         // stopTimer();
//         // resetTimer();
//     }
// }

//----------------------------------------------------------

char w_str[100];

int main(void)
{
// 	char count_str[11] = {0};
// 	const struct device *display_dev;
// 	lv_obj_t *hello_world_label;
// 	lv_obj_t *count_label;

// 	display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
// 	if (!device_is_ready(display_dev)) {
// 		LOG_ERR("Device not ready, aborting test");
// 		return 0;
// 	}

// #ifdef CONFIG_RESET_COUNTER_SW0
// 	if (gpio_is_ready_dt(&button_gpio)) {
// 		int err;

// 		err = gpio_pin_configure_dt(&button_gpio, GPIO_INPUT);
// 		if (err) {
// 			LOG_ERR("failed to configure button gpio: %d", err);
// 			return 0;
// 		}

// 		gpio_init_callback(&button_callback, button_isr_callback,
// 				   BIT(button_gpio.pin));

// 		err = gpio_add_callback(button_gpio.port, &button_callback);
// 		if (err) {
// 			LOG_ERR("failed to add button callback: %d", err);
// 			return 0;
// 		}

// 		err = gpio_pin_interrupt_configure_dt(&button_gpio,
// 						      GPIO_INT_EDGE_TO_ACTIVE);
// 		if (err) {
// 			LOG_ERR("failed to enable button callback: %d", err);
// 			return 0;
// 		}
// 	}
// #endif /* CONFIG_RESET_COUNTER_SW0 */

// #ifdef CONFIG_LV_Z_ENCODER_INPUT
// 	lv_obj_t *arc;
// 	lv_group_t *arc_group;

// 	arc = lv_arc_create(lv_scr_act());
// 	lv_obj_align(arc, LV_ALIGN_CENTER, 0, -15);
// 	lv_obj_set_size(arc, 150, 150);

// 	arc_group = lv_group_create();
// 	lv_group_add_obj(arc_group, arc);
// 	lv_indev_set_group(lvgl_input_get_indev(lvgl_encoder), arc_group);
// #endif /* CONFIG_LV_Z_ENCODER_INPUT */

// #ifdef CONFIG_LV_Z_KEYPAD_INPUT
// 	lv_obj_t *btn_matrix;
// 	lv_group_t *btn_matrix_group;
// 	static const char *const btnm_map[] = {"1", "2", "3", "4", ""};

// 	btn_matrix = lv_btnmatrix_create(lv_scr_act());
// 	lv_obj_align(btn_matrix, LV_ALIGN_CENTER, 0, 70);
// 	lv_btnmatrix_set_map(btn_matrix, (const char **)btnm_map);
// 	lv_obj_set_size(btn_matrix, 100, 50);

// 	btn_matrix_group = lv_group_create();
// 	lv_group_add_obj(btn_matrix_group, btn_matrix);
// 	lv_indev_set_group(lvgl_input_get_indev(lvgl_keypad), btn_matrix_group);
// #endif /* CONFIG_LV_Z_KEYPAD_INPUT */

// 	if (IS_ENABLED(CONFIG_LV_Z_POINTER_INPUT)) {
// 		lv_obj_t *hello_world_button;

// 		hello_world_button = lv_btn_create(lv_scr_act());
// 		lv_obj_align(hello_world_button, LV_ALIGN_CENTER, 0, -15);
// 		lv_obj_add_event_cb(hello_world_button, lv_btn_click_callback, LV_EVENT_CLICKED,
// 				    NULL);
// 		hello_world_label = lv_label_create(hello_world_button);
// 	} else {
// 		hello_world_label = lv_label_create(lv_scr_act());
// 	}

// 	lv_label_set_text(hello_world_label, "Hello world!");
// 	lv_obj_align(hello_world_label, LV_ALIGN_CENTER, 0, 0);

// 	count_label = lv_label_create(lv_scr_act());
// 	lv_obj_align(count_label, LV_ALIGN_BOTTOM_MID, 0, 0);

// 	lv_task_handler();
// 	display_blanking_off(display_dev);

    LOG_INF("Zephyr full code translation of AcaiaArduinoBLE example starting...");

    int err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }
    LOG_INF("Bluetooth initialized");

    // Register connection callbacks
    // bt_conn_cb_register(&conn_callbacks);

    // Initialize scale (scan, connect, discover, subscribe, identify, etc.)
    if (!init_scale()) {
        LOG_INF("init_scale() failed. Exiting main.");
        return;
    }




    LOG_INF("Scale is connected, type = %d", _connected);

    // discover_cccd(default_conn, _read_handle);

	while (1) {
		// if ((count % 100) == 0U) {
		// 	sprintf(count_str, "%d", count/100U);
		// 	lv_label_set_text(count_label, count_str);
		// }
		// lv_task_handler();
		// ++count;
		// k_sleep(K_MSEC(10));


        // If scale requires heartbeat, do it
        // if (heartbeatRequired()) {
        //     heartbeat();
     	// 	k_sleep(K_MSEC(1000));

		// 	notification_req();
        // }

            heartbeat(false);
     		// k_sleep(K_MSEC(100));
			// notification_req(true);
     		// k_sleep(K_MSEC(100));
			// identity_req(true);

     		// k_sleep(K_MSEC(10));
            // heartbeat(false);
     		// k_sleep(K_MSEC(100));
			// notification_req(false);
     		// k_sleep(K_MSEC(100));
			// identity_req(false);
	
		k_sleep(K_MSEC(2500));

        			notification_req(false);
     		// k_sleep(K_MSEC(100));

    LOG_INF("Is connected?, type = %d", _connected);


        // Check for new weight
        if (newWeightAvailable()) {
            float w = getWeight();
            LOG_INF("New weight: %.2f g", w);
		    // sprintf(w_str, "New weight: %.2f g", w);
			// lv_label_set_text(hello_world_label, w_str);
        }
	}
}

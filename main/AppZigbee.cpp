#include "AppZigbee.hpp"

#include <stdint.h>
#include "stdio.h"
#include "string.h"

#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "ha/esp_zigbee_ha_standard.h"

static const char *TAG = "AppZigbee";

// --- LOCAL TYPEDEFS ---

// Mirror of the ZB attributes
typedef struct {
    uint8_t current_position_lift_percentage;
    uint8_t installed_open_limit_lift;
    uint8_t installed_closed_limit_lift;
    uint8_t current_position_tilt_percentage;
    uint8_t installed_open_limit_tilt;
    uint8_t installed_closed_limit_tilt;
} zb_window_covering_attrs_t;

// --- LOCAL VARIABLES  ---
static zb_window_covering_attrs_t window_covering_ctx = {
    .current_position_lift_percentage = 0,
    .installed_open_limit_lift        = 0,
    .installed_closed_limit_lift      = 100,
    // Unused but necessary for the bridge
    .current_position_tilt_percentage = 0,
    .installed_open_limit_tilt        = 0,
    .installed_closed_limit_tilt      = 0,
};

static constexpr uint8_t WINDOW_COVERING_ENDPOINT = 1;

// --- STATIC FUNCTIONS DECLARATION ---
static void ZigbeeStartTopLevelCommissionningHandler(uint8_t mode_mask);
static esp_err_t ZigbeeAttributeHandler(const esp_zb_zcl_set_attr_value_message_t* message);
static esp_err_t ZigbeeHandler(esp_zb_core_action_callback_id_t callback_id, const void *message);
static void ZigbeeTask(void *params);

// --- STATIC FUNCTIONS DEFINITION ---
static void ZigbeeStartTopLevelCommissionningHandler(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee commissioning");
}

// App-level signal handler
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t* p_sg_p        = signal_struct->p_app_signal;
    esp_err_t err_status    = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = static_cast<esp_zb_app_signal_type_t>(*p_sg_p);

    switch (sig_type)
    {
    // Normal start
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
    {
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    }
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
    {
        if (err_status == ESP_OK)
        {
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");

            // Clean start = no network in storage
            if (esp_zb_bdb_is_factory_new())
            {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            }
            else
            {
                ESP_LOGI(TAG, "Device rebooted");
            }
        }
        else
        {
            // Commissioning failed
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }
        break;
    }
    case ESP_ZB_BDB_SIGNAL_STEERING:
    {
        // Network joined
        if (err_status == ESP_OK)
        {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);

            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        }
        // Failed to join network = restart steering process
        else
        {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t) ZigbeeStartTopLevelCommissionningHandler, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    }
    // Unhandled signal
    default:
    {
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
    }
}

// Zigbee attribute handler
static esp_err_t ZigbeeAttributeHandler(const esp_zb_zcl_set_attr_value_message_t* message)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)", message->info.status);

    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);

    if (message->info.dst_endpoint == WINDOW_COVERING_ENDPOINT)
    {
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING)
        {
            ESP_LOGI(TAG, "Received update for attribute %d with value %d", message->attribute.id, (int) message->attribute.data.value);
        }
    }
    return ret;
}

// Zigbee event handler
static esp_err_t ZigbeeHandler(esp_zb_core_action_callback_id_t callback_id, const void* message)
{
    esp_err_t ret = ESP_OK;

    switch (callback_id)
    {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
    {
        // Attribute has to be updated
        ret = ZigbeeAttributeHandler((esp_zb_zcl_set_attr_value_message_t*) message);
        break;
    }
    default:
    {
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    }

    return ret;
}

// Main Zigbee task
static void ZigbeeTask(void *params)
{
    // Initialize Zigbee stack
    esp_zb_cfg_t zb_nwk_cfg = {};
    zb_nwk_cfg.esp_zb_role            = ESP_ZB_DEVICE_TYPE_ROUTER;
    zb_nwk_cfg.install_code_policy    = false;
    zb_nwk_cfg.nwk_cfg.zed_cfg        = {
            .ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_64MIN,
            .keep_alive = 3000,
        };

    // Initialise low-level Zigbee
    esp_zb_init(&zb_nwk_cfg);

    // Configure the endpoint
    esp_zb_window_covering_cfg_t window_covering_cfg        = ESP_ZB_DEFAULT_WINDOW_COVERING_CONFIG();
    // Basic config amend: power source
    window_covering_cfg.basic_cfg.power_source              = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE;
    // Set window covering parameters
    window_covering_cfg.window_cfg.covering_mode            = ESP_ZB_ZCL_ATTR_WINDOW_COVERING_TYPE_LEDS_WILL_DISPLAY_FEEDBACK;
    window_covering_cfg.window_cfg.covering_type            = ESP_ZB_ZCL_ATTR_WINDOW_COVERING_TYPE_ROLLERSHADE_EXTERIOR;
    window_covering_cfg.window_cfg.covering_status          = ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_OPERATIONAL | ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_ONLINE;

    esp_zb_ep_list_t *esp_zb_window_covering_ep  = esp_zb_window_covering_ep_create(WINDOW_COVERING_ENDPOINT, &window_covering_cfg);

    esp_zb_cluster_list_t   *cluster_list       = NULL;
    esp_zb_attribute_list_t *basic_cluster      = NULL;
    esp_zb_attribute_list_t *covering_cluster   = NULL;

    char* manufacturer_id  = "\x07""Balth.D";
    char* device_id        = "\x15""ZB_Things covering v2";

    cluster_list = esp_zb_ep_list_get_ep(esp_zb_window_covering_ep, WINDOW_COVERING_ENDPOINT);
    basic_cluster = esp_zb_cluster_list_get_cluster(cluster_list, ESP_ZB_ZCL_CLUSTER_ID_BASIC, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    // Set device manufacturer and name
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, static_cast<void*>(manufacturer_id)));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,  static_cast<void*>(device_id)));
    
    // Register usecase clusters
    covering_cluster = esp_zb_cluster_list_get_cluster(cluster_list, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    ESP_ERROR_CHECK(esp_zb_window_covering_cluster_add_attr(covering_cluster, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID,  &window_covering_ctx.current_position_lift_percentage));
    ESP_ERROR_CHECK(esp_zb_window_covering_cluster_add_attr(covering_cluster, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_INSTALLED_OPEN_LIMIT_LIFT_ID,         &window_covering_ctx.installed_open_limit_lift));
    ESP_ERROR_CHECK(esp_zb_window_covering_cluster_add_attr(covering_cluster, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_INSTALLED_CLOSED_LIMIT_LIFT_ID,       &window_covering_ctx.installed_closed_limit_lift));

    ESP_ERROR_CHECK(esp_zb_window_covering_cluster_add_attr(covering_cluster, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_TILT_PERCENTAGE_ID,  &window_covering_ctx.current_position_tilt_percentage));
    ESP_ERROR_CHECK(esp_zb_window_covering_cluster_add_attr(covering_cluster, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_INSTALLED_OPEN_LIMIT_TILT_ID,         &window_covering_ctx.installed_open_limit_tilt));
    ESP_ERROR_CHECK(esp_zb_window_covering_cluster_add_attr(covering_cluster, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_INSTALLED_CLOSED_LIMIT_TILT_ID,       &window_covering_ctx.installed_closed_limit_tilt));

    // Register endpoint
    esp_zb_device_register(esp_zb_window_covering_ep);
    
    // Register Zigbee main handler
    esp_zb_core_action_handler_register(ZigbeeHandler);

    // All ZB channels can be used
    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);

    // Start main ZB task
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

// --- EXPOSED FUNCTIONS ---
bool AppZigbee_Init(void)
{
    esp_zb_platform_config_t config = {};
    config.radio_config.radio_mode          = ZB_RADIO_MODE_NATIVE;
    config.host_config.host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE;

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    xTaskCreate(ZigbeeTask, "Task-Zigbee", 4096, NULL, 5, NULL);

    return true;
}

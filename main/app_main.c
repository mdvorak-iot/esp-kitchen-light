#include "app_status.h"
#include <app_rainmaker.h>
#include <app_wifi.h>
#include <double_reset.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_params.h>
#include <esp_rmaker_standard_types.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <string.h>
#include <sys/cdefs.h>
#include <wifi_reconnect.h>

static const char TAG[] = "app_main";

#define APP_DEVICE_NAME CONFIG_APP_DEVICE_NAME
#define APP_DEVICE_TYPE CONFIG_APP_DEVICE_TYPE

// Params
#define APP_RMAKER_DEF_DUTY_NAME "Duty"

static esp_rmaker_param_t *duty_param = NULL;

// Program
static void app_devices_init(esp_rmaker_node_t *node);

void setup()
{
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // System services
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Check double reset
    // NOTE this should be called as soon as possible, ideally right after nvs init
    bool reconfigure = false;
    ESP_ERROR_CHECK_WITHOUT_ABORT(double_reset_start(&reconfigure, DOUBLE_RESET_DEFAULT_TIMEOUT));

    // Setup
    app_status_init();

    struct app_wifi_config wifi_cfg = {
        .security = WIFI_PROV_SECURITY_1,
        .wifi_connect = wifi_reconnect_resume,
    };
    ESP_ERROR_CHECK(app_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MAX_MODEM));
    ESP_ERROR_CHECK(wifi_reconnect_start());

    // RainMaker
    char node_name[APP_RMAKER_NODE_NAME_LEN] = {};
    ESP_ERROR_CHECK(app_rmaker_node_name(node_name, sizeof(node_name)));

    esp_rmaker_node_t *node = NULL;
    ESP_ERROR_CHECK(app_rmaker_init(node_name, &node));

    app_devices_init(node);

    // Hardware
    ESP_ERROR_CHECK(ledc_fade_func_install(0));

    ledc_timer_config_t timerConfig = {};
    timerConfig.timer_num = LEDC_TIMER_0;
    timerConfig.speed_mode = LEDC_HIGH_SPEED_MODE;
    timerConfig.duty_resolution = LEDC_TIMER_10_BIT;
    timerConfig.freq_hz = 75000;
    ESP_ERROR_CHECK(ledc_timer_config(&timerConfig));

    // Channel
    ledc_channel_config_t channelConfig = {};
    channelConfig.timer_sel = LEDC_TIMER_0;
    channelConfig.channel = LEDC_CHANNEL_0;
    channelConfig.gpio_num = GPIO_NUM_23;
    channelConfig.speed_mode = LEDC_HIGH_SPEED_MODE;
    channelConfig.duty = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&channelConfig));

    // Start
    //    ESP_ERROR_CHECK(tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, node_name)); // NOTE this isn't available before WiFi init
    //    ESP_ERROR_CHECK(esp_rmaker_start());
    //    ESP_ERROR_CHECK(app_wifi_start(reconfigure));

    // Done
    ESP_LOGI(TAG, "setup complete");
}

static double percent = 0.0;

_Noreturn void app_main()
{
    setup();

    // Run
    ESP_LOGI(TAG, "life is good");

    for (;;)
    {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        uint32_t duty = percent * 10.23;
        ESP_LOGI(TAG, "duty=%d", duty);
        ledc_set_duty_and_update(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty, 0);

        percent += 10;
        if (percent > 100) percent = 0;
    }
}

static esp_err_t device_write_cb(__unused const esp_rmaker_device_t *device, const esp_rmaker_param_t *param,
                                 const esp_rmaker_param_val_t val, __unused void *private_data,
                                 __unused esp_rmaker_write_ctx_t *ctx)
{
    char *name = esp_rmaker_param_get_name(param);
    if (strcmp(name, APP_RMAKER_DEF_DUTY_NAME) == 0)
    {
        float percent = val.val.f < 0 ? 0 : val.val.f;
        if (percent > 100) percent = 100;

        uint32_t duty = (uint32_t)(percent * 10.23f);
        esp_err_t err = ledc_set_duty_and_update(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty, 0);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "ledc_set_duty_and_update failed: %d %s", err, esp_err_to_name(err));
            return err;
        }

        return esp_rmaker_param_update_and_report(param, val);
    }
    return ESP_OK;
}

static void app_devices_init(esp_rmaker_node_t *node)
{
    // Prepare device
    esp_rmaker_device_t *device = esp_rmaker_device_create(APP_DEVICE_NAME, APP_DEVICE_TYPE, NULL);
    assert(device);

    ESP_ERROR_CHECK(esp_rmaker_device_add_cb(device, device_write_cb, NULL));
    ESP_ERROR_CHECK(esp_rmaker_device_add_param(device, esp_rmaker_name_param_create(ESP_RMAKER_DEF_NAME_PARAM, APP_DEVICE_NAME)));
    ESP_ERROR_CHECK(esp_rmaker_node_add_device(node, device));

    // Register buttons, sensors, etc
    duty_param = esp_rmaker_param_create(APP_RMAKER_DEF_DUTY_NAME, ESP_RMAKER_PARAM_SPEED, esp_rmaker_float(100.0f), PROP_FLAG_READ | PROP_FLAG_WRITE);
    ESP_ERROR_CHECK(esp_rmaker_param_add_ui_type(duty_param, ESP_RMAKER_UI_SLIDER));
    ESP_ERROR_CHECK(esp_rmaker_param_add_bounds(duty_param, esp_rmaker_float(0), esp_rmaker_float(100), esp_rmaker_float(0)));
    ESP_ERROR_CHECK(esp_rmaker_device_add_param(device, duty_param));
}

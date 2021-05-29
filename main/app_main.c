#include "app_status.h"
#include <app_wifi.h>
#include <double_reset.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <string.h>
#include <wifi_reconnect.h>

static const char TAG[] = "app_main";

#define HW_PWM_PIN CONFIG_HW_PWM_PIN
#define HW_PWM_FREQUENCY CONFIG_HW_PWM_FREQUENCY
#define HW_PWM_RESOLUTION CONFIG_HW_PWM_RESOLUTION
#define HW_PWM_MAX_DUTY ((1u << (HW_PWM_RESOLUTION)) - 1)
#define HW_SWITCH_PIN CONFIG_HW_SWITCH_PIN
#define HW_MOTION_OUTPUT_PIN CONFIG_HW_MOTION_OUTPUT_PIN

// Program
void hardware_init();

static void print_qrcode_handler(__unused void *arg, __unused esp_event_base_t event_base,
                                 __unused int32_t event_id, __unused void *event_data)
{
    const char VER[] = "v1";
    char payload[200] = {};
    // {"ver":"%s","name":"%s","pop":"%s","transport":"%s"}
    snprintf(payload, sizeof(payload), "%%7B%%22ver%%22%%3A%%22%s%%22%%2C%%22name%%22%%3A%%22%s%%22%%2C%%22pop%%22%%3A%%22%s%%22%%2C%%22transport%%22%%3A%%22%s%%22%%7D",
             VER, app_wifi_prov_get_service_name(), app_wifi_get_prov_pop(), APP_WIFI_PROV_TRANSPORT);
    // NOTE print this regardless of log level settings
    printf("PROVISIONING: To view QR Code, copy paste the URL in a browser:\n%s?data=%s\n", "https://espressif.github.io/esp-jumpstart/qrcode.html", payload);
}

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
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_PROV_EVENT, WIFI_PROV_START, print_qrcode_handler, NULL, NULL));

    // Hardware
    hardware_init();

    // Start
    ESP_ERROR_CHECK(app_wifi_start(reconfigure));

    // Done
    ESP_LOGI(TAG, "setup complete");
}

void hardware_init()
{
    // PWM
    ESP_ERROR_CHECK(ledc_fade_func_install(0));

    ledc_timer_config_t timerConfig = {};
    timerConfig.timer_num = LEDC_TIMER_0;
    timerConfig.speed_mode = LEDC_HIGH_SPEED_MODE;
    timerConfig.duty_resolution = HW_PWM_RESOLUTION;
    timerConfig.freq_hz = HW_PWM_FREQUENCY;
    ESP_ERROR_CHECK(ledc_timer_config(&timerConfig));

    ledc_channel_config_t channelConfig = {};
    channelConfig.timer_sel = LEDC_TIMER_0;
    channelConfig.channel = LEDC_CHANNEL_0;
    channelConfig.gpio_num = HW_PWM_PIN;
    channelConfig.speed_mode = LEDC_HIGH_SPEED_MODE;
    channelConfig.duty = HW_PWM_MAX_DUTY;
    ESP_ERROR_CHECK(ledc_channel_config(&channelConfig));

    // Switch
    ESP_ERROR_CHECK(gpio_reset_pin(HW_SWITCH_PIN));
    ESP_ERROR_CHECK(gpio_set_direction(HW_SWITCH_PIN, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(HW_SWITCH_PIN, GPIO_PULLUP_ONLY));
    // TODO

    // Motion sensor
    ESP_ERROR_CHECK(gpio_reset_pin(HW_MOTION_OUTPUT_PIN));
    ESP_ERROR_CHECK(gpio_set_direction(HW_MOTION_OUTPUT_PIN, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(HW_MOTION_OUTPUT_PIN, GPIO_PULLDOWN_ONLY));
}

static double percent = 0.0;

_Noreturn void app_main()
{
    setup();

    // Run
    ESP_LOGI(TAG, "life is good");

    for (;;)
    {
        vTaskDelay(700 / portTICK_PERIOD_MS);
        uint32_t duty = percent * 10.23;
        ESP_LOGI(TAG, "duty=%d", duty);
        ledc_set_duty_and_update(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty, 0);

        percent += 2;
        if (percent > 100) percent = 0;
    }
}

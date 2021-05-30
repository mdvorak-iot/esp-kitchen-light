#include "app_status.h"
#include <app_wifi.h>
#include <double_reset.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <freertos/event_groups.h>
#include <nvs_flash.h>
#include <string.h>
#include <wifi_reconnect.h>

static const char TAG[] = "app_main";

// Config
#define HW_PWM_PIN (CONFIG_HW_PWM_PIN)
#define HW_PWM_FREQUENCY (CONFIG_HW_PWM_FREQUENCY)
#define HW_PWM_RESOLUTION (CONFIG_HW_PWM_RESOLUTION)
#define HW_PWM_MAX_DUTY ((1u << (HW_PWM_RESOLUTION)) - 1)
#define HW_PWM_INVERTED (CONFIG_HW_PWM_INVERTED)
#define HW_SWITCH_PIN (CONFIG_HW_SWITCH_PIN)
#define HW_MOTION_OUTPUT_PIN (CONFIG_HW_MOTION_OUTPUT_PIN)

#define APP_SWITCH_AUTO_OFF_SEC (CONFIG_APP_SWITCH_AUTO_OFF_SEC)
#define APP_MOTION_AUTO_OFF_SEC (CONFIG_APP_MOTION_AUTO_OFF_SEC)
#define APP_PWM_FADE_STEP (CONFIG_APP_PWM_FADE_STEP)

#define SEC_TO_MICRO(sec) ((int64_t)(sec)*1000000L)

// State
#define STATE_CHANGED (BIT0)
#define DUTY_PERCENT_MAX (1000)

static EventGroupHandle_t *state_event = NULL;
static int64_t power_auto_off_time = SEC_TO_MICRO(APP_MOTION_AUTO_OFF_SEC);
static uint32_t current_duty_percent = DUTY_PERCENT_MAX;
static uint32_t target_duty_percent = DUTY_PERCENT_MAX;

// Program
void hardware_init();
void switch_handler(__unused void *arg);
void motion_handler(__unused void *arg);

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
    state_event = xEventGroupCreate();

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
#if HW_PWM_INVERTED
    channelConfig.duty = 0;
#else
    channelConfig.duty = HW_PWM_MAX_DUTY;
#endif
    ESP_ERROR_CHECK(ledc_channel_config(&channelConfig));

    // Switch
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    ESP_ERROR_CHECK(gpio_reset_pin(HW_SWITCH_PIN));
    ESP_ERROR_CHECK(gpio_set_direction(HW_SWITCH_PIN, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(HW_SWITCH_PIN, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(gpio_set_intr_type(HW_SWITCH_PIN, GPIO_INTR_POSEDGE));
    ESP_ERROR_CHECK(gpio_intr_enable(HW_SWITCH_PIN));
    ESP_ERROR_CHECK(gpio_isr_handler_add(HW_SWITCH_PIN, switch_handler, NULL));

    ESP_ERROR_CHECK(gpio_reset_pin(HW_MOTION_OUTPUT_PIN));
    ESP_ERROR_CHECK(gpio_set_direction(HW_MOTION_OUTPUT_PIN, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(HW_MOTION_OUTPUT_PIN, GPIO_PULLDOWN_ONLY));
    ESP_ERROR_CHECK(gpio_set_intr_type(HW_MOTION_OUTPUT_PIN, GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_intr_enable(HW_MOTION_OUTPUT_PIN));
    ESP_ERROR_CHECK(gpio_isr_handler_add(HW_MOTION_OUTPUT_PIN, motion_handler, NULL));
}

void switch_handler(__unused void *arg)
{
    target_duty_percent = target_duty_percent ? 0 : DUTY_PERCENT_MAX;
    power_auto_off_time = esp_timer_get_time() + SEC_TO_MICRO(APP_SWITCH_AUTO_OFF_SEC);
    xEventGroupSetBitsFromISR(state_event, STATE_CHANGED, NULL);
}

void motion_handler(__unused void *arg)
{
    if (gpio_get_level(HW_MOTION_OUTPUT_PIN))
    {
        target_duty_percent = DUTY_PERCENT_MAX;
    }
    power_auto_off_time = esp_timer_get_time() + SEC_TO_MICRO(APP_MOTION_AUTO_OFF_SEC);
    xEventGroupSetBitsFromISR(state_event, STATE_CHANGED, NULL);
}

static void set_duty(uint32_t percent)
{
    uint32_t duty = percent * HW_PWM_MAX_DUTY / DUTY_PERCENT_MAX;
#if HW_PWM_INVERTED
    duty = HW_PWM_MAX_DUTY - duty;
#endif
    ESP_LOGI(TAG, "set duty to %d%% (%u)", percent, duty);
    ledc_set_duty_and_update(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty, 0);
}

_Noreturn void app_main()
{
    setup();

    // Run
    ESP_LOGI(TAG, "life is good");

    for (;;)
    {
        // Auto-off
        if (target_duty_percent && (esp_timer_get_time() - power_auto_off_time) > 0)
        {
            target_duty_percent = 0;
        }

        // Set duty cycle
        if (current_duty_percent < target_duty_percent)
        {
            current_duty_percent += APP_PWM_FADE_STEP;
            if (current_duty_percent > DUTY_PERCENT_MAX)
                current_duty_percent = DUTY_PERCENT_MAX;

            set_duty(current_duty_percent);
        }
        if (current_duty_percent > target_duty_percent)
        {
            if (current_duty_percent > APP_PWM_FADE_STEP)
                current_duty_percent -= APP_PWM_FADE_STEP;
            else
                current_duty_percent = 0;

            set_duty(current_duty_percent);
        }

        // Wait for a change
        if (current_duty_percent == target_duty_percent)
        {
            int64_t remaining = power_auto_off_time - esp_timer_get_time();
            TickType_t wait_ticks = target_duty_percent && remaining > 0 ? pdMS_TO_TICKS(remaining / 1000) : portMAX_DELAY;
            ESP_LOGI(TAG, "waiting for %u ms", pdTICKS_TO_MS(wait_ticks));
            xEventGroupWaitBits(state_event, STATE_CHANGED, pdTRUE, pdFALSE, wait_ticks);
        }

        // Sanity wait
        vTaskDelay(1);
    }
}

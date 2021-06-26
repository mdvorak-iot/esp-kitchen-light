#pragma once

#include <esp_err.h>
#include <esp_event_base.h>
#include <hal/gpio_types.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

ESP_EVENT_DECLARE_BASE(SWITCH_EVENT);

enum switch_event
{
    SWITCH_EVENT_ACTION,
};

enum switch_handler_type
{
    SWITCH_HANDLER_ON_PRESS,
    SWITCH_HANDLER_ON_RELEASE,
};

enum switch_mode
{
    SWITCH_MODE_LOW_ON_PRESS = 0,
    SWITCH_MODE_HIGH_ON_PRESS = 1,
};

enum switch_internal_pull
{
    SWITCH_INTERNAL_PULL_ENABLE,
    SWITCH_INTERNAL_PULL_DISABLE,
};

struct switch_data
{
    gpio_num_t pin;
    uint32_t press_length_ms;
    bool long_press;
};

struct switch_config
{
    gpio_num_t pin;
    uint32_t long_press_ms;
    enum switch_mode mode;
    enum switch_internal_pull internal_pull;
    enum switch_handler_type handler_type;
    esp_event_loop_handle_t event_loop;
};

esp_err_t switch_config(const struct switch_config *cfg);

esp_err_t switch_remove(gpio_num_t pin);

#ifdef __cplusplus
}
#endif

#include "switch.h"
#include <driver/gpio.h>
#include <esp_event.h>
#include <esp_log.h>
#include <memory.h>

static const char TAG[] = "switch";

ESP_EVENT_DEFINE_BASE(SWITCH_EVENT);

// TODO Kconfig
#define SWITCH_DEBOUNCE_MS 50

struct switch_state
{
    uint32_t long_press_ms;
    enum switch_mode mode;
    enum switch_handler_type handler_type;
    esp_event_loop_handle_t event_loop;
    esp_timer_handle_t timer;
    volatile int64_t press_start;
    volatile bool pressed;
};

static struct switch_state *switch_states[GPIO_NUM_MAX] = {};

inline static bool IRAM_ATTR is_pressed(const struct switch_state *state, int level)
{
    return level == state->mode;
}

static void on_release(gpio_num_t pin, struct switch_state *state)
{
    // Released
    int64_t now = esp_timer_get_time();
    int64_t press_length_ms = (now - state->press_start) / 1000L; // us to ms

    struct switch_data data = {
        .pin = pin,
        .press_length_ms = press_length_ms,
        .long_press = (press_length_ms >= state->long_press_ms),
    };

    esp_event_isr_post_to(state->event_loop, SWITCH_EVENT, SWITCH_EVENT_ACTION, &data, sizeof(data), NULL);

    // Reset press start, in case of rare race-condition of the timer and ISR
    state->press_start = now;
}

static void debounce_timer_handler(void *arg)
{
    gpio_num_t pin = (size_t)arg; // pin stored directly as the pointer
    struct switch_state *state = switch_states[pin];

    int level = gpio_get_level(pin);
    if (!is_pressed(state, level))
    {
        // Button released during debounce interval
        on_release(pin, state);
    }
    else if (state->handler_type == SWITCH_HANDLER_ON_PRESS && state->long_press_ms > 0)
    {
        // Start new timer, that will fire long-press event, even when button is not released yet
        // TODO
    }

    gpio_intr_enable(pin);
}

static void IRAM_ATTR switch_interrupt_handler(void *arg)
{
    // Dereference
    gpio_num_t pin = (size_t)arg; // pin stored directly as the pointer
    struct switch_state *state = switch_states[pin];

    // Debounce
    int64_t now = esp_timer_get_time();
    if (state->press_start - now > SWITCH_DEBOUNCE_MS * 1000)
    {
        // Ignore this interrupt
        return;
    }

    // Current state
    int level = gpio_get_level(pin);

    if (is_pressed(state, level))
    {
        // NOTE since this is edge handler, button was just pressed
        state->press_start = now;

        // No further interrupts till timer has finished
        gpio_intr_disable(pin);

        // Start timer
        // NOTE error is ignored intentionally
        esp_timer_start_once(state->timer, SWITCH_DEBOUNCE_MS * 1000);
    }
    else
    {
        // NOTE since this is edge handler, button was just released
        on_release(pin, state);
    }
}

esp_err_t switch_config(const struct switch_config *cfg)
{
    if (cfg == NULL || !GPIO_IS_VALID_GPIO(cfg->pin))
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Configure GPIO
    gpio_config_t gpio_cfg = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = BIT64(cfg->pin),
        .intr_type = GPIO_INTR_ANYEDGE,
        .pull_up_en = (cfg->internal_pull && cfg->mode == SWITCH_MODE_LOW_ON_PRESS) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = (cfg->internal_pull && cfg->mode == SWITCH_MODE_HIGH_ON_PRESS) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
    };

    esp_err_t err = gpio_config(&gpio_cfg);
    if (err != ESP_OK)
    {
        return err;
    }

    // Initialize internal state
    struct switch_state *state = switch_states[cfg->pin];

    if (state == NULL)
    {
        state = switch_states[cfg->pin] = malloc(sizeof(*state));
        memset(state, 0, sizeof(*state));

        esp_timer_create_args_t timer_cfg = {
            .callback = debounce_timer_handler,
            .arg = (void *)cfg->pin,
            .name = "switch",
        };
        err = esp_timer_create(&timer_cfg, &state->timer);
        if (err != ESP_OK)
        {
            return err;
        }
    }
    state->mode = cfg->mode;
    state->handler_type = cfg->handler_type;
    state->long_press_ms = cfg->long_press_ms;
    state->event_loop = cfg->event_loop;

    // Register interrupt handler
    err = gpio_isr_handler_add(cfg->pin, switch_interrupt_handler, (void *)cfg->pin); // NOTE using pin value directly in the pointer, not as a pointer
    if (err != ESP_OK)
    {
        return err;
    }

    ESP_LOGI(TAG, "configured switch on pin %d", cfg->pin);
    return ESP_OK;
}

esp_err_t switch_remove(gpio_num_t pin)
{
    struct switch_state *state = switch_states[pin];
    switch_states[pin] = NULL;
    free(state);

    esp_err_t err = gpio_isr_handler_remove(pin);
    if (err != ESP_OK)
    {
        return err;
    }

    return gpio_reset_pin(pin);
}

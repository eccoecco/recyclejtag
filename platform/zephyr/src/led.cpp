#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(rjtag, LOG_LEVEL_INF);

#include "led.h"

#define TARGET_LED_NODE DT_ALIAS(led0)

#if DT_NODE_EXISTS(TARGET_LED_NODE) == 0

#pragma message("Missing LED for board.  LED support disabled")

extern "C"
{
void led_init(void)
{
}

void led_update_status(enum led_normal_status status)
{
    (void)status;
}

void led_update_error(enum led_error_status status)
{
    (void)status;
}
}

#else

extern "C"
{
static void on_led_timer_callback(k_work *work);
}

namespace
{

const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(TARGET_LED_NODE, gpios);

constexpr auto led_work_period = K_MSEC(200);

K_WORK_DELAYABLE_DEFINE(led_delayable_work, on_led_timer_callback);

} // namespace

extern "C"
{

static void on_led_timer_callback(k_work *work)
{
    k_work_schedule(&led_delayable_work, led_work_period);
}

void led_init(void)
{
    k_work_schedule(&led_delayable_work, led_work_period);

    gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);

    gpio_pin_set_dt(&led, 0);
}

void led_update_status(enum led_normal_status status)
{
}

void led_update_error(enum led_error_status status)
{
}
}

#endif
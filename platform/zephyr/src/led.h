/*!
    \file led.h
    \brief Used to control the feedback LED
    \note Callable from C, but internally C++
*/

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

void led_init(void);

enum led_normal_status
{
    LED_IDLE,
    LED_ACTIVE,
};

void led_update_status(enum led_normal_status status);

enum led_error_status
{
    LED_ERROR_NONE,
    LED_ERROR_USB_FAULT,
    LED_ERROR_HARDWARE_FAULT,
};

void led_update_error(enum led_error_status status);

#ifdef __cplusplus
}
#endif

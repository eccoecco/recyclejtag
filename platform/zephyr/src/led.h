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

// These enums are the blink pattern
enum led_normal_status
{
    LED_NORMAL_IDLE = 0x18000, //!< Short time on, long time off
    LED_NORMAL_ACTIVE = 0x3E0, //!< Medium time on, medium time off
};

void led_update_status(enum led_normal_status status);

enum led_error_status
{
    LED_ERROR_NONE = 0,                 //!< No error - no blink
    LED_ERROR_USB_FAULT = 0x14000,      //!< Two fast blinks
    LED_ERROR_HARDWARE_FAULT = 0x15000, //!< Three fast blinks
};

void led_update_error(enum led_error_status status);

#ifdef __cplusplus
}
#endif

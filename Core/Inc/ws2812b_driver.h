#ifndef __WS2812B_DRIVER_H__
#define __WS2812B_DRIVER_H__
#include <stdint.h>

// Sets a specific led in the custom_led_data array using (r,g,b) values.
void set_led_by_rgb(uint8_t led_idx, uint8_t red, uint8_t green, uint8_t blue);

// Sets a specific led in the custom_led_data array using a hexcode RGB.
void set_led_by_code(uint8_t led_idx, uint32_t colorcode);

// Set all leds in the custom_led_data array using (r,g,b) values.
void set_all_leds_by_rgb(uint8_t red, uint8_t green, uint8_t blue);

// Set all leds in the custom_led_data array using hexcode rgb.
void set_all_leds_by_code(uint32_t colorcode);

/**
 * Sends led data (assumed to be 2d array led_data[NUM_LEDS][3]) to the LED matrix. It blocks if a DMA transfer is ongoing.
 * The implementation does the following:
 * Processes each bit in the led_data array:
 * 1) Scale it with the brightness factor (assumed to be in [0, 4095]).
 * 2) Convert into duty cycle data that correspond to the bit logic level
 * 3) Store it in the pwm_dc_buffer.
 * Start timer 1 in DMA mode and drive channel 1 in PWM mode.
 * Once the DMA transfer is done, start another DMA transfer to send reset code.
 */
void send_led_data(uint8_t led_data[][3], uint16_t brightness);

// Blinks led matrix. Implemented in a non-blocking way. 
// Led matrix will spend `duration` time [ms] on with specified color (hexcode rgb) and another `duration` time off.
// Start time is the time [ms] when the animation started, used for determining which led image to display at current time. 
void show_blinking_animation(uint32_t start_time, uint32_t duration, uint32_t color_code, uint16_t brightness);

// Show a husky animation. Impelmented in a non-blocking way. 
void show_husky_animation(uint32_t start_time, uint16_t brightness);
#endif
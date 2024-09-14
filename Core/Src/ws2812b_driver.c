#include "main.h"
#include "ws2812b_driver.h"
#include "led_constants.h"
#include <stdbool.h>

// Global variables for the driver
volatile bool led_data_sent_flag = false;

// Sets a specific led in the custom_led_data array using (r,g,b) values.
void set_led_by_rgb(uint8_t led_idx, uint8_t red, uint8_t green, uint8_t blue) {
    custom_led_data[led_idx][0] = green;
    custom_led_data[led_idx][1] = red;
    custom_led_data[led_idx][2] = blue;
}

// Sets a specific led in the custom_led_data array using a hexcode RGB.
void set_led_by_code(uint8_t led_idx, uint32_t colorcode) {
    custom_led_data[led_idx][0] = (colorcode >> 8) & 0xFF;
    custom_led_data[led_idx][1] = (colorcode >> 16) & 0xFF;
    custom_led_data[led_idx][2] = (colorcode >> 0) & 0xFF;
}

// Set all leds in the custom_led_data array using (r,g,b) values.
void set_all_leds_by_rgb(uint8_t red, uint8_t green, uint8_t blue) {
    for (int i = 0; i < NUM_LEDS; i++) {
        set_led_by_rgb(i, red, green, blue);
    }
}

// Set all leds in the custom_led_data array using hexcode rgb.
void set_all_leds_by_code(uint32_t colorcode) {
    for (int i = 0; i < NUM_LEDS; i++) {
        set_led_by_code(i, colorcode);
    }
}

// Send reset code (signal LOW for 40 cycles of 1.25us) between data frames, required by the LED matrix. 
void send_reset() {
    while (hdma_tim1_ch1.State != HAL_DMA_STATE_READY) {}
    memset(pwm_dc_buffer, 0, sizeof(pwm_dc_buffer));
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t*)pwm_dc_buffer, 30);  // Send 30 cycles is enough (60us tested).
}

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
void send_led_data(uint8_t led_data[][3], uint16_t brightness) {
    // Note: The first two and last two entries of pwm_dc_buffer are by default 0s (0% DC). The actual DC values will be between these padding 0s.
    // The first two is used because during testing, the first PWM cycles sent doesnt have the exact DC as indicated, so I added some 0s DC before sending the actual DC values.
    // The last two is used because it makes sure the signal stays LOW while stopping timer1 with HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1); Otherwise it would produce some garbage. 

    // Do not modify pwm_dc_buffer until previous DMA transfer is done.
    while (hdma_tim1_ch1.State != HAL_DMA_STATE_READY) {}

    // Prepare pwm_dc_buffer from led data.
    for (int i = 0; i < NUM_LEDS; i++) {
        for (int j = 0; j < 3; j++) {
            // Scale each byte by brightness. Note that brightness = 0 doesn't mean black. 
            uint8_t led_grb_byte = led_data[i][j];
            uint8_t led_grb_byte_scaled = ((uint32_t)led_grb_byte * (brightness + 1)) / (ADC_MAX_VALUE + 1);
            // Convert each bit into duty cycle data.
            int bit_idx = 0;
            for (int k = 7; k >= 0; k--) {
                uint8_t bit = led_grb_byte_scaled >> k & 0b1;
                uint16_t dc = (bit == 1) ? HIGH_DC : LOW_DC;
                // Store the duty cycles in pwm_dc_buffer.
                // Led matrix protocol is to send in MSB first. 
                pwm_dc_buffer[2 + i * NUM_BITS_PER_LED + j * 8 + bit_idx] = dc;
                bit_idx++;
            }
        }
    }

    // Send pwm_dc_buffer by starting Timer in DMA mode while driving channel 1 in PWM mode.
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t*)pwm_dc_buffer, sizeof(pwm_dc_buffer) / sizeof(uint16_t));

    // Set the flag so the DMA callback function sends reset code right after this DMA transfer is done. 
    led_data_sent_flag = true;
}

// Blinks led matrix. Implemented in a non-blocking way. 
// Led matrix will spend `duration` time [ms] on with specified color (hexcode rgb) and another `duration` time off.
// Start time is the time [ms] when the animation started, used for determining which led image to display at current time. 
void show_blinking_animation(uint32_t start_time, uint32_t duration, uint32_t color_code, uint16_t brightness) {
    uint32_t current_time = HAL_GetTick();
    uint64_t cycle_duration = 2 * duration;
    if ((current_time - start_time) % cycle_duration <= cycle_duration / 2) {
        set_all_leds_by_code(color_code);
        send_led_data(custom_led_data, brightness);
    } else {
        set_all_leds_by_rgb(0, 0, 0);
        send_led_data(custom_led_data, brightness);
    }
}

// Show a husky animation.
void show_husky_animation(uint32_t start_time, uint16_t brightness) {
    uint32_t current_time = HAL_GetTick();
    uint32_t HUSKY_2_DURATION = 500;
    uint32_t HUSKY_1_DURATION = 1500;
    uint32_t total_duration = HUSKY_1_DURATION + HUSKY_2_DURATION;
    if ((current_time - start_time) % total_duration <= HUSKY_2_DURATION) {
        send_led_data(HUSKY_2, brightness);
    } else {
        send_led_data(HUSKY_1, brightness);
    }
}


// Callback function when the PWM sequence (DMA transfer) is finished.
// It sends the reset code if a led data is just sent. Otherwise do nothing. 
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim) {
    if (led_data_sent_flag == true) {
        send_reset();
        led_data_sent_flag = false;
    }
}


/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
 /* USER CODE END Header */

 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H


/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

// LED array driver parameters
// Everything configurable except PWM_DC_BUFFER_SIZE, because if it's less than NUM_LEDS, it will require slightly more complicated implementation (approach 2 in readme.md).
#define NUM_BITS_PER_LED 24
#define NUM_LEDS 64
#define PWM_DC_BUFFER_SIZE NUM_LEDS  // At least >= 2 Unit: [# LEDS to store]. Code only supports NUM_LEDs at the moment, which makes pwm_dc_buffer use 3KB if 64 leds.
#define ADC_MAX_VALUE 4095           // 12-bit resolution.
#define PERIOD 30                    // period of the timer (autoreload register)
#define LOW_DC 10                    // 1/3 * period = 10.
#define HIGH_DC 20                   // 2/3 * period = 20

// CAN parameters
#define BRAKE_MSG_ID 0x5EC
#define BRAKE_ON  0x01         // First byte of message corresponding to brake state
#define BRAKE_OFF 0x00        //  First byte of message corresponding to no brake state
#define TARGET_MSG_ID BRAKE_MSG_ID  // This is the ID that the CAN controller listen to (the CAN filter will only allow through this value)
#define CAN_DLC 8                   // How many bytes is sent.
#define TIMEOUT_PERIOD 2000 // Assumes MPU continuously send brake message. If it doesn't hear from MPU for this period in ms, then brakelight will blink yellow to indicate error.   

// Conditional compiling parameters
#define DEBUG 0

extern TIM_HandleTypeDef htim1;
extern DMA_HandleTypeDef hdma_tim1_ch1;
extern uint8_t custom_led_data[NUM_LEDS][3];                               // Array that is filled with set_led functions. This essentially "draw" a custom led data in code, useful for testing.
extern uint16_t pwm_dc_buffer[PWM_DC_BUFFER_SIZE * NUM_BITS_PER_LED + 4];  // Array that contains values to be assigned to the compare register TIMx_CCRx (duty cycle). The additional 4 is for padding 0s between the real DC values, which makes PWM + DMA work perfectly without wrong cycles.

// Enum values to identify currently displayed image on the led matrix. 
enum DISPLAY {
    BRAKE_ON_IMG,
    BRAKE_OFF_IMG,
    OTHER_IMG
};


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define LED_ARRAY_DIN_Pin GPIO_PIN_8
#define LED_ARRAY_DIN_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_8
#define LD2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */


#endif /* __MAIN_H */

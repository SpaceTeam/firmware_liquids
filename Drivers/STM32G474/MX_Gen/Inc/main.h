/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_comp.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_cordic.h"
#include "stm32g4xx_ll_dac.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_i2c.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TEMP_0_ADC2_6_Pin LL_GPIO_PIN_0
#define TEMP_0_ADC2_6_GPIO_Port GPIOC
#define TEMP_1_ADC2_7_Pin LL_GPIO_PIN_1
#define TEMP_1_ADC2_7_GPIO_Port GPIOC
#define PRESS_0_ADC2_8_Pin LL_GPIO_PIN_2
#define PRESS_0_ADC2_8_GPIO_Port GPIOC
#define PRESS_1_ADC2_9_Pin LL_GPIO_PIN_3
#define PRESS_1_ADC2_9_GPIO_Port GPIOC
#define CURR_A_COMP3P_ADC2_1_Pin LL_GPIO_PIN_0
#define CURR_A_COMP3P_ADC2_1_GPIO_Port GPIOA
#define CURR_A_COMP1P_Pin LL_GPIO_PIN_1
#define CURR_A_COMP1P_GPIO_Port GPIOA
#define CURR_B_COMP2P_Pin LL_GPIO_PIN_3
#define CURR_B_COMP2P_GPIO_Port GPIOA
#define SPI1_NSS_Pin LL_GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define PRESS_2_ADC2_5_Pin LL_GPIO_PIN_4
#define PRESS_2_ADC2_5_GPIO_Port GPIOC
#define PRESS_3_ADC2_11_Pin LL_GPIO_PIN_5
#define PRESS_3_ADC2_11_GPIO_Port GPIOC
#define CURR_B_COMP4P_ADC1_15_Pin LL_GPIO_PIN_0
#define CURR_B_COMP4P_ADC1_15_GPIO_Port GPIOB
#define AL_TIM1_1N_Pin LL_GPIO_PIN_8
#define AL_TIM1_1N_GPIO_Port GPIOE
#define AH_TIM1_1_Pin LL_GPIO_PIN_9
#define AH_TIM1_1_GPIO_Port GPIOE
#define BL_TIM1_2N_Pin LL_GPIO_PIN_10
#define BL_TIM1_2N_GPIO_Port GPIOE
#define BH_TIM1_2_Pin LL_GPIO_PIN_11
#define BH_TIM1_2_GPIO_Port GPIOE
#define CL_TIM1_3N_Pin LL_GPIO_PIN_12
#define CL_TIM1_3N_GPIO_Port GPIOE
#define CH_TIM1_3_Pin LL_GPIO_PIN_13
#define CH_TIM1_3_GPIO_Port GPIOE
#define IGN_0_Pin LL_GPIO_PIN_14
#define IGN_0_GPIO_Port GPIOE
#define IGN_1_Pin LL_GPIO_PIN_15
#define IGN_1_GPIO_Port GPIOE
#define BRAKE_TIM2_3_Pin LL_GPIO_PIN_10
#define BRAKE_TIM2_3_GPIO_Port GPIOB
#define CURR_C_COMP6P_Pin LL_GPIO_PIN_11
#define CURR_C_COMP6P_GPIO_Port GPIOB
#define FAULT_Pin LL_GPIO_PIN_12
#define FAULT_GPIO_Port GPIOB
#define CURR_C_COMP5P_ADC3_5_Pin LL_GPIO_PIN_13
#define CURR_C_COMP5P_ADC3_5_GPIO_Port GPIOB
#define VSENSE_COMP7P_ADC4_4_Pin LL_GPIO_PIN_14
#define VSENSE_COMP7P_ADC4_4_GPIO_Port GPIOB
#define EN_Pin LL_GPIO_PIN_15
#define EN_GPIO_Port GPIOB
#define CAL_Pin LL_GPIO_PIN_8
#define CAL_GPIO_Port GPIOD
#define TEMP_ADC3_7_Pin LL_GPIO_PIN_10
#define TEMP_ADC3_7_GPIO_Port GPIOD
#define TP_1_Pin LL_GPIO_PIN_6
#define TP_1_GPIO_Port GPIOC
#define TP_2_Pin LL_GPIO_PIN_7
#define TP_2_GPIO_Port GPIOC
#define TP_3_Pin LL_GPIO_PIN_8
#define TP_3_GPIO_Port GPIOA
#define LED_1_Pin LL_GPIO_PIN_11
#define LED_1_GPIO_Port GPIOA
#define LED_2_Pin LL_GPIO_PIN_12
#define LED_2_GPIO_Port GPIOA
#define SPI3_NSS_Pin LL_GPIO_PIN_15
#define SPI3_NSS_GPIO_Port GPIOA
#define DIP_1_Pin LL_GPIO_PIN_1
#define DIP_1_GPIO_Port GPIOD
#define DIP_2_Pin LL_GPIO_PIN_2
#define DIP_2_GPIO_Port GPIOD
#define ENC_I_TIM3_ETR_Pin LL_GPIO_PIN_3
#define ENC_I_TIM3_ETR_GPIO_Port GPIOB
#define ENC_A_TIM3_1_Pin LL_GPIO_PIN_4
#define ENC_A_TIM3_1_GPIO_Port GPIOB
#define ENC_B_TIM3_2_Pin LL_GPIO_PIN_5
#define ENC_B_TIM3_2_GPIO_Port GPIOB
#define DIP_3_Pin LL_GPIO_PIN_6
#define DIP_3_GPIO_Port GPIOB
#define DIP_4_Pin LL_GPIO_PIN_7
#define DIP_4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

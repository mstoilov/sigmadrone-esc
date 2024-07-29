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

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIM1_PERIOD_CLOCKS 12000
#define TIM1_RCR 0
#define SYSTEM_CORE_CLOCK 480000000
#define INPUT_1__Pin GPIO_PIN_2
#define INPUT_1__GPIO_Port GPIOE
#define INPUT_2__Pin GPIO_PIN_4
#define INPUT_2__GPIO_Port GPIOE
#define BRAKE_A_FB_Pin GPIO_PIN_5
#define BRAKE_A_FB_GPIO_Port GPIOE
#define BRAKE_B_FB_Pin GPIO_PIN_6
#define BRAKE_B_FB_GPIO_Port GPIOE
#define DRV1_CS_Pin GPIO_PIN_13
#define DRV1_CS_GPIO_Port GPIOC
#define DRV2_CS_Pin GPIO_PIN_14
#define DRV2_CS_GPIO_Port GPIOC
#define DRV_FAULT__Pin GPIO_PIN_15
#define DRV_FAULT__GPIO_Port GPIOC
#define M2_SOA_M2_IA_FB_Pin GPIO_PIN_0
#define M2_SOA_M2_IA_FB_GPIO_Port GPIOC
#define M2_SOB_M2_IB_FB_Pin GPIO_PIN_1
#define M2_SOB_M2_IB_FB_GPIO_Port GPIOC
#define DRV_SDO_Pin GPIO_PIN_2
#define DRV_SDO_GPIO_Port GPIOC
#define M2_SOC_M2_IC_FB_Pin GPIO_PIN_3
#define M2_SOC_M2_IC_FB_GPIO_Port GPIOC
#define PULSE_M2_TIM5_CH1_Pin GPIO_PIN_0
#define PULSE_M2_TIM5_CH1_GPIO_Port GPIOA
#define M1_SOA_M1_IA_FB_Pin GPIO_PIN_1
#define M1_SOA_M1_IA_FB_GPIO_Port GPIOA
#define M1_SOB_M1_IB_FB_Pin GPIO_PIN_2
#define M1_SOB_M1_IB_FB_GPIO_Port GPIOA
#define ANALOG_SPEED_CTRL_Pin GPIO_PIN_3
#define ANALOG_SPEED_CTRL_GPIO_Port GPIOA
#define M1_SOC_M1_IC_FB_Pin GPIO_PIN_4
#define M1_SOC_M1_IC_FB_GPIO_Port GPIOA
#define PULSE_M1_TIM2_CH1_Pin GPIO_PIN_5
#define PULSE_M1_TIM2_CH1_GPIO_Port GPIOA
#define VBAT_ADC_Pin GPIO_PIN_6
#define VBAT_ADC_GPIO_Port GPIOA
#define M2_AL_Pin GPIO_PIN_7
#define M2_AL_GPIO_Port GPIOA
#define TEMP2_Pin GPIO_PIN_4
#define TEMP2_GPIO_Port GPIOC
#define TEMP1_Pin GPIO_PIN_5
#define TEMP1_GPIO_Port GPIOC
#define M2_CL_Pin GPIO_PIN_1
#define M2_CL_GPIO_Port GPIOB
#define GATE_M2_ENABLE_Pin GPIO_PIN_2
#define GATE_M2_ENABLE_GPIO_Port GPIOB
#define M1_AL_Pin GPIO_PIN_8
#define M1_AL_GPIO_Port GPIOE
#define M1_AH_Pin GPIO_PIN_9
#define M1_AH_GPIO_Port GPIOE
#define M1_BL_Pin GPIO_PIN_10
#define M1_BL_GPIO_Port GPIOE
#define M1_BH_Pin GPIO_PIN_11
#define M1_BH_GPIO_Port GPIOE
#define M1_CL_Pin GPIO_PIN_12
#define M1_CL_GPIO_Port GPIOE
#define M1_CH_Pin GPIO_PIN_13
#define M1_CH_GPIO_Port GPIOE
#define DRV_CAL_Pin GPIO_PIN_14
#define DRV_CAL_GPIO_Port GPIOE
#define GATE_M1_ENABLE_Pin GPIO_PIN_15
#define GATE_M1_ENABLE_GPIO_Port GPIOE
#define QUAD_Z_M1_Pin GPIO_PIN_11
#define QUAD_Z_M1_GPIO_Port GPIOB
#define OUTPUT_1_Pin GPIO_PIN_12
#define OUTPUT_1_GPIO_Port GPIOB
#define OUTPUT_2_Pin GPIO_PIN_13
#define OUTPUT_2_GPIO_Port GPIOB
#define M2_BL_Pin GPIO_PIN_14
#define M2_BL_GPIO_Port GPIOB
#define DRV_SDI_Pin GPIO_PIN_15
#define DRV_SDI_GPIO_Port GPIOB
#define ENCODER_DI_M1_Pin GPIO_PIN_8
#define ENCODER_DI_M1_GPIO_Port GPIOD
#define ENCODER_RO_M1_Pin GPIO_PIN_9
#define ENCODER_RO_M1_GPIO_Port GPIOD
#define M1_SW1_Pin GPIO_PIN_10
#define M1_SW1_GPIO_Port GPIOD
#define M1_SW2_Pin GPIO_PIN_11
#define M1_SW2_GPIO_Port GPIOD
#define ENCODER_DE_M1_Pin GPIO_PIN_12
#define ENCODER_DE_M1_GPIO_Port GPIOD
#define DIRECTION_A_Pin GPIO_PIN_13
#define DIRECTION_A_GPIO_Port GPIOD
#define LED_WARN_Pin GPIO_PIN_14
#define LED_WARN_GPIO_Port GPIOD
#define DIRECTION_B_Pin GPIO_PIN_15
#define DIRECTION_B_GPIO_Port GPIOD
#define M2_AH_Pin GPIO_PIN_6
#define M2_AH_GPIO_Port GPIOC
#define M2_BH_Pin GPIO_PIN_7
#define M2_BH_GPIO_Port GPIOC
#define M2_CH_Pin GPIO_PIN_8
#define M2_CH_GPIO_Port GPIOC
#define LED_STATUS_Pin GPIO_PIN_9
#define LED_STATUS_GPIO_Port GPIOC
#define DBG_USART1_RX_Pin GPIO_PIN_9
#define DBG_USART1_RX_GPIO_Port GPIOA
#define DBG_USART1_TX_Pin GPIO_PIN_10
#define DBG_USART1_TX_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TD_422_UART4_Pin GPIO_PIN_10
#define TD_422_UART4_GPIO_Port GPIOC
#define RD_422_UART4_Pin GPIO_PIN_11
#define RD_422_UART4_GPIO_Port GPIOC
#define BRAKE_B_RELEASE_Pin GPIO_PIN_12
#define BRAKE_B_RELEASE_GPIO_Port GPIOC
#define CAR_RX_Pin GPIO_PIN_0
#define CAR_RX_GPIO_Port GPIOD
#define CAN_TX_Pin GPIO_PIN_1
#define CAN_TX_GPIO_Port GPIOD
#define M2_SW1_Pin GPIO_PIN_2
#define M2_SW1_GPIO_Port GPIOD
#define M2_SW2_Pin GPIO_PIN_3
#define M2_SW2_GPIO_Port GPIOD
#define ENCODER_DE_M2_Pin GPIO_PIN_4
#define ENCODER_DE_M2_GPIO_Port GPIOD
#define ENCODER_DI_M2_Pin GPIO_PIN_5
#define ENCODER_DI_M2_GPIO_Port GPIOD
#define ENCODER_RO_M2_Pin GPIO_PIN_6
#define ENCODER_RO_M2_GPIO_Port GPIOD
#define QUAD_Z_M2_Pin GPIO_PIN_7
#define QUAD_Z_M2_GPIO_Port GPIOD
#define QUAD_A_M1_Pin GPIO_PIN_4
#define QUAD_A_M1_GPIO_Port GPIOB
#define QUAD_B_M1_Pin GPIO_PIN_5
#define QUAD_B_M1_GPIO_Port GPIOB
#define QUAD_A_M2_Pin GPIO_PIN_6
#define QUAD_A_M2_GPIO_Port GPIOB
#define QUAD_B_M2_Pin GPIO_PIN_7
#define QUAD_B_M2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

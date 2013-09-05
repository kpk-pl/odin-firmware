/**
  ******************************************************************************
  * @file    InfraRed/RC5_Decoding_TIM_EXTI/inc/RC5_IR_Emul_Receiver.h
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    25-January-2012
  * @brief   This file contains the RC5 functions prototypes.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * FOR MORE INFORMATION PLEASE READ CAREFULLY THE LICENSE AGREEMENT FILE
  * LOCATED IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RC5_TIM_EXTI_H
#define __RC5_TIM_EXTI_H

#ifdef __cplusplus
 extern "C" {
#endif 
   
/* Includes ------------------------------------------------------------------*/

#include "priorities.h"

/* Exported types ------------------------------------------------------------*/

/** @addtogroup STM32F10x_Infrared_Decoders
  * @{
  */

/** @addtogroup RC5_Decoding_TIM_EXTI
  * @brief RC5 Decoding using Timer and EXTI driver modules
  * @{
  */ 

/** @addtogroup RC5_IR_EMUL_RECEIVER
  * @brief RC5 Decoding using Timer and EXTI driver modules
  * @{
  */ 

typedef enum { NO = 0, YES = !NO} StatusYesOrNo;

/* RC5 frame structure*/
typedef struct
{
  __IO uint8_t ToggleBit;  /* Toggle bit field */
  __IO uint8_t Address;    /* Address field */ 
  __IO uint8_t Command;    /* Command field */ 
} RC5Frame_TypeDef;

/* Exported constants --------------------------------------------------------*/
#define RC5_GPIO_PORT          GPIOD                /* Port which RC5 is connected */
#define RC5_GPIO_CLK           RCC_AHB1Periph_GPIOD /* Clock of Port which RC5 is connected */
#define RC5_GPIO_CLK_FUN	   RCC_AHB1PeriphClockCmd
#define RC5_GPIO_PIN           GPIO_Pin_14          /* Pin which RC5 is connected */
#define RC5_EXTI_PORT_SOURCE   EXTI_PortSourceGPIOD /* RC5 EXTI Port source */
#define RC5_EXTI_PIN_SOURCE    EXTI_PinSource14     /* RC5 EXTI Pin source */
#define RC5_EXTI_IRQn          EXTI15_10_IRQn         /* RC5 EXTI IRQ */
#define RC5_EXTI_LINE          EXTI_Line14          /* RC5 EXTI line */

#define RC5_TIM                TIM4                 /* Timer used for RC5 decoding */
#define RC5_TIM_CLK            RCC_APB1Periph_TIM4  /* Clock of the used timer */
#define RC5_TIM_IRQn           TIM4_IRQn            /* RC5 TIM IRQ */

// Remember to create IRQ Handlers and invoke the right procedures!

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void RC5_Receiver_Init(void);
void RC5_Sample_Data(void);
void RC5_MeasureFirstLowDuration(void);
void RC5_Decode(RC5Frame_TypeDef * RC5_frame);

#ifdef __cplusplus
}
#endif

#endif /* __RC5_IR_EMUL_RECEIVER_H */
/**
  * @}
  */ 

/**
  * @}
  */
  
/**
  * @}
  */
/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/

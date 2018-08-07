/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "stdint.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define CommunicationLED_Pin GPIO_PIN_13
#define CommunicationLED_GPIO_Port GPIOC
#define CTR485B_EN_Pin GPIO_PIN_14
#define CTR485B_EN_GPIO_Port GPIOB
#define CTR485A_EN_Pin GPIO_PIN_1
#define CTR485A_EN_GPIO_Port GPIOC
#define LightSensor_Pin GPIO_PIN_2
#define LightSensor_GPIO_Port GPIOC
#define BSPA_TX_Pin GPIO_PIN_2
#define BSPA_TX_GPIO_Port GPIOA
#define BSPA_RX_Pin GPIO_PIN_3
#define BSPA_RX_GPIO_Port GPIOA
#define W25Q64_NSS_Pin GPIO_PIN_4
#define W25Q64_NSS_GPIO_Port GPIOA
#define W25Q64_SCK_Pin GPIO_PIN_5
#define W25Q64_SCK_GPIO_Port GPIOA
#define W25Q64_MISO_Pin GPIO_PIN_6
#define W25Q64_MISO_GPIO_Port GPIOA
#define W25Q64_MOSI_Pin GPIO_PIN_7
#define W25Q64_MOSI_GPIO_Port GPIOA
#define GentleSensor_Pin GPIO_PIN_4
#define GentleSensor_GPIO_Port GPIOC
#define BSPB_TX_Pin GPIO_PIN_10
#define BSPB_TX_GPIO_Port GPIOB
#define BSPB_RX_Pin GPIO_PIN_11
#define BSPB_RX_GPIO_Port GPIOB
#define MCUAtmosphereLEDR_Pin GPIO_PIN_12
#define MCUAtmosphereLEDR_GPIO_Port GPIOB
#define MCUAtmosphereLEDG_Pin GPIO_PIN_13
#define MCUAtmosphereLEDG_GPIO_Port GPIOB
#define MCU_DS18B20_Pin GPIO_PIN_6
#define MCU_DS18B20_GPIO_Port GPIOC
#define CoreBoard_TX_Pin GPIO_PIN_9
#define CoreBoard_TX_GPIO_Port GPIOA
#define CoreBoard_RX_Pin GPIO_PIN_10
#define CoreBoard_RX_GPIO_Port GPIOA
#define MCU_LED_PWM_Pin GPIO_PIN_4
#define MCU_LED_PWM_GPIO_Port GPIOB
#define MCU_FAN_OUT_Pin GPIO_PIN_5
#define MCU_FAN_OUT_GPIO_Port GPIOB
#define MCU_LED_OUT_Pin GPIO_PIN_6
#define MCU_LED_OUT_GPIO_Port GPIOB
#define RunningLED_Pin GPIO_PIN_9
#define RunningLED_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#define MCU_LED_OFF                     GPIO_PIN_SET
#define MCU_LED_ON                     GPIO_PIN_RESET
#define MCU_FAN_ON                      GPIO_PIN_RESET
#define MCU_FAN_OFF                     GPIO_PIN_SET
#define RunningLED_OFF                   GPIO_PIN_SET
#define RunningLED_ON                  GPIO_PIN_RESET
#define CommunicationLED_OFF             GPIO_PIN_SET
#define CommunicationLED_ON            GPIO_PIN_RESET
#define MCUAtmosphereLEDG_ON            GPIO_PIN_SET
#define MCUAtmosphereLEDG_OFF           GPIO_PIN_RESET
#define MCUAtmosphereLEDR_ON            GPIO_PIN_SET
#define MCUAtmosphereLEDR_OFF           GPIO_PIN_RESET
#define RX_MAX_LENGTH                   256
  /** enum: DS_StatusTypeDef
  **
  ** DESCRIPTION:
  **  --«˝∂Ø∞Â¿‡–Õ
  **
  ** CREATED: 2017/12/7, by bert
  **
  ** FILE: DS_Protocol.h
  **
  ** AUTHOR: Bert.Zhang
  ********************************************************************************
  */
  typedef enum
  {
    DS_OK       = 0x00U,
    DS_ERROR    = 0x01U,
    DS_BUSY     = 0x02U,
    DS_TIMEOUT  = 0x03U,
    DS_NOCMD    = 0x04U
  }DS_StatusTypeDef;
  
/*******************************************************************************
** struct: sGpioStatusDetection
**
** DESCRIPTION:
**  --gpio status 
**
** CREATED: 2017/12/26, by bert
**
** FILE: GentleSensor.h
**
** AUTHOR: Bert.Zhang
********************************************************************************
*/
struct sGpioStatusDetection
{
  uint8_t   GpioCurrentReadVal;                 //Current GPIO value
  uint8_t   GpioLastReadVal;                    //Last GPIO value
  uint8_t   GpioFilterCnt;                      //Filter times
  uint16_t  GpioFilterCntSum;
  uint8_t   GpioStatusVal;                      //GPIO true logic state
  uint8_t   GpioCheckedFlag;                    //vehicle is stilled Flag
  uint8_t   GpioSendDataFlag;                   //Send data flag,it is zero if data been sent within asingle logic ,
                                                //and set to one in the next logic
  uint32_t  GpioValidLogicTimeCnt;              //Gpio vaild logic time counter
}; 

typedef struct sGpioStatusDetection  GPIOSTATUSDETECTION, *pGPIOSTATUSDETECTION;

typedef enum {
  CoreBoard_Dev = 0,
  LeftBoard_Dev = 1,
  RightBoard_Dev = 2
}Dev_Type;
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

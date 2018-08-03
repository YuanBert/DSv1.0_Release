/**
  ******************************************************************************
  * File Name          : USART.h
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */
extern uint8_t coreBoardReceiveBuff[RX_MAX_LENGTH];
extern uint16_t coreBoardReceiveBuffLength;
extern uint8_t gCoreBoardInfoFlag;
extern uint8_t gCoreBoardReceiveInfo[RX_MAX_LENGTH];
extern uint16_t gCoreBoardReceiveInfoLength;

extern uint8_t leftDoorBoardReceiveBuff[RX_MAX_LENGTH];
extern uint16_t leftDoorBoardReceiveBuffLength;
extern uint8_t gLeftDoorBoardInfoFlag;
extern uint8_t gLeftDoorBoardReceiveInfo[RX_MAX_LENGTH];
extern uint16_t gLeftDoorBoardReceiveInfoLength;

extern uint8_t rightDoorBoardReceiveBuff[RX_MAX_LENGTH];
extern uint16_t rightDoorBoardReceiveBuffLength;
extern uint8_t gRightDoorBoardInfoFlag;
extern uint8_t gRightDoorBoardReceiveInfo[RX_MAX_LENGTH];
extern uint16_t gRightDoorBoardReceiveInfoLength;
/* USER CODE END Private defines */

extern void _Error_Handler(char *, int);

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
DS_StatusTypeDef DS_CoreBoardProtocolInit(void);
DS_StatusTypeDef DS_LeftDoorBoardProtocolInit(void);
DS_StatusTypeDef DS_RightDoorBoardProtocolInit(void);


DS_StatusTypeDef DS_SendDataToCoreBoard(uint8_t* pData, uint16_t size, uint32_t Timeout);
DS_StatusTypeDef DS_SendDataToLeftDoorBoard(uint8_t* pData, uint16_t size, uint32_t Timeout);
DS_StatusTypeDef DS_SendDataToRightDoorBoard(uint8_t* pData, uint16_t size, uint32_t Timeout);


DS_StatusTypeDef DS_SendDataToSendingTable(uint8_t portNum, uint8_t sendTimes, uint8_t* pSendData, uint16_t dataLength);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

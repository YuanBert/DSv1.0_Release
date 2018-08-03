/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

#include "gpio.h"
#include "dma.h"

/* USER CODE BEGIN 0 */
#include "string.h"
uint8_t coreBoardReceiveBuff[RX_MAX_LENGTH];
uint16_t coreBoardReceiveBuffLength;
uint8_t gCoreBoardInfoFlag;
uint8_t gCoreBoardReceiveInfo[RX_MAX_LENGTH];
uint16_t gCoreBoardReceiveInfoLength;

uint8_t leftDoorBoardReceiveBuff[RX_MAX_LENGTH];
uint16_t leftDoorBoardReceiveBuffLength;
uint8_t gLeftDoorBoardInfoFlag;
uint8_t gLeftDoorBoardReceiveInfo[RX_MAX_LENGTH];
uint16_t gLeftDoorBoardReceiveInfoLength;

uint8_t rightDoorBoardReceiveBuff[RX_MAX_LENGTH];
uint16_t rightDoorBoardReceiveBuffLength;
uint8_t gRightDoorBoardInfoFlag;
uint8_t gRightDoorBoardReceiveInfo[RX_MAX_LENGTH];
uint16_t gRightDoorBoardReceiveInfoLength;

/* 需要发送的命令表，可以存储四个需要发送的命令 */
uint8_t gSendingCmdTable[4][128];

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = CoreBoard_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CoreBoard_TX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = CoreBoard_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CoreBoard_RX_GPIO_Port, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA1_Channel5;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    GPIO_InitStruct.Pin = BSPA_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BSPA_TX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BSPA_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BSPA_RX_GPIO_Port, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* USART2_RX Init */
    hdma_usart2_rx.Instance = DMA1_Channel6;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_NORMAL;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart2_rx);

  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();
  
    /**USART3 GPIO Configuration    
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX 
    */
    GPIO_InitStruct.Pin = BSPB_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BSPB_TX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BSPB_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BSPB_RX_GPIO_Port, &GPIO_InitStruct);

    /* USART3 DMA Init */
    /* USART3_RX Init */
    hdma_usart3_rx.Instance = DMA1_Channel3;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_NORMAL;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart3_rx);

  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    HAL_GPIO_DeInit(GPIOA, CoreBoard_TX_Pin|CoreBoard_RX_Pin);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    HAL_GPIO_DeInit(GPIOA, BSPA_TX_Pin|BSPA_RX_Pin);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();
  
    /**USART3 GPIO Configuration    
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX 
    */
    HAL_GPIO_DeInit(GPIOB, BSPB_TX_Pin|BSPB_RX_Pin);

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

void DS_CoreBoardUsartReceive_IDLE(void)
{
  uint32_t temp;
  if((__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE) != RESET))
  {
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    HAL_UART_DMAStop(&huart1);
    temp = huart1.hdmarx->Instance->CNDTR;
    coreBoardReceiveBuffLength = RX_MAX_LENGTH - temp;
    
    gCoreBoardInfoFlag = 1;
    memcpy(gCoreBoardReceiveInfo,coreBoardReceiveBuff,coreBoardReceiveBuffLength);
    memset(coreBoardReceiveBuff,0,coreBoardReceiveBuffLength);
    gCoreBoardReceiveInfoLength = coreBoardReceiveBuffLength;
    coreBoardReceiveBuffLength = 0;
    
    HAL_UART_Receive_DMA(&huart1,coreBoardReceiveBuff,RX_MAX_LENGTH);
  }
}

void DS_LeftDoorBoardUsartReceive_IDLE(void)
{
  uint32_t temp;
  if((__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE) != RESET))
  {
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);
    HAL_UART_DMAStop(&huart2);
    temp = huart2.hdmarx->Instance->CNDTR;
    leftDoorBoardReceiveBuffLength = RX_MAX_LENGTH - temp;
    
    gLeftDoorBoardInfoFlag = 1;
    memcpy(gLeftDoorBoardReceiveInfo,leftDoorBoardReceiveBuff,leftDoorBoardReceiveBuffLength);
    memset(leftDoorBoardReceiveBuff,0,leftDoorBoardReceiveBuffLength);
    gLeftDoorBoardReceiveInfoLength = leftDoorBoardReceiveBuffLength;
    leftDoorBoardReceiveBuffLength = 0;
    
    HAL_UART_Receive_DMA(&huart2,leftDoorBoardReceiveBuff,RX_MAX_LENGTH);
    
  }
}

void DS_RightDoorBoardUsartReceive_IDLE(void)
{
  uint32_t temp;
  if((__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE) != RESET))
  {
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    HAL_UART_DMAStop(&huart3);
    temp = huart3.hdmarx->Instance->CNDTR;
    rightDoorBoardReceiveBuffLength = RX_MAX_LENGTH - temp;
    
    gRightDoorBoardInfoFlag = 1;
    memcpy(gRightDoorBoardReceiveInfo,rightDoorBoardReceiveBuff,rightDoorBoardReceiveBuffLength);
    memset(rightDoorBoardReceiveBuff,0,rightDoorBoardReceiveBuffLength);
    gRightDoorBoardReceiveInfoLength = rightDoorBoardReceiveBuffLength;
    rightDoorBoardReceiveBuffLength = 0;
    
    HAL_UART_Receive_DMA(&huart3,rightDoorBoardReceiveBuff,RX_MAX_LENGTH);
    
  }
}

DS_StatusTypeDef DS_CoreBoardProtocolInit(void)
{
  DS_StatusTypeDef state = DS_OK;
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart1,coreBoardReceiveBuff,RX_MAX_LENGTH);
  return state;
}

DS_StatusTypeDef DS_LeftDoorBoardProtocolInit(void)
{
  DS_StatusTypeDef state = DS_OK;
  __HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart2,leftDoorBoardReceiveBuff,RX_MAX_LENGTH);
  HAL_GPIO_WritePin(CTR485A_EN_GPIO_Port,CTR485A_EN_Pin,GPIO_PIN_RESET);
  
  return state;  
}
DS_StatusTypeDef DS_RightDoorBoardProtocolInit(void)
{
  DS_StatusTypeDef state = DS_OK;
  __HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart3,rightDoorBoardReceiveBuff,RX_MAX_LENGTH);
  HAL_GPIO_WritePin(CTR485B_EN_GPIO_Port,CTR485B_EN_Pin,GPIO_PIN_RESET);
  
  return state;
}


DS_StatusTypeDef DS_SendDataToCoreBoard(uint8_t* pData, uint16_t size, uint32_t Timeout)
{
  DS_StatusTypeDef state = DS_OK;
  state = (DS_StatusTypeDef)HAL_UART_Transmit(&huart1, pData,size,Timeout);
  if(DS_OK != state)
  {
    state = DS_ERROR;
  }
  return state; 
}
DS_StatusTypeDef DS_SendDataToLeftDoorBoard(uint8_t* pData, uint16_t size, uint32_t Timeout)
{
  DS_StatusTypeDef state = DS_OK;
  HAL_GPIO_WritePin(CTR485A_EN_GPIO_Port,CTR485A_EN_Pin,GPIO_PIN_SET);
  state = (DS_StatusTypeDef)HAL_UART_Transmit(&huart2, pData, size, Timeout);
  HAL_GPIO_WritePin(CTR485A_EN_GPIO_Port,CTR485A_EN_Pin,GPIO_PIN_RESET);
  return state;   
}
DS_StatusTypeDef DS_SendDataToRightDoorBoard(uint8_t* pData, uint16_t size, uint32_t Timeout)
{
  DS_StatusTypeDef state = DS_OK;
  HAL_GPIO_WritePin(CTR485B_EN_GPIO_Port,CTR485B_EN_Pin,GPIO_PIN_SET);
  state = (DS_StatusTypeDef)HAL_UART_Transmit(&huart3, pData, size, Timeout);
  HAL_GPIO_WritePin(CTR485B_EN_GPIO_Port,CTR485B_EN_Pin,GPIO_PIN_RESET);
  return state;  
}

static uint8_t getXORCode(uint8_t* pData,uint16_t len)
{
  uint8_t ret;
  uint16_t i;
  ret = pData[0];
  for(i = 1; i < len; i++)
  {
    ret ^= pData[i];
  }
  return ret;
}

static DS_StatusTypeDef DS_HandlingUartsData(uint8_t* pDataBuff, uint16_t dataLength)
{
  DS_StatusTypeDef state = DS_OK;
  uint8_t xorTemp;
  uint16_t i;
  
}

DS_StatusTypeDef DS_SendDataToSendingTable(uint8_t portNum, uint8_t sendTimes, uint8_t* pSendData, uint16_t dataLength)
{
  DS_StatusTypeDef state = DS_OK;
  
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

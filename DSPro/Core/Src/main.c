
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define DEBUG       2
#define TIMEOUTNUM            42000UL  //42s
#define WAITINGTIMEOUTNUM     26000UL  //26s
#define ADCWAITINGTIMEOUTNUM  5000UL   //5s进行一次ADC采样


/* 车道机状态定义 */
enum ModeState{
  NORMAL_RUNNING_STATE  = 0, //正常运行状态
  SET_PRAMA_STATE       = 1, //参数设置状态
  COIL_TIGGER_STATE     = 2, //地感线圈触发来车状态
  SOFT_TIGGER_STATE     = 3, //视频流触发来车状态
};

enum ModeState gMachineStateFlag = NORMAL_RUNNING_STATE; //全局的状态变量
GPIOSTATUSDETECTION gGentleSensorStatusDetection;//地感检测结构体

uint16_t gTIM5LEDCheckCnt = 0;
uint8_t  gAtomsphereLedsModeFlag = 0;
uint32_t gCarTimeoutCnt = 0;
uint8_t gCarTimeoutFlag = 0;
uint8_t gSendAndReceiveFlag = 0; //0 -- 表示未发送  1 -- 表示已经发送命令，处在等待接收状态  2 -- 表示接收成功 3 -- 表示入场超时
uint32_t gSendAndReceiveTimeoutCnt  = 0;
uint16_t gTemptureValue = 0;
uint16_t gLightValue = 0;
uint8_t gADCSampleFlag = 0;
uint16_t gADCSampleTimeCnt = 0;

uint16_t gTemptureAlarm = 1800;//默认的温度报警值
uint16_t gLightAlarm;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int fputc(int ch,FILE *f)
{
  HAL_UART_Transmit(&huart1,(uint8_t*)&ch,1,0x02);
  return ch;
}
void DEBUG_PRINT(uint8_t * x,uint8_t prior)
{ 
  if(prior == DEBUG)
      printf("%s\r\n",x);
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

static void SendCarComingCmdToBoard(void)
{
  uint8_t data[7];
  data[0] = 0x5B;
  data[1] = 0xB1;
  data[2] = 0x01;
  data[3] = 0x00;
  data[4] = 0x00;
  data[5] = 0xB0;
  data[6] = 0x5D;
  DS_SendDataToCoreBoard(data, 7, 0x0F); 
}

static void SendAckToBoard(Dev_Type boardType,uint8_t ackCmd,uint8_t ackParam1, uint8_t ackParam2)
{
  uint8_t ackData[6];
  ackData[0] = 0x5B;
  ackData[1] = ackCmd;
  ackData[2] = ackParam1;
  ackData[3] = ackParam2;
  ackData[4] = getXORCode(ackData+1,3);
  ackData[5] = 0x5D;
  if(CoreBoard_Dev == boardType)
  {
    DS_SendDataToCoreBoard(ackData,6,0x0F);
  }
  if(LeftBoard_Dev == boardType)
  {
    DS_SendDataToLeftDoorBoard(ackData,6,0x0F);
  }
  if(RightBoard_Dev == boardType)
  {
    DS_SendDataToRightDoorBoard(ackData,6,0x0F);
  }
}
//上报车辆出入场信息（失败，成功，超时）
static void reportCarEnterOrExitInfo(void)
{
  if(2 == gSendAndReceiveFlag)//车辆入场成功
  {
    gSendAndReceiveFlag = 0;
    gMachineStateFlag = NORMAL_RUNNING_STATE;
    DEBUG_PRINT((uint8_t*)"Car Enter successed",3);
    SendAckToBoard(CoreBoard_Dev,0xAB,0x02,0x00);
  }
  else if(3 == gSendAndReceiveFlag) //车辆入场失败
  {
    gSendAndReceiveFlag = 0;
    gMachineStateFlag = NORMAL_RUNNING_STATE;
    DEBUG_PRINT((uint8_t*)"Car Enter timeout",3);
    SendAckToBoard(CoreBoard_Dev,0xAB,0x02,0x01);
  }
}
/* PWM输出占空比设置100HZ */
void userPWMSetValue(uint16_t value)
{
  TIM_OC_InitTypeDef sConfigOC;
  
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = value;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
}

/* 解析来自CoreBoard的信息 */
void checkCoreBoardInfo(void);
/* 解析来自LeftDoor的信息 */
void checkLeftDoorInfo(void);
/* 解析来自RightDoor的信息 */
void checkRightDoorInfo(void);
/* 获取温度和光照强度值 */
uint32_t getTemptureAndLightValues(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
   
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim5);
  //HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);//PWM调制100Hz
  //HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);//PWM停止输出
  /* 地感检测变量初始化 */
  gGentleSensorStatusDetection.GpioFilterCnt = 0;
  gGentleSensorStatusDetection.GpioFilterCntSum = 50;
  gGentleSensorStatusDetection.GpioValidLogicTimeCnt = 0;
  /* 初始化核心板，道闸A 以及道闸B 的通讯 */
  DS_CoreBoardProtocolInit();
  DS_LeftDoorBoardProtocolInit();
  DS_RightDoorBoardProtocolInit();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    HAL_GPIO_WritePin(RunningLED_GPIO_Port,RunningLED_Pin,RunningLED_ON);//打开工作指示灯
    //printf("Welcome to use the devience\r\n");
    //HAL_UART_Transmit(&huart1,(uint8_t*)"Welcome to use the devience\r\n",30,0x02);
    //HAL_Delay(500);
    /*
    1.检测串口接收到的数据
    2.解析串口接收数据
    3.获取ADC采样值
    */
    /* 解析来自CoreBoard的信息 */
    checkCoreBoardInfo();
    /* 解析来自LeftDoor的信息 */
    checkLeftDoorInfo();
    /* 解析来自RightDoor的信息 */
    checkRightDoorInfo();
    /* 获取温度和光照强度值 */
    getTemptureAndLightValues();
    
    
    /* 正常运行模式 */
    if(NORMAL_RUNNING_STATE == gMachineStateFlag)//正常运行状态
    {
      /*
      1.检测温度值，并根据温度数值决定是否打开风扇
      2.检测光照强度值，并根据光照强度决定是否常开补光灯
      3.氛围灯常量（绿色，不闪烁，红灯灭）
      4.检测是否存在超时
      5.关闭补光灯
      */
      if(gTemptureValue > gTemptureAlarm)//1.
      {
        HAL_GPIO_WritePin(MCU_FAN_OUT_GPIO_Port,MCU_FAN_OUT_Pin,MCU_FAN_ON);
      }
      else
      {
        HAL_GPIO_WritePin(MCU_FAN_OUT_GPIO_Port,MCU_FAN_OUT_Pin,MCU_FAN_OFF);
      }
      
      gAtomsphereLedsModeFlag = 0;//3.
      
      if(2 == gCarTimeoutFlag)//4.
      {
        DEBUG_PRINT((uint8_t*)"coil tigger error",3);
        /* 上报地感故障指令 */
        gCarTimeoutFlag = 0;
      }
      if(1 == gCarTimeoutFlag || 3 == gSendAndReceiveFlag) 
      {
        gCarTimeoutFlag = 0;
        gSendAndReceiveFlag = 0;
        /* 返回车辆入场超时指令 */
        DEBUG_PRINT((uint8_t*)"Car stops timeout",3);
        SendAckToBoard(CoreBoard_Dev,0xAB,0x02,0x01);
        gGentleSensorStatusDetection.GpioValidLogicTimeCnt++;
      }
      HAL_GPIO_WritePin(MCU_LED_OUT_GPIO_Port,MCU_LED_OUT_Pin,MCU_LED_OFF);//5.
      
    }
    /* 参数设置模式 */
    if(SET_PRAMA_STATE == gMachineStateFlag) //参数设置状态
    {
      /*
      1.红灯闪烁，绿灯常量
      2.根据相关协议处理参数设置命令
      */
      gAtomsphereLedsModeFlag = 1;//1.
    }
    /* 地感触发模式下 */
    if(COIL_TIGGER_STATE == gMachineStateFlag) //地感触发状态
    {
      /*
      1.开启补光灯（PWM）
      2.开启氛围灯（绿灯闪烁，红灯长灭）
      3.上报车辆触发地感信息
      4.上报车辆出入场信息（失败，成功，超时）
      */
      HAL_GPIO_WritePin(MCU_LED_OUT_GPIO_Port,MCU_LED_OUT_Pin,MCU_LED_ON);//1.
      gAtomsphereLedsModeFlag = 2;//2.
      if(1 == gGentleSensorStatusDetection.GpioSendDataFlag) //3.上报车辆触发地感信息
      {
        SendCarComingCmdToBoard();
        //gSendAndReceiveFlag = 1;
        gGentleSensorStatusDetection.GpioSendDataFlag = 0;
      }
      //4.上报车辆出入场信息（失败，成功，超时）
      reportCarEnterOrExitInfo();
      
    }
    /* 视频流触发模式下 */
    if(SOFT_TIGGER_STATE == gMachineStateFlag) //视频流触发状态
    {
      /*
      1.开启补光灯（PWM）
      2.开启氛围灯（绿灯闪烁，红灯长灭）
      3.上报车辆出入场信息（失败，成功，超时）
      */
      HAL_GPIO_WritePin(MCU_LED_OUT_GPIO_Port,MCU_LED_OUT_Pin,MCU_LED_ON);//1.
      gAtomsphereLedsModeFlag = 2;//2.
      reportCarEnterOrExitInfo();//3.
    }

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* TIM4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* TIM5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* ADC1_2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
}

/* USER CODE BEGIN 4 */
void  HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  UNUSED(htim);
  /* 0.1ms **********************************************************************************************/
  if(htim4.Instance == htim->Instance)
  {
    /* 添加地感处理 */
    gGentleSensorStatusDetection.GpioCurrentReadVal = HAL_GPIO_ReadPin(GentleSensor_GPIO_Port, GentleSensor_Pin);
    if(0 == gGentleSensorStatusDetection.GpioCurrentReadVal && 0 == gGentleSensorStatusDetection.GpioLastReadVal)
    {
      if(0 == gGentleSensorStatusDetection.GpioCheckedFlag)
      {
        gGentleSensorStatusDetection.GpioFilterCnt++;
        if(gGentleSensorStatusDetection.GpioFilterCnt > gGentleSensorStatusDetection.GpioFilterCntSum)
        {
          gGentleSensorStatusDetection.GpioStatusVal    = 1;
          gGentleSensorStatusDetection.GpioFilterCnt    = 0;
          gGentleSensorStatusDetection.GpioCheckedFlag  = 1;
          gGentleSensorStatusDetection.GpioSendDataFlag = 1;
          gMachineStateFlag =  COIL_TIGGER_STATE;
        }
      }
    }
    else
    {
      if(1 == gGentleSensorStatusDetection.GpioCheckedFlag) //车辆有压住地感到离开地感，进入正常模式
      {
        gMachineStateFlag = NORMAL_RUNNING_STATE;
      }
      gGentleSensorStatusDetection.GpioCheckedFlag	= 0;
      gGentleSensorStatusDetection.GpioFilterCnt		= 0;
      gGentleSensorStatusDetection.GpioStatusVal		= 0;
      gGentleSensorStatusDetection.GpioValidLogicTimeCnt = 1;
    }
    gGentleSensorStatusDetection.GpioLastReadVal = gGentleSensorStatusDetection.GpioCurrentReadVal;
  }
  /* 1ms ************************************************************************************************/
  if(htim5.Instance == htim->Instance)
  {
    /*闪烁频率设置位5Hz*/
    gTIM5LEDCheckCnt++;
    if(gTIM5LEDCheckCnt > 199)
    {
      DEBUG_PRINT((uint8_t*)"Enter LED Check",1);
      gTIM5LEDCheckCnt = 0;
      if(0 == gAtomsphereLedsModeFlag)
      {
        /* 绿灯亮红灯灭 */
        DEBUG_PRINT((uint8_t*)"Green led light and red led reset",1);
        HAL_GPIO_WritePin(MCUAtmosphereLEDR_GPIO_Port,MCUAtmosphereLEDR_Pin,MCUAtmosphereLEDR_OFF);
        HAL_GPIO_WritePin(MCUAtmosphereLEDG_GPIO_Port,MCUAtmosphereLEDG_Pin,MCUAtmosphereLEDG_ON);
      }
      else if(1 == gAtomsphereLedsModeFlag)
      {
        DEBUG_PRINT((uint8_t*)"Green led light and Red led flash",1);
        HAL_GPIO_WritePin(MCUAtmosphereLEDG_GPIO_Port,MCUAtmosphereLEDG_Pin,MCUAtmosphereLEDG_ON);
        HAL_GPIO_TogglePin(MCUAtmosphereLEDR_GPIO_Port,MCUAtmosphereLEDR_Pin);
      }
      else if(2 == gAtomsphereLedsModeFlag)
      {
        DEBUG_PRINT((uint8_t*)"Red led OFF and green led flash",1);
        HAL_GPIO_WritePin(MCUAtmosphereLEDR_GPIO_Port,MCUAtmosphereLEDR_Pin,MCUAtmosphereLEDR_OFF);
        HAL_GPIO_TogglePin(MCUAtmosphereLEDG_GPIO_Port,MCUAtmosphereLEDG_Pin);
      }
      else
      {
        //系统错误 - 001 表示灯模式错误
        DEBUG_PRINT((uint8_t*)" system err code -- 001 ",1);
      }
    }
    
    //车辆压地感超时检测
    if(1 == gGentleSensorStatusDetection.GpioStatusVal)
    {
      gCarTimeoutCnt++;
      if(gCarTimeoutCnt > TIMEOUTNUM)//车辆停留地感超时
      {
        gCarTimeoutCnt = 0;
        if(gGentleSensorStatusDetection.GpioValidLogicTimeCnt  < 10 && gGentleSensorStatusDetection.GpioValidLogicTimeCnt > 0)
        {
          gCarTimeoutFlag = 1;//写车辆停留超时标志
        }
        else if(gGentleSensorStatusDetection.GpioValidLogicTimeCnt  > 9)
        {
          gCarTimeoutFlag = 2;//地感故障
          gGentleSensorStatusDetection.GpioValidLogicTimeCnt  = 0;
        }
        gMachineStateFlag = NORMAL_RUNNING_STATE;//超时后进入正常模式，在正常模式上报超时指令
      }
    }
    else
    {
      gCarTimeoutCnt = 0;
    }
    
    //命令响应超时检测
    if(1 == gSendAndReceiveFlag)
    {
      gSendAndReceiveTimeoutCnt ++ ;
      if(gSendAndReceiveTimeoutCnt > WAITINGTIMEOUTNUM)
      {
        gSendAndReceiveFlag = 3;//等待超时
        gSendAndReceiveTimeoutCnt = 0;
      }
    }
    else if(0 == gSendAndReceiveFlag)
    {
      gSendAndReceiveTimeoutCnt = 0;
    }
    
    //ADC采样计数器
    gADCSampleTimeCnt++;
    if(gADCSampleTimeCnt > ADCWAITINGTIMEOUTNUM)
    {
      gADCSampleFlag = 1;
      gADCSampleTimeCnt = 0;
    }
  }
  /**********************************************************************************************************/
}

/* 解析来自CoreBoard的信息 */
void checkCoreBoardInfo(void)
{
  if(gCoreBoardInfoFlag)
  {
    gCoreBoardInfoFlag = 0;
    DEBUG_PRINT((uint8_t*)"Receive bytes from coreboard",1);
    printf("the uart1 rx_lenth is %d:\n%s\r\n",gCoreBoardReceiveInfoLength,gCoreBoardReceiveInfo);
    DS_SendDataToCoreBoard(gCoreBoardReceiveInfo,gCoreBoardReceiveInfoLength,0x0F);
    /* 解析来自核心板的数据和命令 */
    if(0x5B == gCoreBoardReceiveInfo[0] && 0x5D == gCoreBoardReceiveInfo[gCoreBoardReceiveInfoLength - 1])
    {
      if(0xA0 == (gCoreBoardReceiveInfo[1] & 0xF0))
      {
        DEBUG_PRINT((uint8_t*)"Receive ACK from coreboard",1);
      }
      else
      {
        DEBUG_PRINT((uint8_t*)"Receive CMD from coreboard",1);
        if(0xB2 == gCoreBoardReceiveInfo[1] && 0x01 == gCoreBoardReceiveInfo[2])
        {
          DEBUG_PRINT((uint8_t*)"Receive Open door cmd from coreboard",1);
          /* 发送开闸指令到道闸 */
          DS_SendDataToLeftDoorBoard(gCoreBoardReceiveInfo,gCoreBoardReceiveInfoLength,0x0F);
          DS_SendDataToRightDoorBoard(gCoreBoardReceiveInfo,gCoreBoardReceiveInfoLength,0x0F);
          gSendAndReceiveFlag = 1;//写发送开闸指令标记，用来进行接收时的标记
        }
        if(0xB2 == gCoreBoardReceiveInfo[1] && 0x02 == gCoreBoardReceiveInfo[2])
        {
          DEBUG_PRINT((uint8_t*)"Receive Videos tirggle cmd from coreboard",1);
          /* 发送开闸指令到道闸 */
          gCoreBoardReceiveInfo[2]  = 0x01;
          DS_SendDataToRightDoorBoard(gCoreBoardReceiveInfo,gCoreBoardReceiveInfoLength,0x0F);
          DS_SendDataToLeftDoorBoard(gCoreBoardReceiveInfo,gCoreBoardReceiveInfoLength,0x0F);
          gSendAndReceiveFlag = 1;//写发送开闸指令标记，用来进行接收时的标记
          gMachineStateFlag = SOFT_TIGGER_STATE; //进入视频流触发模式
        }
        /* 数据转发至发送道闸A */
        if(0xC1 == gCoreBoardReceiveInfo[1])
        {
          DS_SendDataToLeftDoorBoard(gCoreBoardReceiveInfo,gCoreBoardReceiveInfoLength,0x0F);
        }
        /* 数据转发至道闸B */
        if(0xC2 == gCoreBoardReceiveInfo[1])
        {
          DS_SendDataToRightDoorBoard(gCoreBoardReceiveInfo,gCoreBoardReceiveInfoLength,0x0F);
        }
      }
    }
    memset(gCoreBoardReceiveInfo,0,gCoreBoardReceiveInfoLength);
    gCoreBoardReceiveInfoLength = 0;
  }
}
/* 解析来自LeftDoor的信息 */
void checkLeftDoorInfo(void)
{
  if(gLeftDoorBoardInfoFlag)
  {
    gLeftDoorBoardInfoFlag = 0;
    DEBUG_PRINT((uint8_t*)"Receive bytes from left door board",2);
    printf("the uart2 rx_lenth is %d:\n %s\r\n",gLeftDoorBoardReceiveInfoLength,gLeftDoorBoardReceiveInfo);
    DS_SendDataToLeftDoorBoard(gLeftDoorBoardReceiveInfo,gLeftDoorBoardReceiveInfoLength,0x0F);
    if(0x5B == gLeftDoorBoardReceiveInfo[0] && 0x5D == gLeftDoorBoardReceiveInfo[gLeftDoorBoardReceiveInfoLength - 1])
    {
      if(0xA0 == (gLeftDoorBoardReceiveInfo[1] & 0xF0))
      {
        /* 收到回复命令 */
      }
      else
      {
        if(1 == gSendAndReceiveFlag && 0xE3 == gLeftDoorBoardReceiveInfo[1])  //接收返回命令
        {
          if(0x01 == gLeftDoorBoardReceiveInfo[2])
          {
            gSendAndReceiveFlag = 3;  //入场超时
          }
          if(0x02 == gLeftDoorBoardReceiveInfo[2])
          {
             gSendAndReceiveFlag = 2;   //入场成功
          }
        }
        /* 接收到来自道闸A的C1类型的数据直接转发给核心板 */
        if(0xC1 == gLeftDoorBoardReceiveInfo[1])
        {
          DS_SendDataToCoreBoard(gLeftDoorBoardReceiveInfo,gLeftDoorBoardReceiveInfoLength,0x0F);
        }
      }
    }
    memset(gLeftDoorBoardReceiveInfo,0,gLeftDoorBoardReceiveInfoLength);
    gLeftDoorBoardReceiveInfoLength = 0;
  }
}
/* 解析来自RightDoor的信息 */
void checkRightDoorInfo(void)
{
  if(gRightDoorBoardInfoFlag)
  {
    gRightDoorBoardInfoFlag = 0;
    DEBUG_PRINT((uint8_t*)"Receive bytes from right door board",2);
//#if 1 == DEBUG
    printf("the uart2 rx_lenth is %d:\n %s\r\n",gRightDoorBoardReceiveInfoLength,gRightDoorBoardReceiveInfo);
//#endif
    DS_SendDataToRightDoorBoard(gRightDoorBoardReceiveInfo,gRightDoorBoardReceiveInfoLength,0x0F);
    
    if(0x5B == gRightDoorBoardReceiveInfo[0] && 0x5D == gRightDoorBoardReceiveInfo[gRightDoorBoardReceiveInfoLength - 1])
    {
      if(0xA0 == (gRightDoorBoardReceiveInfo[1] & 0xF0))
      {
        /* 收到回复命令 */
      }
      else
      {
        /* 接收到来自道闸B的C2类型的数据直接转发给核心板 */
        if(0xC2 == gRightDoorBoardReceiveInfo[1])
        {
          DS_SendDataToCoreBoard(gRightDoorBoardReceiveInfo,gRightDoorBoardReceiveInfoLength,0x0F);
        }
      }
    }
    memset(gRightDoorBoardReceiveInfo,0,gRightDoorBoardReceiveInfoLength);
    gRightDoorBoardReceiveInfoLength = 0;
  }
}

/* 获取温度和光照强度值 :返回数据的高16位表示温度，低16位表示光照强度*/
uint32_t getTemptureAndLightValues(void)
{
  uint32_t ret = 0;
  
  if(1 != gADCSampleFlag)
  {
    return 0xFFFFFFFF;
  }
  gADCSampleFlag = 0;
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);
  
  HAL_ADC_PollForConversion(&hadc1,0x60);
  HAL_ADC_PollForConversion(&hadc2,0x60);
  
  if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1),HAL_ADC_STATE_REG_EOC))
  {
    gTemptureValue = HAL_ADC_GetValue(&hadc1);
  }
  if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc2),HAL_ADC_STATE_REG_EOC))
  {
    gLightValue = HAL_ADC_GetValue(&hadc2);
  }  
  ret = (gTemptureValue << 16) + gLightValue;

//2级DEBUG  
#if 2 == DEBUG
  printf("Tempture is: 0x%04X\t Light is: 0x%04X\n",gTemptureValue,gLightValue);
#endif
  
  return ret;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

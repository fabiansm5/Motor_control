/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdlib.h>


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Modbus.h"

//Define acceleration and deceleration states
#define NOACC 0
#define ACCEL  1
#define DECEL  2
#define STOP  3

//Define all the functions ID
#define FEED_TO_POSITION 103
#define FEED_TO_LENGTH 102
#define SEEK_HOME 110
#define ZERO 2
#define STOP 3
#define RUN_A_SPEED 4


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x30000000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30000200
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30000000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30000200))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
// Modbus variables
modbusHandler_t ModbusH;
int16_t ModbusDATA[200];

//Variables for the acceleration
int F_clk_accel = 275000000;
int PSC_accel = 1499;
double frequency_accel = 0;
double period_accel = 0;

// Struct for a motor
struct motor
{
	int dir;
	int step;
	int enable;
	TIM_TypeDef *timer1;
	TIM_HandleTypeDef htimer1;
	TIM_TypeDef *timer2;
	TIM_HandleTypeDef htimer2;
	int function;
	double current_position;
	double zero;
};
struct motor motors[5]; // motor


//function of operation seek data
// I have no clue what to do here
struct operation
{
		int opcode;
		int angle_of_rotation;
		int max_velocity;
		double initial_velocity;
		double acceleration;
		int direction;
		int Counter;
		int State;
		int total_steps;
		int Acceleration_State;
		double step_rate;
		int absolute_position;
};
struct operation operations[10];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_HS_USB_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//INPUT: RPM required
//DESCRIPTION: Find the period of the PWM to rotate the motor to desired location
//OUTPUT: find period to enter into the appropiate register
double period(double RPM)
{
	// The equation is ARR =  { F_clk/[ freq * (PSC + 1) ] } - 1
	// ARR is the period,
	// psc is the prescalar value
	// f_clk is the clock frequency

	double frequency = (RPM*360*32)/(60*1.8);
	int F_clk = 275000000;
	int PSC = 99;

	if(frequency == 0)
	{
		return 0;
	}
	double period = (F_clk / (frequency * (PSC + 1))) - 1;

	return period;
}

//INPUT: the angle to rotate the motor by
//DESCRIPTION: Calculate the microsteps to reach the angle of rotation
//OUTPUT: the steps to reach rotation
int step_count(double angle)
{
	double micro_angle = 0.05625;
	int num_steps = (angle / micro_angle);
	return abs(num_steps);
}

//INPUT: Current frequency
//DESCRIPTION: Transforms the updated stepping rate (steps/second) to frequency for the pwm
//OUTPUT: the new frequency for the pwm expressed in period
void period_Update(double step_rate, int Motor_ID)
{
	// RPM = step rate (steps/second) (60 seconds / 1 minute) (1 rotation / 200 steps )
	double RPM = step_rate *(60.0/1.0)*(1.0/200.0);
	motors[Motor_ID].timer1->ARR = period(RPM);
	motors[Motor_ID].timer1->CCR1 = motors[Motor_ID].timer1->ARR / 2;
}

//INPUT: CURRENT VELOCITY, INITIAL VELOCITY, TOTAL STEPS, COUNTER
//PURPOSE: Calculate the steps required to stop at the given speed,
//PURPOSE: to check if it is time to decelerate
//OUTPUT: steps to stop at the curren velocity
int Steps_To_Stop(double curr_vel, double init_vel, double accel)
{
	//if no acceleration, no need to enter stop section in acceleration.
	if(curr_vel == init_vel || accel == 0)
	{
		return 0;
	}

	else
	{
		double accel_pulses = curr_vel - init_vel;
		int steps_to_stop = 0;
		for(int i = 0; i < accel_pulses;i++)
		{

			double timer1_frequency = (((curr_vel - i)*(60.0/1.0)*(1.0/200.0))*360*32) / (60*1.8);
			steps_to_stop += timer1_frequency / accel;
		}
		return steps_to_stop;
	}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_HS_USB_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  //----------------------------------------------------
  //----------------Define slave------------------------
  //----------------------------------------------------
  ModbusH.uModbusType = MB_SLAVE;
  ModbusH.port =  &huart3;
  ModbusH.u8id = 17;
  ModbusH.u16timeOut = 1000; // I do not know what this is for.

  ModbusH.u16regs = (uint16_t *) ModbusDATA;
  ModbusH.u16regsize= sizeof(ModbusDATA)/sizeof(ModbusDATA[0]);
  ModbusH.xTypeHW = USART_HW;

  //Initialize Modbus library
  ModbusInit(&ModbusH);

  //Start captsuring traffic on Serial Port
  ModbusStart(&ModbusH);

  //----------------------------------------------------
  //----------------------------------------------------
  //----------------------------------------------------


  //----------------------------------------------------
  //----------------Define Motor timers-----------------
  //----------------------------------------------------

  // motor 0
  motors[0].timer1 = TIM1;
  motors[0].htimer1 = htim1;
  motors[0].timer2 = TIM8;
  motors[0].htimer2 = htim8;
  motors[0].function = -1;
  motors[0].current_position = 0.0;
  motors[0].zero = 0.0;

  // motor 1
  motors[1].function = -1;

  // motor 1
  motors[2].function = -1;

  // motor 1
  motors[3].function = -1;

  // motor 1
  motors[4].function = -1;

  //----------------------------------------------------
  //----------------------------------------------------
  //----------------------------------------------------

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 34;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 99;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 1499;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_HS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_HS_USB_Init(void)
{

  /* USER CODE BEGIN USB_OTG_HS_Init 0 */

  /* USER CODE END USB_OTG_HS_Init 0 */

  /* USER CODE BEGIN USB_OTG_HS_Init 1 */

  /* USER CODE END USB_OTG_HS_Init 1 */
  /* USER CODE BEGIN USB_OTG_HS_Init 2 */

  /* USER CODE END USB_OTG_HS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11|LED_YELLOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_FS_PWR_EN_GPIO_Port, USB_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PE11 LED_YELLOW_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_11|LED_YELLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_ID_Pin */
  GPIO_InitStruct.Pin = USB_FS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
  HAL_GPIO_Init(USB_FS_ID_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PG12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//INPUT: Timer
//Description: count the pulse at every edge detected interrupt
//OUTPUT: None
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	//figure out a way to not use a magic number
	for(int i = 0; i < 5; ++i)
	{



		// first one is for counter and stopping the counter
		if(htim->Instance == motors[i].timer1)
		{
			//motors[motor_ID].current_position
			operations[motors[i].function].Counter += 1;

			if((operations[motors[i].function].total_steps -  operations[motors[i].function].Counter) <=
							Steps_To_Stop(operations[motors[i].function].step_rate, 50, operations[motors[i].function].acceleration) && i == 0)
					{
						int a = Steps_To_Stop(operations[motors[i].function].step_rate, 50, operations[motors[i].function].acceleration);
						operations[motors[i].function].Acceleration_State = STOP;
					}


			if(operations[motors[i].function].direction == 0)
			{
				motors[i].current_position += 0.05625;
				if(motors[i].current_position == 360 && motors[i].current_position < 360.01 && motors[i].current_position > 359.099)
				{
					motors[i].current_position = 0;
				}
			}

			else
			{
				motors[i].current_position = motors[i].current_position - 0.05625;

				if(motors[i].current_position == 0 && motors[i].current_position < 0.01 && motors[i].current_position > 0.099)
				{
					motors[i].current_position = 360;
				}
			}

			if (operations[motors[i].function].Counter >= operations[motors[i].function].total_steps
					&& operations[motors[i].function].Counter != 0)
			{
				//reset the timers
				HAL_TIM_PWM_Stop_IT(&(motors[i].htimer1), TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop_IT(&(motors[i].htimer2), TIM_CHANNEL_1); // can you stop something that has not started

				//reset values of the function
				operations[motors[i].function].Acceleration_State = 0;
				operations[motors[i].function].max_velocity = 0;
				operations[motors[i].function].angle_of_rotation = 0;
				operations[motors[i].function].acceleration = 0;
				operations[motors[i].function].direction = 0;
				operations[motors[i].function].Counter = 0;

				ModbusDATA[124] = 0;
				ModbusDATA[125] = 0;
				ModbusDATA[126] = 0;
				ModbusDATA[127] = 0;

				//reset values of the motor
				motors[i].function = -1;


			}

		}

		// second is for acceleration
		if (htim->Instance == motors[i].timer2)
		{
			switch(operations[motors[i].function].Acceleration_State)
			{
				case(NOACC):
					break;
				case(ACCEL):

						if(operations[motors[i].function].step_rate >= operations[motors[i].function].max_velocity)
						{
							operations[motors[i].function].Acceleration_State = NOACC;
						}

						else
						{
							operations[motors[i].function].step_rate += 1;
							period_Update(operations[motors[i].function].step_rate,i); // change motor
						}

					break;

					break;
				case(STOP):

						if(operations[motors[i].function].step_rate >= operations[motors[i].function].initial_velocity )
						{
							operations[motors[i].function].step_rate = operations[motors[i].function].step_rate - 1;
							period_Update(operations[motors[i].function].step_rate,i); // change motor
						}
					break;
			}
		}
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */


  for(;;)
  {
	  // Read motor ID
	  int motor_ID = ModbusDATA[0];
	  // Only if the motor does not have current
	  //operation running, this is indicated by a
	  // negative value in the function field
	  if(motors[motor_ID].function < 0 && ModbusDATA[124] != 0)
	  {
	 		 // Read opcode
	 		 int opCode =  ModbusDATA[124];

	 		 int function_ID = 0;
	 		 int motor_num = 5;
	 		 for(int i = 0; i < motor_num; ++i)
	 		 {
	 			if( i!= motors[i].function)
	 			{
	 				function_ID = i;
	 				motors[motor_ID].function = function_ID;
	 				break;
	 			}
	 		 }


	 		 switch(opCode)
	 		 {
	 		 	 case FEED_TO_POSITION:

	 		 		// Fill the parameters of the operation
	 		 		operations[function_ID].absolute_position = ModbusDATA[125];
	 		 		operations[function_ID].max_velocity = ModbusDATA[126];
	 		 		operations[function_ID].acceleration = ModbusDATA[127];

	 		 		//Set the direction of the motor
	 		 		if(motors[motor_ID].current_position <= operations[function_ID].absolute_position
	 		 				&& operations[function_ID].absolute_position <= ((int)(motors[motor_ID].current_position) + 180)%360)
	 		 		{
	 		 			operations[function_ID].direction = 0;
	 		 			if(motors[motor_ID].current_position > operations[function_ID].absolute_position)
	 		 			{
	 		 				operations[function_ID].angle_of_rotation = (360 - motors[motor_ID].current_position) + operations[function_ID].absolute_position;
	 		 			}

	 		 			else
	 		 			{
	 		 				operations[function_ID].angle_of_rotation = operations[function_ID].absolute_position - motors[motor_ID].current_position;
	 		 			}
	 		 		}
	 		 		else
	 		 		{
	 		 			operations[function_ID].direction = 1;
	 		 			if(motors[motor_ID].current_position <  operations[function_ID].absolute_position)
	 		 			{
	 		 				operations[function_ID].angle_of_rotation = motors[motor_ID].current_position + (360-operations[function_ID].absolute_position);
	 		 			}

	 		 			else
	 		 			{
	 		 				operations[function_ID].angle_of_rotation = motors[motor_ID].current_position - operations[function_ID].absolute_position;
	 		 			}
	 		 		}

	 		 		// i am not writing the pin of the direction
	 		 		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12,operations[function_ID].direction);

	 				//Define timer 1's variable
	 		 		operations[function_ID].initial_velocity = 50.0;
	 		 		operations[function_ID].step_rate = 50.0;
	 		 		operations[function_ID].total_steps = step_count(operations[function_ID].angle_of_rotation);
	 		 		operations[function_ID].Acceleration_State = 1;
	 				period_Update(operations[function_ID].initial_velocity,motor_ID);

	 				// Define timer 2's variable
	 				F_clk_accel = 275000000;
	 				PSC_accel = 1499;
	 				frequency_accel = operations[function_ID].acceleration;
	 				period_accel = (F_clk_accel / (frequency_accel * (PSC_accel + 1))) - 1;
	 				motors[motor_ID].timer2->ARR = period_accel;
	 				motors[motor_ID].timer2->CCR1 = motors[motor_ID].timer2->ARR / 2;

	 				// start timers
	 				HAL_TIM_PWM_Start_IT(&(motors[motor_ID].htimer1), TIM_CHANNEL_1);
	 				HAL_TIM_PWM_Start_IT(&(motors[motor_ID].htimer2), TIM_CHANNEL_1);

	 			 break;
	 		 	 case FEED_TO_LENGTH:

					// Fill the parameters of the operation
					operations[function_ID].angle_of_rotation = ModbusDATA[125];
					operations[function_ID].max_velocity = ModbusDATA[126];
					operations[function_ID].acceleration = ModbusDATA[127];

					if(operations[function_ID].angle_of_rotation > 0)
					{
						operations[function_ID].direction = 0;
					}

					else
					{
						operations[function_ID].direction = 1;
					}

					HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12,operations[function_ID].direction);

					//Define timer 1's variable
					operations[function_ID].initial_velocity = 50.0;
					operations[function_ID].step_rate = 50.0;
					operations[function_ID].total_steps = step_count(operations[function_ID].angle_of_rotation);
					operations[function_ID].Acceleration_State = 1;
					period_Update(operations[function_ID].initial_velocity,motor_ID);

					// Define timer 2's variable

					F_clk_accel = 275000000;
					PSC_accel = 1499;
					frequency_accel = operations[function_ID].acceleration;
					period_accel = (F_clk_accel / (frequency_accel * (PSC_accel + 1))) - 1;
					motors[motor_ID].timer2->ARR = period_accel;
					motors[motor_ID].timer2->CCR1 = motors[motor_ID].timer2->ARR / 2;

					// start timers
					HAL_TIM_PWM_Start_IT(&(motors[motor_ID].htimer1), TIM_CHANNEL_1);
					HAL_TIM_PWM_Start_IT(&(motors[motor_ID].htimer2), TIM_CHANNEL_1);


	 			 break;

	 			 // seek home and zero are the same thing.
	 		 	 case SEEK_HOME:

	 		 		//Define timer 1's variable
					operations[function_ID].initial_velocity = 50.0;
					operations[function_ID].step_rate = 50.0;
					period_Update(operations[function_ID].initial_velocity,motor_ID);
					HAL_TIM_PWM_Start_IT(&(motors[motor_ID].htimer1), TIM_CHANNEL_1);

					// While is not on.
	 		 		 while(motors[motor_ID].current_position != motors[motor_ID].zero)
	 		 		 {
	 		 			HAL_TIM_PWM_Stop_IT(&(motors[motor_ID].htimer1), TIM_CHANNEL_1);
	 		 			motors[motor_ID].current_position = motors[motor_ID].zero;
	 		 		 }

	 			 break;
	 		 	 case ZERO:
	 		 		motors[motor_ID].current_position = motors[motor_ID].zero;
	 			 break;
	 		 	 case RUN_A_SPEED:

	 		 		 //take input
	 		 		 operations[function_ID].initial_velocity = ModbusDATA[125];
	 		 		 HAL_TIM_PWM_Start_IT(&(motors[motor_ID].htimer1), TIM_CHANNEL_1);
	 		 		 break;
	 		 }
	  }


	  if(motors[motor_ID].function > 0 && ModbusDATA[124] == ZERO)
	  {
		  HAL_TIM_PWM_Stop_IT(&(motors[motor_ID].htimer1), TIM_CHANNEL_1);
		  HAL_TIM_PWM_Stop_IT(&(motors[motor_ID].htimer2), TIM_CHANNEL_1);
	  }

    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

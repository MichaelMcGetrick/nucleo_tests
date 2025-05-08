/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) Michael McGetrick.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Modify print statement to add time-stamp at beginning:
#define print_log(f_, ...) printf("%s ", timestamp()), printf((f_), ##__VA_ARGS__), printf("\n")
// Define flags for UART logging
#define TEST_PERIOD	60
#define UART_DEBUG				//Switch on general debugging
#define UART_DEBUG_SAMPLING		//Switch on debugging during within samplin loop
#define TEST_RTC				// Control flag to measure timer integrity using RTC
#define UART_TRANSFER_TEST	   //Control flag to enable UART transfer time testing

// Define maximum data length buffer for samples
#define MAX_DATA_LEN 8092 //4096
// Define flag to indicate that sampled data be transferred via UART
//#define UART_TRANSFER

//Flag to process raw ADC values
//#define RAW_ADC

// Analog reference voltage (Vref+)
#define VREF                       (3300UL)
//Conversion factor for 12-bit ADC resolution
#define ADC_BIT_FAC					4096 - 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
volatile unsigned int rtc_flg = 0;
volatile unsigned int itr_cnt = 0;
volatile unsigned int uart_transfer_flg = 0;
char * timestamp();
uint16_t adc_mv(unsigned int);
void uart_data_transfer(void);
char *time_str = "";

unsigned int exp_cnts = 0;  //Expected sample count for timer test period
// Sampling attributes
float SYS_CLK  = 3E+8; //300MHz
float SAMPLE_FREQ =  1E+0;
float TIMER1_FREQ = 1E+4;
unsigned int TIMER1_ARR = 0; // Timer1 Counter
unsigned int PSC = 0;		//Timer Prescaler
unsigned int ADC_CONVS_PER_SEC = 0; 	//Number of ADC conversions per sec.
unsigned int adc_val, adc_val_mv;
uint16_t adc_data[MAX_DATA_LEN];
unsigned int num_uart_transfers = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_RTC_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
#if defined(__ICCARM__)
__ATTRIBUTES size_t __write(int, const unsigned char *, size_t);
#endif /* __ICCARM__ */

#if defined(__ICCARM__)
/* New definition from EWARM V9, compatible with EWARM8 */
int iar_fputc(int ch);
#define PUTCHAR_PROTOTYPE int iar_fputc(int ch)
#elif defined ( __CC_ARM ) || defined(__ARMCC_VERSION)
/* ARM Compiler 5/6*/
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#elif defined(__GNUC__)
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#endif /* __ICCARM__ */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)

{

  /* USER CODE BEGIN 1 */
	// Initialise timer variables:

	if (SAMPLE_FREQ >= 1.0E+4)
	{
		TIMER1_ARR = 1;
		PSC =  (int) ( SYS_CLK / (2 * TIMER1_FREQ)) - 1;
		ADC_CONVS_PER_SEC = TIMER1_FREQ / (TIMER1_ARR + 1);
	}
	else
	{
		TIMER1_ARR = (TIMER1_FREQ / SAMPLE_FREQ) - 1;
		PSC =  (int) ( SYS_CLK / (TIMER1_FREQ)) - 1;
		ADC_CONVS_PER_SEC = TIMER1_FREQ / (TIMER1_ARR + 1);
	}
  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Update SystemCoreClock variable according to RCC registers values. */
  SystemCoreClockUpdate();

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* Reset the RTC peripheral and the RTC clock source selection */
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_BACKUPRESET_FORCE();
    __HAL_RCC_BACKUPRESET_RELEASE();

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_RTC_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#ifdef UART_DEBUG
  print_log("Starting timer test for stm32h7s3l8 ..... \n\r");
  print_log("PSC: %d\n\r",PSC);
  print_log("TIMER1_ARR: %d\n\r",TIMER1_ARR);
  print_log ("SAMPLE_FREQ: %d\n\r",(int) SAMPLE_FREQ);
  print_log ("ADC_CONVS_PER_SEC: %d\n\r", ADC_CONVS_PER_SEC);
  /* Clock frequencies */
  /*
  print_log("SysClockFreq: %lu\n\r",HAL_RCC_GetSysClockFreq());
  print_log("HCLKFreq: %lu\n\r",HAL_RCC_GetHCLKFreq());
  print_log("PLK1Freq: %lu\n\r",HAL_RCC_GetPCLK1Freq());
  print_log("PLK2Freq: %lu\n\r",HAL_RCC_GetPCLK2Freq());
  */
  // TEST bit extractor:
  /*
  unsigned int REG_VAL = RCC->CCIPR1;
  unsigned int mask = 0x02;
  unsigned int bitshift = 24;
  unsigned res = ( ( mask << bitshift ) & REG_VAL ) >> bitshift;
  print_log("REG_VAL: %lu\n\r",REG_VAL);
  print_log("mask: %lu\n\r",mask);
  print_log("bitshift: %lu\n\r",bitshift);
  print_log("Selected register bits val: %lu\n\r",res);
  */
#ifdef TEST_RTC
  print_log("(Test period measured using RTC clock) \n\r");
#endif
#endif

  // calibrate ADC for better accuracy and start it w/ interrupt
  if(HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
	  Error_Handler();
  }



#ifdef TEST_RTC
  MX_RTC_Init();  //Start the RTC clock
  print_log("Timestamp -> Start .. \n\r");
#endif
  if(HAL_ADC_Start_IT(&hadc1) != HAL_OK)
  {
   	  Error_Handler();
  }

  // Start TIMER1
  if(HAL_TIM_Base_Start(&htim1) != HAL_OK)
  {
	 Error_Handler();
  }

  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


#ifdef TEST_RTC
	  if (rtc_flg == 1)
	  {
		  rtc_flg = 0;
		  //Stop timer
		  HAL_TIM_Base_Stop(&htim1);
		  //Perform test on UART transfer time
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
#ifdef UART_TRANSFER_TEST
		  unsigned int num_test_runs = 10;
		  print_log ("UART TRANSFER TEST FOR SAMPLED DATA: -------------\n\r");
		  for (int i = 0; i < num_test_runs; i++)
		  {
			  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_0);
			  uart_data_transfer();
			  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_0);
			  HAL_Delay(50);
		  }
#endif

		  break;

	  }
#endif

#ifdef UART_TRANSFER
	  if (uart_transfer_flg == 1)
	  {
		  uart_transfer_flg = 0;

		  // Send sampled data to PC
#ifdef UART_DEBUG_SAMPLING
		  print_log ("1 second of conversion complete .....\n\r");
#endif

		  uart_data_transfer();
#ifdef UART_DEBUG_SAMPLING
		  //print_log ("UART Transfer of %d sample(s) complete\n\r",ADC_CONVS_PER_SEC);
		  print_log ("\n\r");
#endif

		  num_uart_transfers += 1;
		  unsigned int NUM_TEST_TRANSFERS = 10;
		  if(num_uart_transfers == NUM_TEST_TRANSFERS)
		  {
			  HAL_TIM_Base_Stop(&htim1); // Will delete after initial testing
			  break;
		  }

	  }
#endif


  }


#ifdef UART_DEBUG
  	print_log("Timestamp -> End .. \n\r");
#ifdef TEST_RTC
  	print_log("Sample count: %d \n\r",itr_cnt);
#endif
#endif


  /* USER CODE END 3 */
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	  /* USER CODE END ADC1_Init 0 */

	  ADC_MultiModeTypeDef multimode = {0};
	  ADC_ChannelConfTypeDef sConfig = {0};

	  /* USER CODE BEGIN ADC1_Init 1 */

	  /* USER CODE END ADC1_Init 1 */

	  /** Common config
	  */
	  hadc1.Instance = ADC1;
	  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	  hadc1.Init.LowPowerAutoWait = DISABLE;
	  hadc1.Init.ContinuousConvMode = DISABLE;
	  hadc1.Init.NbrOfConversion = 1;
	  hadc1.Init.DiscontinuousConvMode = DISABLE;
	  //hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	  //hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
	  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;

	  hadc1.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
	  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
	  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	  hadc1.Init.OversamplingMode = DISABLE;
	  if (HAL_ADC_Init(&hadc1) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /** Configure the ADC multi-mode
	  */
	  multimode.Mode = ADC_MODE_INDEPENDENT;
	  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /** Configure Regular Channel
	  */
	  sConfig.Channel = ADC_CHANNEL_2;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
	  sConfig.OffsetNumber = ADC_OFFSET_NONE;
	  sConfig.Offset = 0;
	  sConfig.OffsetSign = ADC_OFFSET_SIGN_NEGATIVE;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /* USER CODE BEGIN ADC1_Init 2 */

	  /* USER CODE END ADC1_Init 2 */


}

/* USER CODE BEGIN 4 */

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_PrivilegeStateTypeDef privilegeState = {0};
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = RTC_ASYNCH_PREDIV;
  hrtc.Init.SynchPrediv = RTC_SYNCH_PREDIV;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  hrtc.Init.BinMode = RTC_BINARY_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  privilegeState.rtcPrivilegeFull = RTC_PRIVILEGE_FULL_NO;
  privilegeState.backupRegisterPrivZone = RTC_PRIVILEGE_BKUP_ZONE_NONE;
  privilegeState.backupRegisterStartZone2 = RTC_BKP_DR0;
  privilegeState.backupRegisterStartZone3 = RTC_BKP_DR0;
  if (HAL_RTCEx_PrivilegeModeSet(&hrtc, &privilegeState) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0;
  sAlarm.AlarmTime.Minutes = 1;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
    {
      Error_Handler();
    }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = PSC; //14999; //29999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = TIMER1_ARR;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  //NB: The following trigger definitions worked with Timer only!
  // We require event update configuration
  //sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  //sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  huart3.Init.BaudRate = 460800; //115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_ODD;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */
	GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /*Configure GPIO pin Output Level */
    //HAL_GPIO_WritePin(GPIOD, LD2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);

    /*Configure GPIO pins : LD2_Pin LD1_Pin */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);


    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);

        /*Configure GPIO pins : LD2_Pin LD1_Pin */
        GPIO_InitStruct.Pin = GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);



  /* USER CODE END MX_GPIO_Init_2 */
}



/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART3 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

#if defined(__ICCARM__)
size_t __write(int file, unsigned char const *ptr, size_t len)
{
  size_t idx;
  unsigned char const *pdata = ptr;

  for (idx = 0; idx < len; idx++)
  {
    iar_fputc((int)*pdata);
    pdata++;
  }
  return len;
}
#endif /* __ICCARM__ */

/* USER CODE END 4 */


/* USER CODE BEGIN 5 */
/**
  * @brief  Return timestamp for current time.
  * @param  None
  * @retval char* - timestamp
  */
char * timestamp()
{

	unsigned int tick, rem, num, fac;
	unsigned int ts[4];

	// Get msec tick from system:
	tick = HAL_GetTick();
	num = tick;

	for (int i=3; i > 0; i--)
	{
		fac = (i == 3) ? 1000 : 60;
		rem = num % fac;
		num = num / fac;
		ts[i] = rem;
		ts[i-1] = num;


	}

	sprintf(time_str,"%02d:%02d:%02d.%03d ",ts[0],ts[1],ts[2],ts[3]);

	return time_str;


}

/**
  * @brief  Convert raw ADC value to mV.
  * @param  ADC raw value
  * @retval Value in mV
  */
uint16_t adc_mv(unsigned int val)
{
	return  (VREF * val) / ADC_BIT_FAC;

}


void uart_data_transfer(void)
{
	// Checking data
	uint16_t val;
	for (int i = 0; i < ADC_CONVS_PER_SEC;i++)
	{

#ifdef RAW_ADC
#ifdef UART_DEBUG
		printf("%d\n\r",adc_data[i]);
#endif
		HAL_UART_Transmit(&huart3, (uint8_t *)&adc_data[i], sizeof(adc_data[i]), 1);
#else
#ifdef UART_DEBUG
		//printf("%d\n\r",adc_mv(adc_data[i]));
#endif
		val = adc_mv(adc_data[i]);
		HAL_UART_Transmit(&huart3, (uint8_t *)&val, sizeof(val), 1);
#endif



		// Use this to make transfer more efficient - TBD

	}




}


/* USER CODE END 5 */


/* USER CODE BEGIN 7 */
/**
  * @brief  Alarm callback
  * @param  hrtc : RTC handle
  * @retval None
  */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{

  // Turn LD1 on: Alarm generation
  //BSP_LED_On(LD1);
   rtc_flg = 1;
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);


}

/**
  * @brief  ADC callback
  * @param  hadc : ADC handle
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	adc_data[itr_cnt] = HAL_ADC_GetValue(&hadc1);
	itr_cnt += 1;
#ifdef TEST_RTC
	print_log("Sample count: %d\n\r",itr_cnt);
#else
	if (itr_cnt == ADC_CONVS_PER_SEC)
	{
		uart_transfer_flg = 1;
		itr_cnt = 0;

	}
#endif


	// For test purposes: ----
	// Toggle the Green LED
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_10);
    //Toggle output to scope
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_0);
}


/* USER CODE END 7 */




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


/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "PID.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/*** MOTORES BEGIN PV ***/
typedef enum enumMotorID {MOTOR_ID_NOVALID,
							MOTOR_IZQ_ID,
							MOTOR_DER_ID} enumMotorID;

typedef enum enumMotorFreno {MOTOR_FRENO_FRENAR,
							MOTOR_FRENO_LIBERAR} enumMotorFreno;

typedef enum enumMotorError {MOTOR_ERR_SUCCESS,
							MOTOR_ERR_ID_INVALID,
							MOTOR_ERR_POT_OUT_OF_RANGE,
							MOTOR_ERR_FRENO_ST_NOTVALID} enumMotorError;

enum enumMotorFrenoState {MOTOR_FRENO_ST_LIBERADO,
							MOTOR_FRENO_ST_FRENADO};

struct motores_struct
{
	uint8_t freno_state[MOTOR_DER_ID];	//Valores admitidos para freno_state: enumMotorFrenoState
	uint8_t	potencia[MOTOR_DER_ID];		//Potencia actual de cada motor
	GPIO_TypeDef * frenoPinPort[MOTOR_DER_ID];		//Puerto del freno correspondiente
	uint16_t frenoPin[MOTOR_DER_ID];			//frenoPin del puerto
} MotoresData;

/*** MOTORES END PV ***/

/*** SENSORES BEGIN PV ***/
typedef enum enumSensores_ID {SENS_V_IZQ_5, SENS_V_IZQ_4, SENS_IZQ_3 ,SENS_IZQ_2,	SENS_IZQ_1,	SENS_DER_1,	SENS_DER_2, SENS_DER_3, SENS_V_DER_4, SENS_V_DER_5} enumSensores_ID;
typedef enum enumSensores_States {SENSORES_POS_LINEA_CENTRO, SENSORES_POS_LINEA_IZQ, SENSORES_POS_LINEA_DER} enumSensores_States;
typedef enum enumSensoresError {SENSORES_ERROR_SUCCESS = 0} enumSensoresError;

struct sensores_struct {
	uint16_t maximo_historico; //Valor máximo de la medición de los sensores
	uint16_t ADC_raw[SENSORES_LINEA_COUNT];	//Valores tomados de los ADC
	uint16_t ADC_fil[SENSORES_LINEA_COUNT];	//Valores tomados de los ADC
	int16_t posicion_x[SENSORES_LINEA_COUNT]; //Valores fijos de la posición real de los sensores en décimas de milímetro
	uint8_t pos_linea; //Guarda la posicón de la línea respecto al centro
	uint32_t maximo_historico_acum;
	uint8_t maximos_contados;
} SensoresData;
/*** SENSORES END PV ***/


/*** LEDS BEGIN PV ***/
typedef enum enumLedsID {LED_NOVALID_ID, LED_1_ID, LED_2_ID} enumLedsID;
typedef enum enumLedsStates {LED_ST_OFF, LED_ST_ON} enumLedsStates;
typedef enum enumLedsError {LED_ERR_SUCCESS, LED_ERR_ST_NO_VALID} enumLedsError;

struct leds_struct
{
	GPIO_TypeDef * Port[LED_2_ID];		//Puerto del LED1
	uint16_t Pin[LED_2_ID];				//Pin
} LedsData;

/*** LEDS END PV ***/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                

/* USER CODE BEGIN PFP */

void calcVel(int velMax, int velMin, double newCorrection);
int increasePower(int currPowMax);

void debugPrint(char _out[]);

/*** SENSORES AND MOTORES BEGIN PFP ***/

enumMotorError motorInit (void); //Inicializa estructura de motores
enumMotorError motorSetFreno(enumMotorID motor_ID, enumMotorFreno freno_st); //Setea estado del freno del motor correspondiente
enumMotorError motorSetPotencia(enumMotorID motor_ID, uint8_t potencia); //Setea potencia del motor correspondiente

enumSensoresError sensoresInit(); //Inicializa estructura de Sensores
enumSensoresError sensoresReadyToRace(void); //Reinicia el módulo para iniciar una carrera.
int16_t sensoresGetValActual(void); //Retorna posición de la línea relativa al centro del robot (valores entre -100 y 100)

/*** SENSORES AND MOTORES END PFP***/

/*** LEDS BEGIN PFP ***/
enumLedsError ledsInit(void); //Inicializar Leds
enumLedsError ledsSet(enumLedsID led_ID, enumLedsStates led_st);

/*** LEDS END PFP ***/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* Sensor range defines */
#define SENSOR_SET_POINT_VALUE	0
#define MAX_SENSOR_VALUE		(1000)
#define MIN_SENSOR_VALUE		(-1000)

/* Speed range defines */
#define RACE_POWER_SET_VALUE	70
#define MAX_POWER_VALUE			100
#define MIN_POWER_VALUE			0

/* Driver Mode */
int _driverMode;


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	double sensorValue = 0;
	double correction = 0;

	int powMax = RACE_POWER_SET_VALUE;
	int powMin = MIN_POWER_VALUE;


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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  //int readSpeed = 0;

  /*** SENSORES AND MOTORS USER CODE BEGIN 1 ***/
  motorInit();
  sensoresInit();
  ledsInit();
  /*** SENSORES AND MOTORS USER CODE END 1 ***/

  _driverMode = RACE_MODE;
  /* PID init */
  pidInit();

  /* Sensor PID pointer */
  PID_s *pidSensores;
  /* Sensor get PID */
  if (true != pidGetPID(&pidSensores))
  	return -1;
  /* PID Sensor set */
  pidSet( pidSensores, 0, MAX_SENSOR_VALUE, MIN_SENSOR_VALUE, 1, 0.250, 0.5, 0, 0);

#if 0
  /* Rueda Izq PID pointer */
  PID_s *pidRuedaIzq;
  /* Rueda get PID */
  if (true != pidGetPID(&pidRuedaIzq))
  	return ERROR_FAIL;
  /* PID Sensor set */
  pidSet( pidRuedaIzq, 0, 100, 20, 5, 5, 5, 0, 0);

  /* Rueda Der PID pointer */
  PID_s *pidRuedaDer;
  /* Rueda get PID */
  if (true != pidGetPID(&pidRuedaDer))
  	return -1;
  /* PID Sensor set */
  pidSet( pidRuedaDer, 0, 100, 20, 5, 5, 5, 0, 0);
#endif

  //TEST Potencia de motor
  motorSetPotencia(MOTOR_DER_ID, 1);
  motorSetPotencia(MOTOR_IZQ_ID, 1);
  debugPrint("Seguidor de Linea \n");
  //ledsSet(LED_1_ID, LED_ST_OFF);
  //ledsSet(LED_2_ID, LED_ST_OFF);
  char mensaje[200];
  uint16_t valorActualTest;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //ledsSet(LED_1_ID, LED_ST_OFF);
	  //ledsSet(LED_2_ID, LED_ST_OFF);
	  //HAL_Delay(500);
	  //ledsSet(LED_1_ID, LED_ST_ON);
	  //ledsSet(LED_2_ID, LED_ST_ON);

	  valorActualTest = sensoresGetValActual();
	  sprintf(mensaje, "%06d \r", valorActualTest);
	  //debugPrint(mensaje);

	  HAL_Delay(250);
	  /* Check Mode */
		switch (_driverMode) {
			case ERROR_MODE:
				/* Error mode, display error */
				//setLed();
				_driverMode = RACE_MODE;
				break;

			case CALIBRATE_SENSOR_MODE:
				/* Calibrate Sensor Mode */
				//setLed();
				break;

			case PRE_RACE_MODE:
				/* PreRace Mode */

				/* Led status Race Mode */
				//setLed();

				/* Wait for Button To change Max Power to Motor */
				if(false) /*  */
				{
					/* take new power value */
					powMax = increasePower(powMax);

					/* Led Power Value */
					//setLedBLink(powMax/ledFrequency);
				}

				/* Wait for Button ON to RUN */
				if(false) /* Is button change mode on? */
				{
					//Example pidSet (pidSensores, 0.1, 100, -100, 0.6, 0.02, 0.1, 0, 0);
					/* This break is to be ready for Run, while runButton is ON the robot is waiting for run */
					pidSet( pidSensores, 0.1, MAX_SENSOR_VALUE, MIN_SENSOR_VALUE, 0.6, 0.02, 0.01, 0, 0);
					/* Go to Race */
					_driverMode = RACE_MODE;
				}
				break;

			case RACE_MODE:
				/* Race Mode */

			    /* TODO Cesar */
				/* Get sensor error */
				// sensorValue = mySensores.getValue();

				/* Calculate error PID */
				correction = pidCalculate(pidSensores, SENSOR_SET_POINT_VALUE, sensorValue);

				/* Calculate velocity of a motors and set values */
				calcVel(powMax, powMin, correction);

				break;

			default:
				/* Unknown mode */
				_driverMode = ERROR_MODE;
				break;
		};

		/* Change Mode Button */
		if(false) /* Is button change mode on? */
		{
			_driverMode++;
			if (LAST_MODE <= _driverMode)
			{
				_driverMode = CALIBRATE_SENSOR_MODE;
			}
		}

  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC
                              |RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
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

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the ADC multi-mode 
    */
  multimode.Mode = ADC_DUALMODE_REGSIMULT_ALTERTRIG;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* ADC2 init function */
static void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */
  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */
  /* USER CODE END RTC_Init 1 */

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN RTC_Init 2 */
  /* USER CODE END RTC_Init 2 */

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 24;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = MOTOR_PWM_STEPS;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 24;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = MOTOR_PWM_STEPS;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
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

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_PCB_GPIO_Port, LED_PCB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_1_Pin|LED_2_Pin|BUZZER_OUT_Pin|Freno_DER_Pin 
                          |Freno_IZQ_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_PCB_Pin */
  GPIO_InitStruct.Pin = LED_PCB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PCB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PUL_ARRANQUE_Pin BOOT1_Pin BOTON_2_IN_Pin BOTON_1_IN_Pin */
  GPIO_InitStruct.Pin = PUL_ARRANQUE_Pin|BOOT1_Pin|BOTON_2_IN_Pin|BOTON_1_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_1_Pin LED_2_Pin BUZZER_OUT_Pin Freno_DER_Pin 
                           Freno_IZQ_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin|LED_2_Pin|BUZZER_OUT_Pin|Freno_DER_Pin 
                          |Freno_IZQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void calcVel(int powMax, int powMin, double newCorrection)
{
    int velMotorIzq = powMax;
    int velMotorDer = powMax;

    /* newCorrection es bueno que sea un double asi se puede multiplicar por un fraccional */
    newCorrection = newCorrection * 1.5;

    if(newCorrection > 0)
    {
        velMotorIzq = velMotorIzq - (int) newCorrection;
        if (velMotorIzq < powMin)
        {
            velMotorIzq = powMin;
        }
    }
    else if (newCorrection < 0)
    {
        velMotorDer = velMotorDer + (int) newCorrection;
        if (velMotorDer < powMin)
        {
            velMotorDer = powMin;
        }
    }

    /* TODO Cesar */
    //MotorDer.setPotencia((unsigned short int) velMotorDer);
    //MotorIzq.setPotencia((unsigned short int) velMotorIzq);
}

int increasePower(int currPowMax)
{
	 int newPower = currPowMax;
	    /* increase power on 10% */
	    newPower += 10;
	    /* Check overflow */
	    if(newPower > MAX_POWER_VALUE)
	    {
	    	/* Set half of power 50% */
	    	newPower = MAX_POWER_VALUE/2;
	    }
	    return newPower;
}

/*** MOTORES FUNCTION DEF BEGIN ***/
/**
 * @brief Esta función inicializa la estructura de los motores
 * @param motor_ID: MOTOR_IZQ_ID ó MOTOR_DER_ID
 * @param potencia: Potencia a inyectar al motor, desde 0 a MOTOR_POT_MAX
 */
enumMotorError motorInit (void)
{

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	MotoresData.frenoPinPort[MOTOR_DER_ID] = Freno_DER_GPIO_Port;
	MotoresData.frenoPin[MOTOR_DER_ID] = Freno_DER_Pin;
	MotoresData.frenoPinPort[MOTOR_IZQ_ID] = Freno_IZQ_GPIO_Port;
	MotoresData.frenoPin[MOTOR_IZQ_ID] = Freno_IZQ_Pin;

	return MOTOR_ERR_SUCCESS;
}

/**
 * @brief Esta función envía potencia a los motores en PWM
 * @param motor_ID: MOTOR_IZQ_ID ó MOTOR_DER_ID
 * @param potencia: Potencia a inyectar, desde 0 a MOTOR_POT_MAX
 */
enumMotorError motorSetPotencia(enumMotorID motor_ID, uint8_t potencia)
{
	char mensaje[100];
	sprintf(mensaje, "Setear ID %d, Potencia %d \n\r", motor_ID, potencia);
	debugPrint(mensaje);

	//Verifico ID del motor
	if(motor_ID != MOTOR_IZQ_ID && motor_ID != MOTOR_DER_ID)
	{
		return MOTOR_ERR_ID_INVALID;
	}

	//Verifico estado del freno
	if(MotoresData.freno_state[motor_ID] == MOTOR_FRENO_ST_FRENADO)
	{
		//Liberar freno si está seteado
		HAL_GPIO_WritePin(MotoresData.frenoPinPort[motor_ID], MotoresData.frenoPin[motor_ID], MOTOR_FRENO_OUT_PIN_LIBERAR);
	}

	//Cambiar PWM del motor indicado a la potencia especificada
	if (potencia <= MOTOR_POT_MAX)
	{
		if(motor_ID == MOTOR_DER_ID)
		{
			sprintf(mensaje, "Estoy setenado Potencia \n");
			debugPrint(mensaje);

			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, potencia * MOTOR_PWM_STEPS / MOTOR_POT_MAX);
		}
		else
		{

			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, potencia * MOTOR_PWM_STEPS / MOTOR_POT_MAX);
		}
	}
	else
	{
		return MOTOR_ERR_POT_OUT_OF_RANGE;
	}

	return MOTOR_ERR_SUCCESS;
}

/**
 * @brief Esta función frena el motor especificado
 * @param motor_ID: MOTOR_IZQ_ID ó MOTOR_DER_ID
 * @param freno_st: Estado en que se quiere poner el freno (enumMotorFreno)
 */
enumMotorError motorSetFreno(enumMotorID motor_ID, enumMotorFreno freno_st)
{
	//Verificar ID del motor
	if(motor_ID != MOTOR_IZQ_ID || motor_ID != MOTOR_DER_ID)
	{
		return MOTOR_ERR_ID_INVALID;
	}


	//Setear estado del freno
	if (freno_st == MOTOR_FRENO_FRENAR)
	{
		//Cortar PWM del motor correspondiente si tiene algún valor

		if(motor_ID == MOTOR_DER_ID)
		{
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
		}
		else
		{
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		}

		//Efectuar frenado
		HAL_GPIO_WritePin(MotoresData.frenoPinPort[motor_ID], MotoresData.frenoPin[motor_ID], MOTOR_FRENO_OUT_PIN_FRENAR);
	}
	else if (freno_st == MOTOR_FRENO_LIBERAR)
	{
		//Liberar freno
		HAL_GPIO_WritePin(MotoresData.frenoPinPort[motor_ID], MotoresData.frenoPin[motor_ID], MOTOR_FRENO_OUT_PIN_LIBERAR);
	}
	else
	{
		return MOTOR_ERR_FRENO_ST_NOTVALID;
	}

	return MOTOR_ERR_SUCCESS;
}
/*** MOTORES FUNCTION DEF END ***/

/*** SENSORES FUNCTION DEF BEGIN ***/

/**
 * @brief Esta función imprime por el puerto serie el array de caracteres enviado en _out
 * @param _out: puntero a la cadena de caracteres a imprimir finalizada en null
 */
void debugPrint(char _out[]){
 HAL_UART_Transmit(&huart1, (uint8_t *)_out, strlen(_out), 100);
}


/**
 * @brief Esta función Inicializa la estructura inicial de los sensores.
 */

enumSensoresError sensoresInit(void)
{
	//Inicializo posiciones de los sensores respecto del centro del robot, en centésimas de milímetros
	SensoresData.posicion_x[SENS_V_IZQ_5] = -4500;
	SensoresData.posicion_x[SENS_V_IZQ_4] = -3500;
	SensoresData.posicion_x[SENS_IZQ_3] = -2500;
	SensoresData.posicion_x[SENS_IZQ_2] = -1500;
	SensoresData.posicion_x[SENS_IZQ_1] = -500;
	SensoresData.posicion_x[SENS_DER_1] = 500;
	SensoresData.posicion_x[SENS_DER_2] = 1500;
	SensoresData.posicion_x[SENS_DER_3] = 2500;
	SensoresData.posicion_x[SENS_V_DER_4] = 3500;
	SensoresData.posicion_x[SENS_V_DER_5] = 4500;
	SensoresData.maximo_historico = 0;
	SensoresData.pos_linea = SENSORES_POS_LINEA_CENTRO;
	SensoresData.maximo_historico_acum = 0;
	SensoresData.maximos_contados = 0;

	return SENSORES_ERROR_SUCCESS;
}


/**
 * @brief Esta función Inicializa variables de los sensores para dejarlo listo para largar.
 */

enum enumSensoresError sensoresReadyToRace(void)
{
	//Reiniciar el máximo histórico
	SensoresData.maximo_historico = 0;
	SensoresData.maximo_historico_acum = 0;
	SensoresData.maximos_contados = 0;
	return SENSORES_ERROR_SUCCESS;
}

/**
 * @brief Esta función devuelve el valor actual de los sensores, marcando la diferencia de la
 * posición de la línea con el centro, entre MIN_SENSOR_VALUE y MAX_SENSOR_VALUE
 * @param void: sin parámetros
 */
int16_t sensoresGetValActual(void)
{
	uint8_t i;
	int16_t valActual = 0;
	uint16_t maximo = 0, minimo = 0, promedio = 0;
	long int sumaproductovalores = 0, sumavalores = 0, centro = 0;

	//Inicio de toma de valores de los ADC

	HAL_GPIO_WritePin(LED_PCB_GPIO_Port, LED_PCB_Pin, GPIO_PIN_SET);

	HAL_ADC_Start(&hadc1);

	HAL_ADC_PollForConversion(&hadc1, 1);
	SensoresData.ADC_raw[SENS_IZQ_3] = 4095 - HAL_ADC_GetValue(&hadc1);

	HAL_ADC_PollForConversion(&hadc1, 1);
	SensoresData.ADC_raw[SENS_IZQ_2] = 4095 - HAL_ADC_GetValue(&hadc1);

	HAL_ADC_PollForConversion(&hadc1, 1);
	SensoresData.ADC_raw[SENS_IZQ_1] = 4095 - HAL_ADC_GetValue(&hadc1);

	HAL_ADC_Stop(&hadc1);

	HAL_ADC_Start(&hadc2);

	HAL_ADC_PollForConversion(&hadc2, 1);
	SensoresData.ADC_raw[SENS_DER_1] = 4095 - HAL_ADC_GetValue(&hadc2);

	HAL_ADC_PollForConversion(&hadc2, 1);
	SensoresData.ADC_raw[SENS_DER_2] = 4095 - HAL_ADC_GetValue(&hadc2);

	HAL_ADC_PollForConversion(&hadc2, 1);
	SensoresData.ADC_raw[SENS_DER_3] = 4095 - HAL_ADC_GetValue(&hadc2);

	HAL_ADC_Stop(&hadc2);

	HAL_GPIO_WritePin(LED_PCB_GPIO_Port, LED_PCB_Pin, GPIO_PIN_RESET);

	//Cálculo máximo y mínimo y promedio
	char mensaje[200];
	//sprintf(mensaje, "Sensores fin conversion\n\r");
	//debugPrint(mensaje);

	minimo = maximo = SensoresData.ADC_raw[SENS_IZQ_3];
	sumavalores = 0;
	for (i = SENS_IZQ_3; i <= SENS_DER_3; i++ )
	{
		sprintf(mensaje, "%04d ", SensoresData.ADC_raw[i]);
		debugPrint(mensaje);

		if (SensoresData.ADC_raw[i] < minimo)
		{
			minimo = SensoresData.ADC_raw[i];
		}
		else if (SensoresData.ADC_raw[i] > maximo)
		{
			maximo = SensoresData.ADC_raw[i];
		}
		sumavalores = sumavalores + SensoresData.ADC_raw[i];
	}
	promedio = sumavalores / (i - SENS_IZQ_3);

	//sprintf(mensaje, "\r");
	//debugPrint(mensaje);
	//LOGICA AGREGADA PARA MEJORAR PERFORMANCE
	//Se agregan 4 sensores virtuales más, simulando los que no están a la izquierda y a la derecha,
	//quedarán indicando el máximo valor, tanto a izquierda como a derecha, si pierden la línea
	//Sólo se encenderán si la línea no está en posiciones centrales (SENSORES_POS_LINEA_CENTRO).
	SensoresData.ADC_raw[SENS_V_IZQ_5] = 0;//(SensoresData.pos_linea == SENSORES_POS_LINEA_IZQ)?(SensoresData.maximo_historico - SensoresData.ADC_raw[SENS_IZQ_3] + minimo):minimo;
	SensoresData.ADC_raw[SENS_V_IZQ_4] = 0;//(SensoresData.pos_linea == SENSORES_POS_LINEA_IZQ)?(SensoresData.maximo_historico - SensoresData.ADC_raw[SENS_IZQ_2] + minimo):minimo;
	SensoresData.ADC_raw[SENS_V_DER_4] = 0;//(SensoresData.pos_linea == SENSORES_POS_LINEA_DER)?(SensoresData.maximo_historico - SensoresData.ADC_raw[SENS_DER_2] + minimo):minimo;
	SensoresData.ADC_raw[SENS_V_DER_5] = 0;//(SensoresData.pos_linea == SENSORES_POS_LINEA_DER)?(SensoresData.maximo_historico - SensoresData.ADC_raw[SENS_DER_3] + minimo):minimo;

	//sprintf(mensaje, "\r");
	//debugPrint(mensaje);

	//Cálculo de Valor actual
	//Se calcula como un centro de gravedad de la señal de cada sensor

	//1ro Calculo suma de productos de cada valor del sensor por la distancia al centro de los sensores
	sumaproductovalores = 0;
	sumavalores = 0;
	for(uint8_t i = SENS_IZQ_3; i <= SENS_DER_3; i++)
	{
		SensoresData.ADC_fil[i] = (SensoresData.ADC_raw[i] > promedio)? SensoresData.ADC_raw[i] - promedio : 0;
		sumaproductovalores += SensoresData.ADC_fil[i] * SensoresData.posicion_x[i];
		sumavalores += SensoresData.ADC_fil[i];
	}
	//Valor del centro de gravedad de los sensores
	centro = sumaproductovalores / sumavalores;

	//Valor actual sin escalar
	valActual = (int16_t)centro;


	//Determinación de última posición relativa de la línea según las mediciones actuales,
	//de lo contrario, si no ve más la línea recuerda la última posición y el máximo histórico
	if((maximo - minimo) > SENSORES_DIF_RAW_MIN)
	{
		if(valActual > SensoresData.posicion_x[SENS_DER_2])
		{
			//Si el centro de la línea está más allá del segundo sensor derecho
			SensoresData.pos_linea = SENSORES_POS_LINEA_DER;
		}
		else if (valActual < SensoresData.posicion_x[SENS_IZQ_2])
		{
			//Si el centro de la línea está más allá del segundo sensor izquierdo
			SensoresData.pos_linea = SENSORES_POS_LINEA_IZQ;
		}
		else
		{
			//Si el centro de la línea está entre el segundo sensor izquierdo y el segundo sensor derecho.
			SensoresData.pos_linea = SENSORES_POS_LINEA_CENTRO;
		}

		//Mantenimiento de maximo histórico
		if (SensoresData.maximo_historico == 0)
		{
			SensoresData.maximo_historico = maximo;
			SensoresData.maximo_historico_acum = maximo;
			SensoresData.maximos_contados = 1;
		}
		else
		{
			if (SensoresData.maximos_contados < SENSORES_MAXIMOS_CONTADOS_MAX)
			{
				SensoresData.maximos_contados++;
			}
			else
			{
				//Le resto un valor medio al acum de los máximos
				SensoresData.maximo_historico_acum = SensoresData.maximo_historico_acum / SENSORES_MAXIMOS_CONTADOS_MAX * (SENSORES_MAXIMOS_CONTADOS_MAX - 1);
			}
			SensoresData.maximo_historico = (maximo + SensoresData.maximo_historico_acum) / SensoresData.maximos_contados;
			SensoresData.maximo_historico_acum = SensoresData.maximo_historico * SensoresData.maximos_contados;
		}
	}


	sprintf(mensaje, "%05d %05d %05d %012ld %012ld %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %c\r\n", 	valActual,
												maximo,
												minimo,
												sumaproductovalores,
												sumavalores,
												SensoresData.ADC_fil[SENS_V_IZQ_5],
												SensoresData.ADC_fil[SENS_V_IZQ_4],
												SensoresData.ADC_fil[SENS_IZQ_3],
												SensoresData.ADC_fil[SENS_IZQ_2],
												SensoresData.ADC_fil[SENS_IZQ_1],
												SensoresData.ADC_fil[SENS_DER_1],
												SensoresData.ADC_fil[SENS_DER_2],
												SensoresData.ADC_fil[SENS_DER_3],
												SensoresData.ADC_fil[SENS_V_DER_4],
												SensoresData.ADC_fil[SENS_V_DER_5],
												(SensoresData.pos_linea == SENSORES_POS_LINEA_IZQ)?'i':(SensoresData.pos_linea == SENSORES_POS_LINEA_DER)?'d':'c');
	debugPrint(mensaje);

	//Escalado de valor actual
	valActual = valActual * MAX_SENSOR_VALUE / abs(SensoresData.posicion_x[SENS_DER_3]);

	sprintf(mensaje, "Val actual: %05d", valActual);
	debugPrint(mensaje);

	return valActual;
}

/*** SENSORES FUNCTION DEF END ***/

/*** LEDS FUNCTION DEF BEGIN ***/
enumLedsError ledsInit(void)
{
	LedsData.Port[LED_1_ID] = LED_1_GPIO_Port;
	LedsData.Pin[LED_1_ID] = LED_1_Pin;
	LedsData.Port[LED_2_ID] = LED_2_GPIO_Port;
	LedsData.Pin[LED_2_ID] = LED_2_Pin;
	return LED_ERR_SUCCESS;
}

enumLedsError ledsSet(enumLedsID led_ID, enumLedsStates led_st)
{
	if(led_ID > LED_NOVALID_ID && led_ID <= LED_2_ID)
	{
		if(led_st == LED_ST_ON)
		{
			HAL_GPIO_WritePin(LedsData.Port[led_ID], LedsData.Pin[led_ID], GPIO_PIN_RESET);
		}
		else if (led_st == LED_ST_OFF)
		{
			HAL_GPIO_WritePin(LedsData.Port[led_ID], LedsData.Pin[led_ID], GPIO_PIN_SET);
		}
		else
		{
			return LED_ERR_ST_NO_VALID;
		}
	}
	return LED_ERR_SUCCESS;
}
/*** LEDS FUNCTION DEF END ***/

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

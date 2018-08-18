
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "PID.h"
#include "stdlib.h"
#include <string.h>
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
typedef enum enumMotorID {MOTOR_ID_NOVALID,	MOTOR_IZQ_ID, MOTOR_DER_ID, MOTORES_COUNT_ID} enumMotorID;
typedef enum enumMotorFreno {MOTOR_FRENO_FRENAR, MOTOR_FRENO_LIBERAR} enumMotorFreno;
typedef enum enumMotorError {MOTOR_ERR_SUCCESS, MOTOR_ERR_ID_INVALID, MOTOR_ERR_POT_OUT_OF_RANGE, MOTOR_ERR_FRENO_ST_NOTVALID} enumMotorError;

enum enumMotorFrenoState {MOTOR_FRENO_ST_LIBERADO, MOTOR_FRENO_ST_FRENADO};

struct motores_struct
{
	uint8_t freno_state[MOTORES_COUNT_ID];	//Valores admitidos para freno_state: enumMotorFrenoState
	uint16_t	velocidadSetPoint[MOTORES_COUNT_ID];		//Velocidad seteada para el motor
	GPIO_TypeDef * frenoPinPort[MOTORES_COUNT_ID];		//Puerto del freno correspondiente
	uint16_t frenoPin[MOTORES_COUNT_ID];			//frenoPin del puerto
	PID_s * pidMotor[MOTORES_COUNT_ID];
	uint8_t PWM_Actual[MOTORES_COUNT_ID];
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
typedef enum enumLedsID {LEDS_NOVALID_ID, LED_DER_ID, LED_IZQ_ID, LED_PLAQUITA_ID, LEDS_COUNT_ID} enumLedsID;
typedef enum enumLedsStates {LED_ST_NOVALID, LED_ST_OFF, LED_ST_ON, LED_ST_BLINK} enumLedsStates;
typedef enum enumLedsError {LED_ERR_SUCCESS, LED_ERR_ST_NO_VALID, LED_ERR_NO_VALID_ID} enumLedsError;

struct leds_struct
{
	GPIO_TypeDef * Port[LEDS_COUNT_ID];		//Puerto del LED1
	uint16_t Pin[LEDS_COUNT_ID];				//Pin
	uint8_t Estado[LEDS_COUNT_ID];			//Último estado registrado
	uint8_t RestantesBlinks[LEDS_COUNT_ID];			//Numero de parpadeos restantes
	uint16_t TimeOutBlink[LEDS_COUNT_ID];
} LedsData;

/*** LEDS END PV ***/

/*** BOTONES BEGIN PV ***/
typedef enum enumBotonesID {BOTONES_NOVALID_ID, BOTON_IZQ_ID, BOTON_DER_ID, BOTON_CENTRAL_ID, BOTONES_COUNT_ID} enumBotonesID;
typedef enum enumBotonesStates {BOTON_ST_PRESIONADO, BOTON_ST_NO_PRESIONADO, BOTONES_ST_ERROR_ID} enumBotonesStates;
typedef enum enumBotonesError {BOTONES_ERR_SUCCESS, BOTONES_ERR_ID_NO_VALID} enumBotonesError;
typedef enum enumBotonesEventos {BOTONES_EV_SIN_EVENTO, BOTONES_EV_PRESIONADO, BOTONES_EV_LIBERADO, BOTONES_EV_MANTENIDO_PRESIONADO, BOTONES_EV_ERROR_ID} enumBotonesEventos;
typedef enum enumBotonesTipo {BOTONES_TIPO_NA, BOTONES_TIPO_NC} enumBotonesTipo;

struct botones_struct
{
	GPIO_TypeDef * Port[BOTONES_COUNT_ID];
	uint16_t Pin[BOTONES_COUNT_ID];
	uint8_t Estado[BOTONES_COUNT_ID];
	uint8_t EstadoAnt[BOTONES_COUNT_ID];		//Estado anterior
	uint8_t Evento[BOTONES_COUNT_ID];
	uint8_t Tipo[BOTONES_COUNT_ID];
	int16_t RetardoAPresionado[BOTONES_COUNT_ID]; //Limitado por BOTONES_RETARDO_A_PRESIONADO_CICLOS
} BotonesData;
/*** BOTONES END PV ***/

/*** RUEDAS BEGIN PV ***/
typedef enum enumRuedasID {RUEDAS_NOVALID_ID, RUEDA_IZQ_ID, RUEDA_DER_ID, RUEDAS_COUNT_ID} enumRuedasID;
typedef enum enumRuedasError {RUEDAS_ERR_SUCCESS, RUEDAS_ERR_ID_NOT_VALID} enumRuedasError;

struct ruedas_structt
{
	uint32_t Circunferencia; //Longitud de la circunferencia de la rueda en micrones
	uint16_t Velocidad[RUEDAS_COUNT_ID]; //en (mm / s)
	uint16_t VelocidadAnt[RUEDAS_COUNT_ID]; //Guardar velocidad anterior.
	unsigned long int Odometro[RUEDAS_COUNT_ID]; // Cantidad de mm rodados desde el reset
	uint32_t Count_Raw[RUEDAS_COUNT_ID]; //Valor contado por el registro de los encoders
	unsigned long int TimeOut[RUEDAS_COUNT_ID]; //Cantidad de ciclos de programa transcurridos sin una interrupción del encoder.
	double dt[RUEDAS_COUNT_ID];
}RuedasData;
/*** RUEDAS END PV ***/

/*** PID OBJECT TEST BEGIN ***/
/* Test PID defines */
#define CALIBRACION_PID			1
#define PID_KP_COUNT			40

struct pid_struct
{
	double Kp[PID_KP_COUNT];
	uint16_t KpActualIndex;
	double KpActual;
}PidData;

/*** PID OBJECT TEST END ***/

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

void calcVel(int velRace, double newCorrection);
int changeMaxSpeed(int currSpeedMax);

void debugPrint(char _out[]);

/*** SENSORES AND MOTORES BEGIN PFP ***/

enumMotorError motoresInit (void); //Inicializa estructura de motores
enumMotorError motorSetFreno(enumMotorID motor_ID, enumMotorFreno freno_st); //Setea estado del freno del motor correspondiente
enumMotorError motoresSetPWM(enumMotorID motor_ID, uint8_t porcentaje); //NO USAR. Usar en su lugar motoresSetVelocidad.
enumMotorError motoresSetVelocidad(enumMotorID motor_ID, uint16_t velocidad); //Setea la velocidad deseada del motor en mm/s
uint8_t motorGetPWM(enumMotorID motor_ID);
void motoresService(void); //Presta servicio a los motores


enumSensoresError sensoresInit(); //Inicializa estructura de Sensores
enumSensoresError sensoresReadyToRace(void); //Reinicia el módulo para iniciar una carrera.
int16_t sensoresGetValActual(void); //Retorna posición de la línea relativa al centro del robot (valores entre -100 y 100)

/*** SENSORES AND MOTORES END PFP***/

/*** LEDS BEGIN PFP ***/
enumLedsError ledsInit(void); //Inicializar Leds
enumLedsError ledsSet(enumLedsID led_ID, enumLedsStates led_st);
enumLedsError ledsBlink(enumLedsID led_ID, uint8_t cantidad_blk);
void ledsService(void);
/*** LEDS END PFP ***/

/*** BOTONES BEGIN PFP ***/
enumBotonesError botonesInit(void); //Inicializar Leds
enumBotonesStates botonesGetEstado(enumBotonesID boton_ID);
enumBotonesEventos botonesGetEvento(enumBotonesID boton_ID);

void botonesService(void); //Presta servicio a los pulsadores
/*** BOTONES END PFP ***/

/*** RUEDAS BEGIN PFP ***/
enumRuedasError ruedasInit (void);
uint16_t ruedasGetVelocidad(enumRuedasID ruedas_ID);
unsigned long int ruedasGetOdometro(enumRuedasID ruedas_ID);
enumRuedasError ruedaResetOdometro (enumRuedasID rueda_ID);
void ruedasService(void);
/*** RUEDAS END PFP ***/

/*** PID TEST BEGIN ***/
void pidTestInit(void);
/*** PID TEST END ***/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* Sensor range defines */
#define SENSOR_SET_POINT_VALUE	0
#define MAX_SENSOR_VALUE		(1000)
#define MIN_SENSOR_VALUE		(-1000)

/* Speed range defines */
#define RACE_SPEED_SET_VALUE	(500)	/* Speed in mm/s */
#define MAX_SPEED_VALUE			(3000)
#define MIN_SPEED_VALUE			(0)

/* Motors range defines */
#define MOTOR_PWM_MAX_STEPS		(100)
#define MOTOR_SPEED_MAX			(3000)		/* Máxima velocidad para el PID de motores*/
#define MOTOR_SPEED_MIN			(0)
#define MOTOR_PID_KP_IZQ		(0.1)			/* KP PID MOTOR IZQ */
#define MOTOR_PID_KP_DER		MOTOR_PID_KP_IZQ

/* Leds defines */
#define LEDS_TIMEOUT_BLINK_CICLOS	(1000)		/* ciclos de blink */

/* Botones defines */
#define BOTONES_RETARDO_A_PRESIONADO_CICLOS	(50)	/* Expresado en ciclos del while de aprox 500us */

/* Ruedas defines */
#define RUEDAS_TIMEOUT_CICLOS_0_VEL	(200)		/* Controlar de relacionarlo con CORE_TIEMPO_CICLO, que represente este valor 1 segundo y medio*/
#define RUEDA_ENCODER_DIVS			(10)
/* Core defines */
#define CORE_TIEMPO_CICLO		(0.0005)		/* En segundos, el tiempo en segundos aproximado que consume el ciclo del programa*/


/* Driver Mode */
int _driverMode;

char mensaje[200];
//Función de interrupción
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  //HAL_GPIO_WritePin(LedsData.Port[LED_DER_ID], LedsData.Pin[LED_DER_ID], GPIO_PIN_RESET);

  if (htim->Instance==TIM3)
  {
	  RuedasData.Count_Raw[RUEDA_IZQ_ID] = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);    //read TIM2 channel 1 capture value
	  __HAL_TIM_SET_COUNTER(&htim3, 0);    //reset counter after input capture interrupt occurs
	  RuedasData.Odometro[RUEDA_IZQ_ID] += RuedasData.Circunferencia / 1000;
	  RuedasData.TimeOut[RUEDA_IZQ_ID] = 0; //Reseteo Timeout de espera de la interrupción
	  RuedasData.dt[RUEDA_IZQ_ID] = (double)RuedasData.Count_Raw[RUEDA_IZQ_ID] * 0.00002083333f;
	  RuedasData.Velocidad[RUEDA_IZQ_ID] = (double)RuedasData.Circunferencia /RUEDA_ENCODER_DIVS /((double)RuedasData.dt[RUEDA_IZQ_ID]) /1000;
  }
  if (htim->Instance==TIM4)
  {
	  RuedasData.Count_Raw[RUEDA_DER_ID] = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1);    //read TIM2 channel 1 capture value
	  __HAL_TIM_SET_COUNTER(&htim4, 0);    //reset counter after input capture interrupt occurs
	  RuedasData.Odometro[RUEDA_DER_ID] += RuedasData.Circunferencia / 1000;
	  RuedasData.TimeOut[RUEDA_DER_ID] = 0; //Reseteo Timeout de espera de la interrupción
	  RuedasData.dt[RUEDA_DER_ID] = (double)RuedasData.Count_Raw[RUEDA_DER_ID] * 0.00002083333f;
	  RuedasData.Velocidad[RUEDA_DER_ID] = (double)RuedasData.Circunferencia /RUEDA_ENCODER_DIVS /((double)RuedasData.dt[RUEDA_DER_ID]) /1000;

//	  sprintf(mensaje,"dt %0.7f v %d\r\n", RuedasData.dt[RUEDA_DER_ID], RuedasData.Velocidad[RUEDA_DER_ID]);
//	  debugPrint(mensaje);

  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	double correction = 0;

	int speedRace = RACE_SPEED_SET_VALUE;

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
  MX_USART1_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  //int readSpeed = 0;

  /*** SENSORES AND MOTORS USER CODE BEGIN 1 ***/
  motoresInit();
  sensoresInit();
  ledsInit();
  botonesInit();
  ruedasInit();
  pidTestInit();
  /*** SENSORES AND MOTORS USER CODE END 1 ***/

  /* Driver Modde default value */
  _driverMode = PRE_RACE_MODE;

  /* Sensor PID pointer */
  PID_s *pidSensores;
  /* Sensor get PID */
  if (true != pidGetPID(&pidSensores))
  	return -1;
  /* PID Sensor set default value */
  //pidSet( pidSensores, CORE_TIEMPO_CICLO, MAX_SENSOR_VALUE, MIN_SENSOR_VALUE, 0.7, 0.02, 0.01, 0, 0);

  int16_t SensorValorActual;

  int botonPresionado = false;

  /* Set motor defualt value */
  motoresSetPWM(MOTOR_DER_ID,0);
  motoresSetPWM(MOTOR_IZQ_ID,0);
  //motoresSetVelocidad(MOTOR_DER_ID, 0);
  //motoresSetVelocidad(MOTOR_IZQ_ID, 0);

  /* Set meds default value */
  ledsSet(LED_DER_ID, LED_ST_OFF);
  ledsSet(LED_IZQ_ID, LED_ST_OFF);

  debugPrint("Seguidor de Linea \n");

  //uint16_t valorActualTest;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //uint8_t pwm = 30;
  //uint16_t velocidad = 0;
  while (1)
  {

	  /* servicios */
	  motoresService();
	  botonesService();
	  ruedasService();
	  ledsService();
	  /*** TEST ZONE - Comentar en la versión Release ***/
	  //motorSetPotencia(MOTOR_DER_ID, 40);
	  //motorSetPotencia(MOTOR_IZQ_ID, 40);

//	  //sprintf(mensaje, "%05ld %05ld %d %d\r\n", RuedasData.Count_Raw[RUEDA_IZQ_ID],
//			  	  	  	  	  	  	  	  	  RuedasData.Count_Raw[RUEDA_DER_ID],
//											  ruedasGetVelocidad(RUEDA_IZQ_ID),
//											  ruedasGetVelocidad(RUEDA_DER_ID));
//	  debugPrint(mensaje);
	  //debugPrint("Activo ");
/*
	  if(botonesGetEstado(BOTON_DER_ID) == BOTON_ST_PRESIONADO)
	  {
		  debugPrint("Presionado \n\r");
	  }
	  else
	  {
		  debugPrint("No Presionado \n\r");
	  }
*/

/*
	  sensoresGetValActual();

	  motoresSetVelocidad(MOTOR_DER_ID, 1000);

		if(botonesGetEvento(BOTON_DER_ID) == BOTONES_EV_PRESIONADO)
		{
			if(pwm < 100)
			{
				ledsBlink(LED_DER_ID, 1);
				pwm++;
				velocidad += 100;
			}

			//motoresSetPWM(MOTOR_DER_ID, pwm);
			sprintf(mensaje, "PWM %d \n\r", pwm);
			debugPrint(mensaje);
		}

		if(botonesGetEvento(BOTON_IZQ_ID) == BOTONES_EV_PRESIONADO)
		{
			if(pwm > 0)
			{
				ledsBlink(LED_DER_ID, 2);
				pwm--;
			}
			//motoresSetPWM(MOTOR_DER_ID, pwm);
			sprintf(mensaje, "PWM %d \n\r", pwm);
			debugPrint(mensaje);
		}

		continue;
*/
/*
	  sensoresGetValActual();
	  //motorSetFreno(MOTOR_DER_ID, MOTOR_FRENO_FRENAR);
	  HAL_Delay(100);
	  continue;
*/
	   /*** TEST ZONE - FIN ***/

	  /* Check Mode */
		switch (_driverMode) {
			case ERROR_MODE:

				/* Error mode, display error */
				botonPresionado = false;

				//setLed();
				ledsSet(LED_DER_ID, LED_ST_OFF);
				ledsSet(LED_IZQ_ID, LED_ST_OFF);

				/* Set motor defualt value */
				motoresSetVelocidad(MOTOR_DER_ID, 0);
				motoresSetVelocidad(MOTOR_IZQ_ID, 0);
				/* Go to Pre Race Mode */
				_driverMode = PRE_RACE_MODE;
				break;

			case CALIBRATE_SENSOR_MODE:
				/* Calibrate Sensor Mode */

				botonPresionado = false;

				//setLed();
				ledsSet(LED_DER_ID, LED_ST_OFF);
				ledsSet(LED_IZQ_ID, LED_ST_OFF);

				/*Por ahora no tenemos esta funcionalidad, pero no se queda frenadoe en este punto y vuelve a PRE_RACE_MODE */

				/* Set motor defualt value */
				motoresSetVelocidad(MOTOR_DER_ID, 0);
				motoresSetVelocidad(MOTOR_IZQ_ID, 0);
				/* Go to Pre Race Mode */
				_driverMode = PRE_RACE_MODE;

				ledsBlink(LED_DER_ID, 1);
				ledsSet(LED_IZQ_ID, LED_ST_OFF);

				break;

			case PRE_RACE_MODE:
				/* PreRace Mode */

				/* Retardo del ciclo */
				sensoresGetValActual();

				/* Led status Race Mode */

#ifdef CALIBRACION_PID
				/* Wait for Button To change KP of PID upward */
				if(botonesGetEvento(BOTON_IZQ_ID) == BOTONES_EV_PRESIONADO)
				{
					/* Cambiar de KP */
					if(PidData.KpActualIndex < (PID_KP_COUNT - 1))
						PidData.KpActualIndex++;

					/* guardarlo para enviar a setpid */
					PidData.KpActual = PidData.Kp[PidData.KpActualIndex];

					sprintf(mensaje, "ind %d %0.3f \n\r", PidData.KpActualIndex, PidData.Kp[PidData.KpActualIndex]);
					debugPrint(mensaje);

					/* indicar el valor*/
					ledsBlink(LED_DER_ID, (int)((PidData.KpActual - (int)(PidData.KpActual)) * 10));
					ledsBlink(LED_IZQ_ID, (int)PidData.KpActual);
					/* Led Power Value */
				}
				/* Wait for Button To change KP of PID downward*/
				if(botonesGetEvento(BOTON_DER_ID) == BOTONES_EV_PRESIONADO)
				{
					/* Cambiar de KP */
					if(PidData.KpActualIndex > 0)
						PidData.KpActualIndex--;

					/* guardarlo para enviar a setpid */
					PidData.KpActual = PidData.Kp[PidData.KpActualIndex];

					sprintf(mensaje, "ind %d %0.3f \n\r", PidData.KpActualIndex, PidData.Kp[PidData.KpActualIndex]);
					debugPrint(mensaje);

					/* indicar el valor*/
					ledsBlink(LED_DER_ID, (int)((PidData.KpActual - (int)(PidData.KpActual)) * 10));
					ledsBlink(LED_IZQ_ID, (int)PidData.KpActual);
					/* Led Power Value */
				}

#else
				/* Wait for Button To change Max Power to Motor */
				if(botonesGetEvento(BOTON_IZQ_ID) == BOTONES_EV_PRESIONADO)
				{
					/* take new power value */
					speedRace = changeMaxSpeed(speedRace);

					/* Led Power Value */
					ledsBlink(LED_IZQ_ID, 1);
				}
#endif
				/* Wait for Button ON to RUN */
				if((botonesGetEstado(BOTON_CENTRAL_ID) == BOTON_ST_PRESIONADO) || botonPresionado) /* Is button change mode on? */
				{
					botonPresionado = true;

					//Example pidSet (pidSensores, 0.1, 100, -100, 0.7, 0.02, 0.1, 0, 0);

					/* This break is to be ready for Run, while runButton is ON the robot is waiting for run */
					pidSet( pidSensores, CORE_TIEMPO_CICLO, MAX_SENSOR_VALUE, MIN_SENSOR_VALUE, PidData.KpActual, 0.00001, 0.0001, 0, 0);

					/* default Value */
					correction = 0;

					/* Set led */
					ledsSet(LED_DER_ID, LED_ST_OFF);
					ledsSet(LED_IZQ_ID, LED_ST_ON);

					if(botonesGetEstado(BOTON_CENTRAL_ID) == BOTON_ST_NO_PRESIONADO) /* Is button change mode on? */
					{
						/* boton fue soltado */
						botonPresionado = false;

						/* Set motor Race max value */
						motoresSetVelocidad(MOTOR_DER_ID, speedRace);
						motoresSetVelocidad(MOTOR_IZQ_ID, speedRace);

						/* Set led */
						ledsSet(LED_DER_ID, LED_ST_ON);
						ledsSet(LED_IZQ_ID, LED_ST_ON);

						/* Go to Race */
						_driverMode = RACE_MODE;
					}
				}
				break;

			case RACE_MODE:
				/* Race Mode */

				//debugPrint("RACE Mode \n\r ");

				/* Get sensor error */
				SensorValorActual = sensoresGetValActual();

				/* Calculate error PID */
				correction = pidCalculate(pidSensores, SENSOR_SET_POINT_VALUE, (double) SensorValorActual);

				/* Calculate velocity of a motors and set values */
				calcVel(speedRace, correction);

				break;

			default:
				/* Unknown mode */

				_driverMode = ERROR_MODE;
				break;
		};

		/* Change Mode Button */
		if(botonesGetEvento(BOTON_DER_ID) == BOTONES_EV_PRESIONADO)/* Is button change mode on? */
		{
			/* Por ahora solo vuelve a estado preRace, debido a que no est'an desarrollados los otros modos */
			//_driverMode++;
			//if (LAST_MODE <= _driverMode)
			//{
			//	_driverMode = CALIBRATE_SENSOR_MODE;
			//}

			/* Set motor Race max value */
			motoresSetVelocidad(MOTOR_DER_ID, 0);
			motoresSetVelocidad(MOTOR_IZQ_ID, 0);
			//motoresSetPWM(MOTOR_DER_ID, 0);
			//motoresSetPWM(MOTOR_IZQ_ID, 0);
			speedRace = RACE_SPEED_SET_VALUE;
			botonPresionado = false;
			_driverMode = PRE_RACE_MODE;
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  htim3.Init.Prescaler = 1000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65000;
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
  sConfigIC.ICFilter = 14;
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
  htim4.Init.Prescaler = 1000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65000;
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
  sConfigIC.ICFilter = 14;
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
void calcVel(int velRace, double newCorrection)
{

	/* newCorrection es bueno que sea un double asi se puede multiplicar o dividir por un fraccional */
	//sprintf(mensaje, "c %f \n\r", newCorrection);
	//debugPrint(mensaje);
	    //newCorrection = newCorrection;


	    double auxNew = newCorrection;

	    if(0 > auxNew)
	        auxNew = auxNew * (-1);


	    /*if( auxNew > 300)
	    	velRace = velRace /3;
	    else if( auxNew > 500)
	    	velRace = velRace /7;*/
	    else if( auxNew > 700)
	    	velRace = velRace /2;


    int velMotorIzq = velRace;
    int velMotorDer = velRace;


    if(newCorrection > 0)
    {
        velMotorIzq = velMotorIzq - (int) newCorrection;	// *1.8
        velMotorDer = velMotorDer + (int) newCorrection;	// /3
        if (velMotorIzq < MIN_SPEED_VALUE)
        {
            velMotorIzq = MIN_SPEED_VALUE;
            motorSetFreno(MOTOR_IZQ_ID, MOTOR_FRENO_FRENAR);
        }
        if (velMotorDer > MAX_SPEED_VALUE)
        {
            velMotorDer = MAX_SPEED_VALUE;
        }

    }
    else if (newCorrection < 0)
    {
        velMotorDer = velMotorDer + (int) newCorrection;	// *1.8
        velMotorIzq = velMotorIzq - (int) newCorrection; 	// /3
        if (velMotorDer < MIN_SPEED_VALUE)
        {
            velMotorDer = MIN_SPEED_VALUE;
            motorSetFreno(MOTOR_DER_ID, MOTOR_FRENO_FRENAR);
        }
        if (velMotorIzq > MAX_SPEED_VALUE)
        {
        	velMotorIzq = MAX_SPEED_VALUE;
        }
    }

    /* Set motor speed */

    //sprintf(mensaje, "vD %d, vI %d \r\n", velMotorDer, velMotorIzq);
    //debugPrint(mensaje);

    //motoresSetPWM(MOTOR_DER_ID, (uint8_t)(velMotorDer * MOTOR_PWM_MAX_STEPS /MOTOR_SPEED_MAX));
	//motoresSetPWM(MOTOR_IZQ_ID, (uint8_t)(velMotorIzq * MOTOR_PWM_MAX_STEPS /MOTOR_SPEED_MAX));
    motoresSetVelocidad(MOTOR_DER_ID, velMotorDer);
	motoresSetVelocidad(MOTOR_IZQ_ID, velMotorIzq);

}

int changeMaxSpeed(int currSpeedMax)
{
	 int newSpeed = currSpeedMax;
	    /* increase speed on 10% */
	    newSpeed += 10;
	    /* Check overflow */
	    if(newSpeed > MAX_SPEED_VALUE)
	    {
	    	newSpeed = MAX_SPEED_VALUE;
	    }
	    return newSpeed;
}

/*** MOTORES FUNCTION DEF BEGIN ***/
/**
 * @brief Esta función inicializa la estructura de los motores
 * @param motor_ID: MOTOR_IZQ_ID ó MOTOR_DER_ID
 * @param potencia: Potencia a inyectar al motor, desde 0 a MOTOR_POT_MAX
 */
enumMotorError motoresInit (void)
{

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

	MotoresData.frenoPinPort[MOTOR_DER_ID] = Freno_DER_GPIO_Port;
	MotoresData.frenoPin[MOTOR_DER_ID] = Freno_DER_Pin;
	MotoresData.frenoPinPort[MOTOR_IZQ_ID] = Freno_IZQ_GPIO_Port;
	MotoresData.frenoPin[MOTOR_IZQ_ID] = Freno_IZQ_Pin;

	if (true != pidGetPID(&MotoresData.pidMotor[MOTOR_DER_ID]))
		return -1;
	pidSet( MotoresData.pidMotor[MOTOR_DER_ID], CORE_TIEMPO_CICLO, MOTOR_SPEED_MAX, MOTOR_SPEED_MIN, MOTOR_PID_KP_DER, 0, 0, 0, 0);

	if (true != pidGetPID(&MotoresData.pidMotor[MOTOR_IZQ_ID]))
		return -1;
	pidSet( MotoresData.pidMotor[MOTOR_IZQ_ID], CORE_TIEMPO_CICLO, MOTOR_SPEED_MAX, MOTOR_SPEED_MIN, MOTOR_PID_KP_IZQ, 0, 0, 0, 0);

	MotoresData.PWM_Actual[MOTOR_IZQ_ID] = 0;
	MotoresData.PWM_Actual[MOTOR_DER_ID] = 0;

	MotoresData.velocidadSetPoint[MOTOR_IZQ_ID] = 0;
	MotoresData.velocidadSetPoint[MOTOR_DER_ID] = 0;

	return MOTOR_ERR_SUCCESS;
}

/**
 * @brief Esta función envía potencia a los motores en PWM
 * @param motor_ID: MOTOR_IZQ_ID ó MOTOR_DER_ID
 * @param porcentaje: Valor en porcentaje del ancho del pulso del PWM
 */
enumMotorError motoresSetPWM(enumMotorID motor_ID, uint8_t porcentaje)
{
	//char mensaje[100];
	//sprintf(mensaje, "ID %d, P %d \n\r", motor_ID, porcentaje);
	//debugPrint(mensaje);

	//Verifico ID del motor

	if(motor_ID != MOTOR_IZQ_ID && motor_ID != MOTOR_DER_ID)
	{
		return MOTOR_ERR_ID_INVALID;
	}

	//Verifico estado del freno
	if (porcentaje)
	{
		if(MotoresData.freno_state[motor_ID] == MOTOR_FRENO_ST_FRENADO)
		{
			//Liberar freno si está seteado
			HAL_GPIO_WritePin(MotoresData.frenoPinPort[motor_ID], MotoresData.frenoPin[motor_ID], MOTOR_FRENO_OUT_PIN_LIBERAR);
		}
	}
	//Limitar porcentaje de PWM
	if (porcentaje > MOTOR_POT_MAX)
		porcentaje = MOTOR_POT_MAX;

	//Cambiar PWM del motor indicado a la potencia especificada
	if(motor_ID == MOTOR_DER_ID)
	{
		//sprintf(mensaje, "Estoy setenado Potencia \n");
		//debugPrint(mensaje);

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, porcentaje * MOTOR_PWM_STEPS / MOTOR_POT_MAX);
	}
	else
	{

		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, porcentaje * MOTOR_PWM_STEPS / MOTOR_POT_MAX);
	}

	//Guardar valor seteado en la estructura
	MotoresData.PWM_Actual[motor_ID] = porcentaje;

	return MOTOR_ERR_SUCCESS;
}

uint8_t motorGetPWM(enumMotorID motor_ID)
{
	return MotoresData.PWM_Actual[motor_ID];
}


/**
 * @brief Esta función frena el motor especificado
 * @param motor_ID: MOTOR_IZQ_ID ó MOTOR_DER_ID
 * @param freno_st: Estado en que se quiere poner el freno (enumMotorFreno)
 */
enumMotorError motorSetFreno(enumMotorID motor_ID, enumMotorFreno freno_st)
{
	//Verificar ID del motor
	if(motor_ID != MOTOR_IZQ_ID && motor_ID != MOTOR_DER_ID)
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
			motoresSetVelocidad(MOTOR_DER_ID, 0);
		}
		else
		{
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
			motoresSetVelocidad(MOTOR_IZQ_ID, 0);
		}

		//Efectuar frenado
		HAL_GPIO_WritePin(MotoresData.frenoPinPort[motor_ID], MotoresData.frenoPin[motor_ID], MOTOR_FRENO_OUT_PIN_FRENAR);
		MotoresData.freno_state[motor_ID] = MOTOR_FRENO_ST_FRENADO;
	}
	else if (freno_st == MOTOR_FRENO_LIBERAR)
	{
		//Liberar freno
		HAL_GPIO_WritePin(MotoresData.frenoPinPort[motor_ID], MotoresData.frenoPin[motor_ID], MOTOR_FRENO_OUT_PIN_LIBERAR);
		MotoresData.freno_state[motor_ID] = MOTOR_FRENO_ST_LIBERADO;
	}
	else
	{
		//Liberar freno
		HAL_GPIO_WritePin(MotoresData.frenoPinPort[motor_ID], MotoresData.frenoPin[motor_ID], MOTOR_FRENO_OUT_PIN_LIBERAR);
		MotoresData.freno_state[motor_ID] = MOTOR_FRENO_ST_LIBERADO;
		return MOTOR_ERR_FRENO_ST_NOTVALID;
	}

	return MOTOR_ERR_SUCCESS;
}

/**
 * @brief Esta función mantiene valores de los motores, principalmente regula el PWM para
 * mantener la velocidad seteda
 * @param none
 */
void motoresService (void)
{
	double pwm;
	double correction = 0;
	enumMotorID motor_ID;
	enumRuedasID rueda_ID;


	int16_t	PWM;

	/* CORREGIR MOTORES */
	for (motor_ID = MOTOR_IZQ_ID; motor_ID < MOTORES_COUNT_ID; motor_ID++)
	{
		if(motor_ID == MOTOR_IZQ_ID)
			rueda_ID = RUEDA_IZQ_ID;
		else
			rueda_ID = RUEDA_DER_ID;

		if(MotoresData.velocidadSetPoint[motor_ID] > 0)
		{
			pwm = (double)(MotoresData.velocidadSetPoint[motor_ID]/(RuedasData.Circunferencia/1000)) * 2.941176 + 35;
		}
		else
		{
			pwm = 0;
		}

		//sprintf(mensaje, "pwm %f spv %d \n\r", pwm, MotoresData.velocidadSetPoint[motor_ID]);
		//debugPrint(mensaje);

		motoresSetPWM(motor_ID, pwm);

		#if 0
		if((RuedasData.Velocidad[rueda_ID] != RuedasData.VelocidadAnt[rueda_ID]) || (RuedasData.Velocidad[rueda_ID] != 0))
		{
			//DEBUG
			ledsSet(LED_DER_ID, LED_ST_ON);
			//END DEBUG

			RuedasData.VelocidadAnt[rueda_ID] = RuedasData.Velocidad[rueda_ID];

			pidSet( MotoresData.pidMotor[motor_ID], RuedasData.dt[rueda_ID], MOTOR_SPEED_MAX, -MOTOR_SPEED_MAX,
					MOTOR_PID_KP_DER, 0, 0,
					MotoresData.pidMotor[motor_ID]->pre_error, MotoresData.pidMotor[motor_ID]->integral);
			correction = pidCalculate(MotoresData.pidMotor[motor_ID], MotoresData.velocidadSetPoint[motor_ID], (double) ruedasGetVelocidad(rueda_ID));



			//Llevar a escala de PWM la corrección hecha en velocidad
			correction = correction * MOTOR_PWM_MAX_STEPS / MOTOR_SPEED_MAX;



			//PWM actual
			PWM = motorGetPWM(motor_ID);

			//Corregir PWM
			if((PWM + correction) < 0)
			{
				PWM = 0;
			}
			else if((PWM + correction) > MOTOR_PWM_MAX_STEPS)
			{
				PWM = MOTOR_PWM_MAX_STEPS;
			}
			else
			{
				PWM += correction;
			}

/*
			if(motor_ID == MOTOR_DER_ID)
			{
				sprintf(mensaje, "pw %d setp %d, getv %d, va %d,  corr %d \n\r", PWM, MotoresData.velocidadSetPoint[motor_ID],  ruedasGetVelocidad(rueda_ID), RuedasData.VelocidadAnt[rueda_ID], (uint16_t)correction);
				debugPrint(mensaje);
			}
*/
			//Setear nuevo valor de PWM
			motoresSetPWM(motor_ID, (uint8_t)PWM);
			//motoresSetPWM(MOTOR_DER_ID, 40);

		}
#endif
	}

}

enumMotorError motoresSetVelocidad(enumMotorID motor_ID, uint16_t velocidad)
{
	if(motor_ID > MOTOR_ID_NOVALID && motor_ID < MOTORES_COUNT_ID)
	{
		MotoresData.velocidadSetPoint[motor_ID] = velocidad;
		return MOTOR_ERR_SUCCESS;
	}
	else
	{
		return MOTOR_ERR_ID_INVALID;
	}
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
	//char mensaje[200];
	uint8_t i;
	int16_t valActual = 0;
	uint16_t maximo = 0, minimo = 0, promedio = 0;
	long int sumaproductovalores = 0, sumavalores = 0, centro = 0;

	//Inicio de toma de valores de los ADC

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

	//Cálculo máximo y mínimo y promedio

	//sprintf(mensaje, "Sensores fin conversion\n\r");
	//debugPrint(mensaje);

	minimo = maximo = SensoresData.ADC_raw[SENS_IZQ_3];
	sumavalores = 0;

	for (i = SENS_IZQ_3; i <= SENS_DER_3; i++ )
	{
		//sprintf(mensaje, "%04d ", SensoresData.ADC_raw[i]);
		//debugPrint(mensaje);

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
	//promedio = sumavalores / (i - SENS_IZQ_3);
	//sprintf(mensaje, "mi=%d mx=%d fil ", minimo, maximo);
	//debugPrint(mensaje);

	//LOGICA AGREGADA PARA MEJORAR PERFORMANCE
	//Se agregan 4 sensores virtuales más, simulando los que no están a la izquierda y a la derecha,
	//quedarán indicando el máximo valor, tanto a izquierda como a derecha, si pierden la línea
	//Sólo se encenderán si la línea no está en posiciones centrales (SENSORES_POS_LINEA_CENTRO).
	//Sensor Virtual IZQ 5

/*
	if(SensoresData.maximo_historico >= SensoresData.ADC_raw[SENS_IZQ_3])
		SensoresData.ADC_raw[SENS_V_IZQ_5] = (SensoresData.pos_linea == SENSORES_POS_LINEA_IZQ)?(SensoresData.maximo_historico - SensoresData.ADC_raw[SENS_IZQ_3] + minimo):minimo;
	else
		SensoresData.ADC_raw[SENS_V_IZQ_5] = (SensoresData.pos_linea == SENSORES_POS_LINEA_IZQ)?(-SensoresData.maximo_historico + SensoresData.ADC_raw[SENS_IZQ_3] + minimo):minimo;
	sumavalores = sumavalores + SensoresData.ADC_raw[SENS_V_IZQ_5];
	//Sensor Virtual IZQ 4
	if(SensoresData.maximo_historico >= SensoresData.ADC_raw[SENS_IZQ_2])
		SensoresData.ADC_raw[SENS_V_IZQ_4] = (SensoresData.pos_linea == SENSORES_POS_LINEA_IZQ)?(SensoresData.maximo_historico - SensoresData.ADC_raw[SENS_IZQ_2] + minimo):minimo;
	else
		SensoresData.ADC_raw[SENS_V_IZQ_4] = (SensoresData.pos_linea == SENSORES_POS_LINEA_IZQ)?(-SensoresData.maximo_historico + SensoresData.ADC_raw[SENS_IZQ_2] + minimo):minimo;
	sumavalores = sumavalores + SensoresData.ADC_raw[SENS_V_IZQ_4];

	//Sensor Virtual DER 4
	if(SensoresData.maximo_historico >= SensoresData.ADC_raw[SENS_DER_2])
		SensoresData.ADC_raw[SENS_V_DER_4] = (SensoresData.pos_linea == SENSORES_POS_LINEA_DER)?(SensoresData.maximo_historico - SensoresData.ADC_raw[SENS_DER_2] + minimo):minimo;
	else
		SensoresData.ADC_raw[SENS_V_DER_4] = (SensoresData.pos_linea == SENSORES_POS_LINEA_DER)?(-SensoresData.maximo_historico + SensoresData.ADC_raw[SENS_DER_2] + minimo):minimo;
	sumavalores = sumavalores + SensoresData.ADC_raw[SENS_V_DER_4];

	//Sensor Virtual DER 5
	if(SensoresData.maximo_historico >= SensoresData.ADC_raw[SENS_DER_3])
		SensoresData.ADC_raw[SENS_V_DER_5] = (SensoresData.pos_linea == SENSORES_POS_LINEA_DER)?(SensoresData.maximo_historico - SensoresData.ADC_raw[SENS_DER_3] + minimo):minimo;
	else
		SensoresData.ADC_raw[SENS_V_DER_5] = (SensoresData.pos_linea == SENSORES_POS_LINEA_DER)?(-SensoresData.maximo_historico + SensoresData.ADC_raw[SENS_DER_3] + minimo):minimo;
	sumavalores = sumavalores + SensoresData.ADC_raw[SENS_V_DER_5];
*/

	if(SensoresData.maximo_historico >= SensoresData.ADC_raw[SENS_IZQ_3])
		SensoresData.ADC_raw[SENS_V_IZQ_5] = (SensoresData.pos_linea == SENSORES_POS_LINEA_IZQ)?SensoresData.maximo_historico:minimo;
	else
		SensoresData.ADC_raw[SENS_V_IZQ_5] = (SensoresData.pos_linea == SENSORES_POS_LINEA_IZQ)?SensoresData.maximo_historico:minimo;
	//sumavalores = sumavalores + SensoresData.ADC_raw[SENS_V_IZQ_5];
	//Sensor Virtual IZQ 4
	if(SensoresData.maximo_historico >= SensoresData.ADC_raw[SENS_IZQ_2])
		SensoresData.ADC_raw[SENS_V_IZQ_4] = (SensoresData.pos_linea == SENSORES_POS_LINEA_IZQ)?SensoresData.maximo_historico:minimo;
	else
		SensoresData.ADC_raw[SENS_V_IZQ_4] = (SensoresData.pos_linea == SENSORES_POS_LINEA_IZQ)?SensoresData.maximo_historico:minimo;
	sumavalores = sumavalores + SensoresData.ADC_raw[SENS_V_IZQ_4];

	//Sensor Virtual DER 4
	if(SensoresData.maximo_historico >= SensoresData.ADC_raw[SENS_DER_2])
		SensoresData.ADC_raw[SENS_V_DER_4] = (SensoresData.pos_linea == SENSORES_POS_LINEA_DER)?SensoresData.maximo_historico:minimo;
	else
		SensoresData.ADC_raw[SENS_V_DER_4] = (SensoresData.pos_linea == SENSORES_POS_LINEA_DER)?SensoresData.maximo_historico:minimo;
	sumavalores = sumavalores + SensoresData.ADC_raw[SENS_V_DER_4];

	//Sensor Virtual DER 5
	if(SensoresData.maximo_historico >= SensoresData.ADC_raw[SENS_DER_3])
		SensoresData.ADC_raw[SENS_V_DER_5] = (SensoresData.pos_linea == SENSORES_POS_LINEA_DER)?SensoresData.maximo_historico:minimo;
	else
		SensoresData.ADC_raw[SENS_V_DER_5] = (SensoresData.pos_linea == SENSORES_POS_LINEA_DER)?SensoresData.maximo_historico:minimo;
	//sumavalores = sumavalores + SensoresData.ADC_raw[SENS_V_DER_5];

	//Cálculo de promedio
	promedio = sumavalores / (SENS_V_DER_4 -1 -SENS_V_IZQ_4);

	//sprintf(mensaje, "\r");
	//debugPrint(mensaje);

	//Cálculo de Valor actual
	//Se calcula como un centro de gravedad de la señal de cada sensor

	//1ro Calculo suma de productos de cada valor del sensor por la distancia al centro de los sensores
	sumaproductovalores = 0;
	sumavalores = 0;
	//for(uint8_t i = SENS_IZQ_3; i <= SENS_DER_3; i++)
	for(uint8_t i = SENS_V_IZQ_4; i <= SENS_V_DER_4; i++)
	{
		SensoresData.ADC_fil[i] = (SensoresData.ADC_raw[i] > promedio)? (uint16_t)((double)(SensoresData.ADC_raw[i] - promedio) * 1) : 0;
		sumaproductovalores += SensoresData.ADC_fil[i] * SensoresData.posicion_x[i];
		sumavalores += SensoresData.ADC_fil[i];
		//sprintf(mensaje, "%d ", SensoresData.ADC_fil[i]);
		//debugPrint(mensaje);
	}
	//Valor del centro de gravedad de los sensores
	centro = sumaproductovalores / sumavalores;

	//Valor actual sin escalar
	valActual = (int16_t)centro;

	//sprintf(mensaje, " %ld %d ", centro, valActual);
	//debugPrint(mensaje);

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

/*
	sprintf(mensaje, "%05d %05d %05d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %c\r\n",
												valActual,
												maximo,
												minimo,
												SensoresData.maximo_historico,
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
*/
	//Escalado de valor actual
	valActual = valActual * MAX_SENSOR_VALUE / abs(SensoresData.posicion_x[SENS_V_DER_5]);

	//sprintf(mensaje, "%05d \r\n", valActual);
	//debugPrint(mensaje);

	return valActual;
}

/*** SENSORES FUNCTION DEF END ***/

/*** LEDS FUNCTION DEF BEGIN ***/
enumLedsError ledsInit(void)
{
	LedsData.Port[LED_DER_ID] = LED_2_GPIO_Port;
	LedsData.Pin[LED_DER_ID] = LED_2_Pin;
	LedsData.Port[LED_IZQ_ID] = LED_1_GPIO_Port;
	LedsData.Pin[LED_IZQ_ID] = LED_1_Pin;
	LedsData.Port[LED_PLAQUITA_ID] = LED_PCB_GPIO_Port;
	LedsData.Pin[LED_PLAQUITA_ID] = LED_PCB_Pin;
	return LED_ERR_SUCCESS;
}

enumLedsError ledsSet(enumLedsID led_ID, enumLedsStates led_st)
{
	if(led_ID > LEDS_NOVALID_ID && led_ID < LEDS_COUNT_ID)
	{
		if(led_st == LED_ST_ON)
		{
			HAL_GPIO_WritePin(LedsData.Port[led_ID], LedsData.Pin[led_ID], GPIO_PIN_RESET);
			LedsData.Estado[led_ID] = LED_ST_ON;
		}
		else if (led_st == LED_ST_OFF)
		{
			HAL_GPIO_WritePin(LedsData.Port[led_ID], LedsData.Pin[led_ID], GPIO_PIN_SET);
			LedsData.Estado[led_ID] = LED_ST_OFF;
		}
		else
		{
			return LED_ERR_ST_NO_VALID;
		}
	}
	return LED_ERR_SUCCESS;
}

enumLedsStates ledsGetSt(enumLedsID led_ID)
{
	if(led_ID > LEDS_NOVALID_ID && led_ID < LEDS_COUNT_ID)
	{
		return LedsData.Estado[led_ID];
	}
	return LED_ST_NOVALID;
}

enumLedsError ledsBlink(enumLedsID led_ID, uint8_t cantidad_blk)
{
	if(led_ID > LEDS_NOVALID_ID && led_ID < LEDS_COUNT_ID)
	{
		LedsData.Estado[led_ID] = LED_ST_BLINK;
		LedsData.RestantesBlinks[led_ID] = cantidad_blk;
		LedsData.TimeOutBlink[led_ID] = LEDS_TIMEOUT_BLINK_CICLOS;
		return LED_ERR_SUCCESS;
	}
	else
	{
		return LED_ERR_NO_VALID_ID;
	}
}

void ledsService(void)
{
	//Ejecutar blink
	for(uint8_t led_ID = LEDS_NOVALID_ID + 1; led_ID < LEDS_COUNT_ID; led_ID++)
	{
		if(LedsData.Estado[led_ID] == LED_ST_BLINK)
		{
			if(LedsData.RestantesBlinks[led_ID] > 0)
			{
				if(LedsData.TimeOutBlink[led_ID] > LEDS_TIMEOUT_BLINK_CICLOS/2)
				{
					HAL_GPIO_WritePin(LedsData.Port[led_ID], LedsData.Pin[led_ID], GPIO_PIN_RESET);
					LedsData.TimeOutBlink[led_ID]--;
				}
				else if (LedsData.TimeOutBlink[led_ID] > 0)
				{
					HAL_GPIO_WritePin(LedsData.Port[led_ID], LedsData.Pin[led_ID], GPIO_PIN_SET);
					LedsData.TimeOutBlink[led_ID]--;
				}
				else
				{
					LedsData.RestantesBlinks[led_ID]--;
					LedsData.TimeOutBlink[led_ID] = LEDS_TIMEOUT_BLINK_CICLOS;
				}
			}
			else
			{
				ledsSet(led_ID, LED_ST_OFF);
			}
		}
	}


	//TEST: Por ahora es solo apagar los leds por cuestiones de testing
	//ledsSet(LED_DER_ID, LED_ST_OFF);
	//ledsSet(LED_IZQ_ID, LED_ST_OFF);
}
/*** LEDS FUNCTION DEF END ***/

/*** BOTONES FUNCTION DEF BEGIN ***/
enumBotonesError botonesInit(void)
{
	BotonesData.Port[BOTON_IZQ_ID] = BOTON_1_IN_GPIO_Port;
	BotonesData.Pin[BOTON_IZQ_ID] = BOTON_1_IN_Pin;
	BotonesData.Tipo[BOTON_IZQ_ID] = BOTONES_TIPO_NA;
	BotonesData.Port[BOTON_DER_ID] = BOTON_2_IN_GPIO_Port;
	BotonesData.Pin[BOTON_DER_ID] = BOTON_2_IN_Pin;
	BotonesData.Tipo[BOTON_DER_ID] = BOTONES_TIPO_NA;
	BotonesData.Port[BOTON_CENTRAL_ID] = PUL_ARRANQUE_GPIO_Port;
	BotonesData.Pin[BOTON_CENTRAL_ID] = PUL_ARRANQUE_Pin;
	BotonesData.Tipo[BOTON_CENTRAL_ID] = BOTONES_TIPO_NC;

	for(enumBotonesID botones_ID = BOTONES_NOVALID_ID + 1; botones_ID < BOTONES_COUNT_ID; botones_ID++)
	{
		BotonesData.RetardoAPresionado[botones_ID] = 0;
	}

	return BOTONES_ERR_SUCCESS;
}

enumBotonesStates botonesGetEstado(enumBotonesID boton_ID)
{
	if(boton_ID > BOTONES_NOVALID_ID && boton_ID < BOTONES_COUNT_ID)
	{
		return (BotonesData.Estado[boton_ID]);
	}
	else
	{
		return BOTONES_ST_ERROR_ID;
	}
}

enumBotonesEventos botonesGetEvento(enumBotonesID boton_ID)
{
	if(boton_ID > BOTONES_NOVALID_ID && boton_ID < BOTONES_COUNT_ID)
	{
		return (BotonesData.Evento[boton_ID]);
	}
	else
	{
		return BOTONES_EV_ERROR_ID;
	}
}

void botonesService(void)
{
	for (enumBotonesID boton_ID = BOTONES_NOVALID_ID + 1; boton_ID < BOTONES_COUNT_ID; boton_ID++)
	{

		//Verifico si se presionó algún botón
		if(((HAL_GPIO_ReadPin(BotonesData.Port[boton_ID], BotonesData.Pin[boton_ID]) == GPIO_PIN_SET) && (BotonesData.Tipo[boton_ID] == BOTONES_TIPO_NC))
				||((HAL_GPIO_ReadPin(BotonesData.Port[boton_ID], BotonesData.Pin[boton_ID]) == GPIO_PIN_RESET) && (BotonesData.Tipo[boton_ID] == BOTONES_TIPO_NA)))
		{
			if(BotonesData.RetardoAPresionado[boton_ID] < BOTONES_RETARDO_A_PRESIONADO_CICLOS)
			{
				BotonesData.RetardoAPresionado[boton_ID]++;
			}
			else
			{
				BotonesData.RetardoAPresionado[boton_ID] = BOTONES_RETARDO_A_PRESIONADO_CICLOS;
			}
		}
		else
		{
			if(BotonesData.RetardoAPresionado[boton_ID] > 0)
			{
				BotonesData.RetardoAPresionado[boton_ID]--;
			}
			else
			{
				BotonesData.RetardoAPresionado[boton_ID] = 0;
			}
		}

		//Actualizo el estado del botón si se cumplió el tiempo de retardo
		if(BotonesData.RetardoAPresionado[boton_ID] == BOTONES_RETARDO_A_PRESIONADO_CICLOS)
		{
			//Cambio estado
			BotonesData.Estado[boton_ID] = BOTON_ST_PRESIONADO;
			//Verifico evento
			if(BotonesData.EstadoAnt[boton_ID] == BOTON_ST_NO_PRESIONADO)
			{
				BotonesData.Evento[boton_ID] = BOTONES_EV_PRESIONADO;
			}
			else
			{
				BotonesData.Evento[boton_ID] = BOTONES_EV_MANTENIDO_PRESIONADO;
			}
		}
		else if(BotonesData.RetardoAPresionado[boton_ID] == 0)
		{
			//Cambio estado
			BotonesData.Estado[boton_ID] = BOTON_ST_NO_PRESIONADO;
			//Verifico evento
			if(BotonesData.EstadoAnt[boton_ID] == BOTON_ST_PRESIONADO)
			{
				BotonesData.Evento[boton_ID] = BOTONES_EV_LIBERADO;
			}
			else
			{
				BotonesData.Evento[boton_ID] = BOTONES_EV_SIN_EVENTO;
			}
		}
		else
		{
			//Mantiene estado anterior
			//No hay evento

		}

		//Guardar estado anterior
		BotonesData.EstadoAnt[boton_ID] = BotonesData.Estado[boton_ID];

	}

}
/*** BOTONES FUNCTION DEF END ***/

/*** RUEDA FUNCION DEF INIT ***/

enumRuedasError ruedasInit (void)
{
	RuedasData.Circunferencia = 94248; //Longitud de la circunferencia de la rueda en micrones
	for (enumRuedasID rueda_ID = RUEDAS_NOVALID_ID + 1; rueda_ID < RUEDAS_COUNT_ID ; rueda_ID++)
	{
		RuedasData.Velocidad[rueda_ID] = 0;
		RuedasData.TimeOut[rueda_ID] = 0;
		RuedasData.Odometro[rueda_ID] = 0;
	}

	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
	return RUEDAS_ERR_SUCCESS;
}

uint16_t ruedasGetVelocidad(enumRuedasID rueda_ID)
{
	if(rueda_ID > RUEDAS_NOVALID_ID && rueda_ID < RUEDAS_COUNT_ID)
	{
//		if(RuedasData.Count_Raw[rueda_ID])
//		{
			//RuedasData.Velocidad[rueda_ID] = (double)RuedasData.Circunferencia /4 /((double)RuedasData.Count_Raw[rueda_ID] * 0.00002083333) /1000;
			return RuedasData.Velocidad[rueda_ID];
//		}
//		else
//		{
			//Casos aún no completamente determinados, uno es cuando el robot arranca y nucna giraron las ruedas
//			return 0;
//		}
	}
	else
	{
		return 0;
	}
}

unsigned long int ruedasGetOdometro(enumRuedasID rueda_ID)
{
	if(rueda_ID > RUEDAS_NOVALID_ID && rueda_ID < RUEDAS_COUNT_ID)
	{
		return RuedasData.Odometro[rueda_ID];
	}
	else
	{
		return 0;
	}
}

enumRuedasError ruedaResetOdometro (enumRuedasID rueda_ID)
{
	if(rueda_ID > RUEDAS_NOVALID_ID && rueda_ID < RUEDAS_COUNT_ID)
	{
		RuedasData.Odometro[rueda_ID] = 0;
		return RUEDAS_ERR_SUCCESS;
	}
	else
	{
		return RUEDAS_ERR_ID_NOT_VALID;

	}
}

void ruedasService(void)
{
	enumRuedasID rueda_ID;
	// Contabilizar tiempo hasta la próxima interrupción
	for(rueda_ID = RUEDAS_NOVALID_ID + 1; rueda_ID < RUEDAS_COUNT_ID; rueda_ID++)
	{
		if(rueda_ID == RUEDA_DER_ID)
		{
			//ledsSet(LED_DER_ID, LED_ST_ON);
			//sprintf(mensaje," TO %ld ", (uint32_t)RuedasData.TimeOut[rueda_ID]);
			//debugPrint(mensaje);
		}

		RuedasData.TimeOut[rueda_ID]++;
		if(RuedasData.TimeOut[rueda_ID] > RUEDAS_TIMEOUT_CICLOS_0_VEL)
		{
			//debugPrint("Vel 0");


			RuedasData.Velocidad[rueda_ID] = 0; //Llevar a cero la velocidad porque no han entrado interrupciones en RUEDAS_TIMEOUT_CICLOS_0_VEL * CORE_TIEMPO_CICLO
			RuedasData.TimeOut[rueda_ID] = 0; //Espero Otro TimeOut
		}
	}

}


/*** RUEDA FUNCTION DEF END ***/

/*** PID FUNCION DEF BEGIN ***/
void pidTestInit(void)
{
	//inicializar array de kps a probar
	for(uint16_t pid_n = 0; pid_n < PID_KP_COUNT; pid_n++)
	{
		//if(pid_n < 20)
		{
			PidData.Kp[pid_n] = 0.6 + (double)pid_n * 0.1;
		}
		/*
		else
		{
			PidData.Kp[pid_n] = (double)pid_n - 19 ;
		}*/
	}
	PidData.KpActual = 0.1;
	PidData.KpActualIndex = 0;

}
/*** PID FUNCTION DEF END ***/

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

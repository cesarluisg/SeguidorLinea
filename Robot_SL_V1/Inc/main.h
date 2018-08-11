/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

enum RaceMode
{
	ERROR_MODE = 0,
	CALIBRATE_SENSOR_MODE,
//	PIT_STOP_TEST_MODE,
//	DRIVE_TEST_MODE,
	PRE_RACE_MODE,
	RACE_MODE,
	LAST_MODE
};

enum genError
{
	ERROR_SUCCESS = 0,
	ERROR_FAIL,
	ERROR_FATAL,
	ERROR_HARD,
	ERROR_PID
};


/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define MOTOR_PWM_STEPS 200

#define LED_PCB_Pin GPIO_PIN_13
#define LED_PCB_GPIO_Port GPIOC
#define SENS_DER_1_Pin GPIO_PIN_0
#define SENS_DER_1_GPIO_Port GPIOA
#define SENS_DER_2_Pin GPIO_PIN_1
#define SENS_DER_2_GPIO_Port GPIOA
#define SENS_DER_3_Pin GPIO_PIN_2
#define SENS_DER_3_GPIO_Port GPIOA
#define SENS_IZQ_1_Pin GPIO_PIN_3
#define SENS_IZQ_1_GPIO_Port GPIOA
#define SENS_IZQ_2_Pin GPIO_PIN_4
#define SENS_IZQ_2_GPIO_Port GPIOA
#define SENS_IZQ_3_Pin GPIO_PIN_5
#define SENS_IZQ_3_GPIO_Port GPIOA
#define SENS_DER_4_Pin GPIO_PIN_6
#define SENS_DER_4_GPIO_Port GPIOA
#define SENS_IZQ_4_Pin GPIO_PIN_7
#define SENS_IZQ_4_GPIO_Port GPIOA
#define PUL_ARRANQUE_Pin GPIO_PIN_0
#define PUL_ARRANQUE_GPIO_Port GPIOB
#define BAT_VOLTS_IN_Pin GPIO_PIN_1
#define BAT_VOLTS_IN_GPIO_Port GPIOB
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define LED_1_Pin GPIO_PIN_11
#define LED_1_GPIO_Port GPIOB
#define LED_2_Pin GPIO_PIN_12
#define LED_2_GPIO_Port GPIOB
#define BOTON_2_IN_Pin GPIO_PIN_14
#define BOTON_2_IN_GPIO_Port GPIOB
#define BOTON_1_IN_Pin GPIO_PIN_15
#define BOTON_1_IN_GPIO_Port GPIOB
#define MOT_DER_OUT_NEG_Pin GPIO_PIN_8
#define MOT_DER_OUT_NEG_GPIO_Port GPIOA
#define MOT_IZQ_OUT_NEG_Pin GPIO_PIN_15
#define MOT_IZQ_OUT_NEG_GPIO_Port GPIOA
#define ENC_RUEDA_IZQ_IN_Pin GPIO_PIN_4
#define ENC_RUEDA_IZQ_IN_GPIO_Port GPIOB
#define BUZZER_OUT_Pin GPIO_PIN_5
#define BUZZER_OUT_GPIO_Port GPIOB
#define ENC_RUEDA_DER_IN_Pin GPIO_PIN_6
#define ENC_RUEDA_DER_IN_GPIO_Port GPIOB
#define Freno_DER_Pin GPIO_PIN_8
#define Freno_DER_GPIO_Port GPIOB
#define Freno_IZQ_Pin GPIO_PIN_9
#define Freno_IZQ_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/*** SENSORES BEGIN Private defines ***/
#define SENSORES_ADC_COUNT				(8)
#define SENSORES_LINEA_COUNT			(10)	/*6 Reales y 4 Virtuales */
#define SENSORES_DIF_RAW_MIN			(1024)	/* Diferencia mínima entre máximo y mínimo para que exista línea*/
#define SENSORES_MAXIMOS_CONTADOS_MAX	(10)	/* número máximo de Maximos para promediar */
/*** SENSORES END Private defines ***/

/*** MOTORES BEGIN Private defines ***/
#define MOTOR_POT_MAX					(100)	/* Valor máximo para la potencia de motores*/
#define MOTOR_FRENO_OUT_PIN_FRENAR		GPIO_PIN_SET
#define MOTOR_FRENO_OUT_PIN_LIBERAR		GPIO_PIN_RESET
/*** MOTORES END Private defines ***/


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

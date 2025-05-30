/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
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
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
unsigned short MAX_DC;

unsigned short INVERTED_MAX_DC;
unsigned short FIRST_HALVED;
unsigned short SECOND_HALVED;
unsigned short THIRD_HALVED;
unsigned short STOPPED_STATE;

unsigned short DC_RIGHT;
unsigned short DC_LEFT;

unsigned short RELATIVE_INIT_U1;
unsigned short PULSE_TIME_U1;
unsigned short DISTANCE_U1;

unsigned short RELATIVE_INIT_U2;
unsigned short PULSE_TIME_U2;
unsigned short DISTANCE_U2;
unsigned short COUNT;

bool CRITICAL_CLOSE;
bool CLOSE;
bool MEDIUM;
bool RELATIVELY_FAR;


enum direction {
	FORWARD, BACKWARDS, STOPPED
};
enum turn_state {
	CLEAR,
	FIRST_OBSTACLE_BACKWARDS,
	FIRST_OBSTACLE_FORWARD,
	SECOND_OBSTACLE_BACKWARDS,
	SECOND_OBSTACLE_FORWARD,
	DOOMED
};
enum turn_state TURN_POSITION = CLEAR;
enum direction MOVEMENT_DIRECTION = STOPPED;

uint8_t RxData[10];
uint8_t temp[2];
int indx = 0;

uint8_t CRITICAL_CLOSE_DISTANCE = 5;
uint8_t CLOSE_DISTANCE;
uint8_t MEDIUM_DISTANCE;
uint8_t RELATIVELY_FAR_DISTANCE;
bool ROTATING;
bool DISTANCE_MESSAGE_FLAG;
/* USER CODE END PV */

/* Private function prototypes --------b---------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TS_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void setup_log_message(char *text) {
	char message[40];
	sprintf(message, "[%lu] %s setup OK \r\n", HAL_GetTick(), text);
	HAL_UART_Transmit(&huart1, message, strlen(message), 10000);
}

void general_log_message(char *text) {
	char message[100];
	sprintf(message, "[%lu] %s \r\n", HAL_GetTick(), text);
	HAL_UART_Transmit(&huart1, message, strlen(message), 10000);
}

void speed_log_message(void) {
	uint32_t wheel1_speed = TIM2->CCR3;
	uint32_t wheel2_speed = TIM2->CCR4;

	char message[70]; // Larger buffer for the more complex message

	sprintf(message,
			"[%lu] Speed change. DC changed to (%lu) in wheel 1 and (%lu) in wheel 2\r\n",
			HAL_GetTick(), wheel1_speed, wheel2_speed);

	HAL_UART_Transmit(&huart1, message, strlen(message), 10000);
}

void RESET_DISTANCES() {
	DISTANCE_U1 = 0;
	DISTANCE_U2 = 0;
}

void SET_SPEED(unsigned short dc_left, unsigned short dc_right) {
	if (MOVEMENT_DIRECTION != STOPPED) {
		DC_RIGHT = dc_right;
		DC_LEFT = dc_left;
		TIM2->CCR3 = DC_LEFT;
		TIM2->CCR4 = DC_RIGHT;
	}
}

void TURN_BACKWARDS(unsigned short dc_left, unsigned short dc_right) {
	MOVEMENT_DIRECTION = BACKWARDS;

	FIRST_HALVED = MAX_DC / 2;
	SECOND_HALVED = (MAX_DC * 3) / 4;
	THIRD_HALVED = (MAX_DC * 4) / 5;
	STOPPED_STATE = MAX_DC;

	GPIOB->BSRR = (1 << 13);
	GPIOB->BSRR = (1 << 12);
	SET_SPEED(dc_left, dc_right);

}

void STOP(bool change_state) {
	if (MOVEMENT_DIRECTION == FORWARD) {
		SET_SPEED(0, 0);
	} else if (MOVEMENT_DIRECTION == BACKWARDS) {
		SET_SPEED(100, 100);
	}
	if (change_state) {
		MOVEMENT_DIRECTION = STOPPED;
		general_log_message("Stopping...");
	}
}

void TURN_FORWARD(unsigned short dc_left, unsigned short dc_right) {
	MOVEMENT_DIRECTION = FORWARD;

	FIRST_HALVED = MAX_DC / 2;
	SECOND_HALVED = MAX_DC / 4;
	THIRD_HALVED = MAX_DC / 5;
	STOPPED_STATE = 0;

	GPIOB->BSRR = (1 << 13) << 16;
	GPIOB->BSRR = (1 << 12) << 16;
	SET_SPEED(dc_left, dc_right);

}

void TURN_DIRECTION() {
	if (MOVEMENT_DIRECTION == FORWARD) {
		TURN_BACKWARDS(INVERTED_MAX_DC, INVERTED_MAX_DC);
		general_log_message("Turn backwards");

	} else if (MOVEMENT_DIRECTION == BACKWARDS) {
		STOP(true);

	} else if (MOVEMENT_DIRECTION == STOPPED) {
		TURN_FORWARD(MAX_DC, MAX_DC);
		general_log_message("Turn forwards");

	}
}
void SETUP_WHEELS() {

	GPIOB->MODER |= (1 << (11 * 2 + 1));
	GPIOB->MODER &= ~(1 << 11 * 2);
	GPIOB->AFR[1] &= ~(0xF << 12);
	GPIOB->AFR[1] |= (1 << 12);

	GPIOB->MODER |= (1 << (10 * 2 + 1));
	GPIOB->MODER &= ~(1 << (10 * 2));
	GPIOB->AFR[1] &= ~(0xF << 8);
	GPIOB->AFR[1] |= (1 << 8);

	GPIOB->MODER &= ~(1 << (13 * 2 + 1));
	GPIOB->MODER |= (1 << 13 * 2);

	GPIOB->MODER &= ~(1 << (12 * 2 + 1));
	GPIOB->MODER |= (1 << 12 * 2);

	setup_log_message("Wheels");

}
void TURN_90_RIGHT(bool backwards) {
	enum direction temp = MOVEMENT_DIRECTION;
	ROTATING = true;
	TIM4->DIER = 0x000A;

	STOP(true);
	general_log_message("Turning 90 degrees to the Right");
	COUNT = 0;
	while (COUNT != 5) {
	}

	if (backwards) {
		TURN_BACKWARDS(100, 0);
	} else {
		TURN_FORWARD(0, 100);
	}

	COUNT = 0;
	while (COUNT != 7) {
	}

	STOP(true);
	COUNT = 0;
	while (COUNT != 5) {
	}
	TIM4->DIER = 0x0008;
	ROTATING = false;
	if (temp == FORWARD) {
		TURN_FORWARD(MAX_DC, MAX_DC);
	} else if (temp == BACKWARDS) {
		TURN_BACKWARDS(INVERTED_MAX_DC, INVERTED_MAX_DC);
	}
	COUNT = 0;
}

void TURN_90_LEFT(bool backwards) {
	enum direction temp = MOVEMENT_DIRECTION;
	ROTATING = true;

	TIM4->DIER = 0x000A;
	STOP(true);
	general_log_message("Turning 90 degrees to the left");
	while (COUNT != 5) {
	}
	if (backwards) {

		TURN_BACKWARDS(0, 100);
	} else {

		TURN_FORWARD(100, 0);
	}
	COUNT = 0;
	while (COUNT != 7) {
	}
	STOP(true);
	COUNT = 0;
	while (COUNT != 5) {
	}
	TIM4->DIER = 0x0008;
	ROTATING = false;
	if (temp == FORWARD) {
		TURN_FORWARD(MAX_DC, MAX_DC);
	} else if (temp == BACKWARDS) {
		TURN_BACKWARDS(INVERTED_MAX_DC, INVERTED_MAX_DC);
	}

	COUNT = 0;
}
void SETUP_PWM() {

	TIM2->CR1 = 0x0080;
	TIM2->CR2 = 0x0000;
	TIM2->SMCR = 0x0000;
	TIM2->PSC = 32000;
	TIM2->CNT = 0;
	TIM2->ARR = 99;
	TIM2->DIER = 0x0000;
	TIM2->CCMR2 = 0x6868;
	TIM2->CCER = 0x1100;
	TIM2->EGR |= 0x0001;
	TIM2->CR1 |= 0x0001;
	TIM2->SR = 0;

	setup_log_message("PWM");

}

void CYCLE_TURN_POSITION() {
	switch (TURN_POSITION) {
	case CLEAR:
		TURN_POSITION = FIRST_OBSTACLE_BACKWARDS;
		break;
	case FIRST_OBSTACLE_BACKWARDS:
		TURN_POSITION = FIRST_OBSTACLE_FORWARD;
		break;
	case FIRST_OBSTACLE_FORWARD:
		TURN_POSITION = SECOND_OBSTACLE_BACKWARDS;
		break;
	case SECOND_OBSTACLE_BACKWARDS:
		TURN_POSITION = SECOND_OBSTACLE_FORWARD;
		break;
	case SECOND_OBSTACLE_FORWARD:
		TURN_POSITION = DOOMED;
		break;
	default:
		break;
	}
}

void BUZZ() {
	if (CRITICAL_CLOSE) {
		TIM4->CCMR2 = 0x0050;
	} else if (CLOSE && !ROTATING) {
		TIM4->CCMR2 = 0x0030;
		SET_SPEED(THIRD_HALVED, THIRD_HALVED);
	}

	else if (MEDIUM && !ROTATING) {
		TIM4->CCMR2 = 0x0030;
		SET_SPEED(SECOND_HALVED, SECOND_HALVED);
	}

	else if (RELATIVELY_FAR && !ROTATING) {
		TIM4->CCMR2 = 0x0040;

		SET_SPEED(FIRST_HALVED, FIRST_HALVED);
	}

	else {
		if (!ROTATING) {
			TIM4->CCMR2 = 0x0040;
			if (MOVEMENT_DIRECTION == FORWARD) {
				SET_SPEED(MAX_DC, MAX_DC);

			} else if (MOVEMENT_DIRECTION == BACKWARDS) {
				SET_SPEED(INVERTED_MAX_DC, INVERTED_MAX_DC);

			}
		}
	}
	TIM2->EGR |= 0x0001;
}

void MEASSURE() {

	if (((DISTANCE_U1 / 2) <= CRITICAL_CLOSE_DISTANCE && DISTANCE_U1 != 0)
			|| ((DISTANCE_U2 / 2) <= CRITICAL_CLOSE_DISTANCE && DISTANCE_U2 != 0)) {
		if (TURN_POSITION == CLEAR && MOVEMENT_DIRECTION != STOPPED) {
			TURN_POSITION = FIRST_OBSTACLE_BACKWARDS;
		}

		CRITICAL_CLOSE = true;
		CLOSE = false;
		MEDIUM = false;
		RELATIVELY_FAR = false;
	} else if (((DISTANCE_U2 / 2) > CRITICAL_CLOSE_DISTANCE
			&& (DISTANCE_U2 / 2) <= CLOSE_DISTANCE)
			|| ((DISTANCE_U1 / 2) > CRITICAL_CLOSE_DISTANCE
					&& (DISTANCE_U1 / 2) <= CLOSE_DISTANCE)) {
		TURN_POSITION = CLEAR;
		CRITICAL_CLOSE = false;
		CLOSE = true;
		MEDIUM = false;
		RELATIVELY_FAR = false;

	} else if (((DISTANCE_U2 / 2) > CLOSE_DISTANCE
			&& (DISTANCE_U2 / 2) <= MEDIUM_DISTANCE)
			|| ((DISTANCE_U1 / 2) > CLOSE_DISTANCE
					&& (DISTANCE_U1 / 2) <= MEDIUM_DISTANCE)) {
		TURN_POSITION = CLEAR;
		CRITICAL_CLOSE = false;
		CLOSE = false;
		MEDIUM = true;
		RELATIVELY_FAR = false;

	} else if (((DISTANCE_U2 / 2) > MEDIUM_DISTANCE
			&& (DISTANCE_U2 / 2) <= RELATIVELY_FAR_DISTANCE)
			|| ((DISTANCE_U1 / 2) > MEDIUM_DISTANCE
					&& (DISTANCE_U1 / 2) <= RELATIVELY_FAR_DISTANCE)) {
		TURN_POSITION = CLEAR;
		CRITICAL_CLOSE = false;
		CLOSE = false;
		MEDIUM = false;
		RELATIVELY_FAR = true;

	} else {
		TURN_POSITION = CLEAR;
		CRITICAL_CLOSE = false;
		CLOSE = false;
		MEDIUM = false;
		RELATIVELY_FAR = false;
	}

	if (((DISTANCE_U1 / 2) <= 5 && DISTANCE_U1 != 0)
			|| ((DISTANCE_U2 / 2) <= 5 && DISTANCE_U2 != 0)) {
		if (!DISTANCE_MESSAGE_FLAG) {
			char message[70];
			sprintf(message, "WARNING!! OBSTACLE AT A DISTANCE OF LESS THAN 5 cm \r\n");
			general_log_message(message);
			DISTANCE_MESSAGE_FLAG = true;
		}
	} else if (((DISTANCE_U2 / 2) > 27
			&& (DISTANCE_U2 / 2) <= 33)
			|| ((DISTANCE_U1 / 2) > 27
					&& (DISTANCE_U1 / 2) <= 33)) {
		if (!DISTANCE_MESSAGE_FLAG) {
			char message[70];
			sprintf(message, "BE CAREFUL, OBSTACLE IN APPROXIMATELY 30 cm\r\n");
			general_log_message(message);
			DISTANCE_MESSAGE_FLAG = true;
		}
	}
	else{
		DISTANCE_MESSAGE_FLAG=false;
	}

}

void START_COUNTER_3() {
	TIM3->SR = 0;
	TIM3->EGR |= (1 << 0);
	TIM3->CR1 |= 0x0001;
	NVIC->ISER[0] |= (1 << 29);

}

void START_COUNTER_2() {
	TIM4->SR = 0;
	TIM4->CR1 |= 0x0001;
	NVIC->ISER[0] |= (1 << 30);

}

void TIM3_IRQHandler(void) {

	if ((TIM3->SR & (1 << 2)) != 0) {
		if ((GPIOC->IDR & 0x80) != 0) {
			RELATIVE_INIT_U1 = TIM3->CCR2;
		} else {
			PULSE_TIME_U1 = TIM3->CCR2 - RELATIVE_INIT_U1;
			DISTANCE_U1 = 0.034 * PULSE_TIME_U1;
		}

		TIM3->SR &= ~(0x0004);
	}
	if ((TIM3->SR & (1 << 4)) != 0) {
		if ((GPIOC->IDR & 0x200) != 0) {
			RELATIVE_INIT_U2 = TIM3->CCR4;
		} else {
			PULSE_TIME_U2 = TIM3->CCR4 - RELATIVE_INIT_U2;
			DISTANCE_U2 = 0.034 * PULSE_TIME_U2;
		}
		TIM3->SR &= ~(0x0010);
	}
	if ((TIM3->SR & (1 << 1)) != 0) {
		GPIOC->BSRR = (1 << 6) << 16;
		GPIOC->BSRR = (1 << 8) << 16;

		TIM3->SR &= ~(0x0002);
	}
}

void TIM4_IRQHandler(void) {

	if ((TIM4->SR & TIM_SR_CC3IF) && (TIM4->DIER & TIM_DIER_CC3IE)) {
		BUZZ();

		GPIOC->BSRR = (1 << 6);
		GPIOC->BSRR = (1 << 8);

		START_COUNTER_3();
		TIM4->SR &= ~TIM_SR_CC3IF;
	}

	else if ((TIM4->SR & TIM_SR_CC1IF) && (TIM4->DIER & TIM_DIER_CC1IE)) {
		COUNT++;
		TIM4->SR &= ~TIM_SR_CC1IF;
	}

}

void INIT_TIM3() {
	TIM3->CR1 = 0x0000;
	TIM3->CR2 = 0x0000;
	TIM3->SMCR = 0x0000;
	TIM3->ARR = 0xFFFF;
	TIM3->PSC = 31;
	TIM3->CNT = 0;
	TIM3->CCR1 = 11;
	TIM3->CCR3 = 0xFFFF;
	TIM3->DIER = 0x0016;
	TIM3->CCMR1 = 0x0100;
	TIM3->CCMR2 = 0x0100;
	TIM3->CCER = 0xB0B0;

	setup_log_message("TIM3");

}

void INIT_TIM4() {

	TIM4->CR1 = 0x0080;
	TIM4->CR2 = 0x0000;
	TIM4->SMCR = 0x0000;
	TIM4->ARR = 1000;
	TIM4->PSC = 3199;
	TIM4->CNT = 0;
	TIM4->CCR3 = 1000;
	TIM4->CCR1 = 1000;
	TIM4->DIER = 0x0008;
	TIM4->CCMR2 = 0x0000;
	TIM4->CCMR1 = 0x0000;
	TIM4->CCER = 0x0100;

	setup_log_message("TIM4");

}

void INIT_3V_OUTPUT() {
	GPIOB->MODER |= (1 << (8 * 2 + 1));
	GPIOB->MODER &= ~(1 << (8 * 2));

	GPIOB->AFR[1] &= ~(0xF << (0));
	GPIOB->AFR[1] |= (2 << (0));

	setup_log_message("Buzzer output");

}

void INIT_TRIGG_ECHO() {
	GPIOC->MODER &= ~(1 << (2 * 6 + 1));
	GPIOC->MODER |= (1 << (2 * 6));
	GPIOC->MODER |= (1 << (2 * 7 + 1));
	GPIOC->MODER &= ~(1 << (2 * 7));
	GPIOC->AFR[0] &= ~(0xF << (7 * 4));
	GPIOC->AFR[0] |= (2 << (7 * 4));
	GPIOC->MODER &= ~(1 << (2 * 8 + 1));
	GPIOC->MODER |= (1 << (2 * 8));
	GPIOC->MODER |= (1 << (2 * 9 + 1));
	GPIOC->MODER &= ~(1 << (2 * 9));
	GPIOC->AFR[1] &= ~(0xF << (1 * 4));
	GPIOC->AFR[1] |= (2 << (1 * 4));

	setup_log_message("Ultrasonic GPIO");

}

void EXTI0_IRQHandler(void) {
	if (EXTI->PR != 0) {

		char message[70];
		sprintf(message, "[%lu] User button pressed - Direction: %d\r\n",
				HAL_GetTick(), MOVEMENT_DIRECTION);
		HAL_UART_Transmit(&huart1, message, strlen(message), 10000);

		TURN_DIRECTION();

		EXTI->PR = 0x01;
	}
}

void SETUP_USER_BUTTON() {

	GPIOA->MODER &= ~(1 << (0 * 2 + 1));
	GPIOA->MODER &= ~(1 << (0 * 2));

	EXTI->FTSR |= 0x01;
	EXTI->RTSR &= ~(0x01);
	SYSCFG->EXTICR[0] = 0;
	EXTI->IMR |= 0x01;
	NVIC->ISER[0] |= (1 << 6);

	setup_log_message("User Button");

}

void SETUP_ADC() {

	GPIOA->MODER |= 0x00000030;
	ADC1->CR2 &= ~(0x00000001);
	ADC1->CR1 = 0x00000000;
	ADC1->CR2 = 0x00000412;
	ADC1->SQR1 = 0x00000000;
	ADC1->SQR5 = 0x00000002;
	ADC1->CR2 |= 0x00000001;
	while ((ADC1->SR & 0x0040) == 0)
		ADC1->CR2 |= 0x40000000;

	setup_log_message("ADC");

}

void UPDATE_DC() {
	MAX_DC = (ADC1->DR) * 100 / 4096;
	INVERTED_MAX_DC = 100 - MAX_DC;

	if (MOVEMENT_DIRECTION == FORWARD) {
		TURN_FORWARD(MAX_DC, MAX_DC);
	} else if (MOVEMENT_DIRECTION == BACKWARDS) {
		TURN_BACKWARDS(INVERTED_MAX_DC, INVERTED_MAX_DC);
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
// Process the command
//	TURN_DIRECTION();
	memcpy(RxData + indx, temp, 1);
	if (++indx >= 10)
		indx = 0;
// Reactivate reception for 4 characters
	HAL_UART_Receive_IT(huart, temp, 1);
}

void CHANGE_MIN_DISTANCE(uint8_t new_distance) {
	if (new_distance > 0 && new_distance <= 99) {
		CRITICAL_CLOSE_DISTANCE = new_distance;
	}

	char message[100];
	sprintf(message, "[%lu] Changing new minimun distance to: %d cm \r\n",
			HAL_GetTick(), new_distance);
	HAL_UART_Transmit(&huart1, message, strlen(message), 10000);
}

void CHANGE_MAX_DISTANCE(uint8_t new_distance) {
	if (new_distance > 0 && new_distance <= 99) {
		RELATIVELY_FAR_DISTANCE = new_distance;

		char message[100];
		sprintf(message, "[%lu] Changing new maximum distance to: %d cm \r\n",
				HAL_GetTick(), new_distance);
		HAL_UART_Transmit(&huart1, message, strlen(message), 10000);
	}
}

void EXECUTE_REMOTE_COMMAND() {
	bool isBackward = (MOVEMENT_DIRECTION == FORWARD);

	if (temp[0] == '\n') {

		if (strncmp((char*) RxData, "STOP", 4) == 0) {
			STOP(true);
		} else if (strncmp((char*) RxData, "CYCL", 4) == 0) {
			TURN_DIRECTION();
		} else if (strncmp((char*) RxData, "RIGH", 4) == 0
				&& MOVEMENT_DIRECTION == FORWARD) {
			TURN_90_RIGHT(isBackward);
		} else if (strncmp((char*) RxData, "RIGH", 4) == 0
				&& MOVEMENT_DIRECTION == BACKWARDS) {
			TURN_90_LEFT(isBackward);
		} else if (strncmp((char*) RxData, "LEFT", 4) == 0
				&& MOVEMENT_DIRECTION == FORWARD) {
			TURN_90_LEFT(isBackward);
		} else if (strncmp((char*) RxData, "LEFT", 4) == 0
				&& MOVEMENT_DIRECTION == BACKWARDS) {
			TURN_90_RIGHT(isBackward);
		} else if (strncmp((char*) RxData, "FORW", 4) == 0) {
			TURN_FORWARD(MAX_DC, MAX_DC);
			general_log_message("Going Forward");
		} else if (strncmp((char*) RxData, "BACK", 4) == 0) {
			TURN_BACKWARDS(INVERTED_MAX_DC, INVERTED_MAX_DC);
			general_log_message("Going Backwards");
		} else if (strncmp((char*) RxData, "MD", 2) == 0&& isdigit(RxData[2])
		&& isdigit(RxData[3])) {
			uint8_t new_distance = atoi((char*) &RxData[2]);
			CHANGE_MIN_DISTANCE(new_distance);
		}

		else if (strncmp((char*) RxData, "MA", 2) == 0&& isdigit(RxData[2])
		&& isdigit(RxData[3])) {
			uint8_t new_distance = atoi((char*) &RxData[2]);
			CHANGE_MAX_DISTANCE(new_distance);
		}

		memset(temp, 0, sizeof(temp));
		memset(RxData, 0, sizeof(RxData));
		indx = 0;
	}
}

void INIT_DISTANCES() {
	CLOSE_DISTANCE = CRITICAL_CLOSE + 5;
	MEDIUM_DISTANCE = CRITICAL_CLOSE + 15;
	RELATIVELY_FAR_DISTANCE = CRITICAL_CLOSE + 25;
}





//void receive_string() {
//    uint8_t received_byte;  // Extra byte for NULL terminator
//
//    if (HAL_UART_Receive(&huart1, &received_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
//                HAL_UART_Transmit(&huart1, &received_byte, 1, 100); // Echo the received character
//            }  // Send only valid data
//}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_ADC_Init();
	MX_TS_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	general_log_message("_________ BOARD INIT _________");

	SETUP_ADC();
	UPDATE_DC();
	INIT_DISTANCES();
	INIT_TRIGG_ECHO();
	INIT_3V_OUTPUT();
	INIT_TIM4();
	INIT_TIM3();
	START_COUNTER_2();
	SETUP_WHEELS();

	SETUP_PWM();
	SETUP_USER_BUTTON();
	HAL_UART_Receive_IT(&huart1, temp, 1);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		EXECUTE_REMOTE_COMMAND();
		UPDATE_DC();
		if (CRITICAL_CLOSE) {
			bool isBackward = (MOVEMENT_DIRECTION == FORWARD);
			switch (TURN_POSITION) {
			case FIRST_OBSTACLE_BACKWARDS:
				TURN_90_LEFT(isBackward);
				CYCLE_TURN_POSITION();
				break;

			case FIRST_OBSTACLE_FORWARD:
				TURN_90_LEFT(!isBackward);
				CYCLE_TURN_POSITION();
				break;

			case SECOND_OBSTACLE_BACKWARDS:
				TURN_90_RIGHT(isBackward);
				CYCLE_TURN_POSITION();
				break;

			case SECOND_OBSTACLE_FORWARD:
				TURN_90_RIGHT(!isBackward);
				CYCLE_TURN_POSITION();
				break;

			case DOOMED:
				STOP(true);
				break;

			case CLEAR:
				break;

			default:
				break;
			}
		}

		MEASSURE();

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC_Init(void) {

	/* USER CODE BEGIN ADC_Init 0 */

	/* USER CODE END ADC_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC_Init 1 */

	/* USER CODE END ADC_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc.Instance = ADC1;
	hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
	hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
	hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
	hadc.Init.ContinuousConvMode = DISABLE;
	hadc.Init.NbrOfConversion = 1;
	hadc.Init.DiscontinuousConvMode = DISABLE;
	hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_CC3;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc.Init.DMAContinuousRequests = DISABLE;
	if (HAL_ADC_Init(&hadc) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC_Init 2 */

	/* USER CODE END ADC_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TS Initialization Function
 * @param None
 * @retval None
 */
static void MX_TS_Init(void) {

	/* USER CODE BEGIN TS_Init 0 */

	/* USER CODE END TS_Init 0 */

	/* USER CODE BEGIN TS_Init 1 */

	/* USER CODE END TS_Init 1 */
	/* USER CODE BEGIN TS_Init 2 */

	/* USER CODE END TS_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pins : SEG14_Pin SEG15_Pin SEG16_Pin SEG17_Pin
	 SEG18_Pin SEG19_Pin SEG20_Pin SEG21_Pin
	 SEG22_Pin SEG23_Pin */
	GPIO_InitStruct.Pin = SEG14_Pin | SEG15_Pin | SEG16_Pin | SEG17_Pin
			| SEG18_Pin | SEG19_Pin | SEG20_Pin | SEG21_Pin | SEG22_Pin
			| SEG23_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SEG0_Pin SEG2_Pin COM0_Pin COM1_Pin
	 COM2_Pin SEG12_Pin */
	GPIO_InitStruct.Pin = SEG0_Pin | SEG2_Pin | COM0_Pin | COM1_Pin | COM2_Pin
			| SEG12_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : SEG6_Pin SEG7_Pin SEG8_Pin SEG9_Pin
	 SEG10_Pin SEG11_Pin SEG3_Pin SEG4_Pin
	 SEG5_Pin SEG13_Pin COM3_Pin */
	GPIO_InitStruct.Pin = SEG6_Pin | SEG7_Pin | SEG8_Pin | SEG9_Pin | SEG10_Pin
			| SEG11_Pin | SEG3_Pin | SEG4_Pin | SEG5_Pin | SEG13_Pin | COM3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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

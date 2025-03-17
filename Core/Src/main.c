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
#include <stdbool.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */
unsigned short MAX_DC = 100;

unsigned short LOCAL_MAX_DC;
unsigned short FIRST_HALVED;
unsigned short SECOND_HALVED;
unsigned short THIRD_HALVED;
unsigned short STOPPED_STATE;

unsigned short DC_RIGHT;
unsigned short DC_LEFT;

unsigned short relative_init_u1;
unsigned short pulse_time_u1;
unsigned short distance_u1;

unsigned short relative_init_u2;
unsigned short pulse_time_u2;
unsigned short distance_u2;

bool close;
bool too_close;
bool estado;

enum direction {
	FORWARD, BACKWARDS, STOPPED
};
enum direction movement_direction = STOPPED;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TS_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void set_speed(unsigned short dc_right, unsigned short dc_left) {
	if (movement_direction != STOPPED) {
		DC_RIGHT = dc_right;
		DC_LEFT = dc_left;
		TIM2->CCR3 = DC_LEFT;
		TIM2->CCR4 = DC_RIGHT;
	}
}

void turn_backwards() {
	movement_direction = BACKWARDS;

	LOCAL_MAX_DC = 0;
	FIRST_HALVED = MAX_DC / 2;
	SECOND_HALVED = (MAX_DC * 3) / 4;
	THIRD_HALVED = (MAX_DC * 7) / 8;
	STOPPED_STATE = MAX_DC;

	GPIOB->BSRR = (1 << 13);
	GPIOB->BSRR = (1 << 12);
	set_speed(LOCAL_MAX_DC, LOCAL_MAX_DC);
}

void stop() {
	if (movement_direction == FORWARD) {
		set_speed(0, 0);
	} else if (movement_direction == BACKWARDS) {
		set_speed(MAX_DC, MAX_DC);
	}
	movement_direction = STOPPED;

}

void turn_forward() {
	movement_direction = FORWARD;

	LOCAL_MAX_DC = MAX_DC;
	FIRST_HALVED = MAX_DC / 2;
	SECOND_HALVED = MAX_DC / 4;
	THIRD_HALVED = MAX_DC / 8;
	STOPPED_STATE = 0;

	GPIOB->BSRR = (1 << 13) << 16;
	GPIOB->BSRR = (1 << 12) << 16;
	set_speed(LOCAL_MAX_DC, LOCAL_MAX_DC);

}

void turn_direction() {
	if (movement_direction == FORWARD) {
		turn_backwards();

	} else if (movement_direction == BACKWARDS) {
		stop();

	} else if (movement_direction == STOPPED) {
		turn_forward();
	}
}
void setup_wheels() {

	GPIOB->MODER |= (1 << (11 * 2 + 1));
	GPIOB->MODER &= ~(1 << 11 * 2);
	GPIOB->AFR[1] &= ~(0xF << 12);
	GPIOB->AFR[1] |= (1 << 12);

	GPIOB->MODER |= (1 << (10 * 2 + 1));  // Poner bit 21 en 1
	GPIOB->MODER &= ~(1 << (10 * 2));    // Poner bit 20 en 0
	GPIOB->AFR[1] &= ~(0xF << 8);
	GPIOB->AFR[1] |= (1 << 8);

	GPIOB->MODER &= ~(1 << (13 * 2 + 1));
	GPIOB->MODER |= (1 << 13 * 2);

	GPIOB->MODER &= ~(1 << (12 * 2 + 1));
	GPIOB->MODER |= (1 << 12 * 2);

}

void setup_pwm() {

	TIM2->CR1 = 0x0080; // ARPE = 1 -> Is PWM; CEN = 0; Counter OFF
	TIM2->CR2 = 0x0000; // Always 0 in this course
	TIM2->SMCR = 0x0000; // Always 0 in this course
	TIM2->PSC = 32000; // Pre-scaler=32000 -> f_counter=32000000/32000 = 1000 steps/second
	TIM2->CNT = 0; // Initialize counter to 0
	TIM2->ARR = 99;   // 100 niveles (0-99) -> 100 Hz PWM con 1% de resolución
//	TIM2->CCR3 = MAX_DC;  // 50% duty cycle
//	TIM2->CCR4 = MAX_DC;
	TIM2->DIER = 0x0000; // No IRQ when counting is finished -> CCyIE = 0
	// Output mode
	TIM2->CCMR2 = 0x6868; // CCyS = 0 (TOC, PWM)
	// OCyM = 110 (PWM starting in 1)
	// OCyPE = 1 (with preload)
	TIM2->CCER = 0x1100; // CCyP = 0 (always in PWM)
	// CCyE = 1 (hardware output activated)
	// Counter enabling
	TIM2->EGR |= 0x0001; // UG = 1 -> Generate update event
	TIM2->CR1 |= 0x0001; // CEN = 1 -> Start counter
	TIM2->SR = 0; // Counter flags cleared
}

//void TOGGLE_3V(){
//	 if (!estado){
//		 GPIOB -> BSRR = (1<<8);
//		 	 estado = true;
//			 }
//	 else if (estado){
//		 GPIOB -> BSRR = (1<<8)<<16;
//		 	 estado = false;
//	 }
//}

void BUZZ() {
	if (close) {
//			 TOGGLE_3V();
		TIM4->CCMR2 = 0x0030;
		set_speed(FIRST_HALVED, FIRST_HALVED);

	} else if (too_close) {
//			  GPIOB -> BSRR = (1<<8);
		TIM4->CCMR2 = 0x0050;
		set_speed(SECOND_HALVED, SECOND_HALVED);

	} else {
//			  GPIOB -> BSRR = (1<<8)<<16;
		TIM4->CCMR2 = 0x0040;
		set_speed(LOCAL_MAX_DC, LOCAL_MAX_DC);

	}
	TIM2->EGR |= 0x0001; // Generate update event
}

void MEASSURE() {

	if (((distance_u1 / 2) < 5 && distance_u1 != 0)
			|| ((distance_u2 / 2) < 5 && distance_u2 != 0)) {
		close = false;
		too_close = true;
	} else if (((distance_u2 / 2) > 5 && (distance_u2 / 2) < 30)
			|| ((distance_u1 / 2) > 5 && (distance_u1 / 2) < 30)) {
		close = true;
		too_close = false;
	} else {
		close = false;
		too_close = false;
	}
}

void START_COUNTER_3() {
	// Enabling the counter
	// Clear all flags

	TIM3->SR = 0;
	TIM3->EGR |= (1 << 0);		// UG = 1 -> Update event
	TIM3->CR1 |= 0x0001;		// CEN = 1 -> Starting CNT

	// Enabling IRQ source for TIM3 in NVIC (position 29)
	NVIC->ISER[0] |= (1 << 29);

}

void START_COUNTER_2() {
	// Enabling the counter
	TIM4->SR = 0;
	//	TIM2->EGR |= (1<<0);		// UG = 1 -> Update event
	TIM4->CR1 |= 0x0001;		// CEN = 1 -> Starting CNT

	// Enabling IRQ source for TIM3 in NVIC (position 29)
	NVIC->ISER[0] |= (1 << 30);

}

void TIM3_IRQHandler(void) {

//	Channel 2
	if ((TIM3->SR & (1 << 2)) != 0) {
		// If rising edge
		if ((GPIOC->IDR & 0x80) != 0) {
			relative_init_u1 = TIM3->CCR2;
		} else {
			pulse_time_u1 = TIM3->CCR2 - relative_init_u1;
			distance_u1 = 0.034 * pulse_time_u1;
		}

		TIM3->SR &= ~(0x0004);
	}

// CHANNEL 4
	if ((TIM3->SR & (1 << 4)) != 0) {
		// If rising edge
		if ((GPIOC->IDR & 0x200) != 0) {
			relative_init_u2 = TIM3->CCR4;
		} else {
			pulse_time_u2 = TIM3->CCR4 - relative_init_u2;
			distance_u2 = 0.034 * pulse_time_u2;
		}
		TIM3->SR &= ~(0x0010);
	}

//  Channel 1
	if ((TIM3->SR & (1 << 1)) != 0) {
		GPIOC->BSRR = (1 << 6) << 16;
		GPIOC->BSRR = (1 << 8) << 16;

		TIM3->SR &= ~(0x0002);
	}
}

void TIM4_IRQHandler(void) {

	if ((TIM4->SR & 0x0008) != 0) {
		BUZZ();

		GPIOC->BSRR = (1 << 6);
		GPIOC->BSRR = (1 << 8);

		START_COUNTER_3();
		TIM4->SR &= ~(0x0008);
	}
}

void INIT_TIM3() {
	// Internal clock selection: CR1, CR2, SMRC
	TIM3->CR1 = 0x0000; // ARPE = 0 -> Not periodic
						// CEN = 0; Counter off
	//------------------------------------------
	// DO NOT TOUCH
	TIM3->CR2 = 0x0000; // Always 0x0000 in this subject
	TIM3->SMCR = 0x0000; // Always 0x0000 in this subject
	TIM3->ARR = 0xFFFF;   // Recommended value = FFFF

	//-------------------------------------------
	// Setting up the counter functionality: PSC, CNT, ARR y CCRx
	TIM3->PSC = 31; // Preescaler=32 -> F_counter=32000000/32 = 1000000 steps/second
	TIM3->CNT = 0;	   // Initial value for CNT
	TIM3->CCR1 = 11;	//11 steps = 11us
	TIM3->CCR3 = 0xFFFF;
	// IRQ or no-IRQ selection: DIER
	// Usar el DIER para desenmascarar los canales
	// IRQ for channels 4,3,1
	TIM3->DIER = 0x0016;

	// Counter output mode
	TIM3->CCMR1 = 0x0100;
	TIM3->CCMR2 = 0x0100;

	TIM3->CCER = 0xB0B0;

}

void INIT_TIM4() {

	// Internal clock selection: CR1, CR2, SMRC
	TIM4->CR1 = 0x0080; // ARPE = 1 it is periodic
						// CEN = 0; Counter off
	//------------------------------------------
	// DO NOT TOUCH
	TIM4->CR2 = 0x0000; // Always 0x0000 in this subject
	TIM4->SMCR = 0x0000; // Always 0x0000 in this subject
	TIM4->ARR = 1000;   // Recommended value = FFFF

	//-------------------------------------------
	// Setting up the counter functionality: PSC, CNT, ARR y CCRx
	TIM4->PSC = 3199; // Preescaler=3200 -> F_counter=32000000/3200 = 10000 steps/second
	TIM4->CNT = 0;	   // Initial value for CNT
	TIM4->CCR3 = 1000;

	// IRQ or no-IRQ selection: DIER
	TIM4->DIER = 0x0008;
	TIM4->CCMR2 = 0x0000;
	TIM4->CCER = 0x0100;
}

void INIT_3V_OUTPUT() {
	GPIOB->MODER |= (1 << (8 * 2 + 1));
	GPIOB->MODER &= ~(1 << (8 * 2));

	GPIOB->AFR[1] &= ~(0xF << (0)); // Limpiar completamente los bits de AF para PB8
	GPIOB->AFR[1] |= (2 << (0));  // AF2 para TIM4_CH2 (revisa la hoja de datos)

}

void INIT_TRIGG_ECHO() {
	// PC6 as output for TIM3CH1 (DO)

	GPIOC->MODER &= ~(1 << (2 * 6 + 1)); // MODER = 01 (DO) for PC6
	GPIOC->MODER |= (1 << (2 * 6));

	// PC7 as output for TIM3CH2 (AF)
	GPIOC->MODER |= (1 << (2 * 7 + 1));
	GPIOC->MODER &= ~(1 << (2 * 7));

	GPIOC->AFR[0] &= ~(0xF << (7 * 4)); // Limpiar completamente los bits de AF para PC7
	GPIOC->AFR[0] |= (2 << (7 * 4)); // AF2 para TIM3_CH2 (revisa la hoja de datos)

	GPIOC->MODER &= ~(1 << (2 * 8 + 1)); // MODER = 01 (DO) for PC8
	GPIOC->MODER |= (1 << (2 * 8));

	// PC9 as output for TIM3CH4 (AF)
	GPIOC->MODER |= (1 << (2 * 9 + 1));
	GPIOC->MODER &= ~(1 << (2 * 9));

	GPIOC->AFR[1] &= ~(0xF << (1 * 4)); // Limpiar completamente los bits de AF para PC9
	GPIOC->AFR[1] |= (2 << (1 * 4)); // AF2 para TIM3_CH4 (revisa la hoja de datos)

}

void EXTI0_IRQHandler(void) // ISR for EXTI0.
// PC jumps here when the EXTI0 event occurs
{
	if (EXTI->PR != 0) {
		turn_direction();
		EXTI->PR = 0x01; // Clear the EXTI0 flag
	}
}

void SETUP_USER_BUTTON() {
	GPIOA->MODER &= ~(1 << (0 * 2 + 1));
	GPIOA->MODER &= ~(1 << (0 * 2));

	EXTI->FTSR |= 0x01; // Enables falling edge in EXTI0
	EXTI->RTSR &= ~(0x01); // Disables rising edge in EXTI0
	SYSCFG->EXTICR[0] = 0; // EXTI0 linked to GPIOA (i.e. USER button = PA0)
	EXTI->IMR |= 0x01; // Enables EXTI0
	NVIC->ISER[0] |= (1 << 6);
}
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
	/* USER CODE BEGIN 2 */
	INIT_TRIGG_ECHO();
	INIT_3V_OUTPUT();
	INIT_TIM4();
	INIT_TIM3();
	START_COUNTER_2();
	setup_wheels();

	setup_pwm();
	SETUP_USER_BUTTON();
	turn_forward();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
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

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LD4_Pin | LD3_Pin, GPIO_PIN_RESET);

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

	/*Configure GPIO pins : SEG0_Pin SEG1_Pin SEG2_Pin COM0_Pin
	 COM1_Pin COM2_Pin SEG12_Pin */
	GPIO_InitStruct.Pin = SEG0_Pin | SEG1_Pin | SEG2_Pin | COM0_Pin | COM1_Pin
			| COM2_Pin | SEG12_Pin;
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

	/*Configure GPIO pins : LD4_Pin LD3_Pin */
	GPIO_InitStruct.Pin = LD4_Pin | LD3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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

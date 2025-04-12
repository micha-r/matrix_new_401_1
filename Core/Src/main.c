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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
#define cs_reset() HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET)
#define cs_set() HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET)
#define cs_strob() cs_reset();cs_set()

#define data_reset() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define data_set() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define data_strob() data_reset();data_set()

#define clock_reset() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET)
#define clock_set() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET)
#define clock_strob() clock_reset();clock_set()
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int clock = 0;

const uint8_t char_data[][8] = { { 0x1c, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
		0x1c },  // 0
		{ 0x08, 0x0c, 0x0a, 0x08, 0x08, 0x08, 0x08, 0x3e },  // 1
		{ 0x1c, 0x22, 0x20, 0x10, 0x08, 0x04, 0x02, 0x3e },  // 2
		{ 0x1c, 0x22, 0x20, 0x1c, 0x20, 0x20, 0x22, 0x1c },  // 3
		{ 0x20, 0x30, 0x28, 0x24, 0x22, 0x3e, 0x20, 0x20 },  // 4
		{ 0x3e, 0x02, 0x02, 0x1e, 0x20, 0x20, 0x22, 0x1c },  // 5
		{ 0x1c, 0x22, 0x02, 0x1e, 0x22, 0x22, 0x22, 0x1c },  // 6
		{ 0x3e, 0x20, 0x20, 0x10, 0x08, 0x04, 0x04, 0x04 },  // 7
		{ 0x1c, 0x22, 0x22, 0x1c, 0x22, 0x22, 0x22, 0x1c },  // 8
		{ 0x1c, 0x22, 0x22, 0x22, 0x3c, 0x20, 0x22, 0x1c },  // 9
		};

int disp_1 = 0;
int disp_2 = 0;
int disp_3 = 0;
int disp_4 = 0;
int disp_5 = 0;
int disp_6 = 0;
int disp_7 = 0;
int disp_8 = 0;

//typedef enum {
//    REG_NO_OP           = 0x00,
//    REG_DIGIT_0         = 0x01,
//    REG_DIGIT_1         = 0x02,
//    REG_DIGIT_2         = 0x03,
//    REG_DIGIT_3         = 0x04,
//    REG_DIGIT_4         = 0x05,
//    REG_DIGIT_5         = 0x06,
//    REG_DIGIT_6         = 0x07,
//    REG_DIGIT_7         = 0x08,
//    REG_DECODE_MODE     = 0x09,
//    REG_INTENSITY       = 0x0A,
//    REG_SCAN_LIMIT      = 0x0B,
//    REG_SHUTDOWN        = 0x0C,
//    REG_DISPLAY_TEST    = 0x0F,
//} MAX7219_REGISTERS;

//void write_reg(uint8_t reg, uint8_t value) {
//	uint8_t tx_data[2] = { reg, value };
//	cs_reset();
//	HAL_SPI_Transmit(&hspi1, tx_data, sizeof(tx_data), HAL_MAX_DELAY);
//	cs_set();
//}

//void write_reg(uint8_t reg, uint8_t value) {
//	uint8_t tx_data[2] = { reg, value };
//	cs_reset();
//	if (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_READY) {
//		if (HAL_SPI_Transmit(&hspi1, tx_data, sizeof(tx_data), HAL_MAX_DELAY)
//				!= HAL_OK) {
////			clock_set();
////			data_strob();
////			clock_strob();
//		}
//	}
//	cs_set();
//}

void write_reg(uint8_t reg, uint8_t value) {
	uint8_t tx_data[2] = { reg, value };
	cs_reset();
	//1
	HAL_SPI_Transmit(&hspi1, tx_data, sizeof(tx_data), HAL_MAX_DELAY);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
		;
	//2
	HAL_SPI_Transmit(&hspi1, tx_data, sizeof(tx_data), HAL_MAX_DELAY);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
		;
	//3
	HAL_SPI_Transmit(&hspi1, tx_data, sizeof(tx_data), HAL_MAX_DELAY);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
		;
	//4
	HAL_SPI_Transmit(&hspi1, tx_data, sizeof(tx_data), HAL_MAX_DELAY);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
		;
	//5
	HAL_SPI_Transmit(&hspi1, tx_data, sizeof(tx_data), HAL_MAX_DELAY);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
		;
	//6
	HAL_SPI_Transmit(&hspi1, tx_data, sizeof(tx_data), HAL_MAX_DELAY);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
		;
	//7
	HAL_SPI_Transmit(&hspi1, tx_data, sizeof(tx_data), HAL_MAX_DELAY);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
		;
	//8
	HAL_SPI_Transmit(&hspi1, tx_data, sizeof(tx_data), HAL_MAX_DELAY);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
		;
	cs_set();
}

void write_disp(int i) {
	cs_reset();
	//1
	uint8_t tx_disp_1[2] = { i, char_data[disp_1][8 - i] };
	HAL_SPI_Transmit(&hspi1, tx_disp_1, sizeof(tx_disp_1), HAL_MAX_DELAY);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
		;
	//2
	uint8_t tx_disp_2[2] = { i, char_data[disp_2][8 - i] };
	HAL_SPI_Transmit(&hspi1, tx_disp_2, sizeof(tx_disp_2), HAL_MAX_DELAY);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
		;
	//3
	uint8_t tx_disp_3[2] = { i, char_data[disp_3][8 - i] };
	HAL_SPI_Transmit(&hspi1, tx_disp_3, sizeof(tx_disp_3), HAL_MAX_DELAY);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
		;
	//4
	uint8_t tx_disp_4[2] = { i, char_data[disp_4][8 - i] };
	HAL_SPI_Transmit(&hspi1, tx_disp_4, sizeof(tx_disp_4), HAL_MAX_DELAY);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
		;
	//5
	uint8_t tx_disp_5[2] = { i, char_data[disp_5][8 - i] };
	HAL_SPI_Transmit(&hspi1, tx_disp_5, sizeof(tx_disp_5), HAL_MAX_DELAY);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
		;
	//6
	uint8_t tx_disp_6[2] = { i, char_data[disp_6][8 - i] };
	HAL_SPI_Transmit(&hspi1, tx_disp_6, sizeof(tx_disp_6), HAL_MAX_DELAY);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
		;
	//7
	uint8_t tx_disp_7[2] = { i, char_data[disp_7][8 - i] };
	HAL_SPI_Transmit(&hspi1, tx_disp_7, sizeof(tx_disp_7), HAL_MAX_DELAY);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
		;
	//8
	uint8_t tx_disp_8[2] = { i, char_data[disp_8][8 - i] };
	HAL_SPI_Transmit(&hspi1, tx_disp_8, sizeof(tx_disp_8), HAL_MAX_DELAY);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
		;
	cs_set();
}

void matrix_init() {

	write_reg(0x09, 0x00);       //  no decoding
	write_reg(0x0A, 0x05);       //  brightness intensity
	write_reg(0x0B, 0x07);       //  scan limit = 8 LEDs
	write_reg(0x0C, 0x01);       //  power down =0，normal mode = 1
	write_reg(0x0F, 0x00);       //  no test display

}

void matrix_data(int num) {
	for (int i = 1; i <= 8; i++) {
		write_reg(i, char_data[num][8 - i]);
		HAL_Delay(50);
	}
}

void display_all_clear() {
	for (int i = 1; i <= 8; i++) {
		write_reg(i, 0x00);
	}
}

int lenNumLL(int value) {
	int len = 0;

	do {
		value /= 10;
		len++;
	} while (value);

	return len;
}

void update_data_disp() {

	int len = lenNumLL(clock);

	switch (len) {

	case 1: {
		disp_1 = clock;
		break;
	}

	case 2: {
		disp_1 = clock % 10;
		disp_2 = (clock - (clock % 10)) / 10;
		break;
	}

	case 3: {
		disp_1 = clock % 10;
		disp_2 = clock / 10 % 10;
		disp_3 = (clock - (clock % 100)) / 100;
		break;
	}

	case 4: {
		disp_1 = clock % 10;
		disp_2 = clock / 10 % 10;
		disp_3 = clock / 100 % 10;
		disp_4 = (clock - (clock % 1000)) / 1000;
		break;
	}

	case 5: {
		disp_1 = clock % 10;
		disp_2 = clock / 10 % 10;
		disp_3 = clock / 100 % 10;
		disp_4 = clock / 1000 % 10;
		disp_5 = (clock - (clock % 10000)) / 10000;
		break;
	}

	case 6: {
		disp_1 = clock % 10;
		disp_2 = clock / 10 % 10;
		disp_3 = clock / 100 % 10;
		disp_4 = clock / 1000 % 10;
		disp_5 = clock / 10000 % 10;
		disp_6 = (clock - (clock % 100000)) / 100000;
		break;
	}

	case 7: {
		disp_1 = clock % 10;
		disp_2 = clock / 10 % 10;
		disp_3 = clock / 100 % 10;
		disp_4 = clock / 1000 % 10;
		disp_5 = clock / 10000 % 10;
		disp_6 = clock / 100000 % 10;
		disp_7 = (clock - (clock % 1000000)) / 1000000;
		break;
	}

	case 8: {
		disp_1 = clock % 10;
		disp_2 = clock / 10 % 10;
		disp_3 = clock / 100 % 10;
		disp_4 = clock / 1000 % 10;
		disp_5 = clock / 10000 % 10;
		disp_6 = clock / 100000 % 10;
		disp_7 = clock / 1000000 % 10;
		disp_8 = (clock - (clock % 10000000)) / 10000000;
		break;
	}

	case 9: {
		clock = 0;
		break;
	}

	}

//	disp_1 = clock;
//	disp_2 = clock;
//	disp_3 = clock;
//	disp_4 = clock;
//	disp_5 = clock;
//	disp_6 = clock;
//	disp_7 = clock;
//	disp_8 = clock;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		update_data_disp();
		clock++;
		if (clock > 99999999) {
			clock = 0;
		}
	}
}

void update_disp() {
	for (int i = 1; i <= 8; i++) {
		write_disp(i);
	}
}

//void display_number(uint8_t digit, uint8_t number) {
//	uint8_t data[2];
//	data[0] = digit;
//	data[1] = number;
//	cs_enable();
//	spi1_transmit(data, 2);
//	cs_disable();
//}

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
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	matrix_init();
	HAL_Delay(10);
	display_all_clear();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

//		for (int i = 0; i < 10; i++) {
//			matrix_data(i);
//			HAL_Delay(1000);
//			display_all_clear();
////			HAL_Delay(500);
//		}

		update_disp();

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim2.Init.Prescaler = 15999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
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

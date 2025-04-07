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
SPI_HandleTypeDef hspi1;

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const uint8_t charData[][8] = {
		{ 0x1c, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x1c },  // 0
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

uint8_t disp[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

//void write_reg(uint8_t reg, uint8_t value) {
//	uint8_t tx_data[2] = { reg, value };
//	cs_reset();
//	HAL_SPI_Transmit(&hspi1, tx_data, sizeof(tx_data), HAL_MAX_DELAY);
//	cs_set();
//}


//void write_reg(uint8_t reg, uint8_t value) {
//	cs_reset();
//	HAL_SPI_Transmit(&hspi1, (uint8_t*) &reg, 1, HAL_MAX_DELAY);
//	HAL_Delay(10);
//	HAL_SPI_Transmit(&hspi1, (uint8_t*) &value, 1, HAL_MAX_DELAY);
//	HAL_Delay(10);
//	clock_set();
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


//void write_reg(uint8_t reg, uint8_t value) {
//	uint8_t tx_data[2] = { reg, value };
//	cs_reset();
//	HAL_SPI_Transmit(&hspi1, tx_data, 2, HAL_MAX_DELAY);
//	cs_set();
//}

//void write_reg(uint8_t addr, uint8_t data) {
//	cs_reset(); //активируем Chip Select
//    SPI1->DR = ((uint16_t)addr << 8 | data); //отправляем данные
//        while (!(SPI1->SR & SPI_SR_TXE));
//    while (SPI1->SR & SPI_SR_BSY);
//    cs_set();
//}

void write_reg(uint8_t reg, uint8_t value) {
	uint8_t tx_data[2] = { reg, value };
	cs_reset();
	HAL_SPI_Transmit(&hspi1, tx_data, 2, HAL_MAX_DELAY);
	cs_set();
}


void matrixInit() {
	write_reg(0x09, 0x00); // BCD decode mode
	write_reg(0x0A, 0x01); // Display brightness
	write_reg(0x0B, 0x07); // Scan limit
	write_reg(0x0C, 0x01); // Turn ON display
	write_reg(0x0F, 0x00); // Disable display test
}

void matrixData(int num) {
	for (int i = 1; i <= 8; i++) {
		write_reg(i, charData[num][8 - i]);
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  matrixInit();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		for (int i = 0; i < 10; i++) {
			matrixData(i);
			HAL_Delay(1000);
		}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

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

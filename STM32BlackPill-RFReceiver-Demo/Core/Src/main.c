/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LT8900_RF_CHANNEL	0x04

const uint8_t INIT_REGS[][3] = {
		{ 0, 0x6f, 0xe0 },
		{ 1, 0x56, 0x81 },
		{ 2, 0x66, 0x17 },
		{ 4, 0x9c, 0xc9 },
		{ 5, 0x66, 0x37 },
		{ 7, 0x00, LT8900_RF_CHANNEL },
		{ 8, 0x6c, 0x90 }, //power (default 71af) UNDOCUMENTED
		{ 9, 0x19, 0x40 },
		{ 10, 0x7f, 0xfd },
		{ 11, 0x00, 0x08 },
		{ 12, 0x00, 0x00 },
		{ 13, 0x48, 0xbd },
		{ 22, 0x00, 0xff },
		{ 23, 0x80, 0x05 },
		{ 24, 0x00, 0x67 },
		{ 25, 0x16, 0x59 },
		{ 26, 0x19, 0xe0 },
		{ 27, 0x13, 0x00 },
		{ 28, 0x18, 0x00 },
		{ 32, 0x48, 0x00 }, //8920�??????62.5kbps下同步头只能�??????32�??????16bit},
		{ 33, 0x3f, 0xc7 },
		{ 34, 0x20, 0x00 },
		{ 35, 0x03, 0x00 },
		//重发次数4次},
		{ 36, 0x05, 0x0A },
		//Sync_word 设置1
		{ 37, 0x00, 0x00 },
		{ 38, 0x00, 0x00 },
		/*
		{ 37, 0x00, 0x03 }, //Sync_word 设置2
		{ 38, 0x00, 0x00 }, //Sync_word 设置3
		*/
		{ 39, 0x55, 0xAA },
		//Sync_word 设置4
		{ 40, 0x44, 0x02 },
		{ 41, 0xb8, 0x00 },
		//CRC is ON; scramble is OFF; AUTO_ACK is ON},
		{ 42, 0xfd, 0xb0 },
		//等待RX_ACK时间 176us},
		{ 43, 0x00, 0x0f },
		{ 44, 0x10, 0x00 },
		{ 45, 0x05, 0x52 },
		//62.5k},
		{ 50, 0x00, 0x00 },
		{ 52, 0x80, 0x80 },
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define REG40_VALUE 0x4402
#define LT8900_FLAG_PKT 0B1000000
#define LT8900_FLAG_SYNCWORD_RECV 0x80

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
volatile uint32_t g_ledTurnOffTime = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void LT8900_Init();
void LT8900_WriteHL(uint8_t reg, uint8_t high, uint8_t low);
uint16_t LT8900_ReadRegister(uint8_t reg);
int LT8900_Fifo_Read(void *data, uint8_t maxLength);
void LT8900_ToRecvMode();
void LT8900_ToIdleMode();
int LT8900_HasDataToRead();
void LED_Drive();
void LED_Blink();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void LT8900_Init() {

	// 复位射频芯片
	HAL_GPIO_WritePin(RF_RST_GPIO_Port, RF_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(RF_RST_GPIO_Port, RF_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(200);

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET); // 拉高 SPI_CS

	// 执行初始化序�????????
	for (int i = 0; i < (sizeof(INIT_REGS) / 3); i++) {
		LT8900_WriteHL(INIT_REGS[i][0], INIT_REGS[i][1], INIT_REGS[i][2]);
	}


	HAL_Delay(100);
	uint16_t initCheckResult = LT8900_ReadRegister(40);
	if (initCheckResult != REG40_VALUE) {
		Error_Handler();
	}
}

void LT8900_WriteHL(uint8_t reg, uint8_t high, uint8_t low) {
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	uint8_t bytes[3] = { reg, high, low };
	if (HAL_SPI_Transmit(&hspi1, bytes, 3, HAL_MAX_DELAY) != HAL_OK) {
		Error_Handler();
	}
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

uint16_t LT8900_ReadRegister(uint8_t reg) {

	reg |= 0x80;
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

	// 发送读取寄存器的命令
	if (HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY) != HAL_OK) {
		Error_Handler();
	}

	uint16_t retval;
	// 接收寄存器值
	HAL_StatusTypeDef result = HAL_SPI_Receive(&hspi1, (uint8_t*) (&retval), 2,
			HAL_MAX_DELAY);

	if (result != HAL_OK) {
		Error_Handler();
	}

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	// STM32 是 Little Endian，需要高低位转换
	retval = ((retval & 0x00FF) << 8) | ((retval & 0xFF00) >> 8);
	return retval;
}

int LT8900_Fifo_Read(void *data, uint8_t maxLength) {
	if (maxLength < 2) {
		Error_Handler();
	}

	uint16_t word = LT8900_ReadRegister(50);

	uint8_t packetSize = (uint8_t) ((word & 0xFF00) >> 8);
	if (packetSize == 0 || packetSize > maxLength) {
		return -1;
	}

	uint8_t *buf = (uint8_t*) data;

	buf[0] = (uint8_t) (word & 0x00FF);    //第一个字节已经读取过来了

	//读取剩下 payload 字节
	size_t leftSize = packetSize - 1;
	size_t safeSize = maxLength > leftSize ? maxLength : leftSize;
	for (uint8_t i = 1; i < safeSize; i += 2) {
		word = LT8900_ReadRegister(50);
		memcpy(buf + i, &word, sizeof(word));
	}

	LT8900_WriteHL(52, 0x00, 0x80); // 清空 FIFO 缓冲�???????????

	return packetSize;
}

void LT8900_ToRecvMode() {
	LT8900_WriteHL(7, 0x00, 0x00);
	LT8900_WriteHL(52, 0x80, 0x80);
	LT8900_WriteHL(7, 0x00, 0x80 + LT8900_RF_CHANNEL);
}

void LT8900_ToIdleMode() {
	LT8900_WriteHL(7, 0x00, 0x00);
}

int LT8900_HasDataToRead() {
	uint16_t reg48 = LT8900_ReadRegister(48);
	return (reg48 & LT8900_FLAG_PKT);
}

void LED_Drive() {
	uint32_t now = HAL_GetTick();
	if (now >= g_ledTurnOffTime) {
		HAL_GPIO_WritePin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin, GPIO_PIN_SET);
		g_ledTurnOffTime = 0;
	} else {
		HAL_GPIO_WritePin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin, GPIO_PIN_RESET);
	}
}

void LED_Blink() {
	g_ledTurnOffTime = HAL_GetTick() + 50;
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
	LT8900_Init();

	uint8_t recvBuf[16];
	LT8900_ToIdleMode();
	LT8900_ToRecvMode(); // channel 4
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (LT8900_HasDataToRead()) {
			LED_Blink();	// 让 LED 闪烁

			LT8900_Fifo_Read(recvBuf, 16); // 读取收到的数据
			// recvBuf 中就是射频芯片接收到的数据

			LT8900_ToIdleMode(); // 切换到 IDLE 模式

			LT8900_ToRecvMode(); // 切换到接收模式准备接收下次发送的数据

			HAL_Delay(5);
		}

		LED_Drive(); // 驱动 LED 闪烁计时器

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

  /** Initializes the CPU, AHB and APB busses clocks 
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
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RF_RST_Pin|SPI1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : RF_RST_Pin SPI1_CS_Pin */
  GPIO_InitStruct.Pin = RF_RST_Pin|SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_DEBUG_Pin */
  GPIO_InitStruct.Pin = LED_DEBUG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_DEBUG_GPIO_Port, &GPIO_InitStruct);

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
	// LED 常亮指示出现错误
	HAL_GPIO_WritePin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin, GPIO_PIN_RESET);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

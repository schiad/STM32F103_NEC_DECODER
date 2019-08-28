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
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAXIMDISP
#define VPCOM
//#define DEBUG

uint8_t len = 0;
uint8_t BUFF[80];

struct NEC NEC1;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#ifdef MAXIMDISP

void MAXIM(SPI_HandleTypeDef *hspi, uint8_t addr, uint8_t comm) {
	static uint8_t data[2];
	data[0] = addr;
	data[1] = comm;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);
	HAL_SPI_Transmit(hspi, &data[0], 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);
}

void MAXIM_Init(SPI_HandleTypeDef *hspi) {
	uint8_t i;
	i = 0;
	MAXIM(hspi, 0xA, 0x0); // LED INTENSITY
	while (i < 8) {
		MAXIM(hspi, 0xA, 0x0);
		MAXIM(hspi, i + 1, 0x0);
		i++;
	}
	MAXIM(hspi, 0xC, 0x1);
	MAXIM(hspi, 0xB, 0x7);
	MAXIM(hspi, 0xA, 0x0); // LED INTENSITY
	MAXIM(hspi, 0xF, 0x1);
	MAXIM(hspi, 0xF, 0x0);
	MAXIM(hspi, 0x9, 0x0);
	i = 0;
}

void Maxim_Map(SPI_HandleTypeDef *hspi, uint8_t *map) {
	static uint8_t i;
	i = 0;
	while (i < 8) {
		MAXIM(hspi, i + 1, map[i]);
		i++;
	}
}

#endif

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	NEC1.count = __HAL_TIM_GET_COUNTER(&htim1);
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	NEC1.gpio = HAL_GPIO_ReadPin(NEC_GPIO_Port, NEC_Pin);
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
#ifdef DEBUG
	len = sprintf(BUFF, "DEBUG :\t%u\t%u\t%u\n\0", NEC1.i, NEC1.gpio, NEC1.count);
	CDC_Transmit_FS(BUFF, len);
#endif

	if (NEC1.gpio && NEC1.count > 850 && NEC1.count < 1000) {
		NEC1.init_seq = 1;
		NEC1.complet = 0;
	}

	if (!NEC1.gpio && NEC1.count > 420 && NEC1.count < 480 && NEC1.init_seq) {
		NEC1.i = -1;
		NEC1.repeat = 0;
		NEC1.init_seq = 0;
	}

	if (!NEC1.gpio && NEC1.count > 200 && NEC1.count < 260 && NEC1.init_seq) {
		NEC1.i = -1;
		NEC1.init_seq = 0;
		NEC1.repeat++;
		NEC1.complet = 1;
	}

	if (NEC1.gpio && NEC1.count > 40 && NEC1.count < 70) {
		NEC1.i++;
	}

	if (!NEC1.gpio && NEC1.count > 40 && NEC1.count < 180) {
		switch (NEC1.i / 8) {
		case 0:
			if (NEC1.count > 100) {
				NEC1.addr |= (1 << (NEC1.i % 8));
			} else {
				NEC1.addr &= ~(1 << (NEC1.i % 8));
			}
			break;
		case 1:
			if (NEC1.count > 100) {
				NEC1.addr_inv |= (1 << (NEC1.i % 8));
			} else {
				NEC1.addr_inv &= ~(1 << (NEC1.i % 8));
			}
			break;
		case 2:
			if (NEC1.count > 100) {
				NEC1.cmd |= (1 << (NEC1.i % 8));
			} else {
				NEC1.cmd &= ~(1 << (NEC1.i % 8));
			}
			break;
		case 3:
			if (NEC1.count > 100) {
				NEC1.cmd_inv |= (1 << (NEC1.i % 8));
			} else {
				NEC1.cmd_inv &= ~(1 << (NEC1.i % 8));
			}
			break;
		default:
			break;
		}
	}

	if (NEC1.i == 32) {
		NEC1.complet = 1;
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
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	MAXIM_Init(&hspi1);
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	HAL_TIM_Base_Start(&htim1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1) {

		if (NEC1.complet) {
#ifdef MAXIMDISP
			NEC1.complet = 0;
			MAXIM(&hspi1, 1, NEC1.addr);
			MAXIM(&hspi1, 2, NEC1.addr_inv);
			MAXIM(&hspi1, 3, NEC1.cmd);
			MAXIM(&hspi1, 4, NEC1.cmd_inv);
			MAXIM(&hspi1, 5, NEC1.repeat);
#endif

#ifdef VPCOM
			if (!NEC1.repeat) {
				len =
						sprintf(BUFF,
								"Received :\n\
ADDR:\t%x\n\
!ADDR:\t%x\n\
CMD:\t%x\n\
!CMD:\t%x\n",
								NEC1.addr, NEC1.addr_inv, NEC1.cmd,
								NEC1.cmd_inv);
			} else {
				len = sprintf(BUFF, "Repeat : %d\n", NEC1.repeat);
			}
			CDC_Transmit_FS(BUFF, len);
#endif
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 719;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NEC_Pin */
  GPIO_InitStruct.Pin = NEC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(NEC_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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

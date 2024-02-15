/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "app_subghz_phy.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "com_debug.h"
#include "stdio.h"
#include "string.h"
#include "radio_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define PAYLOAD_LENGTH (16)

#define FREQ_401_MHZ            (401375000) //DLink  space to earth
#define FREQ_402_MHZ			(402375000)	//UPLINK earth to space

#define PA_DUTY_CYCLE           (0x04)
#define HP_MAX                  (0x07)
#define PA_SEL                  (0x00)

//#define POWER                   (0x16)
//#define RAMP_TIME               (0x02)

#define POWER                   (0x0E)      // Reduce power for lower consumption
#define RAMP_TIME               (0x03)      // Adjust ramp time for lower power
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
PacketParams_t pkt_params;
ModulationParams_t mod_params;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

SUBGHZ_HandleTypeDef hsubghz;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint8_t rxCMD[6];
uint8_t rxOBCCMD[4];
uint8_t rxOBCCMDLP[4];

uint8_t p_len = PAYLOAD_LENGTH;

uint8_t tx_cmd[PAYLOAD_LENGTH];
uint8_t rx_cmd[PAYLOAD_LENGTH];

uint8_t ack;
uint8_t pkt_id = 0;
uint8_t counter = 0;

uint16_t crc = 0;
uint8_t DATA_SIZE = 14;
uint8_t rssi_value;

char temp_rx[100];
char temp_OBC_rx[100];
char temp_OBC_rx_LP[100];

uint8_t TX_FLAG = 0;

uint16_t mSEC = 0;
uint8_t sec = 0;
uint8_t min = 0;
uint8_t hour = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
uint16_t calc_CRC(const uint8_t *data, uint8_t data_length);
void radioTxData();

void DioIrqHndlr(RadioIrqMasks_t radioIrq);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim1) {
		mSEC++;

		if (mSEC > 999) {
			mSEC = 0;
			sec++;
		}

		if (sec > 0) {
			if (sec % 5 == 0) {
				TX_FLAG = 1;
			}
		}

		if (sec > 59) {
			sec = 0;
			min++;
		}

		if (min > 59) {
			min = 0;
			hour++;
		}

		if (hour > 23) {
			hour = 0;
			min = 0;
			sec = 0;
		}
	}
}

uint16_t calc_CRC(const uint8_t *data, uint8_t data_length) {

	uint16_t crcReg = 0xFFFF;	// Initialize the CRC register with 0xFFFF
	uint16_t calc = 0x8408;		// Polynomial for CRC-16
	uint16_t w;
	int i, j;
	uint8_t calc_data[DATA_SIZE];  // in 16 bytes, 14 are data bytes

	// Copy data into calc_data
	for (i = 0; i < data_length; i++) {
		calc_data[i] = data[i];
		// Iterate over each byte of data
		for (j = 0; j < 8; j++) {
			w = (crcReg ^ calc_data[i]) & 0x0001; // XOR the LSB of crcReg with the LSB of calc_data
			crcReg = crcReg >> 1;			// Right-shift the crcReg by 1 bit
			if (w == 1) {
				crcReg = crcReg ^ calc;	// If w is 1, XOR the crcReg with the polynomial
			}
			calc_data[i] = calc_data[i] >> 1;// Right-shift the data byte by 1 bit
		}
	}
	crcReg = crcReg ^ 0xFFFF;						// Final XOR with 0xFFFF
	return crcReg;
}

void radioTxData() {
	tx_cmd[0] = 0x42;
	tx_cmd[1] = 0x4D;

	//getSoilSensorData();
	tx_cmd[2] = 0x01; 		       //gst data (GST ID)
	//tx_cmd[3] = read_soil_value >> 8 & 0xff; 	  //gst DATA (SOIL SENSOR - MSB)
	//tx_cmd[4] = read_soil_value & 0xff;     //gst data (SOIL SENSOR LSB)

	tx_cmd[3] = 0x1A; 	  //gst DATA (SOIL SENSOR - MSB)
	tx_cmd[4] = 0x23;     //gst data (SOIL SENSOR LSB)

	tx_cmd[5] = 0x33;    //gst data
	tx_cmd[6] = 0x44;
	tx_cmd[7] = 0x55;
	tx_cmd[8] = 0x66;
	tx_cmd[9] = 0x77;
	tx_cmd[10] = 0x88;
	tx_cmd[11] = 0x99;
	tx_cmd[12] = 0x38;
	tx_cmd[13] = 0xA7;
	crc = 0;
	crc = calc_CRC(tx_cmd, DATA_SIZE);
	tx_cmd[14] = crc >> 8;
	tx_cmd[15] = crc;
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
	MX_SubGHz_Phy_Init();
	MX_USART1_UART_Init();
	MX_TIM2_Init();
	MX_USART2_UART_Init();
	MX_LPUART1_UART_Init();
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start(&htim2);

	pkt_params.PacketType = PACKET_TYPE_LORA;
	pkt_params.Params.LoRa.PayloadLength = PAYLOAD_LENGTH;
	pkt_params.Params.LoRa.PreambleLength = 8;
	pkt_params.Params.LoRa.HeaderType = LORA_PACKET_IMPLICIT;
	pkt_params.Params.LoRa.CrcMode = LORA_CRC_ON;
	pkt_params.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;

	mod_params.PacketType = PACKET_TYPE_LORA;
	mod_params.Params.LoRa.Bandwidth = LORA_BW_031;
	mod_params.Params.LoRa.SpreadingFactor = LORA_SF10;
	mod_params.Params.LoRa.CodingRate = LORA_CR_4_8;
//	mod_params.Params.LoRa.LowDatarateOptimize = 0;
	mod_params.Params.LoRa.LowDatarateOptimize = 1; // Enable low data rate optimization

	SUBGRF_Init(DioIrqHndlr);
	SUBGRF_SetBufferBaseAddress(0x00, 0x00);
	SUBGRF_SetPayload(tx_cmd, PAYLOAD_LENGTH);
	SUBGRF_SetPacketParams(&pkt_params);
	SUBGRF_SetSyncWord(( uint8_t[] ) { 0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00,
					0x00 });
	SUBGRF_SetWhiteningSeed(0x01FF);
	SUBGRF_SetRfFrequency(FREQ_402_MHZ);
	SUBGRF_SetPaConfig(PA_DUTY_CYCLE, HP_MAX, PA_SEL, 0x01);
	//SUBGRF_SetTxParams(RFO_HP, POWER, RAMP_TIME);
	SUBGRF_SetTxParams(RFO_LP, POWER, RAMP_TIME); // Set to RFO_LP for low power
	SUBGRF_SetModulationParams(&mod_params);
	SUBGRF_SetDioIrqParams(
			IRQ_TX_DONE | IRQ_PREAMBLE_DETECTED | IRQ_RX_DONE
					| IRQ_RX_TX_TIMEOUT | IRQ_SYNCWORD_VALID,
			IRQ_TX_DONE | IRQ_PREAMBLE_DETECTED | IRQ_RX_DONE
					| IRQ_RX_TX_TIMEOUT | IRQ_SYNCWORD_VALID, IRQ_RADIO_NONE,
			IRQ_RADIO_NONE);

	myPrintf("\n########## Test code: BEGIN ##########\r\n");
	myPrintf("########## UART1 (PA9_TX, PA10_RX): 		##########\r\n");
	myPrintf("Pin Description: Debug COM_MCU\r\n");
	myPrintf("If this message pop up then debug COM MCU is working!!!\r\n");
	myPrintf("__________________________________________________\r\n");
	myPrintf("________________Waiting to receive COM Command____________\r\n");
	myPrintf("SEND 'UART02' to test UART2 OBC'\r\n");
	myPrintf("SEND 'LPUART' to test LPUART OBC'\r\n");
	myPrintf("SEND '3RADIO' to test RFO LP Radio'\r\n");
	myPrintf("__________________________________________________\r\n");

	HAL_UART_Receive_IT(&huart1, rxCMD, 6);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_RESET);
		delay_us(1000000);
		HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_SET);
		delay_us(1000000);

		if (TX_FLAG) {
			radioTxData();
			SUBGRF_SetRfFrequency(FREQ_402_MHZ);
			//SUBGRF_SetSwitch(RFO_HP, RFSWITCH_TX); /*Set RF switch*/
			SUBGRF_SetSwitch(RFO_LP, RFSWITCH_TX); /*Set RF switch*/
			SUBGRF_SendPayload(tx_cmd, 16, 0);
			counter++;
		}

		/* USER CODE END WHILE */
		MX_SubGHz_Phy_Process();

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

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS_PWR;
	RCC_OscInitStruct.HSEDiv = RCC_HSE_DIV1;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
	RCC_OscInitStruct.PLL.PLLN = 6;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3 | RCC_CLOCKTYPE_HCLK
			| RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief LPUART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_LPUART1_UART_Init(void) {

	/* USER CODE BEGIN LPUART1_Init 0 */

	/* USER CODE END LPUART1_Init 0 */

	/* USER CODE BEGIN LPUART1_Init 1 */

	/* USER CODE END LPUART1_Init 1 */
	hlpuart1.Instance = LPUART1;
	hlpuart1.Init.BaudRate = 115200;
	hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
	hlpuart1.Init.StopBits = UART_STOPBITS_1;
	hlpuart1.Init.Parity = UART_PARITY_NONE;
	hlpuart1.Init.Mode = UART_MODE_TX_RX;
	hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
	if (HAL_UART_Init(&hlpuart1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN LPUART1_Init 2 */

	/* USER CODE END LPUART1_Init 2 */

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
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief SUBGHZ Initialization Function
 * @param None
 * @retval None
 */
void MX_SUBGHZ_Init(void) {

	/* USER CODE BEGIN SUBGHZ_Init 0 */

	/* USER CODE END SUBGHZ_Init 0 */

	/* USER CODE BEGIN SUBGHZ_Init 1 */

	/* USER CODE END SUBGHZ_Init 1 */
	hsubghz.Init.BaudratePrescaler = SUBGHZSPI_BAUDRATEPRESCALER_8;
	if (HAL_SUBGHZ_Init(&hsubghz) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SUBGHZ_Init 2 */

	/* USER CODE END SUBGHZ_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 48 - 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 1000 - 1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

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
	htim2.Init.Prescaler = 48 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1000001 - 1;
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
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LED1_Pin | LED2_Pin | LED3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, FE_CTRL3_Pin | FE_CTRL2_Pin | FE_CTRL1_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
	GPIO_InitStruct.Pin = LED1_Pin | LED2_Pin | LED3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : FE_CTRL3_Pin FE_CTRL2_Pin FE_CTRL1_Pin */
	GPIO_InitStruct.Pin = FE_CTRL3_Pin | FE_CTRL2_Pin | FE_CTRL1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : B1_Pin B2_Pin */
	GPIO_InitStruct.Pin = B1_Pin | B2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : B3_Pin */
	GPIO_InitStruct.Pin = B3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(B3_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart1) {
		sprintf(temp_rx, "%c%c%c%c%c%c", rxCMD[0], rxCMD[1], rxCMD[2], rxCMD[3],
				rxCMD[4], rxCMD[5]);
		if (strcmp(temp_rx, "UART02") == 0) {
			ack = 1;
			myPrintf("\n########## Test code: BEGIN ##########\r\n");
			myPrintf("########## UART2 (PA2_TX, PA3_RX): 		##########\r\n");
			myPrintf("Pin Description: OBC_COM_1\r\n");
			myPrintf(
					"If this message pop up then track in OBC_COM_1_Script_Communicator\r\n");
			myPrintf("------------- Wait for Test Complete --------------\r\n");
			myPrintf("__________________________________________________\r\n");

			myPrintf2(
					"#\n######### Thank you for visiting OBC_COM_1_Script_Communicator ##########\r\n");
			myPrintf2("########## UART2 (PA2_TX, PA3_RX): 		##########\r\n");
			myPrintf2("Pin Description: OBC_COM_1\r\n");
			myPrintf2(
					"Please send 'OK02' to verify test and track into COM_Script_Communicator, i.e.\r\n");
			myPrintf2("1. UART1 (PA9_TX, PA10_RX): 		##########\r\n");
			myPrintf2("2. Pin Description: Debug COM_MCU\r\n");
			myPrintf2("__________________________________________________\r\n");
			HAL_UART_Receive_IT(&huart2, rxOBCCMD, 4);
		} else if (strcmp(temp_rx, "LPUART") == 0) {
			ack = 2;
			myPrintf("\n########## Test code: BEGIN ##########\r\n");
			myPrintf("########## LPUART (PC1_TX, PC0_RX): 		##########\r\n");
			myPrintf("Pin Description: OBC_COM_LPUART\r\n");
			myPrintf(
					"If this message pop up then track in OBC_COM_LPUART_Script_Communicator\r\n");
			myPrintf("------------- Wait for Test Complete --------------\r\n");
			myPrintf("__________________________________________________\r\n");

			myPrintf3(
					"\n########## Thank you for visiting OBC_COM_LPUART_Script_Communicator ##########\r\n");
			myPrintf3("########## LPUART (PC1_TX, PC0_RX): 		##########\r\n");
			myPrintf3("Pin Description: OBC_COM_LPUART\r\n");
			myPrintf3(
					"Please send 'OKLP' to verify test and track into COM_Script_Communicator, i.e.\r\n");
			myPrintf3("1. UART1 (PA9_TX, PA10_RX): 		##########\r\n");
			myPrintf3("2. Pin Description: Debug COM_MCU\r\n");
			myPrintf3("__________________________________________________\r\n");
			HAL_UART_Receive_IT(&hlpuart1, rxOBCCMDLP, 4);
		} else if (strcmp(temp_rx, "3RADIO") == 0) {
			ack = 3;
			myPrintf("\n########## Test code: BEGIN ##########\r\n");
			myPrintf("########## Low Power Radio: 	##########\r\n");
			myPrintf("########## COMMUNICATION PARAMETERS: 		##########\r\n");
			myPrintf("Modulation: LoRa PACKET\r\n");
			myPrintf(
					"FREQUENCY MODS: UPLINK FREQ: %lu\r\n    DOWNLINK FREQ: %lu\r\n",
					FREQ_402_MHZ, FREQ_401_MHZ);
			myPrintf(
					"POWER CONFIG:\r\n    PA_DUTY_CYCLE : %x,    HP_MAX: %x,\n\r    PA_SEL : %x,    POWER TX: %u dBm\n\r",
					PA_DUTY_CYCLE, HP_MAX, PA_SEL, POWER);
			myPrintf("RECEVING BANDWIDTH: 	%d\n\r",
					mod_params.Params.LoRa.Bandwidth);
			myPrintf("Packet Type: 			%d\n\r", pkt_params.PacketType);
			myPrintf("PayloadLength: 		%d\n\r",
					pkt_params.Params.LoRa.PayloadLength);
			myPrintf("PreambleLength: 		%d\n\r",
					pkt_params.Params.LoRa.PreambleLength);
			myPrintf("HeaderType: 			%d\n\r",
					pkt_params.Params.LoRa.HeaderType);

			myPrintf(
					"If this message pop up then track in other NucleoWL55JC2 Receiver\r\n");
			myPrintf("------------- Wait for Test Complete --------------\r\n");
			myPrintf("__________________________________________________\r\n");
			myPrintf(
					"\n\rTo stop the testing first complete transmission and reception:  \r\n");
			myPrintf("##### The Transmitter will transmits packets now. \r\n");

			HAL_TIM_Base_Start_IT(&htim1);
		}
		HAL_UART_Receive_IT(&huart1, rxCMD, 6);
	} else if (huart == &huart2) {
		sprintf(temp_OBC_rx, "%c%c%c%c", rxOBCCMD[0], rxOBCCMD[1], rxOBCCMD[2],
				rxOBCCMD[3]);
		if (strcmp(temp_OBC_rx, "OK02") == 0) {
			myPrintf("\n---------    Test Complete    ---------\r\n");
			myPrintf("----------- Received 'OK02' from:- \r\n");
			myPrintf("########## UART2 (PA2_TX, PA3_RX): 		##########\r\n");
			myPrintf(
					"########## Pin Description: OBC_COM_1     ##########\r\n");
			myPrintf("__________________________________________________\r\n");
			myPrintf("__________________________________________________\r\n");
			myPrintf(
					"________________Waiting to receive COM Command____________\r\n");
			myPrintf("SEND 'UART02' to test UART2 OBC'\r\n");
			myPrintf("SEND 'LPUART' to test LPUART OBC'\r\n");
			myPrintf("SEND '3RADIO' to test RFO LP Radio'\r\n");
			myPrintf("__________________________________________________\r\n");
		}
		HAL_UART_Receive_IT(&huart2, rxOBCCMD, 4);
	} else if (huart == &hlpuart1) {
		sprintf(temp_OBC_rx_LP, "%c%c%c%c", rxOBCCMDLP[0], rxOBCCMDLP[1],
				rxOBCCMDLP[2], rxOBCCMDLP[3]);
		if (strcmp(temp_OBC_rx_LP, "OKLP") == 0) {
			myPrintf("\n---------    Test Complete    ---------\r\n");
			myPrintf("----------- Received 'OKLP' from:- \r\n");
			myPrintf("########## LPUART (PC1_TX, PC0_RX):  		##########\r\n");
			myPrintf(
					"########## Pin Description: OBC_COM_LPUART     ##########\r\n");
			myPrintf("__________________________________________________\r\n");
			myPrintf("__________________________________________________\r\n");
			myPrintf(
					"________________Waiting to receive COM Command____________\r\n");
			myPrintf("SEND 'UART02' to test UART2 OBC'\r\n");
			myPrintf("SEND 'LPUART' to test LPUART OBC'\r\n");
			myPrintf("SEND '3RADIO' to test RFO LP Radio'\r\n");
			myPrintf("__________________________________________________\r\n");
		}
		HAL_UART_Receive_IT(&hlpuart1, rxOBCCMDLP, 4);
	}
}

void DioIrqHndlr(RadioIrqMasks_t radioIrq) {
	if (radioIrq == IRQ_TX_DONE) {
		TX_FLAG = 0;
		myPrintf("\n\r Test Command Transmitted Successful:  \r");
		for (int i = 0; i < 16; i++) {
			myPrintf(" %02x", tx_cmd[i]);
		}
		myPrintf("\n\n");
		pkt_id++;
		if (pkt_id > 5) {
			pkt_id = 0;
			HAL_TIM_Base_Stop(&htim1);
			myPrintf("\n\r Testing, Wait to receive:  \r\n");

			myPrintf("\n########## Receive and Transmits ##########\r\n");
			myPrintf("########## Low Power Radio Receiver: 	##########\r\n");
			myPrintf("########## COMMUNICATION PARAMETERS: 		##########\r\n");
			myPrintf("Modulation: LoRa PACKET\r\n");
			myPrintf(
					"FREQUENCY MODS: UPLINK FREQ: %lu\r\n    DOWNLINK FREQ: %lu\r\n",
					FREQ_402_MHZ, FREQ_401_MHZ);
			myPrintf(
					"POWER CONFIG:\r\n    PA_DUTY_CYCLE : %x,    HP_MAX: %x,\n\r    PA_SEL : %x,    POWER TX: %u dBm\n\r",
					PA_DUTY_CYCLE, HP_MAX, PA_SEL, POWER);
			myPrintf("RECEVING BANDWIDTH: 	%d\n\r",
					mod_params.Params.LoRa.Bandwidth);
			myPrintf("Packet Type: 			%d\n\r", pkt_params.PacketType);
			myPrintf("PayloadLength: 		%d\n\r",
					pkt_params.Params.LoRa.PayloadLength);
			myPrintf("PreambleLength: 		%d\n\r",
					pkt_params.Params.LoRa.PreambleLength);
			myPrintf("HeaderType: 			%d\n\r",
					pkt_params.Params.LoRa.HeaderType);
			myPrintf(
					"--------   If this message pop up then wait to receive   --------\r\n");
			myPrintf("__________________________________________________\r\n");
			SUBGRF_SetRfFrequency(FREQ_401_MHZ);
			SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX); /*Set RF switch*/
			SUBGRF_SetRxBoosted(0xFFFFFF);

			HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_SET);
			delay_us(1000000);  // 500ms delay
			HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_RESET);
			delay_us(1000000);
		}
	}
	if (radioIrq == IRQ_RX_DONE) {
		pkt_id++;
		SUBGRF_GetPayload(rx_cmd, &p_len, PAYLOAD_LENGTH);
		rssi_value = SUBGRF_GetRssiInst();
		myPrintf("\nLora Received\r\n");
		for (int i = 0; i < sizeof(rx_cmd); i++) {
			myPrintf(" %x", rx_cmd[i]);
		}
		myPrintf("\r\n");
		if (pkt_id > 5) {
			myPrintf("\n\rTesting complete, Receive and Transmit done.  \r\n");
			myPrintf("__________________________________________________\r\n");
			myPrintf(
					"________________Waiting to receive COM Command____________\r\n");
			myPrintf("SEND 'UART02' to test UART2 OBC'\r\n");
			myPrintf("SEND 'LPUART' to test LPUART OBC'\r\n");
			myPrintf("SEND '3RADIO' to test RFO LP Radio'\r\n");
			myPrintf("__________________________________________________\r\n");
			pkt_id = 0;
		}
	}
}

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

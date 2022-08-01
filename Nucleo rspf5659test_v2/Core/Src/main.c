/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
I2C_HandleTypeDef hi2c2;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t txBuffer[64] = {0};
uint8_t rxBuffer[64] = {0};
uint16_t uart_send = 1000;
uint16_t uart_send2 = 0;
uint8_t spi_tx_buf[10] = {0};
uint8_t spi_rx_buffer[2] = {0};
uint8_t spi_send = 0;
uint16_t i2c_send = 0, i2c2_send = 0;
uint16_t spi_buf;
uint8_t i2c_buf1, i2c_buf2, i2c_buf3, i2c_buf4;
uint16_t old_spi_buf = 0;
uint32_t i2c_buf, old_i2c_buf, i2c2_buf, old_i2c2_buf;

const unsigned char arr_crc_8[256]=
	{  0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
	 157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220,
		35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98,
	 190,224,  2, 92,223,76, 99, 61,124, 34,192,158, 29, 67,161,255,
		70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7,
	 219,133,103, 57,186,90,  6, 88, 25, 71,165,251,120, 38,196,154,
	 101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36,
	 248,166, 68, 26,153,199, 37,123, 58,53,134,216, 91,  5,231,185,
	 140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
		17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
	 175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
		50,108,142,208, 83, 13,78,177,240,174, 76, 18,145,207, 45,115,
	 202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139,
		87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
	 233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
	 116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53 };
	
	 const uint32_t SB[32] = {0x00000001,0x00000002,0x00000004,0x00000008,
							  0x00000010,0x00000020,0x00000040,0x00000080,
				              0x00000100,0x00000200,0x00000400,0x00000800,
							  0x00001000,0x00002000,0x00004000,0x00008000,
					          0x00010000,0x00020000,0x00040000,0x00080000,
							  0x00100000,0x00200000,0x00400000,0x00800000,
							  0x01000000,0x02000000,0x04000000,0x08000000,
						      0x10000000,0x20000000,0x40000000,0x80000000};
	 
	uint8_t led[15] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
	
	uint8_t i2c_led[32] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
	uint8_t i2c2_led[32] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t i,j = 0;//counter 
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
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  
	USART1->CR1 |= USART_CR1_RXNEIE;
	
	txBuffer[0] = 0x7E;
	txBuffer[1] = 0x00;
	txBuffer[2] = 0x00;
	txBuffer[3] = 0x01;
	txBuffer[4] = 0x02;
	txBuffer[5] = 0x03;
	txBuffer[6] = 0x04;
	txBuffer[7] = 0x00;
	txBuffer[8] = 0x00;
	txBuffer[20] = 0x7E;
	
	spi_tx_buf[0] = 0x09;
	spi_tx_buf[1] = 0x09;
	spi_tx_buf[2] = 0x09;
	spi_tx_buf[3] = 0x09;
	spi_tx_buf[4] = 0x09;
	spi_tx_buf[5] = 0x09;
	spi_tx_buf[6] = 0x09;
	spi_tx_buf[7] = 0x01;
	spi_tx_buf[8] = 0x00;
	spi_tx_buf[9] = 0x00;
		
	uint8_t i2c_rx_buffer1[10];
	uint8_t i2c_rx_buffer2[10];
	uint8_t i2c_rx_buffer3[10];
	uint8_t i2c_rx_buffer4[10];
	
	uint8_t i2c_tx_buffer[32];
	
	i2c_tx_buffer[0] = 0x24;
	i2c_tx_buffer[1] = 0x24;
	i2c_tx_buffer[2] = 0x24;
	i2c_tx_buffer[3] = 0x03;
	
	i2c_rx_buffer1[0] = 0;
	i2c_rx_buffer1[1] = 0;
	i2c_rx_buffer1[2] = 0;
	i2c_rx_buffer2[0] = 0;
	i2c_rx_buffer2[1] = 0;
	i2c_rx_buffer2[2] = 0;
	i2c_rx_buffer3[0] = 0;
	i2c_rx_buffer3[1] = 0;
	i2c_rx_buffer3[2] = 0;
	i2c_rx_buffer4[0] = 0;
	i2c_rx_buffer4[1] = 0;
	i2c_rx_buffer4[2] = 0;
		
	uint8_t pwm = 0;
	uint8_t packet_length = 0;
	
	for (uint8_t i = 0;i<15;i++)
	{
		txBuffer[i] = 0;
	}
	txBuffer[0] = 0x7E;
	txBuffer[1] = 0x00;
	txBuffer[2] = 0x01;
	txBuffer[3] = 0x01;
	txBuffer[4] = 0x01;
	txBuffer[5] = 0x01;
	txBuffer[6] = 0x01;
	txBuffer[7] = 0xFF;
	for (uint8_t i=1;i<7;i++)
	{
		txBuffer[7] = arr_crc_8[txBuffer[7] ^ txBuffer[i]]; 
	}
	if (txBuffer[7] == 0x7E)
	{
		txBuffer[7] = 0x7D;
		txBuffer[8] = 0x5E;
		txBuffer[9] = 0x7E;
		packet_length = 10;
	}
	else if (txBuffer[7] == 0x7D)
	{
		txBuffer[7] = 0x7D;
		txBuffer[8] = 0x5D;
		txBuffer[9] = 0x7E;
		packet_length = 10;
	}
	else
	{
		txBuffer[8] = 0x7E;
		packet_length = 9;
	}
	//sending a package to UART1
	HAL_UART_Transmit(&huart1, txBuffer, packet_length, 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//processor clock delay
	for(i = 0; i < 18; i++)
	{
		__nop();
	}

		if (!uart_send) 
		{
   // RCC->AHBRSTR 
      RCC->APB1RSTR |= RCC_APB1RSTR_I2C2RST;
      RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C2RST;
      MX_I2C2_Init();
			uart_send = 50;
			//HAL_UART_Transmit(&huart3, &txBuffer[20], 1, 100); // test 
			USART1->CR1 |= USART_CR1_RXNEIE;
      
			GPIOB->BSRR |= GPIO_BSRR_BR_6;
			//processor clock delay
			for(i = 0; i < 4; i++)
			{
				__nop();
			}
			HAL_SPI_Receive(&hspi1, spi_rx_buffer,2 , 10);
			GPIOB->BSRR |= GPIO_BSRR_BS_6;
				
			spi_buf = spi_rx_buffer[1] | (spi_rx_buffer[0]<<8);
			if (spi_buf != old_spi_buf)
			{
				old_spi_buf = spi_buf;
				spi_send = 1;
			}
			//processor clock delay
			for(i = 0; i < 10; i++)
			{
				__nop();
			}
				
			//out for 56 and 57
			HAL_I2C_Master_Receive(&hi2c2, 0x0F<<1, i2c_rx_buffer2, 1, 10);
			__nop();
			HAL_I2C_Master_Receive(&hi2c2, 0x0E<<1, i2c_rx_buffer1, 1, 10);
			HAL_I2C_Master_Receive(&hi2c2, 0x0D<<1, i2c_rx_buffer4, 1, 10);
			__nop();
			HAL_I2C_Master_Receive(&hi2c2, 0x0C<<1, i2c_rx_buffer3, 1, 10);
			HAL_I2C_DeInit(&hi2c2);
			MX_I2C2_Init();
					
			i2c_buf =(i2c_rx_buffer1[0]) | (i2c_rx_buffer2[0]<<6);
			if (i2c_buf != old_i2c_buf)
			{
				i2c_send = 1;
				old_i2c_buf = i2c_buf;
			}
				
			i2c2_buf =(i2c_rx_buffer3[0]) | (i2c_rx_buffer4[0]<<8);
			if (i2c2_buf != old_i2c2_buf)
			{
				i2c2_send = 1;
				old_i2c2_buf = i2c2_buf;
			}
		}
//		if (!uart_send2)
//		{
//			uart_send2 = 2000;
//			
//		}
		
		//spi packet handle
		if (spi_send)
		{
			spi_send = 0;
			for(uint8_t i=0;i<15;i++)
			{
				if (spi_buf & SB[i])
				{
					led[i]++;
					if (led[i] == 5) led[i] = 1;
				}
			}
			spi_tx_buf[0] = led[0] | (led[1]<<3);
			spi_tx_buf[1] = led[2] | (led[3]<<3);
			spi_tx_buf[2] = led[4] | (led[5]<<3);
			spi_tx_buf[3] = led[6] | (led[7]<<3);
			spi_tx_buf[4] = led[8] | (led[9]<<3);
			spi_tx_buf[5] = led[10] | (led[11]<<3);
			spi_tx_buf[6] = led[12] | (led[13]<<3);
			spi_tx_buf[7] = led[14] | (pwm<<3);
			spi_tx_buf[8] = 0;
			
			
			GPIOC->BSRR |= GPIO_BSRR_BR_7;
			//processor clock delay
			for(i = 0; i < 4; i++)
			{
				__nop();
			}
			HAL_SPI_Transmit(&hspi2, spi_tx_buf, 9, 100);
			//processor clock delay
			for(i = 0; i < 4; i++)
			{
				__nop();
			}
			GPIOC->BSRR |= GPIO_BSRR_BS_7;
		}
		
		//i2c packet handle
		if (i2c_send)
		{
			i2c_send = 0;
			
			for(uint8_t i=0;i<12;i++)
			{
				if (i2c_buf & SB[i])
				{
					i2c_led[i]++;				
					if (i2c_led[i] == 5) i2c_led[i] = 1;
				}
			}
			
			i2c_tx_buffer[0] = i2c_led[0] | (i2c_led[1]<<3);
			i2c_tx_buffer[1] = i2c_led[2] | (i2c_led[3]<<3);
			i2c_tx_buffer[2] = i2c_led[4] | (i2c_led[5]<<3);
			i2c_tx_buffer[3] = pwm;
			i2c_tx_buffer[4] = i2c_led[6] | (i2c_led[7]<<3);
			i2c_tx_buffer[5] = i2c_led[8] | (i2c_led[9]<<3);
			i2c_tx_buffer[6] = i2c_led[10] | (i2c_led[11]<<3);
			i2c_tx_buffer[7] = pwm;
			
			
			HAL_I2C_Master_Transmit(&hi2c2, (0x0F<<1) | 0x01, i2c_tx_buffer+4, 4, 100);
			HAL_I2C_Master_Transmit(&hi2c2, (0x0E<<1) | 0x01, i2c_tx_buffer, 4, 100);
		}
		
		
		if (i2c2_send)
		{
			i2c2_send = 000;
			
			for(uint8_t i=0;i<14;i++)
			{
				if (i2c2_buf & SB[i])
				{
					i2c2_led[i]++;
					if(i == 5 || i == 6 || i == 7)
					{
						i2c2_led[4]++;
						if(i2c2_led[4] == 5) i2c2_led[4] = 1;
					}							
					if (i2c2_led[i] == 5) i2c2_led[i] = 1;
				}
			}
			
			
			i2c_tx_buffer[8] = i2c2_led[0] | (i2c2_led[1]<<3);
			i2c_tx_buffer[9] = i2c2_led[2] | (i2c2_led[3]<<3);
			i2c_tx_buffer[10] = i2c2_led[4] | (i2c2_led[5]<<3);
			i2c_tx_buffer[11] = pwm;
			i2c_tx_buffer[12] = i2c2_led[8] | (i2c2_led[9]<<3);
			i2c_tx_buffer[13] = i2c2_led[10] | (i2c2_led[11]<<3);
			i2c_tx_buffer[14] = i2c2_led[12] | (i2c2_led[13]<<3);
			i2c_tx_buffer[15] = pwm;
			
			//out for 56 and 57			
			HAL_I2C_Master_Transmit(&hi2c2, (0x0D<<1) | 0x01, i2c_tx_buffer+12, 4, 100);
			HAL_I2C_Master_Transmit(&hi2c2, (0x0C<<1) | 0x01, i2c_tx_buffer+8, 4, 100);
		}
    HAL_IWDG_Refresh(&hiwdg);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x9010DEFF;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

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
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CSS2_GPIO_Port, CSS2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CSS1_GPIO_Port, CSS1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CSS2_Pin */
  GPIO_InitStruct.Pin = CSS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CSS2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CSS1_Pin */
  GPIO_InitStruct.Pin = CSS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CSS1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//uart packet handle
void UartRxAnalyze(void)
{
	uint8_t packet_length;
	uint8_t send_flag;
	switch(rxBuffer[1])
	{
		case 1:
			if(rxBuffer[2]==1) txBuffer[2]++;
			if(txBuffer[2]==5) txBuffer[2] = 1;
			send_flag = 1;
		break;
		case 2:
			if(rxBuffer[2]==1) txBuffer[3]++;
			if(txBuffer[3]==5) txBuffer[3] = 1;
			send_flag = 1;
		break;
		case 3:
        if(rxBuffer[2]==1) txBuffer[4]++;
        if(txBuffer[4]==5) txBuffer[4] = 1;
        send_flag = 1;
		break;
		case 4:
			if(rxBuffer[2]==1) txBuffer[5]++;
			if(txBuffer[5]==5) txBuffer[5] = 1;
			send_flag = 1;
		break;
		case 5:
			if(rxBuffer[2]==1) txBuffer[6]++;
			if(txBuffer[6]==5) txBuffer[6] = 1;
			send_flag = 1;
		break;
		case 6:
			if(rxBuffer[2]==1) txBuffer[6]++;
			if(txBuffer[6]==5) txBuffer[6] = 1;
			send_flag = 1;
		break;
		case 7:
			if(rxBuffer[2]==1) txBuffer[6]++;
			if(txBuffer[6]==5) txBuffer[6] = 1;
			send_flag = 1;
		break;
		default:
			
		break;
	}
	if (send_flag)
	{
		txBuffer[7] = 0xFF;
		for (uint8_t i=1;i<7;i++)
		{
			txBuffer[7] = arr_crc_8[txBuffer[7] ^ txBuffer[i]]; 
		}
		if (txBuffer[7] == 0x7E)
		{
			txBuffer[7] = 0x7D;
			txBuffer[8] = 0x5E;
			txBuffer[9] = 0x7E;
			packet_length = 10;
		}
		else if (txBuffer[7] == 0x7D)
		{
			txBuffer[7] = 0x7D;
			txBuffer[8] = 0x5D;
			txBuffer[9] = 0x7E;
			packet_length = 10;
		}
		else
		{
			txBuffer[8] = 0x7E;
			packet_length = 9;
		}
		HAL_UART_Transmit(&huart1, txBuffer, packet_length, 100);
		send_flag = 0;
	}
	
}
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


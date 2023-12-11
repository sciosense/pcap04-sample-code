/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
//#define IIC_EN

#ifndef IIC_EN
#include "user_spi_interface.c"
#else
#include "user_i2c_interface.c"
#endif

//#include "user_tools.c"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define WR_MEM 		0xA0
#define RD_MEM 		0x20
#define WR_CONFIG 	0xA3C0	//byte wise
#define RD_CONFIG 	0x23C0	//byte wise
#define RD_RESULT 	0x40

#define POR 		0x88
#define INIT 		0x8A
#define CDC_START 	0x8C
#define RDC_START 	0x8E
#define DSP_TRIG 	0x8D
#define NV_STORE 	0x96
#define NV_RECALL 	0x99
#define NV_ERASE 	0x9C
#define TEST_READ 	0x7E

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

#ifdef IIC_EN
HAL_StatusTypeDef ret = HAL_OK;
volatile uint16_t dev_addr = 0x50; //Device Address with R/W Bit 0-1-0-1-0-A1-A0 R/W or without (0x28)
#endif

volatile uint32_t  My_INTN_Counter = 0;
volatile uint8_t   My_INTN_State = 1;

volatile uint32_t MyRawRES0 = 0;
volatile uint32_t MyRawRES1 = 0;
volatile uint32_t MyRawRES2 = 0;
volatile float MyRatioRES0 = 0;
volatile float MyRatioRES1 = 0;
volatile float MyRatioRES2 = 0;

uint32_t My_i2c_timeout = 10;
uint8_t  My_buf[256] = {0x12,0x34};
uint32_t My_ratio = 0;

// Default Configuration <pcap04_default.cfg>, Bytewise
uint8_t  standard_cfg_bytewise[52] = {
		0x1D,0x00,0x58,0x10,   // Register 0, 1, 2, 3
		0x10,0x00,0x0F,0x20,   // Register 4, 5, 6, 7
		0x00,0xD0,0x07,0x00,   // Register 8, 9, 10, 11
		0x00,0x08,0xFF,0x03,   // Register 12, 13, 14, 15
		0x00,0x24,0x00,0x00,   // Register 16, 17, 18, 19
		0x00,0x01,0x50,0x30,   // Register 20, 21, 22, 23
		0x73,0x04,0x50,0x08,   // Register 24, 25, 26, 27
		0x5A,0x00,0x82,0x08,   // Register 28, 29, 30, 31
		0x08,0x00,0x47,0x40,   // Register 32, 33, 34, 35
		0x00,0x00,0x00,0x71,   // Register 36, 37, 38, 39
		0x00,0x00,0x08,0x00,   // Register 40, 41, 42, 43
		0x00,0x00,0x00,0x01,   // Register 44, 45, 46, 47
		0x00,0x00,0x00,0x00    // Register 48, 49, 50, 51
};

// Default Configuration <pcap04_default.cfg>, when Dword is used.
// Is not used in this example!
uint32_t standard_cfg[] = {
		0x1058001D,   // Register 3, 2, 1, 0
		0x200F0010,   // Register 7, 6, 5, 4
		0x0007D000,   // Register 11, 10, 9, 8
		0x03FF0800,   // Register 15, 14, 13, 12
		0x00002400,   // Register 19, 18, 17, 16
		0x30500100,   // Register 23, 22, 21, 20
		0x08500473,   // Register 27, 26, 25, 24
		0x0882005A,   // Register 31, 30, 29, 28
		0x40470008,   // Register 35, 34, 33, 32
		0x71000000,   // Register 39, 38, 37, 36
		0x00080000,   // Register 43, 42, 41, 40
		0x01000000,   // Register 47, 46, 45, 44
		0x00000000   // Register 51, 50, 49, 48
};

// Standard Firmware, <PCap04_standard_v1.hex>, Bytewise
// 34 Rows x 16 Bytes  +  1 Rows x 4 Bytes  = 548 Bytes
uint8_t standard_fw[548] = {
		0x24, 0x05, 0xA0, 0x01, 0x20, 0x55, 0x42, 0x5C, 0x48, 0xB1, 0x07, 0x92, 0x02, 0x20, 0x13, 0x02,
		0x20, 0x93, 0x02, 0xB2, 0x02, 0x78, 0x20, 0x54, 0xB3, 0x06, 0x91, 0x00, 0x7F, 0x20, 0x86, 0x20,
		0x54, 0xB6, 0x03, 0x72, 0x62, 0x20, 0x54, 0xB7, 0x00, 0x00, 0x42, 0x5C, 0xA1, 0x00, 0x49, 0xB0,
		0x00, 0x49, 0x40, 0xAB, 0x5D, 0x92, 0x1C, 0x90, 0x02, 0x7F, 0x20, 0x86, 0x66, 0x67, 0x76, 0x77,
		0x66, 0x7A, 0xCF, 0xCD, 0xE6, 0x43, 0xF1, 0x44, 0x29, 0xE0, 0x7A, 0xDC, 0xE7, 0x41, 0x32, 0xAA,
		0x01, 0x99, 0xFD, 0x7B, 0x01, 0x7A, 0xCF, 0xEB, 0xE6, 0x43, 0xF1, 0x44, 0x29, 0xE0, 0x7A, 0xC1,
		0xE7, 0x41, 0x32, 0x6A, 0xDE, 0x44, 0x7A, 0xCF, 0xEA, 0xE6, 0x43, 0xF1, 0x44, 0x29, 0xE0, 0x6A,
		0xDF, 0x44, 0x7A, 0xC4, 0xE7, 0x41, 0x32, 0xAB, 0x05, 0x7A, 0xC1, 0xE1, 0x43, 0xE0, 0x3A, 0x7A,
		0xC0, 0xE1, 0x43, 0xE0, 0x3A, 0x02, 0x7A, 0xCF, 0xE6, 0xE6, 0x43, 0xF1, 0x44, 0x29, 0xE0, 0x7A,
		0xEF, 0x44, 0x02, 0x20, 0x9D, 0x84, 0x01, 0x21, 0x2E, 0x21, 0x74, 0x20, 0x37, 0xC8, 0x7A, 0xE7,
		0x43, 0x49, 0x11, 0x6A, 0xD4, 0x44, 0x7A, 0xC1, 0xD8, 0xE6, 0x43, 0xE9, 0x44, 0x1C, 0x43, 0x13,
		0xAB, 0x63, 0x6A, 0xDE, 0x41, 0xAB, 0x0B, 0x46, 0x46, 0x46, 0x7A, 0xDF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xE3, 0x41, 0x32, 0x1C, 0x44, 0xE9, 0x13, 0x6A, 0xD4, 0x13, 0x41, 0xAA, 0xDF, 0x7A, 0xC5, 0xE1,
		0x43, 0x49, 0xE0, 0x34, 0x7A, 0xCF, 0xE3, 0xE6, 0x43, 0xF1, 0x44, 0x29, 0xE0, 0xDB, 0xC0, 0x27,
		0xE5, 0x6A, 0xDF, 0x43, 0x7A, 0xC8, 0xE7, 0x41, 0x30, 0xAB, 0x03, 0x86, 0x01, 0x92, 0x37, 0x7A,
		0xC6, 0xE7, 0x41, 0x7A, 0xFA, 0xE7, 0x43, 0xEA, 0x44, 0x7A, 0xC1, 0xE1, 0xE6, 0x43, 0xE9, 0x44,
		0x25, 0xE0, 0x7A, 0xC6, 0xE7, 0x41, 0x7A, 0xFA, 0xE7, 0x43, 0xEA, 0x44, 0x7A, 0xC0, 0xE7, 0x43,
		0xE9, 0x44, 0x25, 0xE0, 0x92, 0x10, 0x7A, 0xE1, 0x44, 0xE2, 0x44, 0xE3, 0x44, 0xE4, 0x44, 0xE5,
		0x44, 0xE6, 0x44, 0xE7, 0x44, 0xE8, 0x44, 0xC1, 0xD8, 0x24, 0x3E, 0x92, 0xFF, 0x02, 0x7A, 0xCF,
		0xD7, 0xE6, 0x43, 0xF1, 0x44, 0x7A, 0xD0, 0xE7, 0x43, 0x2A, 0x2A, 0x32, 0xAB, 0x03, 0x42, 0x5C,
		0x92, 0x03, 0x7A, 0xC0, 0xE1, 0x43, 0xD9, 0x27, 0x90, 0x6A, 0xDF, 0x43, 0x7A, 0xC8, 0xE7, 0x41,
		0x32, 0xAB, 0x03, 0x86, 0x01, 0x92, 0x11, 0x7A, 0xC2, 0x43, 0x7A, 0xE7, 0x44, 0x6A, 0xC6, 0x44,
		0x7A, 0xC3, 0x43, 0x7A, 0xE8, 0x44, 0x6A, 0xC7, 0x44, 0xC1, 0xD4, 0x24, 0x57, 0x7A, 0xC8, 0xE1,
		0x43, 0xE0, 0x3A, 0x02, 0x7A, 0xCF, 0xE7, 0xE6, 0x43, 0xF1, 0x44, 0x29, 0xE0, 0x7A, 0xC7, 0xE1,
		0x41, 0x6A, 0xD4, 0x45, 0x5A, 0x25, 0x36, 0x46, 0x46, 0x46, 0x46, 0x7A, 0xE9, 0x44, 0x7A, 0xC0,
		0xE7, 0x43, 0x55, 0x7A, 0xEA, 0x45, 0x7A, 0xE9, 0x51, 0x1C, 0x43, 0x6A, 0xCA, 0x44, 0x1D, 0x43,
		0x6A, 0xCB, 0x44, 0x7A, 0xC1, 0xCA, 0xE6, 0x43, 0xE9, 0x44, 0x7A, 0xC1, 0xE1, 0x43, 0x7A, 0xCC,
		0xE0, 0xE6, 0x41, 0x2C, 0x42, 0x7A, 0xC5, 0xE1, 0x43, 0x49, 0xE0, 0x34, 0x7A, 0xC1, 0xCC, 0xE6,
		0x43, 0xE9, 0x44, 0x7A, 0xC1, 0xE1, 0x43, 0x2C, 0x70, 0x7A, 0xCC, 0x43, 0x7A, 0xCF, 0x44, 0x7A,
		0xCD, 0x43, 0x7A, 0xCE, 0x44, 0x6A, 0xCA, 0x43, 0xC1, 0xCA, 0x7A, 0xE6, 0x41, 0xE9, 0x45, 0x2B,
		0xAE, 0xEE, 0x44, 0x7A, 0xC1, 0xCA, 0xE6, 0x43, 0xE9, 0x44, 0x7A, 0xC1, 0xE1, 0x43, 0x7A, 0xCC,
		0xEC, 0xE6, 0x41, 0x2C, 0x42, 0x7A, 0xC5, 0xE1, 0x43, 0x49, 0xE0, 0x34, 0x7A, 0xC1, 0xCC, 0xE6,
		0x43, 0xE9, 0x44, 0x7A, 0xC1, 0xE1, 0x43, 0x2C, 0x70, 0x7A, 0xCC, 0x43, 0x7A, 0xCF, 0x44, 0x7A,
		0xCD, 0x43, 0x7A, 0xCE, 0x44, 0x6A, 0xCB, 0x43, 0xC1, 0xCA, 0x7A, 0xE6, 0x41, 0xE9, 0x45, 0x2B,
		0xAE, 0xED, 0x44, 0x02
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* PIN Assignment of STM32L476:
   * SPI1_MOSI PA7
   * SPI1_MISO PA6
   * SPI1_SCK  PA5
   * SPI1_SSN  PB6
   *
   * I2C1_SCL  PB8
   * I2C1_SDA  PB7
   *
   * INTN      PA9
   */


  /* Test SPI/I2C Communication Interface by sending Test Opcode
   * Write opcode 0x7e to SIF and read 1 byte. Compare this byte to following patterns:
   * 0x11: Expected value, read cycle performed correctly
   * 0x88: Failure: there is a big/little-endian swap
   * 0xEE: Failure: during read cycle all bits are inverted
   * 0x77: Failure: inverted bits and bit/little-endian swap
   */
#ifndef IIC_EN
  My_buf[0] = Read_Byte2(TEST_READ);
#else
  ret = I2C_Write_Opcode(dev_addr, TEST_READ);
  ret = HAL_I2C_Master_Receive(&hi2c1, dev_addr, My_buf, 1, 1);
#endif

  // POR + INIT
#ifndef IIC_EN
  Write_Opcode(POR);
  HAL_Delay(500);
  Write_Opcode(INIT);
#else
  ret = I2C_Write_Opcode(dev_addr, POR);
  HAL_Delay(500);
  ret = I2C_Write_Opcode(dev_addr, INIT);
#endif
  // Delay after INIT
  HAL_Delay(10);

  // Write firmware with additional write verification of e.g. 100 bytes
#ifndef IIC_EN
  Write_Byte_Auto_Incr(WR_MEM, 0x00, standard_fw, 548);
  Read_Byte_Auto_Incr(RD_MEM, 0x00, My_buf, 100);
#else
  ret = I2C_Memory_Access(dev_addr, WR_MEM, 0x00, standard_fw, 548);
  ret = I2C_Memory_Access(dev_addr, RD_MEM, 0x00, My_buf, 100);
#endif

  // Write configuration (52 Bytes) with additional write verification
#ifndef IIC_EN
  Write_Byte_Auto_Incr(WR_CONFIG, 0x00, standard_cfg_bytewise, 52);
  Read_Byte_Auto_Incr(RD_CONFIG, 0x00, My_buf, 52);

  Write_Opcode(INIT);
#else
  ret = I2C_Config_Access(dev_addr, WR_CONFIG, 0x00, standard_cfg_bytewise, 52);
  ret = I2C_Config_Access(dev_addr, RD_CONFIG, 0x00, My_buf, 52);

  ret = I2C_Write_Opcode(dev_addr, INIT);
#endif

  // Start CDC measurement
#ifndef IIC_EN
  Write_Opcode(CDC_START);
#else
  ret = I2C_Write_Opcode(dev_addr, CDC_START);
#endif


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // Read result register after INTN = 0
if (My_INTN_State==0) {
#ifndef IIC_EN
	  MyRawRES0 = Read_Dword_Lite(0x40, 0x00);
	  MyRawRES1 = Read_Dword_Lite(0x40, 0x04);
#else
	  MyRawRES0 = I2C_Read_Result(dev_addr, 0x40, 0x00);
	  MyRawRES1 = I2C_Read_Result(dev_addr, 0x40, 0x04);
#endif
}

	  //Post Processing
	  MyRatioRES0 = (float)MyRawRES0 / 134217728; // = 2^27
	  MyRatioRES1 = (float)MyRawRES1 / 134217728; // = 2^27

	  HAL_Delay(50); // used for debugging

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SSN_GPIO_Port, SSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : INTN_Pin */
  GPIO_InitStruct.Pin = INTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SSN_Pin */
  GPIO_InitStruct.Pin = SSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SSN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/* stm32l4xx_hal_gpio.c */

	/* Prevent unused argument(s) compilation warning */
	UNUSED(GPIO_Pin);

	// Note: It takes about 1us after INTN

	if (GPIO_Pin == INTN_Pin) {
		My_INTN_State = (HAL_GPIO_ReadPin(INTN_GPIO_Port, INTN_Pin) == GPIO_PIN_SET); /* low active */
		if (My_INTN_State == 0) {
			My_INTN_Counter += 1;
		}
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

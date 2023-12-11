/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    hi2c1_interface.c
  * @brief   I2C Interface Communication Routines, including UART.
  *
  @verbatim
  ==============================================================================
                     ##### How to use this driver #####
  ==============================================================================
    SPI Timings
      SSN enable to valid latch clock = min. 1.7us (2.6us)
      SSN hold time after SCK falling = min. 2.2us (3.6us)

    Timing in between the bytes
      Transmitting incremental = no additional gap           (recommended usage)
      Transmitting bytewise = approx. 3.5us

      Receiving incremental = approx. 1us                    (recommended usage)
      Receiving bytewise = approx. 5us

    Transmitting and Receiving Examples
    //declaration
      uint8_t timeout = 10; //Timeout duration in millisecond [ms]
      uint8_t spiTX[3];
      uint8_t spiRX[3];
      spiTX[0]=0xAA;
      spiTX[1]=0xBB;
      spiTX[2]=0xCC;

    //Example: transmitting bytes without gap in between           (recommended)
      Set_SSN(LOW);
      HAL_SPI_Transmit(&hspi1, spiTX, 3, timeout);
      Set_SSN(HIGH);

    //Example: transmitting bytewise with gap of approx. 3.5us in between
      Set_SSN(LOW);
      HAL_SPI_Transmit(&hspi1, &spiTX[0], 1, timeout);
      HAL_SPI_Transmit(&hspi1, &spiTX[1], 1, timeout);
      HAL_SPI_Transmit(&hspi1, &spiTX[2], 1, timeout);
      Set_SSN(HIGH);

    //Example: receiving bytewise with gap of ~1us in between      (recommended)
      Set_SSN(LOW);
      HAL_SPI_Receive(&hspi1, spiRX, 3, timeout); //gap between read bytes 1us
      Set_SSN(HIGH);

    //Example: receiving bytewise with gap of ~5us in between
      Set_SSN(LOW);
      HAL_SPI_Receive(&hspi1, &spiRX[0], 1, timeout);
      HAL_SPI_Receive(&hspi1, &spiRX[1], 1, timeout);
      HAL_SPI_Receive(&hspi1, &spiRX[2], 1, timeout);
      Set_SSN(HIGH);

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
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
#include "inc/user_i2c_interface.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern I2C_HandleTypeDef hi2c1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*                            Sweep Device Address                            */
/******************************************************************************/
/**
  * @brief  sweep device address to find the address of connected slave
  * @param  from_addr (7 bit) start at device address
  * @param  to_addr (7 bit) end at devie address
  * @param  *addr_array returns the found device address(es)
  * @retval none
  *
  * @verbatim
  ==============================================================================
                     ##### How to use this routine #####
  ==============================================================================
	uint8_t slave[1] = 0;
	I2C_Sweep_DevAddr(0, 127, slave);
	dev_addr = slave[0];
  */
void I2C_Sweep_DevAddr(uint8_t from_addr, uint8_t to_addr, uint8_t *addr_array)
{
	uint8_t index = 0;
	/* 3 is number of trials, 1ms is timeout */
	uint8_t timeout = 1;
	uint8_t trials = 3;

	for (int check_addr = from_addr; check_addr <= to_addr; check_addr++)
	{
		/* Checks if target device is ready for communication. */
		if (HAL_I2C_IsDeviceReady(&hi2c1, check_addr, trials, timeout) != HAL_OK) {
			check_addr++;
			HAL_Delay(1);
			/* Return error */
			//return HAL_ERROR;
		} else {
		//printf("device address = 0x%02X", check_addr);
		addr_array[index] = check_addr;
		index++;
		// End FOR-Loop
		break;
		}
	}
}

/******************************************************************************/
/*                            Write one byte Opcode                           */
/******************************************************************************/
/**
  * @brief  Write one byte Opcode.
  * @param  slave (7 Bit) decive address
  * @param  one_byte
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef I2C_Write_Opcode(uint8_t slave, uint8_t one_byte)
{
	/* Timeout duration in millisecond [ms] */
	uint8_t timeout = 1;
	uint8_t i2cTX[1];

	i2cTX[0] = one_byte;

	/* 1. Transmit register address */
	while(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)slave, i2cTX, 1, timeout) != HAL_OK ) {
				/* Error_Handler() function is called when Timeout error occurs.
				   When Acknowledge failure occurs (Slave don't acknowledge it's address)
				   Master restarts communication */
				if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
				{
				  Error_Handler();
				}
			}

	while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);

	return HAL_OK;
}

/******************************************************************************/
/*                             Memory Access                            */
/******************************************************************************/
/**
  * @brief  Write one byte.
  * @param  opcode (byte)
  * @param  address (byte)
  * @param  byte (byte)
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef I2C_Memory_Access(uint8_t slave, uint8_t opcode, uint16_t address, uint8_t *byte, uint16_t size)
{
	//opcodes for memory access
	//WR = 0xA0
	//RD = 0x20

	/* Calculation of max timeout using I2C frequency = 100kHz
	 *  ( 1 / 100kHz ) x (2 + size) x (8 bit + ACK bit) = minimum timeout
	 *   */
	/* Timeout duration in millisecond [ms] */
	float savety_factor = 1.2e3; //plus 10% and to get timeout value in [ms]
	uint32_t timeout = (( 1 / 100e3 ) * (2 + size) * 9) * savety_factor; //1000;

	if (opcode == 0xA0)
	{
		uint8_t i2cTX[size + 2];

		i2cTX[0] = opcode | (uint8_t)(address>>8);
		i2cTX[1] = (uint8_t)address;

		for (int i = 0; i < size; i++)
		{
			i2cTX[2 + i] = byte[i];
		}
		//WR
		/* Send WHO_AM_I register address */
		while(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)slave,  i2cTX, (2 + size), timeout) != HAL_OK ) {
			/* Error_Handler() function is called when Timeout error occurs.
			   When Acknowledge failure occurs (Slave don't acknowledge it's address)
			   Master restarts communication */
			if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
			{
			  Error_Handler();
			}
		}
		while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);

	} else {
		uint8_t i2cTX[2];
		uint8_t i2cRX[size];

		i2cTX[0] = opcode | (uint8_t)(address>>8);
		i2cTX[1] = address;

		//RD
		/* Send WHO_AM_I register address */
		while(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)slave, i2cTX, 2, timeout) != HAL_OK) {
			/* Error_Handler() function is called when Timeout error occurs.
			   When Acknowledge failure occurs (Slave don't acknowledge it's address)
			   Master restarts communication */
			if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
			{
			  Error_Handler();
			}
		}
		while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);

		/* Receieve data in the register */
		while(HAL_I2C_Master_Receive(&hi2c1, (uint16_t)slave, i2cRX, size, timeout) != HAL_OK) {
			/* Error_Handler() function is called when Timeout error occurs.
			   When Acknowledge failure occurs (Slave don't acknowledge it's address)
			   Master restarts communication */
			if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
			{
			  Error_Handler();
			}
		}
		while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);

		//copy array
		for (int i = 0; i < size; i++)
		{
			byte[i] = i2cRX[i];
		}

	}

	return HAL_OK;
}

/******************************************************************************/
/*                         Configuration Access                         */
/******************************************************************************/
/**
  * @brief  Write one double word.
  * @param  slave (7 Bit) decive address
  * @param  opcode (byte)
  * @param  address (byte)
  * @param  dword (double word)
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef I2C_Config_Access(uint8_t slave, uint32_t opcode, uint8_t address, uint8_t *byte, uint8_t size)
{
	//opcodes for memory access
	//WR_CFG = 0xA3C0
	//RD_CFG = 0x23C0

	/* Timeout duration in millisecond [ms] */
	uint8_t timeout = 100;
	uint8_t i2cTX[256];
	uint8_t i2cRX[256];

	i2cTX[0] = (uint8_t)(opcode>>8);
	i2cTX[1] = address | (uint8_t)opcode;

	if (opcode == 0xA3C0)
	{
		if (size == 1)
		{
			i2cTX[2] = byte[0];
		} else {
			for (int i = 0; i < size; i++)
			{
				i2cTX[2 + i] = byte[i];
			}
		}
		//WR
		while(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)slave,  i2cTX, (2 + size), timeout) != HAL_OK) {
			/* Error_Handler() function is called when Timeout error occurs.
			   When Acknowledge failure occurs (Slave don't acknowledge it's address)
			   Master restarts communication */
			if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
			{
			  Error_Handler();
			}
		}
		while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);

	} else {
		//RD
		while(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)slave, i2cTX, 2, timeout) != HAL_OK) {
			/* Error_Handler() function is called when Timeout error occurs.
			   When Acknowledge failure occurs (Slave don't acknowledge it's address)
			   Master restarts communication */
			if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
			{
			  Error_Handler();
			}
		}
		while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);

		while(HAL_I2C_Master_Receive(&hi2c1, (uint16_t)slave, i2cRX, size, timeout) != HAL_OK) {
			/* Error_Handler() function is called when Timeout error occurs.
			   When Acknowledge failure occurs (Slave don't acknowledge it's address)
			   Master restarts communication */
			if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
			{
			  Error_Handler();
			}
		}
		while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);

		//copy array
		for (int i = 0; i < size; i++)
		{
			byte[i] = i2cRX[i];
		}

	}

	return HAL_OK;
}

/******************************************************************************/
/*                         Write one data double word                         */
/******************************************************************************/
/**
  * @brief  Write one double word.
  * @param  slave (7 Bit) decive address
  * @param  opcode (byte)
  * @param  address (byte)
  * @param  dword (double word)
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef I2C_Write_Dword(uint8_t slave, uint8_t opcode, uint8_t address, uint32_t dword)
{
	/* Timeout duration in millisecond [ms] */
	uint8_t timeout = 10;
	uint8_t i2cTX[6];
	uint32_t temp_u32 = 0;

	i2cTX[0] = opcode;
	i2cTX[1] = address;
	temp_u32 = dword;
	i2cTX[5] = temp_u32>>24;
	i2cTX[4] = temp_u32>>16;
	i2cTX[3] = temp_u32>>8;
	i2cTX[2] = temp_u32;

    /* 1. Transmit register address */
	while(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)slave,  i2cTX, 6, timeout) != HAL_OK) {
		/* Error_Handler() function is called when Timeout error occurs.
		   When Acknowledge failure occurs (Slave don't acknowledge it's address)
		   Master restarts communication */
		if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
		{
		  Error_Handler();
		}
	}
	while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);

	return HAL_OK;
}

/******************************************************************************/
/*                             Write one data byte                            */
/******************************************************************************/
/**
  * @brief  Write one byte.
  * @param  opcode (byte)
  * @param  address (byte)
  * @param  byte (byte)
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef I2C_Write_Byte(uint8_t slave, uint8_t opcode, uint8_t address, uint8_t byte)
{
	/* Timeout duration in millisecond [ms] */
	uint8_t timeout = 10;
	uint8_t i2cTX[3];

	i2cTX[0] = opcode;
	i2cTX[1] = address;
	i2cTX[2] = byte;

  	/* 1. Transmit register address */
	while(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)slave,  i2cTX, 3, timeout) != HAL_OK) {
		/* Error_Handler() function is called when Timeout error occurs.
		   When Acknowledge failure occurs (Slave don't acknowledge it's address)
		   Master restarts communication */
		if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
		{
		  Error_Handler();
		}
	}
	while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);

	return HAL_OK;
}

/******************************************************************************/
/*                              Read double word                              */
/******************************************************************************/
/**
  * @brief  Read double word.
  * @param  opcode (byte)
  * @param  address (byte)
  * @retval 32-bit value
  */
uint32_t I2C_Read_Dword(uint8_t slave, uint8_t rd_opcode, uint8_t address)
{
  /* Timeout duration in millisecond [ms] */
  uint8_t timeout = 10;
  uint8_t i2cTX[2];
  uint8_t i2cRX[4];
  uint32_t temp_u32 = 0;

  i2cTX[0] = rd_opcode;
  i2cTX[1] = address;

  /* 1. Transmit register address */
  HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)slave, i2cTX, 2, timeout);

  /* 2. Read four bytes */
  HAL_I2C_Master_Receive(&hi2c1, (uint16_t)slave, i2cRX, 4, timeout);

  /* Concatenate of bytes (from MSB to LSB) */
  temp_u32 = (i2cRX[0]<<24) + (i2cRX[1]<<16) + (i2cRX[2]<<8) + (i2cRX[3]);

  return temp_u32;
}

/******************************************************************************/
/*                                  Read byte                                 */
/******************************************************************************/
/**
  * @brief  Read byte.
  * @param  opcode (byte)
  * @param  address (byte)
  * @retval 8-bit value
  */
uint8_t I2C_Read_Byte(uint8_t slave, uint8_t rd_opcode, uint8_t address)
{
  /* Timeout duration in millisecond [ms] */
  uint8_t timeout = 10;
  uint8_t i2cTX[2];
  uint8_t i2cRX[1];
  uint8_t temp_u8 = 0;

  i2cTX[0] = rd_opcode;
  i2cTX[1] = address;

  /* 1. Transmit register address */
  HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)slave, i2cTX, 2, timeout);

  /* 2. Read four bytes */
  HAL_I2C_Master_Receive(&hi2c1, (uint16_t)slave, i2cRX, 1, timeout);

  /* Concatenate of bytes (from MSB to LSB) */
  temp_u8 = i2cRX[0];

  return temp_u8;
}

/******************************************************************************/
/*                            Read Result Register                            */
/******************************************************************************/
/**
  * @brief  Read double word.
  * @param  opcode (byte)
  * @param  address (byte)
  * @retval 32-bit value
  */
uint32_t I2C_Read_Result(uint8_t slave, uint8_t rd_opcode, uint8_t address)
{
	/* Definition of order, which bit is read first.
	 * For example, with PICOCAP #define must be commented out */
//#define MSB2LSB

	/* Timeout duration in millisecond [ms] */
	uint8_t timeout = 10;
	uint8_t i2cTX[1];
	uint8_t i2cRX[4];
	uint32_t temp_u32 = 0;

	i2cTX[0] = rd_opcode | address;

	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)slave, i2cTX, 1, timeout);
	HAL_I2C_Master_Receive(&hi2c1, (uint16_t)slave, i2cRX, 4, timeout);

	/* 1. Transmit register address */
//	while(HAL_I2C_Master_Transmit(&hi2c1, slave, i2cTX, 1, timeout) != HAL_OK) {
		/* Error_Handler() function is called when Timeout error occurs.
		   When Acknowledge failure occurs (Slave don't acknowledge it's address)
		   Master restarts communication */
/*		if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
		{
		  Error_Handler();
		}
	}
	while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
*/
	/* 2. Read four bytes */
//	while(HAL_I2C_Master_Receive(&hi2c1, slave, i2cRX, 4, timeout) != HAL_OK) {
		/* Error_Handler() function is called when Timeout error occurs.
		   When Acknowledge failure occurs (Slave don't acknowledge it's address)
		   Master restarts communication */
/*		if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
		{
		  Error_Handler();
		}
	}
	while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
*/
#ifdef MSB2LSB
	/* Concatenate of bytes (from MSB to LSB) */
	temp_u32 = (i2cRX[0]<<24) + (i2cRX[1]<<16) + (i2cRX[2]<<8) + (i2cRX[3]);
# else
	/* Concatenate of bytes (from LSB to MSB), e.g. used by PICOCAP */
	temp_u32 = (i2cRX[3]<<24) + (i2cRX[2]<<16) + (i2cRX[1]<<8) + (i2cRX[0]);
#endif

	return temp_u32;
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

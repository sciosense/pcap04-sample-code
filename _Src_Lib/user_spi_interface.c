/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    user_spi_interface.c
  * @brief   SPI Interface Communication Routines.
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
#include "inc/user_spi_interface.h"

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

extern SPI_HandleTypeDef hspi1;

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
/*                                  Set SSN                                   */
/******************************************************************************/
/**
  * @brief  Set or clear the level of SSN pin
  * @param  level specifies the level to be written to SSN pin
  *            @arg LOW: to clear the SSN pin
  *            @arg HIGH: to set the SSN pin
  * @retval none
  */
void Set_SSN(uint8_t level)
{
  if(level == LOW) {
    HAL_GPIO_WritePin(SSN_GPIO_Port, SSN_Pin, GPIO_PIN_RESET);
  }
  if(level == HIGH) {
    HAL_GPIO_WritePin(SSN_GPIO_Port, SSN_Pin, GPIO_PIN_SET);
  }

  return;
}

/******************************************************************************/
/*                               Set SCK to low                               */
/******************************************************************************/
/**
  * @brief  Set the SCK level to low.
  * @retval none
  */
void Set_SCK_LOW(void)
{
  /*to set low SPI_SCK*/
  HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET);

  return;
}

/******************************************************************************/
/*                   Waiting for interrupt with set timeout                   */
/******************************************************************************/
/**
  * @brief  Waiting for interrupt.
  * @param  Timeout Timeout duration
  * @retval none
  */
void Waiting_For_INTN(uint32_t timeout)
{
  uint32_t tickstart;
  
  /*Init tickstart for timeout management */
  tickstart = HAL_GetTick();

  /*step a - Waiting for the interrupt INTN (low active) */
  while(HAL_GPIO_ReadPin(INTN_GPIO_Port, INTN_Pin))
  {
    /* Timeout management */
    if ((((HAL_GetTick() - tickstart) >=  timeout) && (timeout != HAL_MAX_DELAY)) || (timeout == 0U))
    {
      printf("TIMEOUT after %u ms\n", (unsigned int)timeout);
    }
  }

  return;
}

/******************************************************************************/
/*                         Set one bit of double word                         */
/******************************************************************************/
/**
  * @brief  Set one bit of double word.
  * @param  opcode (byte)
  * @param  address (byte)
  * @param  bit_no (uint8_t) 31..0
  * @retval none
  */
void Set_Bit_No(uint8_t opcode, uint8_t address, int bit_no)
{
  /* Timeout duration in millisecond [ms] */
  uint8_t timeout = 10;
  uint8_t spiTX[6];
  uint32_t temp_u32 = 1;

  if (bit_no > 31) bit_no = 31;
  if (bit_no < 0) bit_no = 0;
  
  spiTX[0] = opcode;
  spiTX[1] = address;
  temp_u32 <<= bit_no;
  spiTX[2] = temp_u32>>24;
  spiTX[3] = temp_u32>>16;
  spiTX[4] = temp_u32>>8;
  spiTX[5] = temp_u32;
  
  /* 1. Put SSN low - Activate */
  Set_SSN(LOW);
  
  /* 2. Transmit register address */
  HAL_SPI_Transmit(&hspi1, spiTX, 6, timeout);
  
  /* 3. Put SSN high - Deactivate */
  Set_SSN(HIGH);

  return;
}

/******************************************************************************/
/*                            Write one byte Opcode                           */
/******************************************************************************/
/**
  * @brief  Write one byte Opcode.
  * @param  one_byte
  * @retval none
  */
void Write_Opcode(uint8_t one_byte)
{
  /* Timeout duration in millisecond [ms] */
  uint8_t timeout = 10;
  
  /* 1. Put SSN low - Activate */
  PUT_SSN_LOW;
  
  /* 2. Transmit register address */
  HAL_SPI_Transmit(&hspi1, &one_byte, 1, timeout); 
  
  /* 3. Put SSN high - Deactivate */
  PUT_SSN_HIGH;

  return;
}

/******************************************************************************/
/*                               Write two bytes                              */
/******************************************************************************/
/**
  * @brief  Write two bytes.
  * @param  byte1 (e.g. opcode RC_MT_REQ)
  * @param  byte2 (e.g. request EC_MT_REQ_BITx)
  * @retval none
  */
void Write_Opcode2(uint8_t byte1, uint8_t byte2)
{
  /* Timeout duration in millisecond [ms] */
  uint8_t timeout = 10;
  uint8_t spiTX[2];

  spiTX[0] = byte1;
  spiTX[1] = byte2;
      
  /* 1. Put SSN low - Activate */
  Set_SSN(LOW);
  
  /* 2. Transmit register address */
  HAL_SPI_Transmit(&hspi1, spiTX, 2, timeout);
  
  /* 3. Put SSN high - Deactivate */
  Set_SSN(HIGH);

  return;
}

/******************************************************************************/
/*                               Write two bytes                              */
/* Note: Used for Wireless Sensor Node (e.g.AS393x)
 *
 */
/******************************************************************************/
/**
  * @brief  Write two bytes.
  * @param  byte1 (e.g. direct command )
  * @param  byte2 (e.g. command mode)
  * @retval none
  */
void Write_Opcode2_Lite(uint8_t byte1, uint8_t byte2)
{
  /* Timeout duration in millisecond [ms] */
  uint8_t timeout = 10;
  uint8_t spiTX[1];

  spiTX[0] = byte1 | byte2;

  /* 1. Put SSN low - Activate */
  Set_SSN(HIGH);

  /* 2. Transmit register address */
  HAL_SPI_Transmit(&hspi1, spiTX, 1, timeout);

  /* 3. Put SSN high - Deactivate */
  Set_SSN(LOW);

  return;
}

/******************************************************************************/
/*                             Write one data byte                            */
/******************************************************************************/
/**
  * @brief  Write one byte.
  * @param  opcode (byte)
  * @param  address (byte)
  * @param  byte (byte)
  * @retval none
  */
void Write_Byte(uint8_t opcode, uint8_t address, uint8_t byte)
{
  /* Timeout duration in millisecond [ms] */
  uint8_t timeout = 10;
  uint8_t spiTX[3];

  spiTX[0] = opcode;
  spiTX[1] = address;
  spiTX[2] = byte;
      
  /* 1. Put SSN low - Activate */
  Set_SSN(LOW);
  
  /* 2. Transmit register address */
  HAL_SPI_Transmit(&hspi1, spiTX, 3, timeout);
  
  /* 3. Put SSN high - Deactivate */
  Set_SSN(HIGH);

  return;
}

/******************************************************************************/
/*                  Write one data byte with address, 16 bit                  */
/* Note: Used to write FWC into UFCs, e.g. AS6031                             */
/******************************************************************************/
/**
  * @brief  Write one byte.
  * @param  opcode (byte)
  * @param  address (two bytes)
  * @param  byte (byte)
  * @retval none
  */
void Write_Byte2(uint8_t opcode, uint16_t address, uint8_t byte)
{
  /* Timeout duration in millisecond [ms] */
  uint8_t timeout = 10;
  uint8_t spiTX[4];

  spiTX[0] = opcode;
  spiTX[1] = address>>8; //highest byte
  spiTX[2] = address;    //lowest byte
  spiTX[3] = byte;

  /* 1. Put SSN low - Activate */
  Set_SSN(LOW);

  /* 2. Transmit register address */
  HAL_SPI_Transmit(&hspi1, spiTX, 4, timeout);

  /* 3. Put SSN high - Deactivate */
  Set_SSN(HIGH);

  return;
}

/******************************************************************************/
/*                             Write one data byte                            */
/* Note: Used for Wireless Sensor Node (e.g.AS393x)                           */
/******************************************************************************/
/**
  * @brief  Write one byte.
  * @param  opcode (byte)
  * @param  address (byte)
  * @param  byte (byte)
  * @retval none
  */
void Write_Byte_Lite(uint8_t opcode, uint8_t address, uint8_t byte)
{
  /* Timeout duration in millisecond [ms] */
  uint8_t timeout = 10;
  uint8_t spiTX[2];

  spiTX[0] = opcode | address;
  spiTX[1] = byte;

  /* 1. Put SSN low - Activate */
  Set_SSN(HIGH);

  /* 2. Transmit register address */
  HAL_SPI_Transmit(&hspi1, spiTX, 2, timeout);

  /* 3. Put SSN high - Deactivate */
  Set_SSN(LOW);

  return;
}

/******************************************************************************/
/*                         Write one data double word                         */
/******************************************************************************/
/**
  * @brief  Write one double word.
  * @param  opcode (byte)
  * @param  address (byte)
  * @param  dword (double word)
  * @retval none
  */
void Write_Dword(uint8_t opcode, uint8_t address, uint32_t dword)
{
  /* Timeout duration in millisecond [ms] */
  uint8_t timeout = 10;
  uint8_t spiTX[6];
  uint32_t temp_u32 = 0;

  spiTX[0] = opcode;
  spiTX[1] = address;
  temp_u32 = dword;
  spiTX[2] = temp_u32>>24;
  spiTX[3] = temp_u32>>16;
  spiTX[4] = temp_u32>>8;
  spiTX[5] = temp_u32;
      
  /* 1. Put SSN low - Activate */
  Set_SSN(LOW);
  
  /* 2. Transmit register address */
  HAL_SPI_Transmit(&hspi1, spiTX, 6, timeout);
  
  /* 3. Put SSN high - Deactivate */
  Set_SSN(HIGH);

  return;
}

/******************************************************************************/
/*                         Write one data double word                         */
/******************************************************************************/
/**
  * @brief  Write one double word.
  * @param  opcode (byte)
  * @param  address (byte)
  * @param  dword (double word)
  * @retval none
  */
void Write_Dword_Lite(uint8_t opcode, uint8_t address, uint32_t dword)
{
  /* Timeout duration in millisecond [ms] */
  uint8_t timeout = 10;
  uint8_t spiTX[5];
  uint32_t temp_u32 = 0;

  spiTX[0] = opcode | address;
  temp_u32 = dword;
  spiTX[1] = temp_u32>>24;
  spiTX[2] = temp_u32>>16;
  spiTX[3] = temp_u32>>8;
  spiTX[4] = temp_u32;

  /* 1. Put SSN low - Activate */
  Set_SSN(LOW);

  /* 2. Transmit register address */
  HAL_SPI_Transmit(&hspi1, spiTX, 5, timeout);

  /* 3. Put SSN high - Deactivate */
  Set_SSN(HIGH);

  return;
}

/******************************************************************************/
/*                       Write only bits of double word                       */
/******************************************************************************/
/**
  * @brief  This function writes only the specified bits (from msbit to lsbit)
  *             to the address without reading first.
  * @param  opcode (byte)
  * @param  address (byte)
  * @param  msbit (byte) most significat bit
  * @param  lsbit (byte) low significant bit
  * @param  dword (double word)
  * @retval none
  */
void Write_Dword_Bits(uint8_t opcode, uint8_t address, uint8_t msbit, uint8_t lsbit, uint32_t dword)
{
//#define _DEBUGGGING_FUNCTION
  
  uint32_t bit_amount = 0;
  uint32_t bit_mask = 0;
  uint32_t bit_mask_inv = 0xFFFFFFFF;
  uint32_t temp_u32 = 0;
  
  /* out of range [31:0] */
  if (msbit > 31) msbit = 31;
  if (lsbit > 31) lsbit = 31;
  
  if (lsbit > msbit) lsbit = msbit;

  /* build the mask */
  bit_amount = msbit - lsbit;
  for (int i = 0; i < bit_amount + 1; i++) {
    bit_mask <<= 1;
    bit_mask += 1;
  }
  bit_mask <<= lsbit;
  bit_mask_inv -= bit_mask;

  /* Change specified bits at shifted position*/
  temp_u32 = dword << lsbit;
  
  /* within the limits */
  temp_u32 &= bit_mask;

#ifdef _DEBUGGGING_FUNCTION
  /* for debugging */
  puts("Write_Dword_Bits");
  printf(" opcode = 0x%02X\taddress = 0x%02X\n", opcode, address);
  printf(" msb = %u\tlsb = %u\n", msbit, lsbit);
  printf(" value to be written = 0x%08X\n", temp_u32);
  printf(" bit_mask = 0x%08X\tbit_mask_inv = 0x%08X\n", bit_mask, bit_mask_inv);
#endif
  
  /* Write content */
  Write_Dword(opcode, address, temp_u32);

#undef _DEBUGGGING_FUNCTION

  return;
}

/******************************************************************************/
/*                       Update only bits of double word                       */
/******************************************************************************/
/**
  * @brief  This function updates the specified bits (from msbit to lsbit) by 
  *             reading the content of address first. After that, the updated 
  *             content will be written.
  * @param  rd_opcode (byte) for reading double word
  * @param  address (byte)
  * @param  wr_opcode (byte) for writing updated double word
  * @param  msbit (byte) most significat bit
  * @param  lsbit (byte) low significant bit
  * @param  dword (double word)
  * @retval none
  */
void Update_Dword_Bits(uint8_t rd_opcode, uint8_t address, uint8_t wr_opcode, uint8_t msbit, uint8_t lsbit, uint32_t dword)
{
//#define _DEBUGGGING_FUNCTION
  
  uint32_t bit_amount = 0;
  uint32_t bit_mask = 0;
  uint32_t bit_mask_inv = 0xFFFFFFFF;
  uint32_t temp_u32 = 0;
  
  uint32_t address_content = 0;
  
  /* out of range [31:0] */
  if (msbit > 31) msbit = 31;
  if (lsbit > 31) lsbit = 31;
  
  if (lsbit > msbit) lsbit = msbit;

  /* build the mask */
  bit_amount = msbit - lsbit;
  for (int i = 0; i < bit_amount + 1; i++) {
    bit_mask <<= 1;
    bit_mask += 1;
  }
  bit_mask <<= lsbit;
  bit_mask_inv -= bit_mask;

  
  /* Read content of address */
  address_content = Read_Dword(rd_opcode, address);
  
  /* Change specified bits at shifted position*/
  temp_u32 = dword << lsbit;

#ifdef _DEBUGGGING_FUNCTION
  /* for debugging */
  puts("Update_Dword_Bits");
  printf(" RD_opcode = 0x%02X\tWR_opcode = 0x%02X\taddress = 0x%02X\n", rd_opcode, wr_opcode, address);
  printf(" msb = %u\tlsb = %u\n", msbit, lsbit);
  printf(" read content = 0x%08X\tvalue to be written = 0x%08X\n", address_content, temp_u32);
  printf(" bit_mask = 0x%08X\tbit_mask_inv = 0x%08X\n", bit_mask, bit_mask_inv);
#endif
  
  /* Clear specified bits of content */
  address_content &= bit_mask_inv;

#ifdef _DEBUGGGING_FUNCTION
  /* for debugging */
  printf(" cleared range = %08X\n", address_content);
#endif
  
  /* Write content */
  address_content |= temp_u32;
  Write_Dword(wr_opcode, address, address_content);

#ifdef _DEBUGGGING_FUNCTION
  /* for debugging */
  printf(" written = %08X\n", address_content); 
#endif
  
#undef _DEBUGGGING_FUNCTION

  return;
}

/******************************************************************************/
/*                       Write bytes auto incrementally                       */
/* Note: Used for PICOCAP devices                                             */
/******************************************************************************/
/**
  * @brief  Write bytes incrementally.
  * @param  opcode (byte)
  * @param  address (byte)
  * @param  byte_array
  * @param  to_addr (32 bit)
  * @retval none
  */
void Write_Byte_Auto_Incr(int opcode, int address, uint8_t *byte_array, int to_addr)
{
  /* Timeout duration in millisecond [ms] */
  uint8_t timeout = 10;
  uint8_t spiTX[2]; //max index
  
  //opcodes for memory access
  //WR_CFG = 0xA3C0
  //RD_CFG = 0x23C0
  //opcodes for memory access
  //WR = 0xA0
  //RD = 0x20

  if (opcode<0x100) {
	  spiTX[0] = (uint8_t)opcode | (uint8_t)(address>>8);
	  spiTX[1] = (uint8_t)(address);
  } else {
	  spiTX[0] = (uint8_t)(opcode>>8);
	  spiTX[1] = (uint8_t)(opcode) | (uint8_t)(address);
  }

  /* 1. Put SSN low - Activate */
  Set_SSN(LOW);

  /* 2.a Transmit register address */
  HAL_SPI_Transmit(&hspi1, spiTX, 2, timeout);
    
  /* 2.b Transmit register address incrementally */
  for (int i = address; i <= to_addr; i++) {
    HAL_SPI_Transmit(&hspi1, byte_array, 1, timeout);

    byte_array++;
  }
  
  /* 3. Put SSN high - Deactivate */
  Set_SSN(HIGH);

  return;
}

/******************************************************************************/
/*                        Read bytes auto incrementally                       */
/* Note: Used for PICOCAP devices                                             */
/******************************************************************************/
/**
  * @brief  Read byte array auto incrementally.
  * @param  opcode (byte),       "Byte2",       Bit[7:2]
  * @param  address (byte 1),    "Byte2+Byte1", Bit[9:0]
  * @param  byte array (byte 0), "Byte0"
  * @param  to address
  * @retval none
  */
void Read_Byte_Auto_Incr(int opcode, int address, uint8_t *spiRX, int to_addr)
{
  /* Timeout duration in millisecond [ms] */
  uint8_t timeout = 10;
  uint8_t spiTX[2]; //max index

  uint16_t n_byte = 0;
  n_byte = (to_addr - address) + 1;

  //opcodes for memory access
  //WR_CFG = 0xA3C0
  //RD_CFG = 0x23C0
  //opcodes for memory access
  //WR = 0xA0
  //RD = 0x20

  if (opcode<0x100) {
	  spiTX[0] = (uint8_t)(opcode) | (uint8_t)(address>>8);
	  spiTX[1] = (uint8_t)(address);
  } else {
	  spiTX[0] = (uint8_t)(opcode>>8);
	  spiTX[1] = (uint8_t)(opcode) | (uint8_t)(address);
  }

  /* 1. Put SSN low - Activate */
  PUT_SSN_LOW;

  /* 2. Transmit register address */
  HAL_SPI_Transmit(&hspi1, spiTX, 2, timeout);

  /* 3. Read n bytes */
  HAL_SPI_Receive(&hspi1, spiRX, n_byte, timeout);

  /* 4. Put SSN high - Deactivate */
  PUT_SSN_HIGH;

  return;
}

/******************************************************************************/
/*                    Write double words auto incrementally                   */
/******************************************************************************/
/**
  * @brief  Write double words incrementally.
  * @param  opcode (byte)
  * @param  from_addr (byte)
  * @param  dword_array
  * @param  to_addr (32 bit)
  * @retval none
  */
void Write_Dword_Auto_Incr(uint8_t opcode, uint8_t from_addr, uint32_t *dword_array, int to_addr)
{
  /* Timeout duration in millisecond [ms] */
  uint8_t timeout = 10;
  uint8_t spiTX[4];
  uint32_t temp_u32 = 0;

  spiTX[0] = opcode;
  spiTX[1] = from_addr;

  /* to start at expected index */
  dword_array += from_addr; 
  
  /* 1. Put SSN low - Activate */
  Set_SSN(LOW);

  /* 2.a Transmit register address */
  HAL_SPI_Transmit(&hspi1, spiTX, 2, timeout); 
    
  /* 2.b Transmit register address incrementally */
  for (int i = from_addr; i <= to_addr; i++) {
    temp_u32 = *dword_array;
    spiTX[0] = temp_u32>>24;
    spiTX[1] = temp_u32>>16;
    spiTX[2] = temp_u32>>8;
    spiTX[3] = temp_u32;

    HAL_SPI_Transmit(&hspi1, spiTX, 4, timeout);

    dword_array++;
  }
  
  /* 3. Put SSN high - Deactivate */
  Set_SSN(HIGH);

  return;
}

/******************************************************************************/
/*                      Write register auto incrementally                     */
/******************************************************************************/
/**
  * @brief  Write double words incrementally.
  * @param  opcode (byte)
  * @param  from_addr (byte)
  * @param  dword_array
  * @param  to_addr (32 bit)
  * @retval none
  */
void Write_Register_Auto_Incr(uint8_t opcode, uint8_t from_addr, uint32_t *dword_array, int to_addr)
{
  /* Timeout duration in millisecond [ms] */
  uint8_t timeout = 10;
  uint8_t spiTX[4];
  uint32_t temp_u32 = 0;

  spiTX[0] = opcode;
  spiTX[1] = from_addr;

//  /* to start at expected index */
//  dword_array += from_addr; 
  
  /* 1. Put SSN low - Activate */
  Set_SSN(LOW);

  /* 2.a Transmit register address */
  HAL_SPI_Transmit(&hspi1, spiTX, 2, timeout); 
    
  /* 2.b Transmit register address incrementally */
  for (int i = from_addr; i <= to_addr; i++) {
    temp_u32 = *dword_array;
    spiTX[0] = temp_u32>>24;
    spiTX[1] = temp_u32>>16;
    spiTX[2] = temp_u32>>8;
    spiTX[3] = temp_u32;

    HAL_SPI_Transmit(&hspi1, spiTX, 4, timeout);

    dword_array++;
  }
  
  /* 3. Put SSN high - Deactivate */
  Set_SSN(HIGH);

  return;
}

/******************************************************************************/
/*                                 Read byte                                  */
/******************************************************************************/
/**
  * @brief  Read byte.
  * @param  opcode (byte)
  * @param  address (byte)
  * @retval 8-bit value
  */
uint8_t Read_Byte(uint8_t rd_opcode, uint8_t address)
{
  /* Timeout duration in millisecond [ms] */
  uint8_t timeout = 10;
  uint8_t spiTX[2];
  uint8_t spiRX[1];

  spiTX[0] = rd_opcode;
  spiTX[1] = address;

  /* 1. Put SSN low - Activate */
  Set_SSN(LOW);

  /* 2. Transmit register address */
  HAL_SPI_Transmit(&hspi1, spiTX, 2, timeout);

  /*3. Read one bytes */
  HAL_SPI_Receive(&hspi1, spiRX, 1, timeout);

  /* 4. Put SSN high - Deactivate */
  Set_SSN(HIGH);

  return spiRX[0];
}

/******************************************************************************/
/*                                 Read byte2                                 */
/*              E.g., used by PICOCAP by performing a TEST READ               */
/******************************************************************************/
/**
  * @brief  Read byte.
  * @param  opcode (byte)
  * @retval 8-bit value
  */
uint8_t Read_Byte2(uint8_t rd_opcode)
{
  /* Timeout duration in millisecond [ms] */
  uint8_t timeout = 10;
  uint8_t spiTX[1];
  uint8_t spiRX[1];

  spiTX[0] = rd_opcode;

  /* 1. Put SSN low - Activate */
  Set_SSN(LOW);

  /* 2. Transmit register address */
  HAL_SPI_Transmit(&hspi1, spiTX, 1, timeout);

  /*3. Read one bytes */
  HAL_SPI_Receive(&hspi1, spiRX, 1, timeout);

  /* 4. Put SSN high - Deactivate */
  Set_SSN(HIGH);

  return spiRX[0];
}

/******************************************************************************/
/*                                 Read byte                                  */
/* Note: Used for Wireless Sensor Node (e.g.AS393x)
 *
 */
/******************************************************************************/
/**
  * @brief  Read byte.
  * @param  opcode (byte)
  * @param  address (byte)
  * @retval 8-bit value
  */
uint8_t Read_Byte_Lite(uint8_t rd_opcode, uint8_t address)
{
  /* Timeout duration in millisecond [ms] */
  uint8_t timeout = 10;
  uint8_t spiTX[1];
  uint8_t spiRX[1];

  spiTX[0] = rd_opcode | address;

  /* 1. Put SSN low - Activate */
  Set_SSN(HIGH);

  /* 2. Transmit register address */
  HAL_SPI_Transmit(&hspi1, spiTX, 1, timeout);

  /*3. Read one bytes */
  HAL_SPI_Receive(&hspi1, spiRX, 1, timeout);

  /* 4. Put SSN high - Deactivate */
  Set_SSN(LOW);

  return spiRX[0];
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
uint32_t Read_Dword(uint8_t rd_opcode, uint8_t address)
{
  /* Timeout duration in millisecond [ms] */
  uint8_t timeout = 10;
  uint8_t spiTX[2];
  uint8_t spiRX[4];
  uint32_t temp_u32 = 0;
  
  spiTX[0] = rd_opcode;
  spiTX[1] = address;
  
  /* 1. Put SSN low - Activate */
  Set_SSN(LOW);
  
  /* 2. Transmit register address */
  HAL_SPI_Transmit(&hspi1, spiTX, 2, timeout);
  
  /*3. Read four bytes */
  HAL_SPI_Receive(&hspi1, spiRX, 4, timeout);
  
  /* 4. Put SSN high - Deactivate */
  Set_SSN(HIGH);
  
  /*Concatenate of bytes (from MSB to LSB) */
  temp_u32 = (spiRX[0]<<24) + (spiRX[1]<<16) + (spiRX[2]<<8) + (spiRX[3]);
  
  return temp_u32;
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
uint32_t Read_Dword_Lite(uint8_t rd_opcode, uint8_t address)
{
  /* Definition of order, which bit is read first.
   * For example, with PICOCAP #define must be commented out */
//#define MSB2LSB

  /* Timeout duration in millisecond [ms] */
  uint8_t timeout = 10;
  uint8_t spiTX[1];
  uint8_t spiRX[4];
  uint32_t temp_u32 = 0;

  spiTX[0] = rd_opcode | address;

  /* 1. Put SSN low - Activate */
  Set_SSN(LOW);

  /* 2. Transmit register address */
  HAL_SPI_Transmit(&hspi1, spiTX, 1, timeout);

  /*3. Read four bytes */
  HAL_SPI_Receive(&hspi1, spiRX, 4, timeout);

  /* 4. Put SSN high - Deactivate */
  Set_SSN(HIGH);

#ifdef MSB2LSB
  /*Concatenate of bytes (from MSB to LSB) */
  temp_u32 = (spiRX[0]<<24) + (spiRX[1]<<16) + (spiRX[2]<<8) + (spiRX[3]);
#else
  /*Concatenate of bytes (from LSB to MSB), e.g. used by PICOCAP */
  temp_u32 = (spiRX[3]<<24) + (spiRX[2]<<16) + (spiRX[1]<<8) + (spiRX[0]);
#endif

  return temp_u32;
}

/******************************************************************************/
/*                              Read double word                              */
/******************************************************************************/
/**
  * @brief  Read double word.
  * @param  opcode (byte) for reading double word
  * @param  address (byte)
  * @param  msbit (byte) most significat bit
  * @param  lsbit (byte) low significant bit
  * @retval 32-bit value, content of specified bits
  */
uint32_t Read_Dword_Bits(uint8_t rd_opcode, uint8_t address, uint8_t msbit, uint8_t lsbit)
{
//#define _DEBUGGGING_FUNCTION
  
  uint32_t address_content = 0;
  uint32_t bit_amount = 0;
  uint32_t bit_mask = 0;
  uint32_t temp_u32 = 0;
  
  /* out of range [31:0] */
  if (msbit > 31) msbit = 31;
  if (lsbit > 31) lsbit = 31;
  
  if (lsbit > msbit) lsbit = msbit;
  
  /* build the mask */
  bit_amount = msbit - lsbit;
  for (int i = 0; i < bit_amount + 1; i++) {
    bit_mask <<= 1;
    bit_mask += 1;
  }
  bit_mask <<= lsbit;
  
  /* read the register content */
  address_content = Read_Dword(rd_opcode, address);
  temp_u32 = (address_content & bit_mask) >> lsbit;
  
#ifdef _DEBUGGGING_FUNCTION
  /* for debugging */
  puts("Read_Dword_Bits");
  printf(" RD opcode = 0x%02X\taddress = 0x%02X\n", rd_opcode, address);
  printf(" msb = %u\tlsb = %u\n", msbit, lsbit);
  printf(" read content (before) = 0x%08X\tread content (after) = 0x%08X\n", address_content, temp_u32);
  printf(" RD bit_mask = 0x%08X\n", bit_mask);
#endif
  
#undef _DEBUGGGING_FUNCTION
  
  return temp_u32;
}

/* Following lines are added in December 2021 */
/* Added by MHAI */

/******************************************************************************/
/*                       Write bytes auto incrementally                       */
/* Note: Used for TDC (e.g.AS6500)                                            */
/*       Reduces WR Config Timing from 190us down to 30us @20MHz              */
/******************************************************************************/
/**
  * @brief  Write bytes incrementally.
  * @param  opcode (byte), Bit[7:5]
  * @param  from_addr (byte), Bit[4:0]
  * @param  byte_array
  * @param  to_addr (32 bit)
  * @retval none
  */
void Write_Byte_Auto_Incr_Lite(uint8_t opcode, uint8_t from_addr, uint8_t *byte_array, int to_addr)
{
  /* Timeout duration in millisecond [ms] */
  uint8_t timeout = 10;
  uint8_t spiTX[1];
  uint16_t size = 0;

  spiTX[0] = opcode | from_addr;
  size = (to_addr-from_addr) + 1;

  /* 1. Put SSN low - Activate */
  PUT_SSN_LOW;

  /* 2.a Transmit register address */
  HAL_SPI_Transmit(&hspi1, spiTX, 1, timeout);

  /* 2.b Transmit register address incrementally */
  HAL_SPI_Transmit(&hspi1, byte_array, size, timeout);

  /* 3. Put SSN high - Deactivate */
  PUT_SSN_HIGH;

  return;
}

/******************************************************************************/
/*                        Read bytes auto incrementally                       */
/* Note: Used for TDC (e.g.AS6500)                                            */
/*       Without FOR-LOOP (4us), RD Config takes about 48us                   */
/******************************************************************************/
/**
  * @brief  Read byte array auto incrementally.
  * @param  opcode (byte), Bit[7:5]
  * @param  address (byte), Bit[4:0]
  * @param  byte array (byte)
  * @param  to address
  * @retval none
  */
void Read_Byte_Auto_Incr_Lite(uint8_t rd_opcode, uint8_t address, uint8_t *spiRX, int to_addr)
{
  /* Timeout duration in millisecond [ms] */
  uint8_t timeout = 10;
  uint8_t spiTX[1];
  //uint8_t spiRX[128];
  //uint32_t temp_u32 = 0;

  uint16_t n_byte = 0;
  n_byte = (to_addr - address) + 1;

  spiTX[0] = rd_opcode | address;

  /* 1. Put SSN low - Activate */
  PUT_SSN_LOW;

  /* 2. Transmit register address */
  HAL_SPI_Transmit(&hspi1, spiTX, 1, timeout);

  /* 3. Read n bytes */
  HAL_SPI_Receive(&hspi1, spiRX, n_byte, timeout);

  /* 4. Put SSN high - Deactivate */
  PUT_SSN_HIGH;

  return;
}

/******************************************************************************/
/*                             Clearing the FIFOs                             */
/* Note: Used for TDC (e.g.AS6500)                                            */
/*       Jump into subroutine consumes about 1 us                             */
/******************************************************************************/
/**
  * @brief  Clearing the FIFOs by reading as long as INTN goes high.
  * @retval none
  */
void Clearing_All_FIFOs(void)
{
	//Clearing all four FIFOs
	uint32_t count = 0;
	uint8_t read_loop = 0;

	uint8_t spiTX[1];
	uint8_t spiRX[32]; // Max. Reading 4x (REF_IDX[3 Bytes], TSTOP[3 Bytes]), plus 8 Bytes
	uint8_t RD_x_Bytes = 24;
	spiTX[0] = 0x68; // Opcode + Address

	count = 0; //counts the 0xFF

	while (count != RD_x_Bytes)
	{
		// Reading takes about 40us @SPI=20 MHz
		// Note: AS6500 Data Sheet, Section 8.8.4
		//       FIFOs for Adapting Peak and Average Conversion Rate
		SSN_GPIO_Port->BRR = (uint32_t)SSN_Pin;			// 1. Put SSN low - Activate
		HAL_SPI_Transmit(&hspi1, spiTX, 1, 10);			// 2. Transmit register address
		HAL_SPI_Receive(&hspi1, spiRX, RD_x_Bytes, 10);	// 3. Read x bytes
		SSN_GPIO_Port->BSRR = (uint32_t)SSN_Pin;		// 4. Put SSN high - Deactivate
		// Counts each 0xFF
		for (int i=0; i<RD_x_Bytes; i++)
		{
			if (spiRX[i] == 0xFF) {
				count++;
			}
		}
		read_loop++;

	} // End of WHILE() or IF(My_INTN_State==0)

	return;
}

/******************************************************************************/
/*                             Read Result/Status                             */
/* Note: Used for TDC (e.g.AS6500)
 * - status
 * - interrupt
 * - read all data 16XFIFO
 * - read each FIFO as long as 0xFFFFFF, then jump to next
 * - ....
 * - REG 05 (FIFO full and empty flags) FF4 FF3 FF2 FF1 EF4 EF3 EF2 EF1       */
/******************************************************************************/
/**
  * @brief  Read max double word.
  * @param  opcode (byte), Bit[7:5]
  * @param  address (byte), Bit[4:0]
  * @retval none
  */
void Read_Result_Lite(RD_FIFO FIFO_1[FIFO_DEPTH], RD_FIFO FIFO_2[FIFO_DEPTH])
{
	/* Timeout duration in millisecond [ms] */
	uint8_t timeout = 10;
	uint8_t spiTX[1];
	uint8_t spiRX[32];
	//uint32_t temp_u32 = 0;

	//spiTX[0] = rd_opcode | address;
	spiTX[0] = 0x60;

	for (int i=0; i<FIFO_DEPTH; i++)
	{
		/* 1. Put SSN low - Activate */
		Set_SSN(LOW);

		/* 2. Transmit register address */
		HAL_SPI_Transmit(&hspi1, spiTX, 1, timeout);

		/* 3. Read four bytes */
		HAL_SPI_Receive(&hspi1, spiRX, 32, timeout);

		/* 4. Put SSN high - Deactivate */
		Set_SSN(HIGH);

		/*Concatenate of bytes (from MSB to LSB) */
		FIFO_1[i].REFID = (spiRX[8]<<16) + (spiRX[9]<<8) + (spiRX[10]);
		FIFO_1[i].TSTOP = (spiRX[11]<<16) + (spiRX[12]<<8) + (spiRX[13]);
		FIFO_2[i].REFID = (spiRX[14]<<16) + (spiRX[15]<<8) + (spiRX[16]);
		FIFO_2[i].TSTOP = (spiRX[17]<<16) + (spiRX[18]<<8) + (spiRX[19]);
		//FIFO_3[i].REFID = (spiRX[20]<<16) + (spiRX[21]<<8) + (spiRX[22]);
		//FIFO_3[i].TSTOP = (spiRX[23]<<16) + (spiRX[24]<<8) + (spiRX[25]);
		//FIFO_4[i].REFID = (spiRX[26]<<16) + (spiRX[27]<<8) + (spiRX[28]);
		//FIFO_4[i].TSTOP = (spiRX[29]<<16) + (spiRX[30]<<8) + (spiRX[31]);
	}

	return;
}

/******************************************************************************/
/*                            Read amount of bytes                            */
/* Note: Used for TDC (e.g. GP22)                                             */
/******************************************************************************/
/**
  * @brief  Read amount of bytes.
  * @param  opcode (byte)
  * @param  byte array (byte)
  * @param  amount
  * @retval none
  */
void Read_Byte_Amount_Of(uint8_t rd_opcode, uint8_t *spiRX, int amount)
{
  /* Timeout duration in millisecond [ms] */
  uint8_t timeout = 10;
  uint8_t spiTX[1];

  uint16_t n_byte = 0;
  n_byte = amount;

  spiTX[0] = rd_opcode;

  /* 1. Put SSN low - Activate */
  PUT_SSN_LOW;

  /* 2. Transmit register address */
  HAL_SPI_Transmit(&hspi1, spiTX, 1, timeout);

  /* 3. Read n bytes */
  HAL_SPI_Receive(&hspi1, spiRX, n_byte, timeout);

  /* 4. Put SSN high - Deactivate */
  PUT_SSN_HIGH;

  return;
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

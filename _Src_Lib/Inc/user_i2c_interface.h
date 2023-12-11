/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    user_i2c_interface.h
  * @brief   This file contains the headers of the SPI communication.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_I2C_INTERFACE_H
#define __USER_I2C_INTERFACE_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
   
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
   
#define LOW ((uint8_t)0)
#define HIGH ((uint8_t)1)
   
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
   
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/

/* USER CODE BEGIN EFP */
 extern void I2C_Sweep_DevAddr(uint8_t from_addr, uint8_t to_addr, uint8_t *addr_array);
 extern HAL_StatusTypeDef I2C_Write_Opcode(uint8_t slave, uint8_t one_byte);
 extern HAL_StatusTypeDef I2C_Memory_Access(uint8_t slave, uint8_t opcode, uint16_t address, uint8_t *byte, uint16_t size);
 extern HAL_StatusTypeDef I2C_Config_Access(uint8_t slave, uint32_t opcode, uint8_t address, uint8_t *byte, uint8_t size);

 extern HAL_StatusTypeDef I2C_Write_Dword(uint8_t slave, uint8_t opcode, uint8_t address, uint32_t dword);
 extern HAL_StatusTypeDef I2C_Write_Byte(uint8_t slave, uint8_t opcode, uint8_t address, uint8_t byte);

 extern uint32_t I2C_Read_Dword(uint8_t slave, uint8_t rd_opcode, uint8_t address);
 extern uint8_t I2C_Read_Byte(uint8_t slave, uint8_t rd_opcode, uint8_t address);
 extern uint32_t I2C_Read_Result(uint8_t slave, uint8_t rd_opcode, uint8_t address);

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __USER_I2C_INTERFACE_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

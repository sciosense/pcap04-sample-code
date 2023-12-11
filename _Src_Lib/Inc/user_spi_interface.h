/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    user_spi_interface.h
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
#ifndef __USER_SPI_INTERFACE_H
#define __USER_SPI_INTERFACE_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

 // added for AS6500
 typedef struct _RD_FIFO {
 	uint32_t REFID;
 	uint32_t TSTOP;
 } RD_FIFO;

#define FIFO_DEPTH ((uint8_t)16)

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

#define LOW ((uint8_t)0)
#define HIGH ((uint8_t)1)

#define PUT_SSN_LOW (SSN_GPIO_Port->BRR = (uint32_t)SSN_Pin)	// Put SSN low - Activate, SSN -> CLK = < 4us
#define PUT_SSN_HIGH (SSN_GPIO_Port->BSRR = (uint32_t)SSN_Pin)	// Put SSN high - Deactivate, CLK -> SSN = < 9us

 RD_FIFO FIFO_1[FIFO_DEPTH], FIFO_2[FIFO_DEPTH], FIFO_3[FIFO_DEPTH], FIFO_4[FIFO_DEPTH]; //e.g. FIFO Depth = 16

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
   
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/

/* USER CODE BEGIN EFP */
extern void Set_SSN              (uint8_t level);
extern void Set_SCK_LOW          (void); // does not really works, GPIO is blocked!!
extern void Waiting_For_INTN     (uint32_t timeout); // includes printf() --> OBSOLET, because of using EXTI Interrupt Handling (NVIC)!!

//void Write_Byte(uint8_t *pData1, uint16_t Size1, uint8_t *pData2, uint16_t Size2, uint8_t *byte_array, uint8_t timeout);
//pData1=Opcode, pData2=Address, Array and TimeOut
extern void Set_Bit_No              (uint8_t opcode, uint8_t address, int bit_no);
extern void Write_Opcode            (uint8_t one_byte);
extern void Write_Opcode2           (uint8_t byte1, uint8_t byte2); // used for Measure_Task_Request
extern void Write_Opcode2_Lite      (uint8_t byte1, uint8_t byte2); //used by AS393x
extern void Write_Byte              (uint8_t opcode, uint8_t address, uint8_t byte); //not used
extern void Write_Byte2             (uint8_t opcode, uint16_t address, uint8_t byte); //Writing FWC into UFCs
extern void Write_Byte_Lite         (uint8_t opcode, uint8_t address, uint8_t byte); //used by AS393x
extern void Write_Dword             (uint8_t opcode, uint8_t address, uint32_t dword);
extern void Write_Dword_Lite        (uint8_t opcode, uint8_t address, uint32_t dword);
extern void Write_Dword_Bits        (uint8_t opcode, uint8_t address, uint8_t msbit, uint8_t lsbit, uint32_t dword);
extern void Update_Dword_Bits       (uint8_t rd_opcode, uint8_t address, uint8_t wr_opcode, uint8_t msbit, uint8_t lsbit, uint32_t dword);
extern void Write_Byte_Auto_Incr    (int opcode, int address, uint8_t *byte_array, int to_addr); //used by PICOCAP
extern void Read_Byte_Auto_Incr     (int opcode, int address, uint8_t *spiRX, int to_addr); //used by PICOCAP
extern void Write_Dword_Auto_Incr   (uint8_t opcode, uint8_t from_addr, uint32_t *dword_array, int to_addr);
extern void Write_Register_Auto_Incr(uint8_t opcode, uint8_t from_addr, uint32_t *dword_array, int to_addr);

extern uint8_t  Read_Byte      (uint8_t rd_opcode, uint8_t address);
extern uint8_t  Read_Byte2     (uint8_t rd_opcode); //used by PICOCAP
extern uint8_t  Read_Byte_Lite (uint8_t rd_opcode, uint8_t address); //used by AS393x
extern uint32_t Read_Dword     (uint8_t rd_opcode, uint8_t address);
extern uint32_t Read_Dword_Lite(uint8_t rd_opcode, uint8_t address); //function has to be configured
extern uint32_t Read_Dword_Bits(uint8_t rd_opcode, uint8_t address, uint8_t msbit, uint8_t lsbit);

// added for AS6500
//RD_FIFO FIFO_1[FIFO_DEPTH], FIFO_2[FIFO_DEPTH], FIFO_3[FIFO_DEPTH], FIFO_4[FIFO_DEPTH]; //e.g. FIFO Depth = 16

extern void Write_Byte_Auto_Incr_Lite(uint8_t opcode, uint8_t from_addr, uint8_t *byte_array, int to_addr);
extern void Read_Byte_Auto_Incr_Lite (uint8_t rd_opcode, uint8_t address, uint8_t *byte_array, int to_addr);
extern void Clearing_All_FIFOs(void);

extern void Read_Result_Lite     (RD_FIFO FIFO_1[FIFO_DEPTH], RD_FIFO FIFO_2[FIFO_DEPTH]);

extern void Read_Byte_Amount_Of(uint8_t rd_opcode, uint8_t *spiRX, int amount);


/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __USER_SPI_INTERFACE_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

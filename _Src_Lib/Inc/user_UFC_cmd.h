/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    user_as6031_cmd.h
  * @brief   This file contains the headers of the sequences for AS6031.
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
#ifndef __USER_UFC_CMD_H
#define __USER_UFC_CMD_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "user_spi_interface.h"
//#include "user_tools.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define LOW  ((uint8_t)0)
#define HIGH ((uint8_t)1)

#define NOT_LOCKED ((uint8_t)0)
#define LOCKED     ((uint8_t)1)

#define FIRMWARE ((uint8_t)0)
#define ERASED   ((uint8_t)1)

 //Declaration of default HighSpeed/LowSpeed Clock
#ifndef HS_CLOCK
#define HS_CLOCK        (float)4e6		//4 MHz HighSpeed Clock
#endif

#ifndef T_REF
#define T_REF           (float)(1/HS_CLOCK)
#endif

#ifndef LS_CLOCK
#define LS_CLOCK        (float)32768	//32.768 kHz LowSpeed Clock
#endif

#ifndef T_LS_CLK
#define T_LS_CLK		(float)(1/LS_CLOCK)
#endif

//#define _DEBUGGGING_FUNCTION

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/

/* USER CODE BEGIN EFP */
#ifdef UFC_DEVICE_GP30
 extern void GP30_Download_FWC_FWD     (uint8_t *fwcode, uint32_t fwc_size, uint32_t *fwdata, uint8_t lock); //related to GP30
 extern void GP30_Download_FWC_ONLY    (uint8_t *fwcode, uint32_t fwc_size, uint8_t lock); //related to GP30
 extern void GP30_Download_FWD_ONLY    (uint32_t *fwdata, uint8_t lock); //related to GP30
 extern void GP30_Checksum_Verification(void); //related to GP30
 extern void GP30_Erase_FWC_FWD        (void); //related to GP30
#endif

 extern void Waiting_For_INTN_Flag2      (int flag_no, uint32_t timeout); //printf() --> obsolet, because of using EXTI Interrupt Handling (NVIC)!!
#ifdef UFC_DEVICE_AS6031
 extern void Waiting_For_INTN_Poll_Status(uint32_t timeout);
#endif
#ifdef UFC_DEVICE_AS6031
 extern void Sending_System_Reset        (void);
#endif
#ifdef UFC_DEVICE_AS6031
 extern void Sequence_Erase_NVRAM     (void); //puts()
 extern void Sequence_Download_FWC_FWD(uint8_t *fwcode, uint32_t fwc_size, uint32_t *fwdata, uint8_t lock); //puts()
 extern void Sequence_Download_FWD    (uint32_t *fwdata); //puts()
#endif

#ifdef UFC_DEVICE_AS6031
 extern uint32_t Read_Dword_with_Status(uint8_t rd_opcode, uint8_t address);
#endif

 extern void Start_with_System_Reset(void);
 extern void Perform_FW_Transaction (int shr_rc_bit);
#ifdef UFC_DEVICE_AS6031
 extern void Preparation       (void);
 extern void Preparation_Eval  (void); //printf()
#endif
 extern void Download_FWC_FWD  (uint8_t *fwcode, uint32_t fwc_size, uint32_t *fwdata, uint8_t lock);
 extern void Download_FWD_ONLY (uint32_t *fwdata);
 extern void Erase_FWC_FWD     (void);
 extern int  FW_Retention_Check(void);
 extern int  Reading_SHR_GPO   (void);
 extern void Printf_SHR_GPO    (void); //printf()

 extern void Write_FWC(uint8_t *fwcode, uint32_t fwc_size);

 extern void Fill_Up_FWC_with_Zero(uint16_t from_address);

 extern void Write_FWC_Auto_Incr(uint16_t address, uint8_t *byte_array, int limit);

 extern uint32_t Calc_Checksum_FWC(uint8_t *byte_array, int array_size);
 extern uint32_t Calc_Checksum_FWD(uint32_t *dword_array, int from_addr, int to_addr);
#ifdef UFC_DEVICE_AS6031
 extern void Update_Checksum    (uint8_t *fwcode, uint32_t fwc_size, uint32_t *fwdata); //related to GP30
#endif
 extern void Printf_Checksum    (void); //printf()
#ifdef UFC_DEVICE_AS6031
 extern int  System_Status      (void);
 extern void Print_System_Status(void); //printf()
 extern void SYS_Status_Delay   (uint32_t delay); //printf()
#endif
 extern void Update_TimeStamp   (void);
 extern void Printf_TimeStamp   (void); //printf()
 extern void Printf_VCC_VAL     (void); //printf()

 extern void  Put_UFC_Into_Idle  (void);
 extern float Calc_HighSpeedClock(float time_unit);
 extern float Calc_Amplitude     (uint32_t AM_address, uint32_t AMC_VH, uint32_t AMC_VL);
 extern float Calc_TimeOfFlight  (uint32_t TOF_address);

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __USER_UFC_CMD_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

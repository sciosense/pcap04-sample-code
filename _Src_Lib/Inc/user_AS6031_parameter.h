/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    user_as6031_parameter.h
  * @brief   This file contains the headers of determined addresses 
  *             and used variables.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#ifndef __USER_AS6031_PARAMETER_H
#define __USER_AS6031_PARAMETER_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* Due to reassignment, the current
 * definitions of identifier has to be removed */
#if defined(__USER_GP30_PARAMETER_H)
#undef FDB_US_TOF_0_D
#undef FDB_US_TOF_1_D
#undef FDB_US_TOF_2_D
#undef FDB_US_TOF_3_D
#undef FDB_US_TOF_4_D
#undef FDB_US_TOF_5_D
#undef FDB_US_TOF_6_D
#undef FDB_US_TOF_7_D
#endif

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

// used for function Update_TimeStamp()
static uint32_t TS_HOUR;
static uint32_t TS_MIN;
static uint32_t TS_SEC;

// Measured value of VCC voltage
static uint32_t VCC_VAL;

// Firmware Code User Revision
static uint32_t FWU_REV;
// Firmware Code User Range
static uint32_t FWU_RNG;
// Firmware Code User Checksum
static uint32_t FWCU_CS_by_HW;
static uint32_t FWCU_CS_by_SW;
static uint32_t FWCU_CS_by_NVRAM;

// Firmware Code ams Revision
static uint32_t FWA_REV;
// Firmware Code ams Checksum (Note: FWCA_CS is READ ONLY)
static uint32_t FWCA_CS_by_HW;
static uint32_t FWCA_CS_by_NVRAM;

// Firmware Data User Checksum
static uint32_t FWDU_CS_by_HW;
static uint32_t FWDU_CS_by_SW;
static uint32_t FWDU_CS_by_NVRAM;

// Firmware Data ams Checksum
static uint32_t FWDA_CS_by_HW;
static uint32_t FWDA_CS_by_SW;
static uint32_t FWDA_CS_by_NVRAM;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define WD_DIS_CODE ((uint32_t) 0x48DBA399) //Disables watchdog

// Resets & Inits
#define RC_SYS_RST   ((uint8_t) 0x99) //Resets GP30 completely
#define RC_SYS_INIT  ((uint8_t) 0x9A) //Resets whole GP30 without configuration registers
#define RC_SV_INIT   ((uint8_t) 0x9C) //Resets Supervisor

//Memory Access
#define RC_RAA_WR_RAM    ((uint8_t) 0x5A) //Write to RAM or register area
#define RC_RAA_WR_NVRAM  ((uint8_t) 0x5B) //Write to FW data area (NVRAM)
#define RC_RAA_WRS_RAM   ((uint8_t) 0x5E) //Write to RAM or register area, with read system status before write
#define RC_RAA_WRS_NVRAM ((uint8_t) 0x5F) //Write to FW data area (NVRAM), with read system status before write
#define RC_RAA_RD_RAM    ((uint8_t) 0x7A) //Read from RAM or register area
#define RC_RAA_RD_NVRAM  ((uint8_t) 0x7B) //Read to FW data area (NVRAM)
#define RC_RAA_RDS_RAM   ((uint8_t) 0x7E) //Read from RAM or register area, with read system status before read
#define RC_RAA_RDS_NVRAM ((uint8_t) 0x7F) //Read to FW data area (NVRAM), with read system status before read
#define RC_FWC_WR        ((uint8_t) 0x5C) //Write to FW code area (NVRAM)
#define RC_RD_STATUS     ((uint8_t) 0x8F) //Read system status only

//Measurement Task Request
#define RC_MT_REQ      ((uint8_t) 0xDA) //Measure Task Request
#define EC_MT_REQ_BIT0 ((uint8_t) 0x01) //VCC Voltage Measurement
#define EC_MT_REQ_BIT1 ((uint8_t) 0x02) //not used
#define EC_MT_REQ_BIT2 ((uint8_t) 0x04) //Time Of Flight Measurement
#define EC_MT_REQ_BIT3 ((uint8_t) 0x08) //Amplitude Measurement
#define EC_MT_REQ_BIT4 ((uint8_t) 0x10) //Amplitude Measurement Calibration
#define EC_MT_REQ_BIT5 ((uint8_t) 0x20) //Temperature Measurement
#define EC_MT_REQ_BIT6 ((uint8_t) 0x40) //High Speed Clock Calibration
#define EC_MT_REQ_BIT7 ((uint8_t) 0x80) //Zero Cross Calibration
   
//Debug System Commands
#define RC_TSC_CLR   ((uint8_t) 0x86) //Time stamp counter clear
#define RC_BM_RLS    ((uint8_t) 0x87) //Bus master release
#define RC_BM_REQ    ((uint8_t) 0x88) //Bus master request
#define RC_RF_CLR    ((uint8_t) 0x89) //Reset flag clear (RST_FLAG in SYS_STATUS)
#define RC_MCT_OFF   ((uint8_t) 0x8A) //Measure Cycle Timer Off
#define RC_MCT_ON    ((uint8_t) 0x8B) //Measure Cycle Timer On
#define RC_GPR_REQ   ((uint8_t) 0x8C) //General Purpose Request
#define RC_IF_CLR    ((uint8_t) 0x8D) //Interrupt Flags Clear
#define RC_COM_REQ   ((uint8_t) 0x8E) //Communication Request
#define RC_FW_CHKSUM ((uint8_t) 0xB8) //Builds checksum of all FW memories

//Read Results Addresses (Time Of Flight data)
#define FDB_US_TOF_ADD_ALL_U ((uint8_t) 0x80) //Ultrasonic TOF UP, sum of all TOF hits, up direction
#define FDB_US_PW_U          ((uint8_t) 0x81) //Ultrasonic pulse width ratio, up direction
#define FDB_US_AM_U          ((uint8_t) 0x82) //Ultrasonic amplitude value, up direction
#define FDB_US_AMC_VH        ((uint8_t) 0x83) //Ultrasonic amplitude calibration value, high
#define FDB_US_TOF_ADD_ALL_D ((uint8_t) 0x84) //Ultrasonic TOF DOWN, sum of all TOF hits, down direction
#define FDB_US_PW_D          ((uint8_t) 0x85) //Ultrasonic pulse width ratio, down direction
#define FDB_US_AM_D          ((uint8_t) 0x86) //Ultrasonic amplitude value, down direction
#define FDB_US_AMC_VL        ((uint8_t) 0x87) //Ultrasonic amplitude calibration value, low
                                         
#define FDB_US_TOF_0_U ((uint8_t) 0x88) //Ultrasonic TOF UP values 0, up direction
#define FDB_US_TOF_1_U ((uint8_t) 0x89) //Ultrasonic TOF UP values 1, up direction
#define FDB_US_TOF_2_U ((uint8_t) 0x8A) //Ultrasonic TOF UP values 2, up direction
#define FDB_US_TOF_3_U ((uint8_t) 0x8B) //Ultrasonic TOF UP values 3, up direction
#define FDB_US_TOF_4_U ((uint8_t) 0x8C) //Ultrasonic TOF UP values 4, up direction
#define FDB_US_TOF_5_U ((uint8_t) 0x8D) //Ultrasonic TOF UP values 5, up direction
#define FDB_US_TOF_6_U ((uint8_t) 0x8E) //Ultrasonic TOF UP values 6, up direction
#define FDB_US_TOF_7_U ((uint8_t) 0x8F) //Ultrasonic TOF UP values 7, up direction
#define FDB_US_TOF_8_U ((uint8_t) 0x90) //Ultrasonic TOF UP values 8, up direction
#define FDB_US_TOF_9_U ((uint8_t) 0x91) //Ultrasonic TOF UP values 9, up direction
#define FDB_US_TOF_0_D ((uint8_t) 0x92) //Ultrasonic TOF DOWN values 0, down direction
#define FDB_US_TOF_1_D ((uint8_t) 0x93) //Ultrasonic TOF DOWN values 1, down direction
#define FDB_US_TOF_2_D ((uint8_t) 0x94) //Ultrasonic TOF DOWN values 2, down direction
#define FDB_US_TOF_3_D ((uint8_t) 0x95) //Ultrasonic TOF DOWN values 3, down direction
#define FDB_US_TOF_4_D ((uint8_t) 0x96) //Ultrasonic TOF DOWN values 4, down direction
#define FDB_US_TOF_5_D ((uint8_t) 0x97) //Ultrasonic TOF DOWN values 5, down direction
#define FDB_US_TOF_6_D ((uint8_t) 0x98) //Ultrasonic TOF DOWN values 6, down direction
#define FDB_US_TOF_7_D ((uint8_t) 0x99) //Ultrasonic TOF DOWN values 7, down direction
#define FDB_US_TOF_8_D ((uint8_t) 0x9A) //Ultrasonic TOF DOWN values 8, down direction
#define FDB_US_TOF_9_D ((uint8_t) 0x9B) //Ultrasonic TOF DOWN values 9, down direction

//Read Results Addresses (Temperature Measurement data)
//...

//Read Results Addresses using Flow Meter Firmware (e.g. GP30-F01 or AS6031F1)
#define RAM_R_FLOW_VOLUME_INT      ((uint8_t) 0x00) //Integer part of total volume of water flow in cubic meters (fd0)
#define RAM_R_FLOW_VOLUME_FRACTION ((uint8_t) 0x01) //Fractional part of total volume of water flow in cubic meters (fd32)
#define RAM_R_FLOW_LPH             ((uint8_t) 0x02) //Presently calculated flow volume (l/h), unfiltered (fd16)
#define RAM_FILTERED_FLOW_LPH      ((uint8_t) 0x03) //Filtered flow volume (l/h) (fd16)
#define RAM_R_THETA                ((uint8_t) 0x04) //Temperature (�C) calculated from SUMTOF (fd16)
#define RAM_SOUND_VEL              ((uint8_t) 0x05) //Velocity of sound (m/s) (fd8)
#define RAM_FLOW_SPEED             ((uint8_t) 0x06) //Calculated speed of flow (m/s) (fd16)
#define RAM_R_TOF_DIFF             ((uint8_t) 0x07) //Current DIFTOF in raw TDC units (fd16)
#define RAM_R_TOF_SUM              ((uint8_t) 0x08) //Current SUMTOF in raw TDC units (fd16)

//Read Configuration Registers
#define CR_WD_DIS   ((uint8_t) 0x0C0) //Watchdog Disable
#define CR_IFC_CTRL ((uint8_t) 0x0C1) //Interfaces Control
#define CR_GP_CTRL  ((uint8_t) 0x0C2) //General Purpose Control
#define CR_USM_OPT  ((uint8_t) 0x0C3) //USM: Options
#define CR_IEH      ((uint8_t) 0x0C4) //Interrupt & Error Handling
#define CR_CPM      ((uint8_t) 0x0C5) //Clock & Power Management
#define CR_MRG_TS   ((uint8_t) 0x0C6) //Measure Rate Generator & Task Sequencer
#define CR_TPM      ((uint8_t) 0x0C7) //Temperature Measurement
#define CR_USM_PRC  ((uint8_t) 0x0C8) //USM: Processing
#define CR_USM_FRC  ((uint8_t) 0x0C9) //USM: Fire & Receive Control
#define CR_USM_TOF  ((uint8_t) 0x0CA) //USM: Time of Flight
#define CR_USM_AM   ((uint8_t) 0x0CB) //USM: Amplitude Measurement
#define CR_TRIM1    ((uint8_t) 0x0CC) //Trim Parameter
#define CR_TRIM2    ((uint8_t) 0x0CD) //Trim Parameter
#define CR_TRIM3    ((uint8_t) 0x0CE) //Trim Parameter

//Read Special Handlung Registers
#define SHR_TOF_RATE          ((uint8_t) 0xD0) //Time-of-Flight rate
#define SHR_USM_RLS_DLY_U     ((uint8_t) 0xD1) //Multi-hit Start Delay Up
#define SHR_USM_RLS_DLY_D     ((uint8_t) 0xD2) //Multi-hit Start Delay Down
#define SHR_GPO               ((uint8_t) 0xD3) //General Purpose Out
#define SHR_PI_NPULSE         ((uint8_t) 0xD4) //Pulse Interface Number of Pulses
#define SHR_PI_TPA            ((uint8_t) 0xD5) //Pulse Interface Time Pulse Distance
#define SHR_PI_IU_TIME        ((uint8_t) 0xD6) //Pulse Interface, Internal Update Time Distance
#define SHR_PI_IU_NO          ((uint8_t) 0xD7) //Pulse Interface Number of internal Update
                                       
#define SHR_ZCD_LVL           ((uint8_t) 0xD9) //Zero Cross Detection, Level
#define SHR_ZCD_FHL_U         ((uint8_t) 0xDA) //Zero Cross Detection, First Hit Level Up
#define SHR_ZCD_FHL_D         ((uint8_t) 0xDB) //Zero Cross Detection, First Hit Level Down
#define SHR_CPU_REQ           ((uint8_t) 0xDC) //CPU Requests
#define SHR_EXC               ((uint8_t) 0xDD) //Executables
#define SHR_RC                ((uint8_t) 0xDE) //Remote Control
#define SHR_RC_RLS            ((uint8_t) 0xDF) //Release Code for actions of SHR_RC

//Read Status & Result Registers
#define SRR_IRQ_FLAG    ((uint8_t) 0xE0) //Interrupt Flags
#define SRR_ERR_FLAG    ((uint8_t) 0xE1) //Error Flags
#define SRR_FEP_STF     ((uint8_t) 0xE2) //Frontend Processing Status Flags
#define SRR_GPI         ((uint8_t) 0xE3) //General Purpose In
#define SRR_HCC_VAL     ((uint8_t) 0xE4) //High-Speed Clock Calibration Value
#define SRR_VCC_VAL     ((uint8_t) 0xE5) //Measurement Value for VCC Voltage
#define SRR_TSV_HOUR    ((uint8_t) 0xE6) //Time Stamp Value: Hours
#define SRR_TSV_MIN_SEC ((uint8_t) 0xE7) //Time Stamp Value: Minutes & Seconds
                                    
#define SRR_TS_TIME     ((uint8_t) 0xE9) //Task Sequencer Time
#define SRR_MSC_STF     ((uint8_t) 0xEA) //Miscellaneous Status Flags
#define SRR_I2C_RD      ((uint8_t) 0xEB) //2-wire Master Interface Read Data
#define SRR_FWU_RNG     ((uint8_t) 0xEC) //Range Firmware Code User
#define SRR_FWU_REV     ((uint8_t) 0xED) //Revision Firmware Code User
#define SRR_FWA_REV     ((uint8_t) 0xEE) //Revision Firmware Code ams


//Bit Mask of SRR_IRQ_FLAG (Interrupt Flags) 0x0E0
#define TSQ_FNS_mask      ((uint8_t) 0x00) //check bit 0
#define FW_TRANS_FNS_mask ((uint8_t) 0x02) //check bit 1
#define BLD_FNS_mask      ((uint8_t) 0x04) //check bit 2
#define CHKSUM_FNS_mask   ((uint8_t) 0x08) //check bit 3

//Bit Mask of SRR_FEP_STF (Frontend Processing Status Flags) 0x0E2
//Updated in Cycle Trigger A (flow) = bit 9, 8, 6, 5, 4
//Updated in Cycle Trigger B (temperature) = bit 3, 1, 0
#define US_AMC_UPD_mask  ((uint32_t) (1<<9)) //Ultrasonic Update for AMC measurement
#define US_AM_UPD_mask   ((uint32_t) (1<<8)) //Ultrasonic Update for AM measurement
#define US_TOF_EDGE_mask ((uint32_t) (1<<7)) //TOF Measurement Edge
#define US_TOF_UPD_mask  ((uint32_t) (1<<6)) //Ultrasonic Update for TOF measurement
#define US_D_UPD_mask    ((uint32_t) (1<<5)) //Ultrasonic Update in Down direction
#define US_U_UPD_mask    ((uint32_t) (1<<4)) //Ultrasonic Update in Up direction
#define TPM_ST_mask      ((uint32_t) (1<<3)) //Temperature Subtask
#define TM_UPD_mask      ((uint32_t) (1<<1)) //Temperature Measurement Update
#define HCC_UPD_mask     ((uint32_t) (1<<0)) //High-Speed Clock Calibration Update

//CR_IEH (Interfaces Control) Addr. 0x0C4
#define IRQ_EN_TSQ_FNS_mask    ((uint32_t) 0x10000) //Interrupt Request Enable, Task Sequencer finished
#define IRQ_EN_TRANS_FNS_mask  ((uint32_t) 0x20000) //Interrupt Request Enable, FW Transaction finished
#define IRQ_EN_BLD_FNS_mask    ((uint32_t) 0x40000) //Interrupt Request Enable, Bootload finished
#define IRQ_EN_CHKSUM_FNS_mask ((uint32_t) 0x80000) //Interrupt Request Enable, Checksum generation finished


//Firmware Status


//CR_USM_TOF (Ultrasonic Measurement Time of Flight) 0x0CA
#define TOF_HIT_SUM_NO_mask ((uint32_t) 0x00001F00) //TOF_HIT_NO: Number of TOF hits taken for TDC measurement

volatile uint32_t TOF_HIT_SUM_NO = 0;


/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __USER_AS6031_PARAMETER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

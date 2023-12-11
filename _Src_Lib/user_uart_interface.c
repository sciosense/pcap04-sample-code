/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    huart2_interface.c
  * @brief   SPI Interface Communication Routines, including UART.
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

#include "inc/user_uart_interface.h"
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

extern UART_HandleTypeDef huart2;

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
  * @brief  Waiting for interrupt and polling the UART RX.
  * @param  RX_STR pointer to RX buffer
  * @param  size amount of data to be received
  * @retval none
  */
void Waiting_For_INTN_UART(char *RX_STR) //size is missing (15-1)
{
  /* waiting for the interrupt INTN (low active) */
  while(HAL_GPIO_ReadPin(INTN_GPIO_Port, INTN_Pin))
  {
    HAL_UART_Receive(&huart2, (uint8_t*)RX_STR, 15-1, 1);
  }

  return;
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

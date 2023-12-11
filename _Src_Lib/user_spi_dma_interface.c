/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    user_spi_dma_interface.c
  * @brief   SPI Interface Communication Routines, including DMA Controller.
  *
  @verbatim
  ==============================================================================
                     ##### How to use this driver #####
  ==============================================================================
    SPI Timings
      SSN enable to valid latch clock = approx. typ.= 6us (min=300ns)
      SSN hold time after SCK falling = approx. typ.=12us (min=300ns)

    Transmitting and Receiving Examples
    //declaration
      uint8_t spiTX[] = { 0, 0, 0 };
      uint8_t spiRX[] = { 0, 0, 0 };
      spiTX[0]=0xAA; //opcode
      spiTX[1]=0xBB; //dummy byte
      spiTX[2]=0xCC; //dummy byte

    //Example: transmitting bytes without gap in between
      Set_SSN(LOW);
      HAL_SPI_DMA_Transmit(&hspi1, spiTX, 3);
    //SSN goes high by using HAL_SPI_TxCpltCallback()

    //Example: transmitting and receiving byte(s) without gap in between
      Set_SSN(LOW);
      HAL_SPI_TransmitReceive_DMA(&hspi1, spiTX, spiRX, 3);
    //SSN goes high by using HAL_SPI_TxRxCpltCallback()

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

#include "inc/user_spi_dma_interface.h"
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

extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;

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
/*                         Tx + Rx auto incrementally                         */
/* Note: Used for TDC (e.g.AS6500)                                            */
/*       Jump into subroutine consumes about 1 us                             */
/*       Without using any Callback Routines                                  */
/******************************************************************************/
/**
  * @brief  Transmit and Receive an amount of data in non-blocking mode with DMA.
  * @param  pTxData pointer to transmission data buffer
  * @param  pRxData pointer to reception data buffer
  * @param  Size amount of data to be sent
  * @retval none
  */
void My_TransmitReceive_DMA(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
	/** @hspi1         pointer to a SPI_HandleTypeDef structure that contains
	  *                  the configuration information for SPI module.
	  * @hdma_spi1_tx  pointer to a DMA_HandleTypeDef structure that contains
	  *                  the configuration information for DMA module.
	  * @hdma_spi1_rx  pointer to a DMA_HandleTypeDef structure that contains
	  *                  the configuration information for DMA module. */

	hspi1.Instance->CR2 |= SPI_CR2_TXDMAEN;							// Enable Tx DMA Request
	hspi1.Instance->CR2 |= SPI_CR2_RXDMAEN;							// Enable Rx DMA Request

	hdma_spi1_tx.Instance->CCR &= ~DMA_CCR_EN;						// Channel enable
	hdma_spi1_tx.Instance->CPAR = (uint32_t)&hspi1.Instance->DR;	// DMA channel x peripheral address register
	hdma_spi1_tx.Instance->CMAR = (uint32_t)pTxData;				// DMA channel x memory address register
	hdma_spi1_tx.Instance->CNDTR = Size;							// DMA channel x number of data register

	// Put SSN low - Activate
	PUT_SSN_LOW;

	hdma_spi1_tx.Instance->CCR |= DMA_CCR_EN;						// Channel enable
	hdma_spi1_rx.Instance->CCR &= ~DMA_CCR_EN;						// Channel enable
	hdma_spi1_rx.Instance->CPAR = (uint32_t)&hspi1.Instance->DR;	// DMA channel x peripheral address register
	hdma_spi1_rx.Instance->CMAR = (uint32_t)pRxData;				// DMA channel x memory address register
	hdma_spi1_rx.Instance->CNDTR = Size;							// DMA channel x number of data register
	hdma_spi1_rx.Instance->CCR |= DMA_CCR_EN;						// Channel enable

	hspi1.Instance->CR1 |= SPI_CR1_SPE;								// SPI Enable

	while((hspi1.Instance->SR & SPI_SR_RXNE)!=RESET); // Receive buffer Not Empty
	while((hspi1.Instance->SR & SPI_SR_BSY)!=RESET); // Busy flag

	// Put SSN high - Deactivate
	PUT_SSN_HIGH;

	return;
}

/******************************************************************************/
/*                           Tx auto incrementally                            */
/* Note: Used for TDC (e.g.AS6500)                                            */
/*       Jump into subroutine consumes about 1 us                             */
/*       Without using any Callback Routines                                  */
/******************************************************************************/
/**
  * @brief  Transmit and Receive an amount of data in non-blocking mode with DMA.
  * @param  pTxData pointer to transmission data buffer
  * @param  Size amount of data to be sent
  * @retval none
  */
void My_Transmit_DMA(uint8_t *pTxData, uint16_t Size)
{
	/** @hspi1         pointer to a SPI_HandleTypeDef structure that contains
	  *                  the configuration information for SPI module.
	  * @hdma_spi1_tx  pointer to a DMA_HandleTypeDef structure that contains
	  *                  the configuration information for DMA module.
	  * @hdma_spi1_rx  pointer to a DMA_HandleTypeDef structure that contains
	  *                  the configuration information for DMA module. */

	// Work Around
	// Tx and Rx uses the same DMA channel x memory address register
	hspi1.Instance->CR2 |= SPI_CR2_TXDMAEN;							// Enable Tx DMA Request
	hspi1.Instance->CR2 |= SPI_CR2_RXDMAEN;							// Enable Rx DMA Request

	hdma_spi1_tx.Instance->CCR &= ~DMA_CCR_EN;						// Channel enable
	hdma_spi1_tx.Instance->CPAR = (uint32_t)&hspi1.Instance->DR;	// DMA channel x peripheral address register
	hdma_spi1_tx.Instance->CMAR = (uint32_t)pTxData;				// DMA channel x memory address register
	hdma_spi1_tx.Instance->CNDTR = Size;							// DMA channel x number of data register

	// Put SSN low - Activate
	SSN_GPIO_Port->BRR = (uint32_t)SSN_Pin;

	hdma_spi1_tx.Instance->CCR |= DMA_CCR_EN;						// Channel enable
	hdma_spi1_rx.Instance->CCR &= ~DMA_CCR_EN;						// Channel enable
	hdma_spi1_rx.Instance->CPAR = (uint32_t)&hspi1.Instance->DR;	// DMA channel x peripheral address register
	hdma_spi1_rx.Instance->CMAR = (uint32_t)pTxData;				// DMA channel x memory address register
	hdma_spi1_rx.Instance->CNDTR = Size;							// DMA channel x number of data register
	hdma_spi1_rx.Instance->CCR |= DMA_CCR_EN;						// Channel enable

	hspi1.Instance->CR1 |= SPI_CR1_SPE;								// SPI Enable

	while((hspi1.Instance->SR & SPI_SR_RXNE)!=RESET); // Receive buffer Not Empty
	while((hspi1.Instance->SR & SPI_SR_BSY)!=RESET); // Busy flag

	// Put SSN high - Deactivate
	SSN_GPIO_Port->BSRR = (uint32_t)SSN_Pin;

	return;
}

/******************************************************************************/
/*                             Clearing the FIFOs                             */
/* Note: Used for TDC (e.g.AS6500)                                            */
/*       Jump into subroutine consumes about 1 us                             */
/*       Without using any Callback Routines                                  */
/******************************************************************************/
/**
  * @brief  Clearing the FIFOs by reading as long as INTN goes high.
  * @retval none
  */
void My_Clearing_All_FIFOs(void)
{
	//Clearing all four FIFOs
	uint32_t count = 0;
	uint8_t read_loop = 0;

	uint8_t spiTX[33]; // using same sizes (Rx, Tx)
	uint8_t spiRX[33]; // Max. Reading 4x (REF_IDX[3 Bytes], TSTOP[3 Bytes]), plus 8 Bytes plus opcode
	uint8_t RD_x_Bytes = 24;
	spiTX[0] = 0x68; // Opcode + Address

	count = 0; //counts the 0xFF

	while (count != RD_x_Bytes)
	{
		// Reading takes about 10us @SPI=20 MHz
		// Note: AS6500 Data Sheet, Section 8.8.4
		//       FIFOs for Adapting Peak and Average Conversion Rate
		My_TransmitReceive_DMA(spiTX, spiRX, RD_x_Bytes+1);

		// Counts each 0xFF
		for (int i=1; i<RD_x_Bytes+1; i++)
		{
			if (spiRX[i] == 0xFF) {
				count++;
			}
		}
		read_loop++;

	} // End of WHILE() or IF(My_INTN_State==0)

	return;
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

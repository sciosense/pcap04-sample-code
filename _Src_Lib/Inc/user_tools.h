/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    user_tools.h
  * @brief   This file contains the headers of useful tools.
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
#ifndef __USER_TOOLS_H
#define __USER_TOOLS_H

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

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/

/* USER CODE BEGIN EFP */
extern void   Delay_100us                (uint32_t delay);
extern float  Two_s_Complement_Conversion(uint32_t raw_number, int bit, float mult_factor);
extern int    convert_hex2dec            (char *char_input);
extern int    convert_gray2dec           (int gray_input);
extern char   *dtostrf                   (double val, signed char width, unsigned char prec, char *sout);
extern double MyPower                    (int y);
extern void   Printf_Current_Date        (void);
extern void   Printf_Current_Time        (void);

extern uint32_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint32_t BufferLength); //static vs extern

// Output to the SWV Data Trace console
extern int   _write      (int32_t file, uint8_t *ptr, int32_t len);
extern int   __io_putchar(int ch);

extern void Pulse_Generator(GPIO_TypeDef* gpio_port, uint16_t gpio_pin, int number, int delay);
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __USER_TOOLS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

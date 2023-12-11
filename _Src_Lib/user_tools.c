/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    user_tools.c
  * @brief   Useful Tools and Routines.
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
  *
  * How to use Debug Exception and Monitor Control Register (DEMCR)
  *
  * First, enable the cycle counter once at startup:
  *    //Enable trace and debug block DEMCR
  *    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  *    DWT->CYCCNT = 0;                                // Reset cycle counter
  *    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // Enable cycle counter
  *
  * then, you can access its value:
  *    unsigned long t1 = DWT->CYCCNT; //Read cycle counter register
  *    // do something
  *    unsigned long t2 = DWT->CYCCNT; //Read cycle counter register
  *    unsigned long diff = t2 - t1;
  *    // or difference in seconds
  *    float diff_sec = (float)(t2 - t1) / SystemCoreClock;
  *
  * Second, disable the cycle counter once at the end:
  *    //Disable trace and debug block DEMCR
  *    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
  *    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; // Disable cycle counter
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "inc\user_tools.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <time.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CLEAR(array) memset(&(array), '\0', sizeof(array))

#define ENABLE_CYCLE_COUNTER  	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;		\
								DWT->CYCCNT = 0; 									\
								DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk

#define TIME_DIFF_TO(x) 		((float)(DWT->CYCCNT - x) / SystemCoreClock)

#define TIME_MILLI(x)			(float)(x/1000.0)		// calculates x into [ms]
#define TIME_MICRO(x)			(float)(x/1000000.0)	// calculates x into [us]
#define TIME_NANO(x)			(float)(x/1000000000.0)	// calculates x into [ns]

#define TIME_ms(x)				(float)(x*1000.0)		// result in [ms]
#define TIME_us(x)				(float)(x*1000000.0)	// result in [us]
#define TIME_ns(x)				(float)(x*1000000000.0)	// result in [ns]

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
time_t t;
struct tm * ts;

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
/*                               Delay of 100us                               */
/******************************************************************************/
/**
  * @brief  This function creates delay in steps of 100us
  * @param  delay (uint32_t) duration in steps of 100 microsecond [100us]
  * @retval none
  */
void Delay_100us(uint32_t delay)
{
  uint32_t counter = 0;
  uint32_t adjust = 2000;
  
  delay *= adjust;
  
  while (counter < delay)
  {
    counter++;
  }
}

/******************************************************************************/
/*                    Two's Complement to Decimal Conversion                  */
/******************************************************************************/
/**
  * @brief  This function takes about ~88�s, using POW() two times! Means,
  *             each function call of POW() takes approx. 40�s AND header 
  *             file is needed! 
  *                     #include <tgmath.h>
  *                     [..]
  *                     exp = POW(2, bit);
  *                     half_exp = POW(2, bit-1);
  *
  *             ON THE OTHER HAND, using if-clauses, THIS function takes ~2�s
  *                     [..]
  *                     if (bit==16) exp = 65536;
  *                     half_exp = exp / 2;
  *
  *             Definition Two's Complement: Negative numbers in binary
  *                     Given a set of all possible N-bit values, we can assign
  *                     the lower (by the binary value) half to be the integers 
  *                     from 0 to (2^N-1 - 1) inclusive and the upper half to 
  *                     be (-2N-1) to -1 inclusive
  *
  *             Example:
  *                     // divided by 2^16 (fpp), multipled by 250ns (e.g. T_ref)
  *                     FLOAT_Value = Two_s_Complement_Conversion(HEX_value, 16, 250E-9); 
  *
  * @param  raw_number (uint32_t) 
  * @param  bit (int) 
  * @param  mult_factor (float) 
  * @retval Two's Complement (float)
  */
float Two_s_Complement_Conversion(uint32_t raw_number, int bit, float mult_factor)
{
  float number;
  double exp, half_exp;

  /* determine the 'power of 2' */
  if (bit==32) exp = 4294967296;  /* = 2^32 */
  if (bit==24) exp = 16777216;    /* = 2^24 */
  if (bit==16) exp = 65536;       /* = 2^16 */
  if (bit==8)  exp = 256;         /* = 2^8 */

  half_exp = exp / 2;
  
  number = raw_number / exp;

  if (number <= (half_exp - 1)) {
    /*positive number, nothing to do */
  } else { /**/
    /*to get negative number */
    number -= exp;
  }

  /*to get the correct result by multiplication factor */
  number *= mult_factor;
  
  return number;
}

/******************************************************************************/
/*                             Converts HEX to DEC                            */ 
/******************************************************************************/
/**
  * @brief  This function converts up to 8 hex characters (incl. character 0..9, 
  *             A..F, a..f) into corresponding decimal value.
  *             #include <string.h>
  *
  *             Example:
  *                     char* hex_str = "ba34F";
  *                     uint32_t dec_val = convert_hex2dec(hex_str);
  *                     printf("hex = 0x%s \tdec = %u \n", hex_str, dec_val);
  *
  *             Error Handling:
  *                     hex character > 8 --> return max value (0xFFFFFFFF)
  *                     undefined hex character --> return 0
  *
  * @param  char_input is the pointer of HEX-string, max. 8 characters
  * @retval integer value of corresponding decimal value
  */
int convert_hex2dec(char *char_input)
{
  int length = strlen(char_input);

  int final_val = 0;
  int dec_val = 0;
  int factor = 0;

  if (length > 8) {
    /* overloaded */
    return -1; /* 0xFFFF FFFF = 2^32 - 1 */
  }
  
  for (int i = 0; i < length; i++) {
    /* multiplication factor for related character*/
    if ((length-1) - i == 0) factor = 1;         /* 2^0 */
    if ((length-1) - i == 1) factor = 16;        /* 2^4 */
    if ((length-1) - i == 2) factor = 256;       /* 2^8 */
    if ((length-1) - i == 3) factor = 4096;      /* 2^12 */
    if ((length-1) - i == 4) factor = 65536;     /* 2^16 */
    if ((length-1) - i == 5) factor = 1048576;   /* 2^20 */
    if ((length-1) - i == 6) factor = 16777216;  /* 2^24 */
    if ((length-1) - i == 7) factor = 268435456; /* 2^28 */
    
    /* set value to zero*/
    dec_val = 0;
    
    /* in case ASCII character is not allowed - Error Handling */
    if (*char_input < 48 || (*char_input > 70 && *char_input < 97) || *char_input >102) return 0;
    
    /* convert ASCII code into decimal */
    if (*char_input == 48)                       dec_val =  0; /* char(0) */
    if (*char_input == 49)                       dec_val =  1; /* char(1) */
    if (*char_input == 50)                       dec_val =  2; /* char(2) */
    if (*char_input == 51)                       dec_val =  3; /* char(3) */
    if (*char_input == 52)                       dec_val =  4; /* char(4) */
    if (*char_input == 53)                       dec_val =  5; /* char(5) */
    if (*char_input == 54)                       dec_val =  6; /* char(6) */
    if (*char_input == 55)                       dec_val =  7; /* char(7) */
    if (*char_input == 56)                       dec_val =  8; /* char(8) */
    if (*char_input == 57)                       dec_val =  9; /* char(9) */
    if (*char_input == 65 || *char_input == 97)  dec_val = 10; /* char(A) */
    if (*char_input == 66 || *char_input == 98)  dec_val = 11; /* char(B) */
    if (*char_input == 67 || *char_input == 99)  dec_val = 12; /* char(C) */
    if (*char_input == 68 || *char_input == 100) dec_val = 13; /* char(D) */
    if (*char_input == 69 || *char_input == 101) dec_val = 14; /* char(E) */
    if (*char_input == 70 || *char_input == 102) dec_val = 15; /* char(F) */
  
    dec_val *= factor;
    final_val += dec_val;
    
    char_input++;
  }
  
  return final_val;
}

/******************************************************************************/
/*                          Converts Gray to Decimal                          */ 
/******************************************************************************/
/**
  * @brief  Converts 2-bit gray code to decimal
  *         Definition of Gray code, also known as reflected binary code,
  *             is a code having digits 0 and 1. Gray code do not have place 
  *             value for its digits. Any successive codes in Gray code system 
  *             have only one bit changes.
  *
  *         Error Handling:
  *             gray code is not allowed --> return 99
  *
  * @param  gray_input (integer)
  * @retval decimal
  */
int convert_gray2dec(int gray_input)
{
  int dec_val = 0;
  
  /* in case gray code is not allowed - Error Handling */
  if (gray_input < 0 || gray_input > 3) return 99; 
   
  if (gray_input == 0) dec_val = 0; /* Gray Code 00 (dec. 0) --> 0 */
  if (gray_input == 1) dec_val = 1; /* Gray Code 01 (dec. 1) --> 1 */
  if (gray_input == 3) dec_val = 2; /* Gray Code 11 (dec. 3) --> 2 */
  if (gray_input == 2) dec_val = 3; /* Gray Code 10 (dec. 2) --> 3 */

  return dec_val;
}

/******************************************************************************/
/*                          Converts Float to String                          */ 
/******************************************************************************/
/**
  * @brief  This function converts the double value passed in val into an ASCII 
  *             representation that will be stored under sout. The caller is 
  *             responsible for providing sufficient storage in sout.
  *
  *         Conversion is done in the format "[-]d.ddd". The minimum field 
  *             width of the output string (including the possible '.' and the 
  *             possible sign for negative values) is given in width, and prec 
  *             determines the number of digits after the decimal sign. width 
  *             is signed value, negative for left adjustment.
  *
  *         Example:
  *             dtostrf(Value*1E9, 7, 3, string);
  *
  * @param  val (double)
  * @param  width (signed char)
  * @param  prec (unsigned char)
  * @param  sout (char) pointer to the converted string sout
  * @retval decimal
  */
char *dtostrf (double val, signed char width, unsigned char prec, char *sout) {

  char fmt[20];

  sprintf(fmt, "%%%d.%df", width, prec);

  sprintf(sout, fmt, val);

  return sout;

}

/******************************************************************************/
/*                            Simple Power Function                           */
/******************************************************************************/
/**
  * @brief  This function executes 2.0 raised to the power @param y
  * @param  y (int) power value
  * @retval power of y, x^y
  */
double MyPower(int y)
{
	/* Range of a double
	 * A double is a 64-bit IEEE 754 floating point. */
	double factor = 0;

	/* Alternative Solution
	 * double pow(double base, double power)
	 *   needed library, #include <math.h>
	 *   disadvantage, consumes too many time */

	/* Lookup Table */
	if (y == 0 ) factor = 1;                     /* 2^0  */
	if (y == 1 ) factor = 2;                     /* 2^1  */
	if (y == 2 ) factor = 4;                     /* 2^2  */
	if (y == 3 ) factor = 8;                     /* 2^3  */
	if (y == 4 ) factor = 16;                    /* 2^4  */
	if (y == 5 ) factor = 32;                    /* 2^5  */
	if (y == 6 ) factor = 64;                    /* 2^6  */
	if (y == 7 ) factor = 128;                   /* 2^7  */
	if (y == 8 ) factor = 256;                   /* 2^8  */
	if (y == 9 ) factor = 512;                   /* 2^9  */
	if (y == 10) factor = 1024;                  /* 2^10 */
	if (y == 11) factor = 2048;                  /* 2^11 */
	if (y == 12) factor = 4096;                  /* 2^12 */
	if (y == 13) factor = 8192;                  /* 2^13 */
	if (y == 14) factor = 16384;                 /* 2^14 */
	if (y == 15) factor = 32768;                 /* 2^15 */
	if (y == 16) factor = 65536;                 /* 2^16 */
	if (y == 17) factor = 131072;                /* 2^17 */
	if (y == 18) factor = 262144;                /* 2^18 */
	if (y == 19) factor = 524288;                /* 2^19 */
	if (y == 20) factor = 1048576;               /* 2^20 */
	if (y == 21) factor = 2097152;               /* 2^21 */
	if (y == 22) factor = 4194304;               /* 2^22 */
	if (y == 23) factor = 8388608;               /* 2^23 */
	if (y == 24) factor = 16777216;              /* 2^24 */
	if (y == 25) factor = 33554432;              /* 2^25 */
	if (y == 26) factor = 67108864;              /* 2^26 */
	if (y == 27) factor = 134217728;             /* 2^27 */
	if (y == 28) factor = 268435456;             /* 2^28 */
	if (y == 29) factor = 536870912;             /* 2^29 */
	if (y == 30) factor = 1073741824;            /* 2^30 */
	if (y == 31) factor = 2147483648;            /* 2^31 */
	if (y == 32) factor = 4294967296;            /* 2^32 */
	if (y == 33) factor = 8589934592;            /* 2^33 */
	if (y == 34) factor = 17179869184;           /* 2^34 */
	if (y == 35) factor = 34359738368;           /* 2^35 */
	if (y == 36) factor = 68719476736;           /* 2^36 */
	if (y == 37) factor = 137438953472;          /* 2^37 */
	if (y == 38) factor = 274877906944;          /* 2^38 */
	if (y == 39) factor = 549755813888;          /* 2^39 */
	if (y == 40) factor = 1099511627776;         /* 2^40 */
	if (y == 41) factor = 2199023255552;         /* 2^41 */
	if (y == 42) factor = 4398046511104;         /* 2^42 */
	if (y == 43) factor = 8796093022208;         /* 2^43 */
	if (y == 44) factor = 17592186044416;        /* 2^44 */
	if (y == 45) factor = 35184372088832;        /* 2^45 */
	if (y == 46) factor = 70368744177664;        /* 2^46 */
	if (y == 47) factor = 140737488355328;       /* 2^47 */
	if (y == 48) factor = 281474976710656;       /* 2^48 */
	if (y == 49) factor = 562949953421312;       /* 2^49 */
	if (y == 50) factor = 1125899906842620;      /* 2^50 */
	if (y == 51) factor = 2251799813685250;      /* 2^51 */
	if (y == 52) factor = 4503599627370500;      /* 2^52 */
	if (y == 53) factor = 9007199254740990;      /* 2^53 */
	if (y == 54) factor = 18014398509482000;     /* 2^54 */
	if (y == 55) factor = 36028797018964000;     /* 2^55 */
	if (y == 56) factor = 72057594037927900;     /* 2^56 */
	if (y == 57) factor = 144115188075856000;    /* 2^57 */
	if (y == 58) factor = 288230376151712000;    /* 2^58 */
	if (y == 59) factor = 576460752303423000;    /* 2^59 */
	if (y == 60) factor = 1152921504606850000;   /* 2^60 */
	if (y == 61) factor = 2305843009213690000;   /* 2^61 */
	if (y == 62) factor = 4611686018427390000;   /* 2^62 */
	if (y == 63) factor = 9223372036854780000.;   /* 2^63 */ /* Dot is needed, because of Warning Message: */
	if (y == 64) factor = 18446744073709600000.;  /* 2^64 */ /*   Integer constant is too large that it is unsigned */

	return factor;
}

/******************************************************************************/
/*                      Print current date on Terminal IO                     */
/******************************************************************************/
/**
  * @brief  This function prints current date on Terminal IO
  * @retval none
  */
void Printf_Current_Date(void)
{
  /* Reading the time information */
  t = time(NULL);
  ts = localtime(&t);
  
//  printf("%s", asctime(ts));
  
  /* Format of the printed date, US-Format
     e.g. "Thursday, October 12th, 2018" */
  switch (ts->tm_wday)
  {
  case 1:
    printf("Monday");           // Mon
    break;
  case 2:
    printf("Tuesday");          // Tue
    break;  
  case 3:
    printf("Wednesday");        // Wed
    break;  
  case 4:
    printf("Thursday");         // Thu
    break;  
  case 5:
    printf("Friday");           // Fri
    break;  
  case 6:
    printf("Saturday");         // Dat
    break;  
  case 7:
    printf("Sunday");           // Sun
    break;
  default:
    printf(" ");
  }
  printf(", ");
  switch (ts->tm_mon+1)
  {
  case 1:
    printf("January");          // Jan
    break;
  case 2:
    printf("February");         // Feb
    break;  
  case 3:
    printf("March");            // Mar
    break;  
  case 4:
    printf("April");            // Apr
    break;  
  case 5:
    printf("May");              // May
    break;  
  case 6:
    printf("June");             // June
    break;  
  case 7:
    printf("July");             // July
    break;
  case 8:
    printf("August");           // Aug
    break;
  case 9:
    printf("September");        // Sept
    break;
  case 10:
    printf("October");          // Oct
    break;
  case 11:
    printf("November");         // Nov
    break;
  case 12:
    printf("December");         // Dec
    break;
  default:
    printf(" ");
  }
  printf(" ");
  switch (ts->tm_mday)
  {
  case 1:
    printf("1st");
    break;
  case 2:
    printf("2nd");
    break;  
  case 3:
    printf("3rd");
    break;
  default:
    printf("%dth", ts->tm_mday);
  }
  printf(", ");
  printf("%d\n", ts->tm_year+1900);
}

/******************************************************************************/
/*                      Print current time on Terminal IO                     */
/******************************************************************************/
/**
  * @brief  This function prints current time on Terminal IO
  * @retval none
  */
void Printf_Current_Time(void)
{
  /* Reading the time information */
  t = time(NULL);
  ts = localtime(&t);
  
  /* Format of the printed time
     e.g. "10:02:17" */
  printf("%02d:%02d:%02d\n", ts->tm_hour+2, ts->tm_min, ts->tm_sec);
}

/******************************************************************************/
/*                            Compare two buffers                             */
/******************************************************************************/
/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval 0  : pBuffer1 identical to pBuffer2
  *         >0 : pBuffer1 differs from pBuffer2
  */
extern uint32_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint32_t BufferLength) //static vs extern
{
  while (BufferLength--)
  {
    if((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}

/******************************************************************************/
/*            redirect printf output to the SWV Data Trace console            */
/******************************************************************************/
/**
  * @brief  Using SWV Trace (with printf) on STM32
  *         Just make sure stdio.h is included
  * @retval none
  */
int _write(int32_t file, uint8_t *ptr, int32_t len) {
	/* Implement your write code here, this is used by puts and printf for example */
	int i = 0;
	for(i=0; i < len; i++) {
		ITM_SendChar((*ptr++));
	}
	return len;
}

/******************************************************************************/
/*                 overwrite the more low level __io_putchar()                */
/******************************************************************************/
/**
  * @brief  Switch printf() to the debug interface
  *         Often you find the describtion to overwrite _write().
  *         That’s correct.
  *         In case of the STM32CubeIDE generated code it is also possible
  *         to overwrite the more low level __io_putchar().
  * @retval none
  */


int __io_putchar(int ch) {
    ITM_SendChar(ch);
    return ch;
}


void Pulse_Generator(GPIO_TypeDef* gpio_port, uint16_t gpio_pin, int number, int delay) {
	int i = 0, j = 0;
#define _Pulse_Generator_Quick_GPIO

	for (i = 0; i < number; i++) {
#ifndef _Pulse_Generator_Quick_GPIO
		HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_SET);
#else
		gpio_port->BSRR = (uint32_t)gpio_pin;
#endif
		//Delay_100us(delay);
		for (j = 0; j < delay; j++) {}

#ifndef _Pulse_Generator_Quick_GPIO
		HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_RESET);
#else
		gpio_port->BRR = (uint32_t)gpio_pin;
#endif
		//Delay_100us(delay);
		for (j = 0; j < delay; j++) {}
	}
}



/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

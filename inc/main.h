/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define FLASH_START_ADRESS 0x08000000
#define FLASH_END_ADDRESS  0x08040000
#define DUMMY_64		   0x0123456789abcdef
#define VRT1_Pin GPIO_PIN_0
#define VRT1_GPIO_Port GPIOA
#define VDS_Pin GPIO_PIN_4
#define VDS_GPIO_Port GPIOA
#define VREF_Pin GPIO_PIN_5
#define VREF_GPIO_Port GPIOA
#define TAG_IRQ_Pin GPIO_PIN_2
#define TAG_IRQ_GPIO_Port GPIOB
#define TAG_IRQ_EXTI_IRQn EXTI2_IRQn
#define SPI2_SS_Tag_Pin GPIO_PIN_12
#define SPI2_SS_Tag_GPIO_Port GPIOB
#define DA_Pin GPIO_PIN_5
#define DA_GPIO_Port GPIOB
#define DB_Pin GPIO_PIN_6
#define DB_GPIO_Port GPIOB
#define DC_Pin GPIO_PIN_7
#define DC_GPIO_Port GPIOB
#define MIO32(addr)			(*(volatile uint32_t* )(addr))
/* USER CODE BEGIN Private defines */

/* CHOOSE WHICH TAG EEPROM VERSION */

//#define EEPROM_VERSION_2K_
#define EEPROM_VERSION_4K_


#define DATA_SIZE 				(sizeof(uint16_t))
#define TRUE 					0x01
#define FALSE 					0x00
#define PAGE_SIZE				1024
#define MAX_CONNECTION_TRYOUTS	50

#define WAIT_CMD_PHONE			0x00

#define SAMPLING_CASE_1_INIT	0x10
#define SAMPLING_CASE_1			0x11
#define TRANSMIT_CASE_1_INIT	0x12
#define TRANSMIT_CASE_1			0x13

#define SAMPLING_CASE_2_INIT	0x20
#define SAMPLING_CASE_2			0x21
#define TRANSMIT_CASE_2_INIT	0x22
#define TRANSMIT_CASE_2			0x23

#define SAMPLING_CASE_3_INIT	0x30
#define SAMPLING_CASE_3			0x31
#define TRANSMIT_CASE_3_INIT	0x32
#define TRANSMIT_CASE_3			0x33

#define FINISH					0x40

#define CASE_3_SENSOR_1			0x01
#define CASE_3_SENSOR_2			0x02
#define CASE_3_SENSOR_3			0x03
#define CASE_3_SENSOR_4			0x04
#define CASE_3_END				0x05

#define SAME					0x01
#define NEXT					0x00


//timers clock frequencies [kHz]
#define TIM6_INT_FREQ			2
#define TIM2_INT_FREQ			250

#define TIM6_CNT_6MS			(TIM6_INT_FREQ*6)+1
#define TIM6_CNT_320MS			(TIM6_INT_FREQ*320)+1
#define TIM6_CNT_20MS			(TIM6_INT_FREQ*20)+1

#define TIM2_CNT_320MS			(TIM2_INT_FREQ*320)+1

//#define TIM6_PER_SET			20
//#define TIM6_PER_RESET			2500


// depends on clock settings
#define TIM2CNT_OFFSET			3 // tim counter_period/4us *1.5us + 1  ---> 16/4 * 1.5 + 1 = 7

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

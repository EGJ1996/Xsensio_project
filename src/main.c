/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stm32l4xx_ll_spi.h"
#include "nfc.h"
#include "sensor.h"
#include "uart_com.h"
#include <string.h>
#include <stdbool.h>
#include <assert.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

__attribute__((__section__(".user_data"))) const uint32_t userConfig[32];


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* DECLARATION OF THE 4 SENSORS IN A DEFAULT STATE */
uint8_t sensor1_data[SENSOR1_BUFFER_SIZE];
uint8_t sensor1_leakage[SENSOR1_BUFFER_SIZE];
SENSOR_TypeDef sensor1 = {
		.number		 	= 1, // sensor 1 on SPI 1A
		.selected		= FALSE,
		.buffer 		= sensor1_data,
		.buffer_leakage = sensor1_leakage,
		.head_leakage	= 0,
		.head 			= 0,
		.transmit_head 	= 0,
		.nb_samples 	= 0,
		.maxLen 		= SENSOR1_BUFFER_SIZE
};

uint8_t sensor2_data[SENSOR2_BUFFER_SIZE];
uint8_t sensor2_leakage[SENSOR1_BUFFER_SIZE];
SENSOR_TypeDef sensor2 = {
		.number 		= 2,
		.selected 		= FALSE,
		.buffer 		= sensor2_data,
		.buffer_leakage = sensor2_leakage,
		.head_leakage	= 0,
		.head			= 0,
		.transmit_head 	= 0,
		.nb_samples 	= 0,
		.maxLen 		= SENSOR2_BUFFER_SIZE
};

uint8_t sensor3_data[SENSOR3_BUFFER_SIZE];
uint8_t sensor3_leakage[SENSOR1_BUFFER_SIZE];
SENSOR_TypeDef sensor3 = {
		.number 		= 3, // sensor 3 on SPI 1C
		.selected 		= FALSE,
		.buffer 		= sensor3_data,
		.buffer_leakage = sensor3_leakage,
		.head_leakage	= 0,
		.head 			= 0,
		.transmit_head 	= 0,
		.nb_samples 	= 0,
		.maxLen 		= SENSOR3_BUFFER_SIZE
};

uint8_t sensor4_data[SENSOR4_BUFFER_SIZE];
uint8_t sensor4_leakage[SENSOR1_BUFFER_SIZE];
SENSOR_TypeDef sensor4 = {
		.number 		= 4,
		.selected 		= FALSE,
		.buffer 		= sensor4_data,
		.buffer_leakage = sensor4_leakage,
		.head_leakage	= 0,
		.head 			= 0,
		.transmit_head 	= 0,
		.nb_samples 	= 0,
		.maxLen 		= SENSOR4_BUFFER_SIZE
};

//static void MX_SPI2_Temp_Init(void); // use SPI2 for temperature sensor readout
								// An ADC with SPI interface is communicating with the MCU through SPI2

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void 	manageInterrupt();
void 	readoutCase1_init(void);
void 	readoutCase2_init(void);
void 	readoutCase3_init(void);
void 	processCMD_PHONE(uint8_t *data);
uint8_t transmit_sensors_data();
uint8_t case_3_sampling(SENSOR_TypeDef *s);
void 	reset(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* dummy buffers used for testing */
//uint8_t dummy_data_buffer[32] = {0x00, 0x00, 0x0F, 0xFF,
//								0x0F, 0xFF, 0x0F, 0xFF,
//								0x0F, 0xFF, 0x0F, 0xFF,
//								0x0F, 0xFF, 0x0F, 0xFF,
//								0x0F, 0xFF, 0x0F, 0xFF,
//								0x0F, 0xFF, 0x0F, 0xFF,
//								0x0F, 0xFF, 0x0F, 0xFF,
//								0x0F, 0xFF, 0x0F, 0xFF};
//
//uint8_t dummy_data_buffer2[32] = {0x00, 0x11, 0x22, 0x33,
//								0x44, 0x55, 0x66, 0x77,
//								0x88, 0x99, 0xaa, 0xbb,
//								0xcc, 0xdd, 0xee, 0xff,
//								0x00, 0x11, 0x22, 0x33,
//								0x44, 0x55, 0x66, 0x77,
//								0x88, 0x99, 0xaa, 0xbb,
//								0xcc, 0xdd, 0xee, 0xff};

volatile uint16_t convCompleted = 0;
volatile uint32_t i = 0;
volatile uint16_t TIM6_PER_SET = 20;
volatile uint32_t TIM6_PER_RESET = 2500;
//volatile uint32_t PER_SET = 20;
volatile uint16_t reduce_samp_rate = 0;
volatile uint16_t delay = 0;
volatile uint16_t holdtime = 250;					// let the system run for holdtime before starting to sample
volatile uint8_t main_state = WAIT_CMD_PHONE;		// actual state for the FSM

volatile uint32_t count_tim6 = 0;					// period elapsed counter for tim6. Used for timing purpose and toggling DBi pins
volatile uint32_t dout_count = 0;					// external interrupt counter for case 3


volatile uint8_t tag_interupt_detected = FALSE; 	// interrupt detected on line tagIRQ
volatile uint8_t tag_rx_end = FALSE;				// flag set if tagIRQ was triggered
volatile uint8_t phi_rst_ok = FALSE;				// flag say if reset signal is finished (case 3)


volatile uint16_t number = 0;						// variable in algrithm of changing the sampling
													// frequency automatically in the code 'SAMPLING_CASE_2'
volatile uint16_t DACoutput1 = 450;
volatile uint16_t DACoutput2 = 450;
volatile uint16_t nb_samples_case_1 = 0;			// nb of samples asked by the phone for case 1
volatile uint16_t nb_samples_case_2 = 0;			// nb of samples asked by the phone for case 2
volatile uint16_t nb_samples_case_3 = 0;			// nb of samples asked by the phone for case 3
volatile uint8_t Sensor_MUX = 0;					// sensor to be selected by the 1:16 multiplexer
volatile uint16_t MCU_FREQ1 = 0x50;					// MCU frequency 0->0.1MHz, 11->48MHz
volatile uint16_t MCU_FREQ2 = 0x50;					// MCU frequency 0->0.1MHz, 11->48MHz
volatile uint16_t MCU_FREQ = 0x50;					// MCU frequency 0->0.1MHz, 11->48MHz
volatile uint8_t FREQ_buffer = 0;

uint8_t E;
uint8_t D;
uint8_t C;
uint8_t B;
uint8_t A;
uint8_t CBA;
uint8_t sensor1_selected; // buffer to store the sensor1.selected state while changing the CPU freq to
								// get data from sensor1 again with the calculated ideal frequency
uint8_t sensor2_selected; // buffer to store the sensor2.selected state while changing the CPU freq to
								// get data from sensor1 again with the calculated ideal frequency
uint8_t flag1 = 0; // flag to see if we have already calculated the ideal frequency for sensor 1
uint8_t flag2 = 0; // flag to see if we have already calculated the ideal frequency for sensor 2
uint8_t Autoscale = 0; // if true, automatically change the sampling frequency
/* MCU_FREQ			00		10		20		30		40		50		60		70		80		90		A0		B0
 * MCU freq (MHz)	0.1		0.2		0.4		0.8		1		2		4		8		16		24		32		48
*/
volatile uint8_t sampling_rate_divider = 1;			// divider asked by the phone (means we actually want 1/divider samples at 11.5kHz)
													// uncomment the code in processCMD_PHONE when Michael will have implemented the feature on the phone
volatile double divide_by_divider = 1.0;

volatile uint8_t sampling_case_3_get_sample = FALSE;
volatile uint8_t sampling_case_3_state = CASE_3_SENSOR_1; // state for case 3

volatile uint8_t connection_tryouts = 0;			// when this reach MAX_CONNECTION_TRYOUTS, we consider that the NFC device connection is lost
volatile uint8_t DB_i_state = GPIO_PIN_RESET;		// state of the DBi pins i = [1,2,3,4]pin
volatile uint8_t counter_DC2 = 0;	// use to toggle the DC2 pin in TIM6 period elapse callback(for testing)


/* USER CODE END 0 */

void enter_LPRun( void )
{
    /* 1. Each digital IP clock must be enabled or disabled by using the
                   RCC_APBxENR and RCC_AHBENR registers */
	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    /* 2. The frequency of the system clock must be decreased to not exceed the
          frequency of f_MSI range1. */
	SystemClock_Config_Low_Power();
	// Reinitialize peripherals dependent on clock speed
    /* 3. The regulator is forced in low-power mode by software
          (LPRUN and LPSDSR bits set ) */

	//Reset the peripherals that depend on the work
	initialize_peripherals();

    PWR->CR1 &= ~PWR_CR1_LPR;
    PWR->CR1 |= PWR_CR1_LPMS_STANDBY; // must be set before LPRUN
    PWR->CR1 |= PWR_CR1_LPR; // enter low power run mode
}


//Exit low-power run mode by re-entering run mode

void enter_Run( void )
{
    /* Enable Clocks */
	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;

    /* Force the regulator into main mode */
    // Reset LPRUN bit

	// optional step. See if it reduces power consumption
	FLASH->ACR |= FLASH_ACR_RUN_PD;

	PWR->CR1 &= ~(PWR_CR1_LPR);

	// LPSDSR can be reset only when LPRUN bit = 0;

	PWR->CR1 &= ~(PWR_CR1_LPMS_STANDBY);
    /* Set HSI16 oscillator as system clock */
    SystemClock_Config_Low_Power();
    // Reinitialize peripherals dependent on clock speed
    initialize_peripherals();
}

// Initialize the peripherals that depend on the system clock
void initialize_peripherals(void){
	  MX_GPIO_Init();
	  MX_DMA_Init();
	  MX_SPI2_Init();
	  MX_TIM6_Init();
	  MX_TIM2_Init();
	  MX_ADC1_Init();
	  MX_DAC1_Init();
	  MX_USART1_UART_Init();
}
void SystemClock_Config_Low_Power(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

// Lock the flash memory
void Flash_lock(void){
	FLASH->CR |= FLASH_CR_LOCK;
}


// Before writing, the flash memory must be unlocked by setting the FLASH_KEY1 and FLASH_KEY2 values to the specified registers
bool Flash_unlock(void){

	//set the key values to the specified values (datasheet pg 83)
	//FLASH->KEYR = 0x45670123;
    //FLASH->KEYR = 0xCDEF89AB;
	//FLASH->KEYR = FLASH_KEY1;
	//FLASH->KEYR = FLASH_KEY2;
	FLASH->OPTKEYR = FLASH_OPTKEY1;
	FLASH->OPTKEYR = FLASH_OPTKEY2;
    // if flash is unlocked return 1, otherwise return 0

    if(FLASH->CR & FLASH_CR_LOCK)
    	return 0;

    return 1;

}


// Erase was succesful if the values in the cleared memory addresses are set to default
// Need 8 bits to access all the pages (256 pages each of 1 KB)
bool Pg_Erase_Is_Succesful(uintptr_t address, size_t sz){

	uintptr_t addr = address;
	while(sz > DATA_SIZE){ //sizeof(uint16_t) return the size of our data, stored in words of 4 bytes
		if(*(volatile uint32_t*)addr != (uint16_t)(-1)) // -1 (unsigned) == 0xFF....
			return false;
		addr += DATA_SIZE;
		sz -= DATA_SIZE;
	}
	// If sz is not a multiple of our data size, check the remaining data byte by byte

	while(sz > 0){
		if(*(char*)addr != 0xff) // Use (char*) conversion because char is 1 byte
			return false;
		++addr;
		--sz;
	}
	return true;
}

// Function for erasing a 2KB page
// page_num is the number of the page we want to erase (8 bits -> 255 available pages)

bool Flash_PageErase(uint8_t page_num){

	// If flash could not be unlocked, return false
	if(!Flash_unlock())
		return false;

	// Check that no memory operation is ongoing by checking the BSY bit in the FLASH_SR register
	while(FLASH->SR && FLASH_SR_BSY);

	// Clear all error programming flags due to previous programming
	//FLASH->SR &= ~( FLASH_SR_FASTERR | FLASH_SR_OPTVERR | FLASH_SR_RDERR | FLASH_SR_MISERR | FLASH_SR_PGSERR)
					//(FLASH_SR_SIZERR) | (FLASH_SR_PGAERR) | (FLASH_SR_WRPERR) | (FLASH_SR_PROGERR) | (FLASH_SR_OPERR));

	FLASH->SR &= (~FLASH_SR_FASTERR | ~FLASH_SR_OPTVERR | ~FLASH_SR_RDERR | ~FLASH_SR_MISERR | ~FLASH_SR_PGSERR | ~FLASH_SR_SIZERR | ~FLASH_SR_PGAERR | ~FLASH_SR_WRPERR | ~FLASH_SR_PROGERR | ~FLASH_SR_OPERR);

	// After clearing the error registers above, FLASH_SR_PGSERR must be equal to 0. Otherwise, throw an error
	assert((FLASH->SR & FLASH_SR_PGSERR) == 0);


	// set the PER bit
	FLASH->CR |= FLASH_CR_PER;

	// select the page number we want to erase
	// FLASH->CR register is 32 bits and the last 8 bits belong to the PNB register (page number => 256 total pages of 1 KB each)
	FLASH->CR &= 0xffffff00;

	FLASH->CR |= page_num;

	//set the start bit to begin the delete operation
	FLASH->CR |= FLASH_CR_STRT;

	// wait for the BSY bit to be cleared
	while((FLASH->SR & FLASH_SR_BSY));

	// Clear the sector erase flag

	FLASH->CR &= ~(FLASH_CR_PER);
	// Lock flash again
	Flash_lock();

	// Erase has to be done by page. Since we have 8 bits to store the number of pages => 256 total pages
	// Given that the size of the flash memory is 256 KB => size of one page = 1KB
	if(!Pg_Erase_Is_Succesful(FLASH_START_ADRESS + (page_num * 1024),PAGE_SIZE))
		return false;

	return true;
}

// Note that it is only possible to program double word (64 bits) (specified in pg.84 of the datasheet)
// Write to the specified address
bool FLASH_write(uint32_t address,uint64_t data){
	// Check that no flash memory operation is ongoing

	/*if(!Flash_unlock())
		return false;*/

	Flash_unlock();
	while(FLASH->SR & FLASH_SR_BSY);

	// Clear all previous programming errors
	FLASH->SR &= (~FLASH_SR_FASTERR | ~FLASH_SR_OPTVERR | ~FLASH_SR_RDERR | ~FLASH_SR_MISERR | ~FLASH_SR_PGSERR | ~FLASH_SR_SIZERR | ~FLASH_SR_PGAERR | ~FLASH_SR_WRPERR | ~FLASH_SR_PROGERR | ~FLASH_SR_OPERR);

	// Assert that the PGSERR bit is cleared after the above step

	unsigned long st = (FLASH->SR & FLASH_SR_PGSERR);
	assert((FLASH->SR & FLASH_SR_PGSERR) == 0);

	// Set the PG bit
	FLASH->CR |= FLASH_CR_PG;

	// Note a jump from one memory address to another is worth 1 word == 4 bytes
	MIO32(address+0x8) = *((uint32_t*) data);
	//MIO32(address+0xc) = *((uint32_t*)(data>>32));

	MIO32(address+0x10) = *((uint32_t*)(data>>32));
	//MIO64(address+0x10) = *((uint32_t*)(data>>32));
	// Write the data word by word, where size of the word is 4 bytes
	//MIO64(address+0x10) = (uint32_t) (data >> 32);

	// check if sizerr is set
	/*unsigned long tmp1 = (FLASH->SR & FLASH_SR_SIZERR);
	unsigned long tmp2 = (FLASH->SR & FLASH_SR_PGAERR);*/
	// Wait until the BSY bit is cleared in the FLASH_SR register
	while(FLASH->SR & FLASH_SR_BSY);

	// Assert that EOP(End Of Programming) flag is set
	//unsigned long tmp = (FLASH->SR & FLASH_SR_EOP);

	// Note that normally we would wait until FLASH_SR_EOP becomes 0, but that is not happening
	// Debug it later;
	//while(!(FLASH->SR & FLASH_SR_EOP));

	//unsigned long flash_sr = FLASH_SR_EOP;

	//FLASH->SR |= FLASH_SR_EOP;
	//HAL_Delay(1000);


	// Wait until end of programming, i.e., EOP flag is set
	while(!(FLASH->SR & FLASH_SR_EOP));
	// Bug: This should return true
	//assert((FLASH->SR & FLASH_SR_EOP));

	// Clear FLASH_SR_EOP flag
	FLASH->SR &= ~(FLASH_SR_EOP);

	// Clear the bit PG in FLASH_CR register, i.e, disables writes to flash
	FLASH->CR &= ~(FLASH_CR_PG);

	// Lock the flash register
	HAL_FLASH_Lock();
	return true;
}

uint64_t Flash_read(uintptr_t address){
	unsigned long dif = address - FLASH_START_ADRESS;
	uint64_t data;
	uint32_t data1,data2;
	if(dif % 4 == 0){ // the address constitutes the 2nd part of the data
		data2 = *((uint32_t*)address);
		data1 = *((uint32_t*)(address - 0x4));
	}
	else{ // the address constitutes the 1st part of the data
		data1 = *((uint32_t*)address);
		data2 = *((uint32_t*)(address+0x4));
	}
	data = (data1 << 32) + data2;
	return data;
}

//int main(void)
//{
//  /* USER CODE BEGIN 1 */
//
//  /* USER CODE END 1 */
//
//  /* MCU Configuration----------------------------------------------------------*/
//
//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();
//
//  /* USER CODE BEGIN Init */
//
//  /* USER CODE END Init */
//
//  /* Configure the system clock */
//  SystemClock_Config();
//
//  /* USER CODE BEGIN SysInit */
//
//  /* USER CODE END SysInit */
//
//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_DMA_Init();
//  MX_SPI2_Init();
//  MX_TIM6_Init();
//  MX_TIM2_Init();
//  MX_ADC1_Init();
//  MX_DAC1_Init();
//  MX_USART1_UART_Init();
//
//  /* Low power run mode
//  SystemClock_Config_Low_Power();
//  initialize_peripherals();
//  enter_LPRun(); */
//
//
//  /* USER CODE BEGIN 2 */
////  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
//  //save power resources, peripherals will be enabled only when needed
//	__HAL_RCC_SPI1_CLK_DISABLE();
////	__HAL_RCC_TIM2_CLK_DISABLE();
//	__HAL_RCC_TIM6_CLK_DISABLE();
////	__HAL_RCC_GPIOC_CLK_DISABLE();
////	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn); 	//disable interrupt line 9-5 (related to adc counter dout line 6)
////	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
//
//	uint8_t rx_data_buffer[16]; 		//storage of NFC received buffer data
//
//
//	//clearScreenUART();
//
//	/* uncomment the following line to configure the tag.
//	 * Tag must be powered ON.
//	 * Should be executed slowly -> in debug line by line (don't know why)
//	 */
////	configure_EEPROM();
//
//	// Info on terminal for debugging (self-explaining function)
//	//printUART_MSG("Just entered main function\n");
//	cmdCLEAR_BUFFER();
//    printUART_CONFIG();
//    printUART_RFID_STATUS();
//    printUART_BUFFER_STATUS();
//
//    /* uncomment/comment the two following lines to enable/disable tim6 --> toggling of SW pins
//     * When commented, don't forget to enable it in the code (readoutCase2_init) and to stop it
//     * at the end of sampling case 2
//     *
//     * to enable: 							to disable:
//     *  __HAL_RCC_TIM6_CLK_ENABLE();		HAL_TIM_Base_Stop_IT(&htim6);
//     *  HAL_TIM_Base_Start_IT(&htim6);		__HAL_RCC_TIM6_CLK_DISABLE();
//     */
////     __HAL_RCC_TIM6_CLK_ENABLE();
////    HAL_TIM_Base_Start_IT(&htim6);
//// the code is set to have SW at constant low/high with the aid of line 1229/1230
//    float temp;
//    char msg[20];
//	uint16_t rawValues[3];
//	HAL_TIM_Base_Start(&htim2);
//	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)rawValues, 3);
//
////	while(1) {
////		while(!convCompleted);
////
////		for(uint8_t i = 0; i < hadc1.Init.NbrOfConversion; i++) {
////			temp = ((float)rawValues[i]) / 4095 * 2500;
////			temp = ((temp - 760.0) / 2.5) + 25;
////			sprintf(msg, "rawValue %d: %hu\r\n", i, rawValues[i]);
////			HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
//////			sprintf(msg, "Temperature %d: %f\r\n",i, temp);
//////			HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
////		}
////		convCompleted = 0;
////	}
//
//  /* USER CODE END 2 */
//
//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
//
//
//		if(tag_interupt_detected)
//		{
//			manageInterrupt(); //tag interrupt handling fct
//		}
//
//		/* MAIN FINITE STATE MACHINE
//		 * WAIT_CMD_PHONE	<--------
//		 * 							|
//		 * SAMPLING_CASE_1_INIT		|
//		 * SAMPLING_CASE_1			|
//		 * TRANSMIT_CASE_1_INIT		|
//		 * TRANSMIT_CASE_1			|
//		 * 							|
//		 * SAMPLING_CASE_2_INIT		|
//		 * SAMPLING_CASE_2			|
//		 * TRANSMIT_CASE_2_INIT		|
//		 * TRANSMIT_CASE_2			|
//		 * 							|
//		 * SAMPLING_CASE_3_INIT		|
//		 * SAMPLING_CASE_3			|
//		 * TRANSMIT_CASE_3_INIT		|
//		 * TRANSMIT_CASE_3			|
//		 * 							|
//		 * FINISH -------------------
//		 */
//		switch (main_state) {
//
//			case WAIT_CMD_PHONE:
//				if( device_detected()) //NFC device detected  (state = selectedX + PICC AFE is active (MSB) )
//				{
//					if(tag_rx_end) //end of receive from phone (from interruption)
//					{
//						printUART_MSG("tag_rx_end \r\n");
//						if (extendedRECEIVE(rx_data_buffer)==RECEIVE_OK) //buffer read
//						{
//							tag_rx_end = FALSE;
////							printUART_BUFFER(rx_data_buffer); 	// display received buffer
////							printUART_BUFFER_STATUS();
//							processCMD_PHONE(rx_data_buffer); 	// process the received data
//							RCC_OscInitTypeDef RCC_OscInitStruct;
//							/**Initializes the CPU, AHB and APB busses clocks
//							*/
//							RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
//							RCC_OscInitStruct.MSIState = RCC_MSI_ON;
//							RCC_OscInitStruct.MSICalibrationValue = 0;
//							RCC_OscInitStruct.MSIClockRange = MCU_FREQ;
//							RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//							if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//							{
//							_Error_Handler(__FILE__, __LINE__);
//							}
////							MX_SPI1_Init();
//							MX_SPI2_Init();
////							hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
////							MX_USART1_UART_Init();
////							clearScreenUART();
////							cmdCLEAR_BUFFER();
////						    printUART_CONFIG();
////						    printUART_RFID_STATUS();
////						    printUART_BUFFER_STATUS();
//							main_state = SAMPLING_CASE_1_INIT;
//						}
//					}
//				}
//
////				RCC_ClkInitTypeDef RCC_ClkInitStruct;
////				/**Initializes the CPU, AHB and APB busses clocks
////				*/
////				RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
////				                          |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
////				RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
////				RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
////				RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
////				RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;
////				if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
////				{
////				  _Error_Handler(__FILE__, __LINE__);
////				}
//
//				break;
//
//
//			case SAMPLING_CASE_1_INIT:
//				if (nb_samples_case_1 == 0)
//				{
//					// jump directly to case 2 if no samples are asked for case 1
//					main_state = SAMPLING_CASE_2_INIT;
//				}
//				else
//				{
//					main_state = SAMPLING_CASE_1;
//					printUART_MSG("Sampling case 1\r\n");
//					readoutCase1_init();
//				}
//
//				break;
//
//			case SAMPLING_CASE_1:
//				if(sensor1.selected && sensor1.nb_samples<nb_samples_case_1)
//				{
//					sensor_get1Sample(&sensor1, DEST_DATA);
//					// uncomment the following lines to sample the "leakage" signal, for differential measurements
//					/*
//					//change switch state for leakage
//					HAL_GPIO_WritePin(DC2_GPIO_Port, DC2_Pin, GPIO_PIN_SET);
//					sensor_get1Sample(&sensor1, DEST_LEAKAGE); // store sample in leakage buffer
//					//change switch state back for data
//					HAL_GPIO_WritePin(DC2_GPIO_Port, DC2_Pin, GPIO_PIN_RESET);
//					*/
//				}
//				if(sensor2.selected && sensor2.nb_samples<nb_samples_case_1)
//				{
//					sensor_get1Sample(&sensor2, DEST_DATA);
//					// uncomment the following lines to sample the "leakage" signal, for differential measurements
//					/*
//					//change switch state for leakage
//					HAL_GPIO_WritePin(DC2_GPIO_Port, DC2_Pin, GPIO_PIN_SET);
//					sensor_get1Sample(&sensor2, DEST_LEAKAGE); // store sample in leakage buffer
//					//change switch state back for data
//					HAL_GPIO_WritePin(DC2_GPIO_Port, DC2_Pin, GPIO_PIN_RESET);
//					*/
//				}
//				if(sensor3.selected && sensor3.nb_samples<nb_samples_case_1)
//				{
//					sensor_get1Sample(&sensor3, DEST_DATA);
//					// uncomment the following lines to sample the "leakage" signal, for differential measurements
//					/*
//					//change switch state for leakage
//					HAL_GPIO_WritePin(DC2_GPIO_Port, DC2_Pin, GPIO_PIN_SET);
//					sensor_get1Sample(&sensor3, DEST_LEAKAGE); // store sample in leakage buffer
//					//change switch state back for data
//					HAL_GPIO_WritePin(DC2_GPIO_Port, DC2_Pin, GPIO_PIN_RESET);
//					*/
//				}
//				if(sensor4.selected && sensor4.nb_samples<nb_samples_case_1)
//				{
//					sensor_get1Sample(&sensor4, DEST_DATA);
//					// uncomment the following lines to sample the "leakage" signal, for differential measurements
//					/*
//					//change switch state for leakage
//					HAL_GPIO_WritePin(DC2_GPIO_Port, DC2_Pin, GPIO_PIN_SET);
//					sensor_get1Sample(&sensor4, DEST_LEAKAGE); // store sample in leakage buffer
//					//change switch state back for data
//					HAL_GPIO_WritePin(DC2_GPIO_Port, DC2_Pin, GPIO_PIN_RESET);
//					*/
//				}
////				for(i=0;i<250;i++)
////				{}
////					i=0;
//
//				if ( (!sensor1.selected || sensor1.nb_samples >= nb_samples_case_1) &&
//					(!sensor2.selected || sensor2.nb_samples >= nb_samples_case_1) &&
//					(!sensor3.selected || sensor3.nb_samples >= nb_samples_case_1) &&
//					(!sensor4.selected || sensor4.nb_samples >= nb_samples_case_1) )
//				{
//					//all sensor have received enough samples
//
////					HAL_TIM_Base_Stop_IT(&htim6);
//
//					if(sensor1.selected)
//					{
//						// uncomment to process sensors buffers for differential measurements
//						// sensor_differentiateSignals(&sensor1, sensor1.nb_samples);
//
//						printUART_MSG("sensor1\r\n");
//						printUART_SENSOR_DATA(sensor1.buffer, nb_samples_case_1);
//					}
//					if(sensor2.selected)
//					{
//						// uncomment to process sensors buffers for differential measurements
//						// sensor_differentiateSignals(&sensor2, sensor2.nb_samples);
//
//						printUART_MSG("sensor2\r\n");
//						printUART_SENSOR_DATA(sensor2.buffer, nb_samples_case_1);
//					}
//					if(sensor3.selected)
//					{
//						// uncomment to process sensors buffers for differential measurements
//						// sensor_differentiateSignals(&sensor3, sensor3.nb_samples);
//
//						printUART_MSG("sensor3\r\n");
//						printUART_SENSOR_DATA(sensor3.buffer, nb_samples_case_1);
//					}
//					if(sensor4.selected)
//					{
//						// uncomment to process sensors buffers for differential measurements
//						// sensor_differentiateSignals(&sensor4, sensor4.nb_samples);
//
//						printUART_MSG("sensor4\r\n");
//						printUART_SENSOR_DATA(sensor4.buffer, nb_samples_case_1);
//					}
//					main_state = TRANSMIT_CASE_1_INIT;
//				}
//
//				break;
//
//			case TRANSMIT_CASE_1_INIT:
//				LL_SPI_Disable(SPI1);
//				__HAL_RCC_SPI1_CLK_DISABLE();
////				LL_SPI_Disable(SPI2);
////				__HAL_RCC_SPI2_CLK_DISABLE();
////				LL_SPI_Disable(SPI3);
////				__HAL_RCC_SPI3_CLK_DISABLE();
////				LL_SPI_Disable(SPI4);
////				__HAL_RCC_SPI4_CLK_DISABLE();
////				__HAL_RCC_TIM6_CLK_DISABLE();
////				__HAL_RCC_GPIOC_CLK_DISABLE();
//
//				__HAL_RCC_SPI2_CLK_ENABLE();
//				HAL_NVIC_EnableIRQ(EXTI2_IRQn);
//
//				printUART_MSG("TRANSMITING 2\r\n");
//				main_state = TRANSMIT_CASE_1;
//				break;
//
//			case TRANSMIT_CASE_1:
////				printUART_RFID_STATUS();
//				if( device_detected() ) //NFC device detected -> state = selectedX + PICC AFE is active (MSB)
//				{
//					connection_tryouts = 0;
//					if (transmit_sensors_data()==TRANSMIT_FINISH)
//					{
//						printUART_MSG("TRANSMIT DONE 2\r\n");
//						main_state = SAMPLING_CASE_2_INIT;
//					}
//				}
//				else
//				{
//					connection_tryouts++;
//					if (connection_tryouts>MAX_CONNECTION_TRYOUTS)
//					{
//						printUART_MSG("device connection lost\r\n");
//						reset();
//						main_state = WAIT_CMD_PHONE;
//					}
//				}
//
//				break;
//
//			case SAMPLING_CASE_2_INIT:
//				if (nb_samples_case_2 == 0)
//				{
//					// jump directly to case 3 if no samples are asked for case 2
//					main_state = SAMPLING_CASE_3_INIT;
//				}
//				else
//				{
//					main_state = SAMPLING_CASE_2;
//					readoutCase2_init();
//					//////////////////////////////////////////////////////
////					MX_SPI2_Temp_Init();// use SPI2 for temperature sensor readout
//					// An ADC with SPI interface is communicating with the MCU through SPI2
//					// This init function is for reinitialize the SPI2 for ADC reading
//					__HAL_RCC_SPI2_CLK_ENABLE();
//					LL_SPI_Enable(SPI2);
//					//////////////////////////////////////////////////////
////					printUART_MSG("Sampling case 2\r\n");
//					HAL_Delay(holdtime);	// hold before start recording the samples
//
//				}
//				break;
//
//			case SAMPLING_CASE_2:
//
////				if(sensor1.selected && sensor1.nb_samples<nb_samples_case_2)
////				{
////					sensor_get1Sample(&sensor1, DEST_DATA);
////					uint16_t rawValues[1];
////					HAL_ADC_Start_DMA(&hadc1, (uint32_t*)rawValues, 1);
////					while(!convCompleted);
////					HAL_ADC_Stop_DMA(&hadc1);
////					SENSOR_TypeDef* sensor = &sensor3;
//////					uint16_t* data_16 = &rawValues[0];
////					// next is where head will point to after this write.
////					uint16_t next = sensor->head + 2;
////					if (next >= sensor->maxLen)
////						next = 0;
////					sensor->buffer[sensor->head] = (uint8_t)(rawValues[0]>>8);
////					sensor->buffer[(sensor->head)+1] = (uint8_t)(rawValues[0] & 0xff);
////					sensor->head = next;
////					sensor->nb_samples++;
////
////					// uncomment the following lines to sample the "leakage" signal, for differential measurements
////					/*
////					//change switch state for leakage
////					HAL_GPIO_WritePin(DC2_GPIO_Port, DC2_Pin, GPIO_PIN_SET);
////					sensor_get1Sample(&sensor1, DEST_LEAKAGE); // store sample in leakage buffer
////					//change switch state back for data
////					HAL_GPIO_WritePin(DC2_GPIO_Port, DC2_Pin, GPIO_PIN_RESET);
////					*/
////				}
//
//				if(sensor1.selected && sensor2.selected && sensor3.selected)
//				{
////					SENSOR_TypeDef* sensor01 = &sensor1;
////					SENSOR_TypeDef* sensor02 = &sensor2;
////					SENSOR_TypeDef* sensor03 = &sensor3;
//
//					HAL_TIM_Base_Start(&htim2);
//					HAL_ADC_Start_DMA(&hadc1, (uint32_t*)rawValues, 3);
//					while(sensor1.nb_samples<nb_samples_case_2) {
//
//						while(!convCompleted);
//// sensor 1, VREF for pH sensor
//						Sensor_MUX = 2;
//						MUX_conf();
//						// needs a delay here
//						HAL_Delay(50);
//						uint16_t next1 = sensor1.head + 2;
//						if (next1 >= sensor1.maxLen)
//							next1 = 0;
//						sensor1.buffer[sensor1.head] = (uint8_t)(rawValues[0]>>8);
//						sensor1.buffer[(sensor1.head)+1] = (uint8_t)(rawValues[0] & 0xff);
//						sensor1.head = next1;
//						sensor1.nb_samples++;
//// sensor 2, VREF for Na sensor
//						Sensor_MUX = 9;
//						MUX_conf();
//						// needs a delay here
//						HAL_Delay(50);
//						uint16_t next2 = sensor2.head + 2;
//						if (next2 >= sensor2.maxLen)
//							next2 = 0;
//						sensor2.buffer[sensor2.head] = (uint8_t)(rawValues[1]>>8);
//						sensor2.buffer[(sensor2.head)+1] = (uint8_t)(rawValues[1] & 0xff);
//						sensor2.head = next2;
//						sensor2.nb_samples++;
//// sensor 3, Temperature sensor
//						uint16_t next3 = sensor3.head + 2;
//						if (next3 >= sensor3.maxLen)
//							next3 = 0;
//						sensor3.buffer[sensor3.head] = (uint8_t)(rawValues[2]>>8);
//						sensor3.buffer[(sensor3.head)+1] = (uint8_t)(rawValues[2] & 0xff);
//						sensor3.head = next3;
//						sensor3.nb_samples++;
//
//						convCompleted = 0;
//					}
//
//					HAL_ADC_Stop_DMA(&hadc1);
//
//				}
//
//				if ( (!sensor1.selected || sensor1.nb_samples >= nb_samples_case_2) &&
//					(!sensor2.selected || sensor1.nb_samples >= nb_samples_case_2) &&
//					(!sensor3.selected || sensor1.nb_samples >= nb_samples_case_2) &&
//					(!sensor4.selected || sensor1.nb_samples >= nb_samples_case_2) )
//				{
//					//all sensor have received enough samples
//					main_state = TRANSMIT_CASE_2_INIT;
////										if(sensor1.selected)
////										{
////											// uncomment to process sensors buffers for differential measurements
////											// sensor_differentiateSignals(&sensor1, sensor1.nb_samples);
////
////											printUART_MSG("sensor1\r\n");
////											printUART_SENSOR_DATA(sensor1.buffer, nb_samples_case_2);
////										}
////										if(sensor2.selected)
////										{
////											// uncomment to process sensors buffers for differential measurements
////											// sensor_differentiateSignals(&sensor2, sensor2.nb_samples);
////
////											printUART_MSG("sensor2\r\n");
////											printUART_SENSOR_DATA(sensor2.buffer, nb_samples_case_2);
////										}
////										if(sensor3.selected)
////										{
////											// uncomment to process sensors buffers for differential measurements
////											// sensor_differentiateSignals(&sensor3, sensor3.nb_samples);
////
////											printUART_MSG("sensor3\r\n");
////											printUART_SENSOR_DATA(sensor3.buffer, nb_samples_case_2);
////										}
////										if(sensor4.selected)
////										{
////										}
//				}
//
//				break;
//
//			case TRANSMIT_CASE_2_INIT:
//				LL_SPI_Disable(SPI1);
//				__HAL_RCC_SPI1_CLK_DISABLE();
//
///////////////////////////////////////////////////////////////
////				MX_SPI2_Init();  // use SPI2 for temperature sensor readout
//				// An ADC with SPI interface is communicating with the MCU through SPI2
//				// This init function is for reinitialize the SPI2 for NFC communication
///////////////////////////////////////////////////////////////
//
////				LL_SPI_Disable(SPI2);
////				__HAL_RCC_SPI2_CLK_DISABLE();
////				LL_SPI_Disable(SPI3);
////				__HAL_RCC_SPI3_CLK_DISABLE();
////				LL_SPI_Disable(SPI4);
////				__HAL_RCC_SPI4_CLK_DISABLE();
////				__HAL_RCC_TIM6_CLK_DISABLE();
////				__HAL_RCC_GPIOC_CLK_DISABLE();
//
//				__HAL_RCC_SPI2_CLK_ENABLE();
//				HAL_NVIC_EnableIRQ(EXTI2_IRQn);
//
////				printUART_MSG("TRANSMITING 2\r\n");
//				main_state = TRANSMIT_CASE_2;
//				break;
//
//			case TRANSMIT_CASE_2:
////				printUART_RFID_STATUS();
//				if( device_detected() ) //NFC device detected -> state = selectedX + PICC AFE is active (MSB)
//				{
//					connection_tryouts = 0;
//					if (transmit_sensors_data()==TRANSMIT_FINISH)
//					{
////						printUART_MSG("TRANSMIT DONE 2\r\n");
//						main_state = SAMPLING_CASE_3_INIT;
//					}
//				}
//				else
//				{
//					connection_tryouts++;
//					if (connection_tryouts>MAX_CONNECTION_TRYOUTS)
//					{
////						printUART_MSG("device connection lost\r\n");
//						reset();
//						main_state = WAIT_CMD_PHONE;
//					}
//				}
//
//				break;
//
//
//			case SAMPLING_CASE_3_INIT:
//				if (nb_samples_case_3 == 0)
//				{
//					// jump directly to case finish if no samples are asked for case 3
//					main_state = FINISH;
//				}
//				else
//				{
//					main_state = SAMPLING_CASE_3;
//					printUART_MSG("Sampling case 3\r\n");
//					readoutCase3_init();
//				}
//
//				break;
//
//			case SAMPLING_CASE_3:
//				// we start by acquiring the data from sensor1 then 2 , 3 and finish by sensor 4 (obviously for selected sensors only)
//				switch (sampling_case_3_state) {
//					case CASE_3_SENSOR_1:
//						if (case_3_sampling(&sensor1)==NEXT)
//						{
//							sampling_case_3_state = CASE_3_SENSOR_2;
//						}
//
//						break;
//					case CASE_3_SENSOR_2:
//						if (case_3_sampling(&sensor2)==NEXT)
//						{
//							sampling_case_3_state = CASE_3_SENSOR_3;
//						}
//
//
//						break;
//					case CASE_3_SENSOR_3:
//						if (case_3_sampling(&sensor3)==NEXT)
//						{
//							sampling_case_3_state = CASE_3_SENSOR_4;
//						}
//
//						break;
//					case CASE_3_SENSOR_4:
//						if (case_3_sampling(&sensor4)==NEXT)
//						{
//							sampling_case_3_state = CASE_3_END;
//						}
//
//						break;
//					case CASE_3_END:
//						HAL_TIM_Base_Stop_IT(&htim6);
////						HAL_GPIO_WritePin(PHI_RST_GPIO_Port, PHI_RST_Pin, GPIO_PIN_RESET);
//						main_state = TRANSMIT_CASE_3_INIT;
//
//						break;
//					default:
//						break;
//				}
//
//				break;
//
//			case TRANSMIT_CASE_3_INIT:
//				__HAL_RCC_TIM6_CLK_DISABLE();
//				__HAL_RCC_TIM2_CLK_DISABLE();
////				__HAL_RCC_GPIOC_CLK_DISABLE();
////				HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
//
//				__HAL_RCC_SPI2_CLK_ENABLE();
//				HAL_NVIC_EnableIRQ(EXTI2_IRQn);
//
//				printUART_MSG("TRANSMITING 3\r\n");
//				main_state = TRANSMIT_CASE_3;
//				break;
//
//			case TRANSMIT_CASE_3:
////				printUART_RFID_STATUS();
//				if( device_detected() ) //NFC device detected -> state = selectedX + PICC AFE is active (MSB)
//				{
//					connection_tryouts = 0;
//					if (transmit_sensors_data()==TRANSMIT_FINISH)
//					{
//						printUART_MSG("TRANSMIT DONE 3\r\n");
//						main_state = FINISH;
//					}
//				}
//				else
//				{
//					connection_tryouts++;
//					if (connection_tryouts>MAX_CONNECTION_TRYOUTS)
//					{
//						printUART_MSG("device connection lost\r\n");
//						reset();
//						main_state = WAIT_CMD_PHONE;
//					}
//				}
//
//				break;
//
//			case FINISH:
//				printUART_MSG("FINISH");
//				reset();
//				main_state = WAIT_CMD_PHONE;
//				break;
//
//			default:
//				printUART_MSG("unknown state");
//				reset();
//				main_state = WAIT_CMD_PHONE;
//				break;
//		}
//
//  /* USER CODE END WHILE */
//
//  /* USER CODE BEGIN 3 */
//
//  }
//  /* USER CODE END 3 */
//
//}

//HAL_StatusTypeDef FLASH_ErasePage(uint32_t Page_Address)
//{
//	HAL_StatusTypeDef status;
//	/* Check the parameters */
//	assert_param(IS_FLASH_PROGRAM_ADDRESS(Page_Address));
//	/* Wait for last operation to be completed */
//	status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
//	if
//	(status == HAL_OK)
//	{
//	/* If the previous operation is completed, proceed to erase the page */
//	FLASH->CR |= FLASH_CR_PER;
//	//FLASH->AR = Page_Address;
//	FLASH->ACR = Page_Address;
//	FLASH->CR |= FLASH_CR_STRT;
//	/* Wait for last operation to be completed */
//	//status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
//	/* Disable the PER Bit */
//	FLASH->CR &= ~FLASH_CR_PER;
//	}
//	/* Return the Erase Status */
//	return status;
//}
//
//
//void Flash_write(uint32_t data){
//
//	 	 HAL_FLASH_Unlock();
//	    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );
//
//	    FLASH_EraseInitTypeDef EraseInitStruct;
//
//	    /*EraseInitStruct.NbPages = 256;
//	    EraseInitStruct.Page = 64;*/
//
//	    EraseInitStruct.NbPages = 1;
//	    EraseInitStruct.Page = 0;
//	    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
//
//
//	    /*bool status = FLASH_ErasePage(userConfig);
//	    while(FLASH->SR & FLASH_SR_BSY);
//	    HAL_FLASH_Unlock();*/
//	    //bool status = FLASH_PageErase(userConfig,0);
//
//	    uint32_t PageError = 0;
//
//	    /*if(HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
//	        //Erase error!
//	    }*/
//
//	    // Page requests the address of the page or page number?
//	    /*HAL_FLASH_Program(TYPEPROGRAM_WORD,userConfig,inter->moder);
//	    HAL_FLASH_Program(TYPEPROGRAM_WORD,userConfig+1,inter->prc);
//	    HAL_FLASH_Program(TYPEPROGRAM_WORD,userConfig+2,inter->cpr);
//	    HAL_FLASH_Program(TYPEPROGRAM_WORD,userConfig+3,inter->pos_mul_factor);
//	    HAL_FLASH_Program(TYPEPROGRAM_WORD,userConfig+4,inter->speed_mul_factor);
//	    HAL_FLASH_Program(TYPEPROGRAM_WORD,userConfig+5,inter->PWM_prc);*/
//	    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,userConfig,data);
//	    //FLASH->CR &= ~FLASH_CR_PER;
//	    CLEAR_BIT(FLASH->CR,(FLASH_CR_PER));
//	    CLEAR_BIT(FLASH->CR,(FLASH_CR_PG));
//	    HAL_FLASH_Lock();
//}

#define APPLICATION_START_ADDRESS					0x8020000

void flashErase(uint8_t startPage, uint8_t numberOfPages)
{

	FLASH_EraseInitTypeDef EraseInitStruct;
    HAL_FLASH_Unlock();

    EraseInitStruct.TypeErase= FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Page = startPage;
    EraseInitStruct.NbPages = numberOfPages;

    uint32_t PageError = 0;

    if(HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
    {
    	;
    }

    HAL_FLASH_Lock();
}

int main(void){

	HAL_Init();

	SystemClock_Config();
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_SPI2_Init();
	MX_TIM6_Init();
	MX_TIM2_Init();
	MX_ADC1_Init();
	MX_DAC1_Init();
	MX_USART1_UART_Init();

	flashErase(64,1);
    __IO uint64_t word =  0x1234567812345678UL;
    __IO uint64_t word1 = 0x123UL;
    HAL_FLASH_Unlock();

    //Note that a 64 bits word is split into 2 32 bits word
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, APPLICATION_START_ADDRESS, word);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, APPLICATION_START_ADDRESS+0x8, word1);
    uint32_t data  = userConfig[0] ;
    uint32_t data1 = userConfig[2];
    int dummy_data = 12345;

	//	while(1){
//
//	}
}
/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 2;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV8;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* DAC1 init function */
static void MX_DAC1_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 99;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 19;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
     PA0   ------> SharedAnalog_PA0
     PA4   ------> SharedAnalog_PA4
     PA5   ------> SharedAnalog_PA5
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_SS_Tag_GPIO_Port, SPI2_SS_Tag_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DA_Pin|DB_Pin|DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PH0 PH1 PH3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : VRT1_Pin PA1 PA2 PA3 
                           VDS_Pin VREF_Pin PA6 PA7 
                           PA8 PA10 PA11 PA12 
                           PA15 */
  GPIO_InitStruct.Pin = VRT1_Pin|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |VDS_Pin|VREF_Pin|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12 
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11 
                           PB3 PB4 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TAG_IRQ_Pin */
  GPIO_InitStruct.Pin = TAG_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TAG_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_SS_Tag_Pin DA_Pin DB_Pin DC_Pin */
  GPIO_InitStruct.Pin = SPI2_SS_Tag_Pin|DA_Pin|DB_Pin|DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	convCompleted = 1;
}
/* Function: 	Used to put the MCU in the configuration to wait for a phone's cmd
 * 				this function is called when MAX_CONNECTION_TRYOUTS is reached or
 * 				when a complete take is finished.
 * Return: 		none
 * Param:		none
 */
void reset(void)
{
	LL_SPI_Disable(SPI1);
	__HAL_RCC_SPI1_CLK_DISABLE();
//	LL_SPI_Disable(SPI2);
//	__HAL_RCC_SPI2_CLK_DISABLE();
//	LL_SPI_Disable(SPI3);
//	__HAL_RCC_SPI3_CLK_DISABLE();
//	LL_SPI_Disable(SPI4);
//	__HAL_RCC_SPI4_CLK_DISABLE();
//	__HAL_RCC_TIM2_CLK_DISABLE();
//	__HAL_RCC_TIM6_CLK_DISABLE();
//	__HAL_RCC_GPIOC_CLK_DISABLE();
//	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn); //disable interrupt line 6 (related to adc counter)

	__HAL_RCC_SPI2_CLK_ENABLE();
	HAL_NVIC_EnableIRQ(EXTI2_IRQn); // enable interrupt line 2 (related to tagIRQ)

	tag_rx_end = FALSE;
	nb_samples_case_1 = 0;
	nb_samples_case_2 = 0;
	nb_samples_case_3 = 0;
	flag1 = 0;
	flag2 = 0;
	connection_tryouts = 0;
}


/* Function: 	Process the data received by the phone
 * Return: 		none
 * Param:		data: pointer on the received data
 */
void processCMD_PHONE(uint8_t *data)
{
	// First received byte defined which sensor is enabled |0|0|0|0|s1|s2|s3|s4|
	sensor1.selected = (data[0] & 0x08)>>3;
	sensor2.selected = (data[0] & 0x04)>>2;
	sensor3.selected = (data[0] & 0x02)>>1;
	sensor4.selected = (data[0] & 0x01);

	/* byte 2,3 defined nb of samples wanted for case 1
	 * byte 4,5 defined nb of samples wanted for case 2
	 * byte 6,7 defined nb of samples wanted for case 3
	 */
	DACoutput1 = ((uint16_t)data[2] | ((uint16_t)data[1])<<8)*sampling_rate_divider;
	DACoutput2 = ((uint16_t)data[6] | ((uint16_t)data[5])<<8)*sampling_rate_divider;
//	nb_samples_case_1 = ((uint16_t)data[2] | ((uint16_t)data[1])<<8)*sampling_rate_divider;
	nb_samples_case_2 = ((uint16_t)data[4] | ((uint16_t)data[3])<<8)*sampling_rate_divider;
//	nb_samples_case_3 = ((uint16_t)data[6] | ((uint16_t)data[5])<<8)*sampling_rate_divider;
	//nb_samples_case_3 = 0;
	// 4 LSBs of byte 7 define sensor to be selected by multiplexer
	Sensor_MUX = (uint16_t)data[7] & 0x0F;

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DACoutput1);

	// 4 MSBs of byte 7 define MCU frequency
	MCU_FREQ = (uint16_t)data[7] & 0xF0;
	MCU_FREQ1 = MCU_FREQ;
	MCU_FREQ2 = MCU_FREQ;
	FREQ_buffer = MCU_FREQ >> 4;
	if (MCU_FREQ > 0x40 && MCU_FREQ < 0xB0) // fixed frequency 2,4,8,16,24,32 MHz
	{
		TIM6_PER_SET = (FREQ_buffer - 4) * 20;
		TIM6_PER_RESET = 2500;
		reduce_samp_rate = 0;
	}
	else if (MCU_FREQ >= 0xB0) // automatic calculate the ideal frequency, initial value be 2 MHz
	{
		TIM6_PER_SET = 20;
		TIM6_PER_RESET = 2500;
		Autoscale = 1;
		MCU_FREQ = 0x50;
		MCU_FREQ1 = MCU_FREQ;
		MCU_FREQ2 = MCU_FREQ;
	}
	else // fixed CPU frequency at 2 MHz, but reduce sampling and reset rate to that equivalent to a CPU frequency of 125,250,500,1000 KHz
	{
		reduce_samp_rate = 240 / (0x01 << FREQ_buffer);
		TIM6_PER_SET = 20;
		TIM6_PER_RESET = 80000 / (0x01 << FREQ_buffer);
		TIM6_PER_RESET = TIM6_PER_RESET < 0xFFFF? TIM6_PER_RESET : 0xFFFF;
		MCU_FREQ = 0x50;
	}

	// sampling_rate_divider = data[7]; //uncomment the two following lines to get the divider from the phone when it will be implemented by the phone
	// divide_by_divider = 1.0/sampling_rate_divider;

	//the following is for debugging purpose only (display on terminal)
	printUART_MSG("Sensors selected: \r\n");
	printUART_MSG("1: ");
	sensor1.selected ? printUART_MSG("on\r\n") : printUART_MSG("off\r\n");
	printUART_MSG("2: ");
	sensor2.selected ? printUART_MSG("on\r\n") : printUART_MSG("off\r\n");
	printUART_MSG("3: ");
	sensor3.selected ? printUART_MSG("on\r\n") : printUART_MSG("off\r\n");
	printUART_MSG("4: ");
	sensor4.selected ? printUART_MSG("on\r\n") : printUART_MSG("off\r\n");
	printUART_MSG("Divider (HEX): ");
	printUART(&sampling_rate_divider, 1);

	printUART_MSG("Case 1 samples: ");
	printUART(((uint8_t*)&nb_samples_case_1)+1, 1);
	printUART((uint8_t*)&nb_samples_case_1, 1);
	printUART_MSG("Case 2 samples: ");
	printUART(((uint8_t*)&nb_samples_case_2)+1, 1);
	printUART((uint8_t*)&nb_samples_case_2, 1);
	printUART_MSG("Case 3 samples: ");
	printUART(((uint8_t*)&nb_samples_case_3)+1, 1);
	printUART((uint8_t*)&nb_samples_case_3, 1);


}

/* Function: 	For each selected sensor, transmit all the data that have not been transmitted yet.
 * 				Data are transmitted sequentially starting with sensor1
 * Return: 		status: transmission Finished or Not finished
 * Param:		none
 */
uint8_t transmit_sensors_data()
{
	uint8_t state = TRANSMIT_NOT_FINISH;
	uint8_t size;
	uint8_t tx_data[12];
	if (sensor1.selected && sensor_transmit_state(&sensor1)!=TRANSMIT_FINISH)
	{
		// 12 is the maximum number of bytes that can be sent at once in extended mode
		// the following line compute how many data should be sent (12 or less)
		size = sensor1.head>=(sensor1.transmit_head+(12*sampling_rate_divider)) ? 12 : (uint8_t)((sensor1.head-sensor1.transmit_head)*divide_by_divider);
		for (uint8_t i = 0; i < size; i=i+2)
		{
			// create a new array with the samples that needs to be sent taking into account the divider
			tx_data[i] = sensor1.buffer[sensor1.transmit_head+(i*sampling_rate_divider)];
			tx_data[i+1] = sensor1.buffer[sensor1.transmit_head+(i*sampling_rate_divider)+1];
		}
		if (extendedTRANSMISSION(tx_data, size)==TRANSMIT_OK) // send the data
		{
			sensor1.transmit_head += size*sampling_rate_divider;
		}
	}
	else if (sensor2.selected && sensor_transmit_state(&sensor2)!=TRANSMIT_FINISH)
	{
		// 12 is the maximum number of bytes that can be sent at once in extended mode
		// the following line compute how many data should be sent (12 or less)
		size = sensor2.head>=(sensor2.transmit_head+(12*sampling_rate_divider)) ? 12 : (uint8_t)((sensor2.head-sensor2.transmit_head)*divide_by_divider);
		for (uint8_t i = 0; i < size; i=i+2)
		{
			// create a new array with the samples that needs to be sent taking into account the divider
			tx_data[i] = sensor2.buffer[sensor2.transmit_head+(i*sampling_rate_divider)];
			tx_data[i+1] = sensor2.buffer[sensor2.transmit_head+(i*sampling_rate_divider)+1];
		}
		if (extendedTRANSMISSION(tx_data, size)==TRANSMIT_OK) // send the data
		{
			sensor2.transmit_head += size*sampling_rate_divider; // update the transmit head
		}
	}
	else if (sensor3.selected && sensor_transmit_state(&sensor3)!=TRANSMIT_FINISH)
	{
		// 12 is the maximum number of bytes that can be sent at once in extended mode
		// the following line compute how many data should be sent (12 or less)
		size = sensor3.head>=(sensor3.transmit_head+(12*sampling_rate_divider)) ? 12 : (uint8_t)((sensor3.head-sensor3.transmit_head)*divide_by_divider);
		for (uint8_t i = 0; i < size; i=i+2)
		{
			// create a new array with the samples that needs to be sent taking into account the divider
			tx_data[i] = sensor3.buffer[sensor3.transmit_head+(i*sampling_rate_divider)];
			tx_data[i+1] = sensor3.buffer[sensor3.transmit_head+(i*sampling_rate_divider)+1];
		}
		if (extendedTRANSMISSION(tx_data, size)==TRANSMIT_OK) // send the data
		{
			sensor3.transmit_head += size*sampling_rate_divider; // update the transmit head
		}
	}
	else if (sensor4.selected && sensor_transmit_state(&sensor4)!=TRANSMIT_FINISH)
	{
		// 12 is the maximum number of bytes that can be sent at once in extended mode
		// the following line compute how many data should be sent (12 or less)
		size = sensor4.head>=(sensor4.transmit_head+(12*sampling_rate_divider)) ? 12 : (uint8_t)((sensor4.head-sensor4.transmit_head)*divide_by_divider);
		for (uint8_t i = 0; i < size; i=i+2)
		{
			// create a new array with the samples that needs to be sent taking into account the divider
			tx_data[i] = sensor4.buffer[sensor4.transmit_head+(i*sampling_rate_divider)];
			tx_data[i+1] = sensor4.buffer[sensor4.transmit_head+(i*sampling_rate_divider)+1];
		}
		if (extendedTRANSMISSION(tx_data, size)==TRANSMIT_OK) // send the data
		{
			sensor4.transmit_head += size*sampling_rate_divider; // update the transmit head
		}
	}
	else
	{
		// all selected sensors have transmitted all their data
		state = TRANSMIT_FINISH;
	}

	return state;
}


/* Function: 	sampling case 3
 * 				HASN'T BEEN TESTED!
 * Return: 		status: same (stay with the same sensor or next (acquisition for that sensor is finished, -> next sensor)
 * Param:		s: pointer on the sensor
 */
uint8_t case_3_sampling(SENSOR_TypeDef *s)
{
	if (s->selected && s->nb_samples<nb_samples_case_3)
	{
		if(!phi_rst_ok && count_tim6>=TIM6_CNT_6MS)
		{
//			HAL_GPIO_WritePin(PHI_RST_GPIO_Port, PHI_RST_Pin, GPIO_PIN_RESET);
			phi_rst_ok = TRUE;

			// start signal phi1 and phi 2
			count_tim6 = 0;
			TIM2->CNT = TIM2CNT_OFFSET;
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
//			HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

		}
		else if (phi_rst_ok && count_tim6>=TIM6_CNT_20MS)
		{
			sampling_case_3_get_sample = TRUE;
		}
		if (phi_rst_ok && sampling_case_3_get_sample)
		{
			//for testing
//			dout_count = dummy_data_buffer[s->nb_samples];

			sensor_pushDataBuffer_2B_DOUT(s, dout_count);
			dout_count = 0;
			sampling_case_3_get_sample = FALSE;
			count_tim6 = 0;
		}
		return SAME; // stay with the same sensor
	}
	else
	{
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
		phi_rst_ok = FALSE;
		sampling_case_3_get_sample = FALSE;
//		HAL_GPIO_WritePin(PHI_RST_GPIO_Port, PHI_RST_Pin, GPIO_PIN_SET);
		count_tim6 = 0;
		dout_count = 0;

		return NEXT; // read for that sensor is finished -> next sensor
	}

}


/* Function: 	initializations for readout and sampling case 1
 * Return: 		none
 * Param:		none
 */
void readoutCase1_init(void)
{
	HAL_NVIC_DisableIRQ(EXTI2_IRQn);

	__HAL_RCC_SPI2_CLK_DISABLE();
//	__HAL_RCC_GPIOC_CLK_ENABLE();

	__HAL_RCC_SPI1_CLK_ENABLE();
	LL_SPI_Enable(SPI1);
//	__HAL_RCC_SPI2_CLK_ENABLE();
//	LL_SPI_Enable(SPI2);
//	__HAL_RCC_SPI3_CLK_ENABLE();
//	LL_SPI_Enable(SPI3);
//	__HAL_RCC_SPI4_CLK_ENABLE();
//	LL_SPI_Enable(SPI4);

	sensor_reset(&sensor1);
	sensor_reset(&sensor2);
	sensor_reset(&sensor3);
	sensor_reset(&sensor4);

//	HAL_GPIO_WritePin(DA1_GPIO_Port, DA1_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(DC1_GPIO_Port, DC1_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(SW_GPIO_Port, SW_Pin, GPIO_PIN_RESET);

//	HAL_GPIO_WritePin(DA2_GPIO_Port, DA2_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(DC2_GPIO_Port, DC2_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(SW_GPIO_Port, SW_Pin, GPIO_PIN_RESET);

//	HAL_GPIO_WritePin(DA3_GPIO_Port, DA3_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(DC3_GPIO_Port, DC3_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(SW_GPIO_Port, SW_Pin, GPIO_PIN_RESET);

//	HAL_GPIO_WritePin(DA4_GPIO_Port, DA4_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(DC4_GPIO_Port, DC4_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(SW_GPIO_Port, SW_Pin, GPIO_PIN_RESET);

}


/* Function: 	initializations for readout and sampling case 2
 * Return: 		none
 * Param:		none
 */
void readoutCase2_init(void)
{
	    __HAL_RCC_TIM6_CLK_ENABLE();
	    HAL_TIM_Base_Start_IT(&htim6);
	HAL_NVIC_DisableIRQ(EXTI2_IRQn);

// commented by junrui, 20190809, because now we are going to
// update the MUX control bits during the measurements
//    MUX_conf();

//	__HAL_RCC_SPI2_CLK_DISABLE();

//	__HAL_RCC_TIM6_CLK_ENABLE();

//	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_SPI1_CLK_ENABLE();
	LL_SPI_Enable(SPI1);
//	__HAL_RCC_SPI2_CLK_ENABLE();
//	LL_SPI_Enable(SPI2);
//	__HAL_RCC_SPI3_CLK_ENABLE();
//	LL_SPI_Enable(SPI3);
//	__HAL_RCC_SPI4_CLK_ENABLE();
//	LL_SPI_Enable(SPI4);


	sensor_reset(&sensor1);
	sensor_reset(&sensor2);
	sensor_reset(&sensor3);
	sensor_reset(&sensor4);

//	HAL_GPIO_WritePin(DA1_GPIO_Port, DA1_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(DC1_GPIO_Port, DC1_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(SW_GPIO_Port, SW_Pin, GPIO_PIN_SET);

//	HAL_GPIO_WritePin(DA2_GPIO_Port, DA2_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(DC2_GPIO_Port, DC2_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(SW_GPIO_Port, SW_Pin, GPIO_PIN_RESET);

//	HAL_GPIO_WritePin(DA3_GPIO_Port, DA3_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(DC3_GPIO_Port, DC3_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(SW_GPIO_Port, SW_Pin, GPIO_PIN_RESET);

//	HAL_GPIO_WritePin(DA4_GPIO_Port, DA4_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(DC4_GPIO_Port, DC4_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(SW_GPIO_Port, SW_Pin, GPIO_PIN_RESET);

	DB_i_state = GPIO_PIN_SET;

	//all DBi should be toggle by timer 4
//	HAL_TIM_Base_Start_IT(&htim6);

}


/* Function: 	initializations for readout and sampling case 3
 * Return: 		none
 * Param:		none
 */
void readoutCase3_init(void)
{
	HAL_NVIC_DisableIRQ(EXTI2_IRQn);

	__HAL_RCC_SPI2_CLK_DISABLE();

//	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_TIM6_CLK_ENABLE();
	TIM6->ARR = TIM6_PER_SET;
	__HAL_RCC_TIM2_CLK_ENABLE();

	sensor_reset(&sensor1);
	sensor_reset(&sensor2);
	sensor_reset(&sensor3);
	sensor_reset(&sensor4);
	sampling_case_3_state = CASE_3_SENSOR_1;
	phi_rst_ok = FALSE;
	sampling_case_3_get_sample = FALSE;

//	HAL_GPIO_WritePin(DA1_GPIO_Port, DA1_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(DC1_GPIO_Port, DC1_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(SW_GPIO_Port, SW_Pin, GPIO_PIN_RESET);

//	HAL_GPIO_WritePin(DA2_GPIO_Port, DA2_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(DC2_GPIO_Port, DC2_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(SW_GPIO_Port, SW_Pin, GPIO_PIN_RESET);

//	HAL_GPIO_WritePin(DA3_GPIO_Port, DA3_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(DC3_GPIO_Port, DC3_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(SW_GPIO_Port, SW_Pin, GPIO_PIN_RESET);

//	HAL_GPIO_WritePin(DA4_GPIO_Port, DA4_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(DC4_GPIO_Port, DC4_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(SW_GPIO_Port, SW_Pin, GPIO_PIN_RESET);

	//HAL_NVIC_EnableIRQ(EXTI9_5_IRQn); //enabled later in the code (just after pwm generation);

	//start timer4 to count 6ms;
	count_tim6 = 0;
	dout_count = 0;
//	HAL_GPIO_WritePin(PHI_RST_GPIO_Port, PHI_RST_Pin, GPIO_PIN_SET);
	HAL_TIM_Base_Start_IT(&htim6);
}


/* Function: 	function to see where the tag interrupt comes from (rx end, tx end, etc.)
 * Return: 		none
 * Param:		none
 */
void manageInterrupt()
{
	uint8_t rx_data[1];
	readRegister(INT_REG_0, rx_data); //read interrupt register 0 to determine the interrupt. AFTER READING THIS REGISTER, THE TAG CLEAR ALL FLAGS
//	printUART(rx_data, 1);
	switch (rx_data[0] & 0x06) {
		case 0x04:
			tag_rx_end = TRUE;
			printUART_MSG("-rx end int   \r\n");
			break;
		case 0x02:
//			printUART_MSG("-tx end int   \r\n");
			break;
		default:
			{
//				printUART_MSG("-unknown int  \r\n");
//				readRegister(INT_REG_1); //read interrupt register 1 to determine the interrupt. AFTER READING THIS REGISTER, THE TAG CLEAR ALL FLAGS
//				printUART(rx_data, 1);
//				switch (rx_data[0] & 0x80) {
//					case 0x80:
//						strcpy(msg, "-rx start int \r\n");
//						printUART_MSG(msg);
//						break;
//					default:
////						strcpy(msg, "-unknown int  \r\n");
//						break;
//				}
			}
			break;
	}
	tag_interupt_detected = FALSE; //disable
}


/* Function: 	External interrupt callback for line 2 (TagIRQ) or line 3 (Dout_count)
 * Return: 		none
 * Param:		GPIO_Pin: interrupt line
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//interrupt TAGIRQ
	if (GPIO_Pin==GPIO_PIN_2)
	{
		tag_interupt_detected = TRUE; //toggle interrupt flag
	}

	//interrupt DOUT COUNT
	if (GPIO_Pin==GPIO_PIN_3)
	{
		dout_count++;
	}

}


/* Function: 	Timer period elapsed callback for tim2 and tim6
 * Return: 		none
 * Param:		htim: pointer on the timer handle
 */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if (htim->Instance == TIM6)
//	{
//		if(1)//(main_state==SAMPLING_CASE_2)
//		{
//			if(DB_i_state ==  GPIO_PIN_RESET)
//			{
////				HAL_GPIO_WritePin(SW_GPIO_Port, SW_Pin, GPIO_PIN_SET);
//				TIM6->ARR = TIM6_PER_SET;
//				DB_i_state = GPIO_PIN_SET;
//			}
//			else //pins are set
//			{
////				HAL_GPIO_WritePin(SW_GPIO_Port, SW_Pin, GPIO_PIN_RESET);
//				TIM6->ARR = TIM6_PER_RESET;
//				DB_i_state = GPIO_PIN_RESET;
//			}
//		}
//
//		count_tim6++;
//	}
//
//	if (htim->Instance == TIM2)
//	{
////		count_tim2++;
//	}
//}

void MUX_conf(void)
{
	uint8_t cmd[1];

	// by Junrui, 20190709, to select sensors on the patch, feeding a current into one of the drains, and measure the reference voltage
    // where three digital signals DC/DB/DA are selecting 8 sensors from Xsensio's chip. via MAX4781 (DC DB DA).
	// Sensor_MUX is a 4 digit number, decimal value of which ranges from 01 to 12
	// Sensor_MUX = 0010 (2) --> drain2 is selected
	// Sensor_MUX = 0011 (3) --> drain3 is selected
	// Sensor_MUX = 0100 (4) --> drain4 is selected
	// Sensor_MUX = 0101 (5) --> drain5 is selected
	// Sensor_MUX = 0110 (6) --> drain6 is selected
	// Sensor_MUX = 0111 (7) --> drain7 is selected
	// Sensor_MUX = 1000 (8) --> drain8 is selected
	// Sensor_MUX = 1001 (9) --> drain9 is selected
	uint8_t MUX_value = Sensor_MUX;
	uint8_t CBA = 010;
	switch (MUX_value) {
		case 0x0010:
			CBA = 010; // drain 2
			break;
		case 0x0011:
			CBA = 001; // drain 3
			break;
		case 0x02:
			CBA = 000; // drain 4
			break;
		case 0x03:
			CBA = 011; // drain 5
			break;
		case 0x04:
			CBA = 100; // drain 6
			break;
		case 0x05:
			CBA = 110; // drain 7
			break;
		case 0x06:
			CBA = 111; // drain 8
			break;
		case 0x07:
			CBA = 101; // drain 9
			break;
	}

	uint8_t DCBA[3];
	DCBA[0] =  CBA & 0x01; 		 // A
	DCBA[1] = (CBA >> 1) & 0x01; // B
	DCBA[2] = (CBA >> 2) & 0x01; // C

    HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, DCBA[2]);
    HAL_GPIO_WritePin(DB_GPIO_Port, DB_Pin, DCBA[1]);
    HAL_GPIO_WritePin(DA_GPIO_Port, DA_Pin, DCBA[0]);

}

void Temperature(void)
{
	uint8_t cmd[1];

	cmd[0] = Sensor_MUX;
//	cmd[0] = MUX_REGISTER;// command code transmit buffer
//	cmd[1] = 0x00;

//	HAL_GPIO_WritePin(SPI2_SS1_GPIO_Port, SPI2_SS1_Pin, GPIO_PIN_RESET);
//	HAL_Delay(SPI2_SS1_TIMEOUT);
//	HAL_SPI_Transmit(&hspi2, cmd, 1, SPI2_TIMEOUT);
//	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
//	HAL_GPIO_WritePin(SPI2_SS1_GPIO_Port, SPI2_SS1_Pin, GPIO_PIN_SET);

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

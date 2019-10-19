/*
 * uart_com.c
 *
 *  Created on: 13 juil. 2017
 *      Author: jeremie.willemin@gmail.com
 *
 *
 *	This file contain functions used for ease of debugging purpose only.
 *	It uses the UART2 interface of the microcontroller. UART2 is connected
 *	to the ST-LINK interface on the NUCLEO board which enable a virtual com port on the computer.
 *	It is thus possible to use these functions to print information on a terminal
 *	for debugging purposes.
 *
 *	For the final version of the firmware these functions shoud not be used.
 */

#include "stm32l4xx_hal.h"
#include <string.h>
#include "uart_com.h"
#include "nfc.h"
extern UART_HandleTypeDef huart1;
//extern uint8_t rx_data[4];
//extern uint8_t rx_data_buffer[32];

/* Function: 	clear the terminal
 * Return: 		none
 * Param:		none
 */
void clearScreenUART(void)
{

	//clear screen
	HAL_UART_Transmit(&huart1, (uint8_t*)"\033[0;0H", strlen("\033[0;0H"), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1, (uint8_t*)"\033[2J", strlen("\033[2J"), HAL_MAX_DELAY);

}


/* Function: 	Print a message on the terminal
 * Return: 		none
 * Param:		msg: pointer on the string
 */
void printUART_MSG(char *msg)
{

	//message
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

}


/* Function: 	print sensor's data on the terminal in HEX format
 * Return: 		none
 * Param:		data:		pointer on sensor's data
 * 				nb_sample:	nb of sample to print
 */
void printUART_SENSOR_DATA(uint8_t* data, uint16_t nb_sample)
{
	/*
		 * Data printed are displayed in HEX format!
		 */

		uint16_t i;
		char num_str[3];
		for (i=0; i<nb_sample; i++){
			if (1)//(i%SAMPLES_RATIO==0) // if you want to send all samples (serial) set this statement to 1 (always true)
			{
				sprintf( num_str, "%02X", data[2*i] );
				HAL_UART_Transmit(&huart1, (uint8_t*)num_str, 3, HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart1, (uint8_t*)"  ", 2, HAL_MAX_DELAY);

				sprintf( num_str, "%02X", data[2*i+1] );
				HAL_UART_Transmit(&huart1, (uint8_t*)num_str, 3, HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart1, (uint8_t*)"  \r\n", 4, HAL_MAX_DELAY);
			}

		}


		//end of line
		//HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
}

/* Function: 	print data in HEX format on the terminal
 * Return: 		none
 * Param:		data: pointer on the data
 * 				size: size of the data buffer
 */
void printUART(uint8_t* data, uint8_t size)
{
	/*
	 * Data printed are displayed in HEX format!
	 */

	uint8_t i;
	char num_str[3];
	for (i=0; i<size; i++){
		if (i!=0 && i%4==0){
			//end of line
			HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
		}
		sprintf( num_str, "%02X", data[i] );
		HAL_UART_Transmit(&huart1, (uint8_t*)num_str, 3, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart1, (uint8_t*)"  ", 2, HAL_MAX_DELAY);

	}


	//end of line
	HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

}



/* Function: 	print the tag configuration on the terminal
 * Return: 		none
 * Param:		none
 */
void printUART_CONFIG(void)
{
	uint8_t rx_data[4];
	//message
	printUART_MSG("CONFIG\r\n");

	readEEPROM(EEPROM_CONFIG_LINE_1, rx_data);
	printUART(rx_data, 4);
	readEEPROM(EEPROM_CONFIG_LINE_2, rx_data);
	printUART(rx_data, 4);
	readEEPROM(EEPROM_CONFIG_LINE_3, rx_data);
	printUART(rx_data, 4);
	readEEPROM(EEPROM_CONFIG_LINE_4, rx_data);
	printUART(rx_data, 4);

}


/* Function: 	print the EEPROM content on the terminal
 * Return: 		none
 * Param:		none
 */
void printUART_DATA(void)
{
	uint8_t rx_data[4];
	uint8_t	address;
	for (address = EEPROM_DATA_1ST_LINE ; address<EEPROM_DATA_LAST_LINE; address++)
	{
		readEEPROM(address, rx_data);
		printUART(rx_data, 4);
	}

}


/* Function: 	print the buffer content (32 bytes) on the terminal
 * Return: 		none
 * Param:		buffer: pointer on the buffer
 */
void printUART_BUFFER(uint8_t *buffer)
{

	printUART_MSG("BUFFER DATA\r\n");
	printUART(buffer, 32);

}

/* Function: 	print buffer status on the terminal
 * Return: 		none
 * Param:		none
 */
void printUART_BUFFER_STATUS(void)
{
	uint8_t rx_data[1];
	printUART_MSG("BUFFER STATUS\r\n");

	readRegister(BUFFER_STATUS_2, rx_data);
	printUART(rx_data, 1);
	readRegister(BUFFER_STATUS_1, rx_data);
	printUART(rx_data, 1);
}


/* Function: 	print RFID status on the terminal
 * Return: 		none
 * Param:		none
 */
void printUART_RFID_STATUS(void)
{
	uint8_t rx_data[1];
	printUART_MSG("RFID STATUS\r\n");

	char msg[16];
	readRegister(RFID_STATUS, rx_data);
	printUART(rx_data, 1);
	switch (rx_data[0] & 0x7c) {
		case STATE_OFF:
			strcpy(msg, "-off       \r\n");
			break;
		case STATE_SENSE:
			strcpy(msg, "-sense     \r\n");
			break;
		case STATE_RESOLUTION:
			strcpy(msg, "-resolution\r\n");
			break;
		case STATE_SELECTED:
			strcpy(msg, "-selected  \r\n");
			break;
		case STATE_SELECTEDX:
			strcpy(msg, "-selectedX \r\n");
			break;
		case STATE_SENSEX:
			strcpy(msg, "-senseX    \r\n");
			break;
		case STATE_SLEEP:
			strcpy(msg, "-sleep     \r\n");
			break;
		default:
			strcpy(msg, "-other     \r\n");
			break;
	}
	printUART_MSG(msg);
}


/* Function: 	print the last connection address on the terminal
 * Return: 		none
 * Param:		none
 */
void printUART_LAST_ADDRESS(void)
{
	uint8_t rx_data[1];
	printUART_MSG("LAST ADDRESS\r\n");

	readRegister(LAST_NFC, rx_data);
	printUART(rx_data, 1);
}


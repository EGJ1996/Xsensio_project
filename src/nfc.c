/*
 * nfc.c
 *
 *  Created on: 13 juil. 2017
 *      Author: jeremie.willemin@gmail.com
 *
 * This file contains ready-to-use function related to the NFC chip AS3955.
 * The following possibilities are implemented:
 * 		- write data in EEPROM
 * 		- read data from EEPROM
 * 		- configure EEPROM with a predefinied configuration
 *
 * 		- read data from register
 *
 * 		- load data into the buffer (for extended communication)
 * 		- read data from the buffer (extended communication)
 *
 * 		- and a set of command that can be send to the chip (clear_buffer, transmit_buffer, etc)
 * 		(ref to datasheet of AS3955)
 *
 * 	TWO VERSION OF AS3955 EXIST, ONE WITH 4-Kb EEPROM AND ONE WITH 2-Kb EEPROM.
 * 	A CORRECT SELECTION OF EEPROM SIZE SHOULD BE DONE --> selection in main.h
 */

#include "stm32l4xx_hal.h"
#include "main.h"
#include "nfc.h"
extern SPI_HandleTypeDef hspi2;
//extern uint8_t rx_data[4];
//extern uint8_t rx_data_buffer[32];



/* Function: 	write data in the EEPROM at a specified address
 * Return: 		none
 * Param:		address:	address where to write the data
 * 				data:		pointer on the data that we want to write
 */
void writeEEPROM(uint8_t address, uint8_t* data)
{
	uint8_t cmd[6];

	cmd[0] = EEPROM_WRITE;// EEPROM WRITE
	cmd[1] = address<<1;

	cmd[2] = data[0];
	cmd[3] = data[1];
	cmd[4] = data[2];
	cmd[5] = data[3];

	HAL_GPIO_WritePin(SPI2_SS_Tag_GPIO_Port, SPI2_SS_Tag_Pin, GPIO_PIN_RESET);
	HAL_Delay(SPI2_SS_Tag_TIMEOUT);
	HAL_SPI_Transmit(&hspi2, cmd, 6, SPI2_TIMEOUT);
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(SPI2_SS_Tag_GPIO_Port, SPI2_SS_Tag_Pin, GPIO_PIN_SET);
}


/* Function: 	Read data in the EEPROM at a specified address
 * Return: 		none
 * Param:		address:	address where to read the data
 * 				data:		pointer on the data where we store the data read
 */
void readEEPROM(uint8_t address, uint8_t *rx_data)
{
	uint8_t cmd[2];

	cmd[0] = EEPROM_READ; //EEPROM READ
	cmd[1] = address<<1;

	HAL_GPIO_WritePin(SPI2_SS_Tag_GPIO_Port, SPI2_SS_Tag_Pin, GPIO_PIN_RESET);
	HAL_Delay(SPI2_SS_Tag_TIMEOUT);
//	asm("nop");
	HAL_SPI_Transmit(&hspi2, cmd, 2, SPI2_TIMEOUT);
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
	HAL_SPI_Receive(&hspi2, rx_data, 4, SPI2_TIMEOUT);
	HAL_GPIO_WritePin(SPI2_SS_Tag_GPIO_Port, SPI2_SS_Tag_Pin, GPIO_PIN_SET);
}


/* Function: 	Read data in a specific register
 * Return: 		none
 * Param:		address:	address of the register
 * 				data:		pointer on the data where we store the readings
 */
void readRegister(uint8_t address, uint8_t *rx_data)
{
	uint8_t cmd[1];

	cmd[0] = REGISTER_READ | address; //REGISTER READ + address

	HAL_GPIO_WritePin(SPI2_SS_Tag_GPIO_Port, SPI2_SS_Tag_Pin, GPIO_PIN_RESET);
	HAL_Delay(SPI2_SS_Tag_TIMEOUT);
	HAL_SPI_Transmit(&hspi2, cmd, 1, SPI2_TIMEOUT);
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
	HAL_SPI_Receive(&hspi2, rx_data, 1, SPI2_TIMEOUT);
	HAL_GPIO_WritePin(SPI2_SS_Tag_GPIO_Port, SPI2_SS_Tag_Pin, GPIO_PIN_SET);
}


/* Function: 	send a buffer load cmd to the AS3955 chip with the data that
 * 				we want to load.
 * Return: 		none
 * Param:		data:	pointer on the data
 * 				size:	size of the data that we want to load
 */
void buffer_LOAD(uint8_t *data, uint8_t size)
{
	uint8_t cmd[size+1];
	uint8_t i;

	cmd[0] = BUFFER_LOAD; //buffer load cmd

	for (i=0; i<size; i++)
	{
		cmd[i+1] = data[i];
	}

	HAL_GPIO_WritePin(SPI2_SS_Tag_GPIO_Port, SPI2_SS_Tag_Pin, GPIO_PIN_RESET);
	HAL_Delay(SPI2_SS_Tag_TIMEOUT);
	HAL_SPI_Transmit(&hspi2, cmd, size+1, SPI2_TIMEOUT);
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(SPI2_SS_Tag_GPIO_Port, SPI2_SS_Tag_Pin, GPIO_PIN_SET);
}

/* Function: 	send a buffer read cmd to the AS3955 chip.
 * Return: 		none
 * Param:		rx_data_buffer:	pointer on the data where to store the readings
 * 				size:	size of the data that we want to read
 */
void buffer_READ(uint8_t size, uint8_t *rx_data_buffer)
{
	uint8_t cmd[1];
	cmd[0] = BUFFER_READ; //buffer read cmd

	HAL_GPIO_WritePin(SPI2_SS_Tag_GPIO_Port, SPI2_SS_Tag_Pin, GPIO_PIN_RESET);
	HAL_Delay(SPI2_SS_Tag_TIMEOUT);
	HAL_SPI_Transmit(&hspi2, cmd, 1, SPI2_TIMEOUT);
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
	HAL_SPI_Receive(&hspi2, rx_data_buffer, size, SPI2_TIMEOUT);
	HAL_GPIO_WritePin(SPI2_SS_Tag_GPIO_Port, SPI2_SS_Tag_Pin, GPIO_PIN_SET);
}


/* Function: 	Configure the TAG in a predefined configuration
 * 				TAG MUST BE POWERED ON OBVIOUSLY
 * 				For some reasons, this function doesn't work sometimes. Execute it slowly in debug mode
 * Return: 		none
 * Param:		none
 */
void configure_EEPROM(void)
{

	/* Delays used to make the function running slower. It seems to work better that way */

	uint8_t tx_data[4];
	tx_data[0] = 0x00;
	tx_data[1] = 0x00;
	tx_data[2] = 0x00;
	tx_data[3] = 0x00;
	HAL_Delay(1);
	writeEEPROM(EEPROM_CONFIG_LINE_1, tx_data);
	HAL_Delay(1);

	tx_data[0] = 0x00;
	tx_data[1] = 0x77;
	tx_data[2] = 0xff;
	tx_data[3] = 0x00;
	HAL_Delay(1);
	writeEEPROM(EEPROM_CONFIG_LINE_2, tx_data);
	HAL_Delay(1);

	tx_data[0] = 0x44;
	tx_data[1] = 0x00;
	tx_data[2] = 0x00;
	tx_data[3] = 0x00;
	HAL_Delay(1);
	writeEEPROM(EEPROM_CONFIG_LINE_3, tx_data);
	HAL_Delay(1);

//	tx_data[0] = 0x00 | 0x01;				//Vreg voltage level set to 3.3v, Rout set to 100 Ohm
//	tx_data[1] = 0x80 | 0x20 | 0x08 | 0x01;	//Enable conf over rf, enable extended mode, enable auth_set RF, power mode 1
//	tx_data[2] = 0xff ^ (0x02 | 0x04); 		//enable interrupt tx end, rx end
//	tx_data[3] = 0xff;
	tx_data[0] = 0x33;
	tx_data[1] = 0xA9;
	tx_data[2] = 0xF9;
	tx_data[3] = 0xFF;
	HAL_Delay(1);
	writeEEPROM(EEPROM_CONFIG_LINE_4, tx_data);
	HAL_Delay(1);
}

/* Function: 	send a set to default cmd to the chip (ref AS3955 datasheet)
 * Return: 		none
 * Param:		none
 */
void cmdSET_DEFAULT(void)
{
	uint8_t cmd[2];

	cmd[0] = CMD_SET_DEFAULT;// command code set default
	cmd[1] = 0x00;

	HAL_GPIO_WritePin(SPI2_SS_Tag_GPIO_Port, SPI2_SS_Tag_Pin, GPIO_PIN_RESET);
	HAL_Delay(SPI2_SS_Tag_TIMEOUT);
	HAL_SPI_Transmit(&hspi2, cmd, 2, SPI2_TIMEOUT);
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(SPI2_SS_Tag_GPIO_Port, SPI2_SS_Tag_Pin, GPIO_PIN_SET);
}


/* Function: 	send a restart transceiver cmd to the chip (ref AS3955 datasheet)
 * Return: 		none
 * Param:		none
 */
void cmdRESTART_TRANSCEIVER(void)
{
	uint8_t cmd[2];

	cmd[0] = CMD_RESTART_TRANSCEIVER;// command code
	cmd[1] = 0x00;

	HAL_GPIO_WritePin(SPI2_SS_Tag_GPIO_Port, SPI2_SS_Tag_Pin, GPIO_PIN_RESET);
	HAL_Delay(SPI2_SS_Tag_TIMEOUT);
	HAL_SPI_Transmit(&hspi2, cmd, 2, SPI2_TIMEOUT);
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(SPI2_SS_Tag_GPIO_Port, SPI2_SS_Tag_Pin, GPIO_PIN_SET);
}

/* Function: 	send a clear buffer cmd to the chip. This set and reset some flags
 * Return: 		none
 * Param:		none
 */
void cmdCLEAR_BUFFER(void)
{
	uint8_t cmd[2];

	cmd[0] = CMD_CLEAR_BUFFER;// command code clear buffer
	cmd[1] = 0x00;

	HAL_GPIO_WritePin(SPI2_SS_Tag_GPIO_Port, SPI2_SS_Tag_Pin, GPIO_PIN_RESET);
	HAL_Delay(SPI2_SS_Tag_TIMEOUT);
	HAL_SPI_Transmit(&hspi2, cmd, 2, SPI2_TIMEOUT);
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(SPI2_SS_Tag_GPIO_Port, SPI2_SS_Tag_Pin, GPIO_PIN_SET);
}


/* Function: 	send a transmit buffer cmd to the chip (ref AS3955 datasheet)
 * Return: 		none
 * Param:		none
 */
void cmdTRANSMIT_BUFFER(void)
{
	uint8_t cmd[2];

	cmd[0] = CMD_TRANSMIT_BUFFER;// command code transmit buffer
	cmd[1] = 0x00;

	HAL_GPIO_WritePin(SPI2_SS_Tag_GPIO_Port, SPI2_SS_Tag_Pin, GPIO_PIN_RESET);
	HAL_Delay(SPI2_SS_Tag_TIMEOUT);
	HAL_SPI_Transmit(&hspi2, cmd, 2, SPI2_TIMEOUT);
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(SPI2_SS_Tag_GPIO_Port, SPI2_SS_Tag_Pin, GPIO_PIN_SET);
}


/* Function: 	implement the extended transmission to tranmit a certain number of bytes
 * Return: 		status: Busy, Ready or Ok
 * Param:		data: pointer on the data that we want to transmit
 * 				size: size of the data buffer (max is 12)
 */
uint8_t extendedTRANSMISSION(uint8_t *data, uint8_t size)
{
	//read flags
	uint8_t rx_data[1];
	readRegister(BUFFER_STATUS_2, rx_data);//buffer status register 2
	if( rx_data[0] )
	{
		//still data to read out
//		printMSG("still data to read\r\n");
		return TRANSMIT_BUSY;
	}
	else{
		readRegister(BUFFER_STATUS_1, rx_data);//buffer status register 1
		if( rx_data[0] & 0x08 ) //MASK test io_data_ready=1
		{
			//transmission is already done, data needs to be read by nfc device
//			printMSG("transmission is ready\r\n");
			return TRANSMIT_READY;
		}
		else{
			cmdCLEAR_BUFFER();
			buffer_LOAD(data, size);
			cmdTRANSMIT_BUFFER();
			return TRANSMIT_OK;
		}

	}
}


/* Function: 	implement the function to read the bytes received in extended transmission
 * Return: 		status: length not valid, Ready or Ok
 * Param:		rx_data_buffer: pointer on the buffer where we want to store the data
 */
uint8_t extendedRECEIVE(uint8_t *rx_data_buffer)
{
	uint8_t rx_data[1];

	readRegister(BUFFER_STATUS_1, rx_data);//buffer status register 1
	if( rx_data[0] & 0x10 ) //MASK test rf_data_ready=1 (data received store in buffer waiting to be processed)
	{

		readRegister(BUFFER_STATUS_2, rx_data);//buffer status register 2
		if( rx_data[0] & 0x80) //MASK test buf_len_invalid
		{
			//Buffer content is being changed - data length not valid
			return RECEIVE_LENGTH_NOT_VALID;
		}
		else
		{
			buffer_READ(rx_data[0] & 0x3f, rx_data_buffer); //MAKS keeps only 5 LSB
			cmdCLEAR_BUFFER();
			return RECEIVE_OK;
		}
	}
	else
	{
		return RECEIVE_NOT_READY;
	}

}


/* Function: 	Check if a NFC Device (phone) is connected to the Tag
 * Return: 		status: True or False
 * Param:		none
 */
uint8_t device_detected(void)
{
	uint8_t rx_data[1];

	readRegister(RFID_STATUS, rx_data);//RFID status register
	if( rx_data[0]==0xF0) //NFC device detected -> state = selectedX + PICC AFE is active (MSB)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}




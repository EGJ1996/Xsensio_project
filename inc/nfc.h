/*
 * nfc.h
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

#ifndef NFC_H_
#define NFC_H_

//#ifdef EEPROM_VERSION_2K_

#define EEPROM_DATA_1ST_LINE 	0x04
#define EEPROM_DATA_LAST_LINE 	0x39 // FOR 2-KB EEPROM TAG
#define EEPROM_CONFIG_LINE_1	0x3c // FOR 2-KB EEPROM TAG
#define EEPROM_CONFIG_LINE_2	0x3d // FOR 2-KB EEPROM TAG
#define EEPROM_CONFIG_LINE_3	0x3e // FOR 2-KB EEPROM TAG
#define EEPROM_CONFIG_LINE_4	0x3f // FOR 2-KB EEPROM TAG

//#endif /* EEPROM_VERSION_2K_ */

/*#ifdef EEPROM_VERSION_4K_

#define EEPROM_DATA_1ST_LINE 	0x04
#define EEPROM_DATA_LAST_LINE 	0x79 // FOR 4-KB EEPROM TAG
#define EEPROM_CONFIG_LINE_1	0x7c // FOR 4-KB EEPROM TAG
#define EEPROM_CONFIG_LINE_2	0x7d // FOR 4-KB EEPROM TAG
#define EEPROM_CONFIG_LINE_3	0x7e // FOR 4-KB EEPROM TAG
#define EEPROM_CONFIG_LINE_4	0x7f // FOR 4-KB EEPROM TAG

#endif*/ /* EEPROM_VERSION_4K_ */

#define SPI2_TIMEOUT 	2	//ms
#define SPI2_SS_Tag_TIMEOUT	1	//ms
#define SPI2_SS1_TIMEOUT 1 //ms

#define IO_CONF			0x00
#define IC_CONF_0 		0x01
#define IC_CONF_1 		0x02
#define IC_CONF_2 		0x03
#define RFID_STATUS 	0x04
#define IC_STATUS		0x05
#define INT_REG_0 		0x0a
#define INT_REG_1 		0x0b
#define BUFFER_STATUS_2 0x0c
#define BUFFER_STATUS_1 0x0d
#define LAST_NFC 		0x0e

#define EEPROM_WRITE 	0x40
#define EEPROM_READ 	0x7f
#define REGISTER_READ 	0x20
#define BUFFER_LOAD 	0x80
#define BUFFER_READ 	0xbf//0xA0

#define CMD_SET_DEFAULT 		0xc2
#define CMD_RESTART_TRANSCEIVER 0xc6
#define CMD_CLEAR_BUFFER 		0xc4
#define CMD_TRANSMIT_BUFFER 	0xc8

#define STATE_OFF			0b00000000
#define STATE_SENSE			0b00001000
#define STATE_RESOLUTION	0b00011000
#define STATE_SELECTED		0b00110000
#define STATE_SELECTEDX		0b01110000
#define STATE_SENSEX		0b01011000
#define STATE_SLEEP			0b01001000

#define CMD_PHONE_SENSOR_SELECT 0x00
#define CMD_PHONE_PROCESS_OK	0x00
#define CMD_PHONE_PROCESS_FAIL	0x01

#define TRANSMIT_OK		0x00
#define TRANSMIT_BUSY	0x01
#define TRANSMIT_READY	0x02

#define RECEIVE_OK					0x00
#define RECEIVE_LENGTH_NOT_VALID	0x01
#define RECEIVE_NOT_READY			0x02

/*
 * FCT Prototype
 */

void writeEEPROM(uint8_t address, uint8_t* data);
void readEEPROM(uint8_t address, uint8_t *rx_data);

void readRegister(uint8_t address, uint8_t *rx_data);

void buffer_LOAD(uint8_t *data, uint8_t size);
void buffer_READ(uint8_t size, uint8_t *rx_data);

void configure_EEPROM(void);

void cmdSET_DEFAULT(void);
void cmdRESTART_TRANSCEIVER(void);
void cmdCLEAR_BUFFER(void);
void cmdTRANSMIT_BUFFER(void);

uint8_t extendedTRANSMISSION(uint8_t *data, uint8_t size);
uint8_t extendedRECEIVE(uint8_t *rx_data_buffer);

uint8_t device_detected(void);


#endif /* NFC_H_ */

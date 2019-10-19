/*
 * uart_com.h
 *
 *  Created on: 13 juil. 2017
 *      Author: jeremie.willemin@gmail.com
 *
 *  This file contain functions used for ease of debugging purpose only.
 *	It uses the UART2 interface of the microcontroller. UART2 is connected
 *	to the ST-LINK interface on the NUCLEO board which enable a virtual com port on the computer.
 *	It is thus possible to use these functions to print information on a terminal
 *	for debugging purposes.
 *
 *	For the final version of the firmware these functions shoud not be used.
 */

#ifndef UART_COM_H_
#define UART_COM_H_

void clearScreenUART(void);
void printUART_MSG(char *msg);
void printUART_SENSOR_DATA(uint8_t* data, uint16_t nb_sample);
void printUART(uint8_t* data, uint8_t size);
void printUART_CONFIG(void);
void printUART_DATA(void);
void printUART_BUFFER(uint8_t *buffer);
void printUART_BUFFER_STATUS(void);
void printUART_RFID_STATUS(void);
void printUART_LAST_ADDRESS(void);

#endif /* UART_COM_H_ */

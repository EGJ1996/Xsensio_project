/*
 * sensor.h
 *
 *  Created on: 19 juil. 2017
 *      Author: jeremie.willemin@gmail.com
 *
 * File containing data structure of sensors as well as functions related
 * to this data structure. Push data, reset sensor, etc
 *
 * The function to get samples use a SPI peripheral (SPI1).
 *
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#define SENSOR1_BUFFER_SIZE		4500 //bytes, (2 bytes = 1 sample)
#define SENSOR2_BUFFER_SIZE		4500 //bytes, (2 bytes = 1 sample)
#define SENSOR3_BUFFER_SIZE		4500 //bytes, (2 bytes = 1 sample)
#define SENSOR4_BUFFER_SIZE		4500 //bytes, (2 bytes = 1 sample)

#define SPI_SENSOR_TIMEOUT 		2	//ms
#define SPI_SENSOR_SS_TIMEOUT	1	//ms


#define TRANSMIT_FINISH			0x00
#define TRANSMIT_NOT_FINISH		0x01

#define DEST_DATA		0x00
#define DEST_LEAKAGE	0x01
#define MUX_REGISTER	0x0B

typedef struct __SENSOR_TypeDef{
	uint8_t		number; 		// sensor 1 2 3 ou 4
	uint8_t		selected; 		// true or false
	uint8_t		* const buffer; // data (the pointer is const not the data)
	uint8_t		* const buffer_leakage; // leakage samples for differentiate measurements (the pointer is const not the data)
	uint16_t	head;			// point to the address at the end of the data inside the buffer
	uint16_t	head_leakage;	// point to the address at the end of the leakage buffer
	uint16_t	transmit_head;	// point to the address at the end of the data that have been transmitted already
	uint16_t	nb_samples;		// nb samples inside the buffer (= buffer length in bytes divided by 2 )
	uint16_t	maxLen;			// size of the buffer

} SENSOR_TypeDef;

void TEST_sensor_push_dummy_data_2B(SENSOR_TypeDef* sensor, uint8_t* data);

void sensor_pushDataBuffer_2B_DOUT(SENSOR_TypeDef* sensor, uint16_t data);

void sensor_pushDataBuffer_2B(SENSOR_TypeDef* sensor, uint16_t* data);
void sensor_pushLeakageBuffer_2B(SENSOR_TypeDef* sensor, uint16_t* data_16);
HAL_StatusTypeDef sensor_get1Sample(SENSOR_TypeDef* sensor, uint8_t dest);
void sensor_differentiateSignals(SENSOR_TypeDef* sensor, uint16_t nb_samples);

void sensor_reset(SENSOR_TypeDef* s);


void MUX_conf(void);
uint8_t sensor_transmit_state(SENSOR_TypeDef* sensor);

#endif /* SENSOR_H_ */

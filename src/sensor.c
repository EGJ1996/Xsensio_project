/*
 * sensor.c
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

#include "stm32l4xx_hal.h"
#include "spi_ll_com.h"
#include "sensor.h"

//extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

uint16_t rx_data_sensor1_2b = 0;
uint16_t rx_data_sensor2_2b = 0;
uint16_t rx_data_sensor3_2b = 0;
uint16_t rx_data_sensor4_2b = 0;


void TEST_sensor_push_dummy_data_2B(SENSOR_TypeDef* sensor, uint8_t* data)
{
	uint16_t next = sensor->head + 2;
	if (next >= sensor->maxLen)
		next = 0;

	sensor->buffer[sensor->head] = data[0];
	sensor->buffer[(sensor->head)+1] = data[1];

	sensor->head = next;
	sensor->nb_samples++;
}
/* Function: 	push data into the sensor's data structure
 * Return: 		None
 * Param:		sensor: pointer on the sensor structure
 * 				data:	data to push into the buffer
 */
void sensor_pushDataBuffer_2B_DOUT(SENSOR_TypeDef* sensor, uint16_t data)
{
	uint8_t data_2b[2];
	data_2b[1] = (uint8_t)(data & 0xff);
	data_2b[0] = (uint8_t)(data >> 8);

	// next is where head will point to after this write.
	uint16_t next = sensor->head + 2;
	if (next >= sensor->maxLen)
		next = 0;

	sensor->buffer[sensor->head] = data_2b[0];
	sensor->buffer[(sensor->head)+1] = data_2b[1];

	sensor->head = next;
	sensor->nb_samples++;
}


/* Function: 	push data into the sensor's data structure with  bitwise operations (formating)
 * Return: 		None
 * Param:		sensor: pointer on the sensor structure
 * 				data:	pointer on data to push into the buffer
 */
void sensor_pushDataBuffer_2B(SENSOR_TypeDef* sensor, uint16_t* data_16)
{
	uint8_t data[2];
	data[1] = (uint8_t)(*data_16 & 0xff);
	data[0] = (uint8_t)(*data_16 >> 8);

	// next is where head will point to after this write.
	uint16_t next = sensor->head + 2;
	if (next >= sensor->maxLen)
		next = 0;

	/* bitwise operation to transform
	 * data[0]						data[1]
	 * |0|0|D11|D10|D9|D8|D7|D6|  |D5|D4|D3|D2|D1|D0|x|x|
	 * to
	 *
	 *   head --> | 0| 0| 0| 0|D11|D10|D9|D8|
	 * head+1 --> |D7|D6|D5|D4| D3| D2|D1|D0|
	 */

	sensor->buffer[sensor->head] = (data[0]>>2 & 0x0F);
	sensor->buffer[(sensor->head)+1] = (data[0]<<6) | (data[1]>>2);


	sensor->head = next;
	sensor->nb_samples++;
}


/* Function: 	push data into the sensor's leakage data structure with  bitwise operations (formating)
 * Return: 		None
 * Param:		sensor: pointer on the sensor structure
 * 				data:	pointer on data to push into the buffer
 */
void sensor_pushLeakageBuffer_2B(SENSOR_TypeDef* sensor, uint16_t* data_16)
{
	uint8_t data[2];
	data[1] = (uint8_t)(*data_16 & 0xff);
	data[0] = (uint8_t)(*data_16 >> 8);

	// next is where head will point to after this write.
	uint16_t next = sensor->head_leakage + 2;
	if (next >= sensor->maxLen)
		next = 0;

	/* bitwise operation to transform
	 * data[0]						data[1]
	 * |0|0|D11|D10|D9|D8|D7|D6|  |D5|D4|D3|D2|D1|D0|x|x|
	 * to
	 *
	 *   head --> | 0| 0| 0| 0|D11|D10|D9|D8|
	 * head+1 --> |D7|D6|D5|D4| D3| D2|D1|D0|
	 */

	sensor->buffer_leakage[sensor->head_leakage] = (data[0]>>2 & 0x0F);
	sensor->buffer_leakage[(sensor->head_leakage)+1] = (data[0]<<6) | (data[1]>>2);


	sensor->head_leakage = next;
//	sensor->nb_samples++; // This is already done by the data
}

/* Function: 	get 1 sample using SPI1
 * Return: 		status: Ok or Busy
 * Param:		sensor: sensor from which we want 1 sample
 * 				dest: in which buffer we want to store the sample (data or leakage)
 */
HAL_StatusTypeDef sensor_get1Sample(SENSOR_TypeDef* sensor, uint8_t dest)
{
//	switch (sensor->number) {
//		case 1:
//			if (hspi1.State == HAL_SPI_STATE_READY)
//			{
////				HAL_StatusTypeDef statusRx;
////				HAL_GPIO_WritePin(SPI1_SSA_GPIO_Port, SPI1_SSA_Pin, GPIO_PIN_RESET);
////				statusRx = HAL_SPI_Receive(&hspi1, rx_data_sensor1, 1, 10);
////				while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
////				HAL_GPIO_WritePin(SPI1_SSA_GPIO_Port, SPI1_SSA_Pin, GPIO_PIN_SET);
////				return HAL_OK;
////				return HAL_SPI_Receive_DMA(&hspi1, rx_data_sensor1, 1);
//
//				/* LL library for faster spi com */
////				HAL_GPIO_WritePin(SPI1_SSA_GPIO_Port, SPI1_SSA_Pin, GPIO_PIN_RESET);
////				HAL_GPIO_WritePin(SPI1_SSC_GPIO_Port, SPI1_SSC_Pin, GPIO_PIN_RESET);
////				rx_data_sensor1_2b = SPI1_Read2Bytes();
////				HAL_GPIO_WritePin(SPI1_SSA_GPIO_Port, SPI1_SSA_Pin, GPIO_PIN_SET);
////				HAL_GPIO_WritePin(SPI1_SSC_GPIO_Port, SPI1_SSC_Pin, GPIO_PIN_SET);
//				if(dest==DEST_LEAKAGE)
//				{
//					sensor_pushLeakageBuffer_2B(sensor, &rx_data_sensor1_2b);
//				}
//				else //DEST_DATA
//				{
//					sensor_pushDataBuffer_2B(sensor, &rx_data_sensor1_2b);
//				}
//
//				return HAL_OK;
//
//
//			}
//			else
//			{
//				return HAL_BUSY;
//			}
//
//			break;
//		case 2:
//			if (hspi1.State == HAL_SPI_STATE_READY)
//			{
////				HAL_GPIO_WritePin(SPI1_SSB_GPIO_Port, SPI1_SSB_Pin, GPIO_PIN_RESET);
////				return HAL_SPI_Receive_DMA(&hspi2, rx_data_sensor2, 1);
//
//				/* LL library for faster spi com */
////				HAL_GPIO_WritePin(SPI1_SSB_GPIO_Port, SPI1_SSB_Pin, GPIO_PIN_RESET);
////				rx_data_sensor2_2b = SPI1_Read2Bytes();
////				HAL_GPIO_WritePin(SPI1_SSB_GPIO_Port, SPI1_SSB_Pin, GPIO_PIN_SET);
//				if(dest==DEST_LEAKAGE)
//				{
//					sensor_pushLeakageBuffer_2B(sensor, &rx_data_sensor2_2b);
//				}
//				else //DEST_DATA
//				{
//					sensor_pushDataBuffer_2B(sensor, &rx_data_sensor2_2b);
//				}
//
//				return HAL_OK;
//			}
//			else
//			{
//				return HAL_BUSY;
//			}
//
//			break;
//		case 3:  // temperature sensor use SPI2
//			if (hspi2.State == HAL_SPI_STATE_READY)
//			{
////				HAL_GPIO_WritePin(SPI1_SSC_GPIO_Port, SPI1_SSC_Pin, GPIO_PIN_RESET);
////				return HAL_SPI_Receive_DMA(&hspi3, rx_data_sensor3, 1);
//
//				/* LL library for faster spi com */
////				HAL_GPIO_WritePin(SPI1_SSC_GPIO_Port, SPI1_SSC_Pin, GPIO_PIN_RESET);
////				HAL_GPIO_WritePin(SPI1_SSC_GPIO_Port, SPI1_SSC_Pin, GPIO_PIN_RESET);
////				rx_data_sensor3_2b = SPI1_Read2Bytes();
////				HAL_GPIO_WritePin(SPI1_SSC_GPIO_Port, SPI1_SSC_Pin, GPIO_PIN_SET);
////				HAL_GPIO_WritePin(SPI1_SSC_GPIO_Port, SPI1_SSC_Pin, GPIO_PIN_SET);
////				sensor_pushDataBuffer_2B(sensor, &rx_data_sensor3_2b);
//				if(dest==DEST_LEAKAGE)
//				{
//					sensor_pushLeakageBuffer_2B(sensor, &rx_data_sensor3_2b);
//				}
//				else //DEST_DATA
//				{
//					sensor_pushDataBuffer_2B(sensor, &rx_data_sensor3_2b);
//				}
//
//				return HAL_OK;
//			}
//			else
//			{
//				return HAL_BUSY;
//			}
//
//			break;
////		case 4:
////			if (hspi1.State == HAL_SPI_STATE_READY)
////			{
//////				HAL_GPIO_WritePin(SPI1_SSD_GPIO_Port, SPI1_SSD_Pin, GPIO_PIN_RESET);
//////				return HAL_SPI_Receive_DMA(&hspi4, rx_data_sensor4, 1);
////
////				/* LL library for faster spi com */
////				HAL_GPIO_WritePin(SPI1_SSD_GPIO_Port, SPI1_SSD_Pin, GPIO_PIN_RESET);
////				rx_data_sensor4_2b = SPI1_Read2Bytes();
////				HAL_GPIO_WritePin(SPI1_SSD_GPIO_Port, SPI1_SSD_Pin, GPIO_PIN_SET);
////				if(dest==DEST_LEAKAGE)
////				{
////					sensor_pushLeakageBuffer_2B(sensor, &rx_data_sensor4_2b);
////				}
////				else //DEST_DATA
////				{
////					sensor_pushDataBuffer_2B(sensor, &rx_data_sensor4_2b);
////				}
////
////				return HAL_OK;
////			}
////			else{
////				return HAL_BUSY;
////			}
////
////
////			break;
//		default:
//			return HAL_ERROR;
//			break;
//	}
}


/* Function: 	reset the sensor
 * Return: 		None
 * Param:		s: pointer on the sensor structure
 */
void sensor_reset(SENSOR_TypeDef* s)
{
	s->head = 0;
	s->head_leakage = 0;
	s->transmit_head = 0;
	s->nb_samples = 0;
}


/* Function: 	return status Finish if all data inside the sensor's data structure are already transmitted
 * Return: 		transmission status: Finish or Not Finish
 * Param:		sensor: pointer on the sensor structure
 */
uint8_t sensor_transmit_state(SENSOR_TypeDef* sensor)
{
	if(sensor->transmit_head < sensor->head)
	{
		return TRANSMIT_NOT_FINISH;
	}
	else
	{
		return TRANSMIT_FINISH;
	}

}

/* Function: 	make the differential computation (sample1-leakage1, sample2-leakage2 etc)
 * 				New data are stored in the buffer sensor
 * Return: 		none
 * Param:		sensor: pointer on the sensor structure
 * 				nb_samples: nb samples that contain the buffer
 */
void sensor_differentiateSignals(SENSOR_TypeDef* sensor, uint16_t nb_samples)
{
	uint16_t data, leakage, new, i;
	for(i=0; i<nb_samples ; i+=2)
	{
		// 2 bytes = 1 sample
		data 		= (((uint16_t)sensor->buffer[i])<<8) + ((uint16_t)sensor->buffer[i+1]);
		leakage 	= (((uint16_t)sensor->buffer_leakage[i])<<8) + ((uint16_t)sensor->buffer_leakage[i+1]);

		// difference
		new = (data - leakage);

		// put the new sample into 2 bytes
		sensor->buffer[i] 	= (uint8_t)(new >> 8);
		sensor->buffer[i+1] = (uint8_t)(new & 0xff);

	}
}





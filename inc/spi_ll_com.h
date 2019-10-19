/*
 * spi_ll_com.h
 *
 *  Created on: 2 août 2017
 *      Author: jeremie.willemin@gmail.com
 *
 * 	This file implement functions for SPI communication using
 * 	the low-layer library (ll), for a faster communication than HAL.
 *
 * 	The configuration of the peripheral is still done with HAL.
 */

#ifndef SPI_LL_COM_H_
#define SPI_LL_COM_H_

#define SPI_READ_TIMEOUT	1


uint16_t SPI1_Read2Bytes(void);
uint16_t SPI2_Read2Bytes(void);
uint16_t SPI3_Read2Bytes(void);
uint16_t SPI4_Read2Bytes(void);




#endif /* SPI_LL_COM_H_ */

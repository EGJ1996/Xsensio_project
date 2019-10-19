/*
 * spi_ll_com.c
 *
 *  Created on: 2 août 2017
 *      Author: jeremie.willemin@gmail.com
 *
 * 	This file implement functions for SPI communication using
 * 	the low-layer library (ll), for a faster communication than HAL.
 *
 * 	The configuration of the peripheral is still done with HAL.
 */


#include "stm32l4xx_hal.h"
#include "stm32l4xx_ll_spi.h"
#include "spi_ll_com.h"


/* Function: 	Read 2 bytes from SPI1
 * Return: 		data received
 * Param:		none
 */
uint16_t SPI1_Read2Bytes(void)
{
    uint32_t start_time_r;
    uint32_t this_time_r;

    while (LL_SPI_IsActiveFlag_RXNE(SPI1))
    {
        (void)LL_SPI_ReceiveData16(SPI1);      // flush any FIFO content
    }

    while (!LL_SPI_IsActiveFlag_TXE(SPI1))
    {
        ;
    }

    start_time_r = HAL_GetTick();

//    HAL_DMA_Start_IT(&hdma_spi1_rx, SPI1.DR, dstaddress, datalength);

    LL_SPI_TransmitData16(SPI1, 0xFFFF);   // send dummy byte
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1))
    {
        this_time_r = HAL_GetTick() - start_time_r;
        if (this_time_r > SPI_READ_TIMEOUT)
        {
            // timeout error!
            break;
        }
    	;
    }
    return LL_SPI_ReceiveData16(SPI1);
}

/* Function: 	Read 2 bytes from SPI2
 * Return: 		data received
 * Param:		none
 */
uint16_t SPI2_Read2Bytes(void)
{
    uint32_t start_time_r;
    uint32_t this_time_r;

    while (LL_SPI_IsActiveFlag_RXNE(SPI2))
    {
        (void)LL_SPI_ReceiveData16(SPI2);      // flush any FIFO content
    }

    while (!LL_SPI_IsActiveFlag_TXE(SPI2))
    {
        ;
    }

    start_time_r = HAL_GetTick();

//    HAL_DMA_Start_IT(&hdma_spi1_rx, SPI1.DR, dstaddress, datalength);

    LL_SPI_TransmitData16(SPI2, 0xFFFF);   // send dummy byte
    while (!LL_SPI_IsActiveFlag_RXNE(SPI2))
    {
        this_time_r = HAL_GetTick() - start_time_r;
        if (this_time_r > SPI_READ_TIMEOUT)
        {
            // timeout error!
            break;
        }
    	;
    }
    return LL_SPI_ReceiveData16(SPI2);
}

/* Function: 	Read 2 bytes from SPI3
 * Return: 		data received
 * Param:		none
 */
//uint16_t SPI3_Read2Bytes(void)
//{
//    uint32_t start_time_r;
//    uint32_t this_time_r;
//
//    while (LL_SPI_IsActiveFlag_RXNE(SPI3))
//    {
//        (void)LL_SPI_ReceiveData16(SPI3);      // flush any FIFO content
//    }
//
//    while (!LL_SPI_IsActiveFlag_TXE(SPI3))
//    {
//        ;
//    }
//
//    start_time_r = HAL_GetTick();
//
////    HAL_DMA_Start_IT(&hdma_spi1_rx, SPI1.DR, dstaddress, datalength);
//
//    LL_SPI_TransmitData16(SPI3, 0xFFFF);   // send dummy byte
//    while (!LL_SPI_IsActiveFlag_RXNE(SPI3))
//    {
//        this_time_r = HAL_GetTick() - start_time_r;
//        if (this_time_r > SPI_READ_TIMEOUT)
//        {
//            // timeout error!
//            break;
//        }
//    	;
//    }
//    return LL_SPI_ReceiveData16(SPI3);
//}
//
///* Function: 	Read 2 bytes from SPI4
// * Return: 		data received
// * Param:		none
// */
//uint16_t SPI4_Read2Bytes(void)
//{
//    uint32_t start_time_r;
//    uint32_t this_time_r;
//
//    while (LL_SPI_IsActiveFlag_RXNE(SPI4))
//    {
//        (void)LL_SPI_ReceiveData16(SPI4);      // flush any FIFO content
//    }
//
//    while (!LL_SPI_IsActiveFlag_TXE(SPI4))
//    {
//        ;
//    }
//
//    start_time_r = HAL_GetTick();
//
////    HAL_DMA_Start_IT(&hdma_spi1_rx, SPI1.DR, dstaddress, datalength);
//
//    LL_SPI_TransmitData16(SPI4, 0xFFFF);   // send dummy byte
//    while (!LL_SPI_IsActiveFlag_RXNE(SPI4))
//    {
//        this_time_r = HAL_GetTick() - start_time_r;
//        if (this_time_r > SPI_READ_TIMEOUT)
//        {
//            // timeout error!
//            break;
//        }
//    	;
//    }
//    return LL_SPI_ReceiveData16(SPI4);
//}

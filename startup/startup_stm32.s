/**
  ******************************************************************************
  * @file      startup_stm32.s dedicated to STM32L443CCTx device
  * @author    Ac6
  * @version   V1.0.0
  * @date      2019-10-12
  ******************************************************************************
  */


.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

.global g_pfnVectors
.global Default_Handler

/* start address for the initialization values of the .data section.
defined in linker script */
.word _sidata
/* start address for the .data section. defined in linker script */
.word _sdata
/* end address for the .data section. defined in linker script */
.word _edata
/* start address for the .bss section. defined in linker script */
.word _sbss
/* end address for the .bss section. defined in linker script */
.word _ebss

/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied main() routine is called.
 * @param  None
 * @retval : None
*/

  .section .text.Reset_Handler
  .weak Reset_Handler
  .type Reset_Handler, %function
Reset_Handler:
  ldr   r0, =_estack
  mov   sp, r0          /* set stack pointer */

/* Copy the data segment initializers from flash to SRAM */
  ldr r0, =_sdata
  ldr r1, =_edata
  ldr r2, =_sidata
  movs r3, #0
  b LoopCopyDataInit

CopyDataInit:
  ldr r4, [r2, r3]
  str r4, [r0, r3]
  adds r3, r3, #4

LoopCopyDataInit:
  adds r4, r0, r3
  cmp r4, r1
  bcc CopyDataInit
  
/* Zero fill the bss segment. */
  ldr r2, =_sbss
  ldr r4, =_ebss
  movs r3, #0
  b LoopFillZerobss

FillZerobss:
  str  r3, [r2]
  adds r2, r2, #4

LoopFillZerobss:
  cmp r2, r4
  bcc FillZerobss

/* Call the clock system intitialization function.*/
  bl  SystemInit
/* Call static constructors */
  bl __libc_init_array
/* Call the application's entry point.*/
  bl main

LoopForever:
    b LoopForever


.size Reset_Handler, .-Reset_Handler

/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 *
 * @param  None
 * @retval : None
*/
    .section .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b Infinite_Loop
  .size Default_Handler, .-Default_Handler
/******************************************************************************
*
* The STM32L443CCTx vector table.  Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*
******************************************************************************/
   .section .isr_vector,"a",%progbits
  .type g_pfnVectors, %object
  .size g_pfnVectors, .-g_pfnVectors


g_pfnVectors:
  .word _estack
  .word Reset_Handler
  .word NMI_Handler
  .word HardFault_Handler
  .word	MemManage_Handler
  .word	BusFault_Handler
  .word	UsageFault_Handler
  .word	0
  .word	0
  .word	0
  .word	0
  .word	SVC_Handler
  .word	DebugMon_Handler
  .word	0
  .word	PendSV_Handler
  .word	SysTick_Handler
  .word	WWDG_IRQHandler         			/* Window Watchdog interrupt                                           */
  .word	PVD_IRQHandler          			/* PVD through EXTI line detection                                     */
  .word	TAMP_STAMP_IRQHandler   			/* Tamper and TimeStamp interrupts                                     */
  .word	RTC_WKUP_IRQHandler     			/* RTC Tamper or TimeStamp /CSS on LSE through EXTI line 19 interrupts */
  .word	FLASH_IRQHandler        			/* Flash global interrupt                                              */
  .word	RCC_IRQHandler          			/* RCC global interrupt                                                */
  .word	EXTI0_IRQHandler        			/* EXTI Line 0 interrupt                                               */
  .word	EXTI1_IRQHandler        			/* EXTI Line 1 interrupt                                               */
  .word	EXTI2_IRQHandler        			/* EXTI Line 2 interrupt                                               */
  .word	EXTI3_IRQHandler        			/* EXTI Line 3 interrupt                                               */
  .word	EXTI4_IRQHandler        			/* EXTI Line4 interrupt                                                */
  .word	DMA1_Channel1_IRQHandler			/* DMA1 Channel1 global interrupt                                      */
  .word	DMA1_Channel2_IRQHandler			/* DMA1 Channel2 global interrupt                                      */
  .word	DMA1_Channel3_IRQHandler			/* DMA1 Channel3 interrupt                                             */
  .word	DMA1_Channel4_IRQHandler			/* DMA1 Channel4 interrupt                                             */
  .word	DMA1_Channel5_IRQHandler			/* DMA1 Channel5 interrupt                                             */
  .word	DMA1_Channel6_IRQHandler			/* DMA1 Channel6 interrupt                                             */
  .word	DMA1_Channel7_IRQHandler			/* DMA1 Channel 7 interrupt                                            */
  .word	ADC1_IRQHandler         			/* ADC1 and ADC2 global interrupt                                      */
  .word	CAN1_TX_IRQHandler      			/* CAN1 TX interrupts                                                  */
  .word	CAN1_RX0_IRQHandler     			/* CAN1 RX0 interrupts                                                 */
  .word	CAN1_RX1_IRQHandler     			/* CAN1 RX1 interrupts                                                 */
  .word	CAN1_SCE_IRQHandler     			/* CAN1 SCE interrupt                                                  */
  .word	EXTI9_5_IRQHandler      			/* EXTI Line5 to Line9 interrupts                                      */
  .word	TIM15_IRQHandler        			/* Timer 15 global interrupt                                           */
  .word	TIM16_IRQHandler        			/* Timer 16 global interrupt                                           */
  .word	0                       			/* Reserved                                                            */
  .word	TIM1_CC_IRQHandler      			/* TIM1 Capture Compare interrupt                                      */
  .word	TIM2_IRQHandler         			/* TIM2 global interrupt                                               */
  .word	0                       			/* Reserved                                                            */
  .word	0                       			/* Reserved                                                            */
  .word	I2C1_EV_IRQHandler      			/* I2C1 event interrupt                                                */
  .word	I2C1_ER_IRQHandler      			/* I2C1 error interrupt                                                */
  .word	I2C2_EV_IRQHandler      			/* I2C2 event interrupt                                                */
  .word	I2C2_ER_IRQHandler      			/* I2C2 error interrupt                                                */
  .word	SPI1_IRQHandler         			/* SPI1 global interrupt                                               */
  .word	SPI2_IRQHandler         			/* SPI2 global interrupt                                               */
  .word	USART1_IRQHandler       			/* USART1 global interrupt                                             */
  .word	USART2_IRQHandler       			/* USART2 global interrupt                                             */
  .word	USART3_IRQHandler       			/* USART3 global interrupt                                             */
  .word	EXTI15_10_IRQHandler    			/* EXTI Lines 10 to 15 interrupts                                      */
  .word	RTC_ALARM_IRQHandler    			/* RTC alarms through EXTI line 18 interrupts                          */
  .word	0                       			/* Reserved                                                            */
  .word	0                       			/* Reserved                                                            */
  .word	0                       			/* Reserved                                                            */
  .word	0                       			/* Reserved                                                            */
  .word	0                       			/* Reserved                                                            */
  .word	ADC3_IRQHandler         			/* ADC3 global Interrupt                                               */
  .word	0                       			/* Reserved                                                            */
  .word	SDMMC_IRQHandler        			/* SDMMC global Interrupt                                              */
  .word	0                       			/* Reserved                                                            */
  .word	SPI3_IRQHandler         			/* SPI3 global Interrupt                                               */
  .word	0                       			/* Reserved                                                            */
  .word	0                       			/* Reserved                                                            */
  .word	TIM6_DAC_IRQHandler     			/* TIM6 global and DAC1 and 2 underrun error interrupts                */
  .word	TIM7_IRQHandler         			/* TIM7 global interrupt                                               */
  .word	DMA2_Channel1_IRQHandler			/* DMA2 Channel 1 global Interrupt                                     */
  .word	DMA2_Channel2_IRQHandler			/* DMA2 Channel 2 global Interrupt                                     */
  .word	DMA2_Channel3_IRQHandler			/* DMA2 Channel 3 global Interrupt                                     */
  .word	DMA2_Channel4_IRQHandler			/* DMA2 Channel 4 global Interrupt                                     */
  .word	DMA2_Channel5_IRQHandler			/* DMA2 Channel 5 global Interrupt                                     */
  .word	0                       			/* Reserved                                                            */
  .word	0                       			/* Reserved                                                            */
  .word	0                       			/* Reserved                                                            */
  .word	COMP_IRQHandler         			/* COMP1 and COMP2 interrupts                                          */
  .word	LPTIM1_IRQHandler       			/* LP TIM1 interrupt                                                   */
  .word	LPTIM2_IRQHandler       			/* LP TIM2 interrupt                                                   */
  .word	0                       			/* Reserved                                                            */
  .word	DMA2_Channel6_IRQHandler			/* DMA2 Channel 6 global Interrupt                                     */
  .word	DMA2_Channel7_IRQHandler			/* DMA2 Channel 7 global Interrupt                                     */
  .word	0                       			/* Reserved                                                            */
  .word	0                       			/* Reserved                                                            */
  .word	I2C3_EV_IRQHandler      			/* I2C3 event interrupt                                                */
  .word	I2C3_ER_IRQHandler      			/* I2C3 error interrupt                                                */
  .word	SAI1_IRQHandler         			/* SAI1 global interrupt                                               */
  .word	0                       			/* Reserved                                                            */
  .word	SWPMI1_IRQHandler       			/* SWPMI1 global interrupt                                             */
  .word	TSC_IRQHandler          			/* TSC global interrupt                                                */
  .word	LCD_IRQHandler          			/* LCD global interrupt                                                */
  .word	RNG_IRQHandler          			/* RNG global interrupt                                                */
  .word	CRS_IRQHandler          			/* CRS global interrupt                                                */
  .word	FPU_IRQHandler          			/* Floating point interrupt                                            */

/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/

  	.weak	NMI_Handler
	.thumb_set NMI_Handler,Default_Handler

  	.weak	HardFault_Handler
	.thumb_set HardFault_Handler,Default_Handler

  	.weak	MemManage_Handler
	.thumb_set MemManage_Handler,Default_Handler

  	.weak	BusFault_Handler
	.thumb_set BusFault_Handler,Default_Handler

	.weak	UsageFault_Handler
	.thumb_set UsageFault_Handler,Default_Handler

	.weak	SVC_Handler
	.thumb_set SVC_Handler,Default_Handler

	.weak	DebugMon_Handler
	.thumb_set DebugMon_Handler,Default_Handler
	
	.weak	PendSV_Handler
	.thumb_set PendSV_Handler,Default_Handler

	.weak	SysTick_Handler
	.thumb_set SysTick_Handler,Default_Handler

	.weak	WWDG_IRQHandler
	.thumb_set WWDG_IRQHandler,Default_Handler
	
	.weak	PVD_IRQHandler
	.thumb_set PVD_IRQHandler,Default_Handler
	
	.weak	TAMP_STAMP_IRQHandler
	.thumb_set TAMP_STAMP_IRQHandler,Default_Handler
	
	.weak	RTC_WKUP_IRQHandler
	.thumb_set RTC_WKUP_IRQHandler,Default_Handler
	
	.weak	FLASH_IRQHandler
	.thumb_set FLASH_IRQHandler,Default_Handler
	
	.weak	RCC_IRQHandler
	.thumb_set RCC_IRQHandler,Default_Handler
	
	.weak	EXTI0_IRQHandler
	.thumb_set EXTI0_IRQHandler,Default_Handler
	
	.weak	EXTI1_IRQHandler
	.thumb_set EXTI1_IRQHandler,Default_Handler
	
	.weak	EXTI2_IRQHandler
	.thumb_set EXTI2_IRQHandler,Default_Handler
	
	.weak	EXTI3_IRQHandler
	.thumb_set EXTI3_IRQHandler,Default_Handler
	
	.weak	EXTI4_IRQHandler
	.thumb_set EXTI4_IRQHandler,Default_Handler
	
	.weak	DMA1_Channel1_IRQHandler
	.thumb_set DMA1_Channel1_IRQHandler,Default_Handler
	
	.weak	DMA1_Channel2_IRQHandler
	.thumb_set DMA1_Channel2_IRQHandler,Default_Handler
	
	.weak	DMA1_Channel3_IRQHandler
	.thumb_set DMA1_Channel3_IRQHandler,Default_Handler
	
	.weak	DMA1_Channel4_IRQHandler
	.thumb_set DMA1_Channel4_IRQHandler,Default_Handler
	
	.weak	DMA1_Channel5_IRQHandler
	.thumb_set DMA1_Channel5_IRQHandler,Default_Handler
	
	.weak	DMA1_Channel6_IRQHandler
	.thumb_set DMA1_Channel6_IRQHandler,Default_Handler
	
	.weak	DMA1_Channel7_IRQHandler
	.thumb_set DMA1_Channel7_IRQHandler,Default_Handler
	
	.weak	ADC1_IRQHandler
	.thumb_set ADC1_IRQHandler,Default_Handler
	
	.weak	CAN1_TX_IRQHandler
	.thumb_set CAN1_TX_IRQHandler,Default_Handler
	
	.weak	CAN1_RX0_IRQHandler
	.thumb_set CAN1_RX0_IRQHandler,Default_Handler
	
	.weak	CAN1_RX1_IRQHandler
	.thumb_set CAN1_RX1_IRQHandler,Default_Handler
	
	.weak	CAN1_SCE_IRQHandler
	.thumb_set CAN1_SCE_IRQHandler,Default_Handler
	
	.weak	EXTI9_5_IRQHandler
	.thumb_set EXTI9_5_IRQHandler,Default_Handler
	
	.weak	TIM15_IRQHandler
	.thumb_set TIM15_IRQHandler,Default_Handler
	
	.weak	TIM16_IRQHandler
	.thumb_set TIM16_IRQHandler,Default_Handler
	
	.weak	TIM1_CC_IRQHandler
	.thumb_set TIM1_CC_IRQHandler,Default_Handler
	
	.weak	TIM2_IRQHandler
	.thumb_set TIM2_IRQHandler,Default_Handler
	
	.weak	I2C1_EV_IRQHandler
	.thumb_set I2C1_EV_IRQHandler,Default_Handler
	
	.weak	I2C1_ER_IRQHandler
	.thumb_set I2C1_ER_IRQHandler,Default_Handler
	
	.weak	I2C2_EV_IRQHandler
	.thumb_set I2C2_EV_IRQHandler,Default_Handler
	
	.weak	I2C2_ER_IRQHandler
	.thumb_set I2C2_ER_IRQHandler,Default_Handler
	
	.weak	SPI1_IRQHandler
	.thumb_set SPI1_IRQHandler,Default_Handler
	
	.weak	SPI2_IRQHandler
	.thumb_set SPI2_IRQHandler,Default_Handler
	
	.weak	USART1_IRQHandler
	.thumb_set USART1_IRQHandler,Default_Handler
	
	.weak	USART2_IRQHandler
	.thumb_set USART2_IRQHandler,Default_Handler
	
	.weak	USART3_IRQHandler
	.thumb_set USART3_IRQHandler,Default_Handler
	
	.weak	EXTI15_10_IRQHandler
	.thumb_set EXTI15_10_IRQHandler,Default_Handler
	
	.weak	RTC_ALARM_IRQHandler
	.thumb_set RTC_ALARM_IRQHandler,Default_Handler
	
	.weak	ADC3_IRQHandler
	.thumb_set ADC3_IRQHandler,Default_Handler
	
	.weak	SDMMC_IRQHandler
	.thumb_set SDMMC_IRQHandler,Default_Handler
	
	.weak	SPI3_IRQHandler
	.thumb_set SPI3_IRQHandler,Default_Handler
	
	.weak	TIM6_DAC_IRQHandler
	.thumb_set TIM6_DAC_IRQHandler,Default_Handler
	
	.weak	TIM7_IRQHandler
	.thumb_set TIM7_IRQHandler,Default_Handler
	
	.weak	DMA2_Channel1_IRQHandler
	.thumb_set DMA2_Channel1_IRQHandler,Default_Handler
	
	.weak	DMA2_Channel2_IRQHandler
	.thumb_set DMA2_Channel2_IRQHandler,Default_Handler
	
	.weak	DMA2_Channel3_IRQHandler
	.thumb_set DMA2_Channel3_IRQHandler,Default_Handler
	
	.weak	DMA2_Channel4_IRQHandler
	.thumb_set DMA2_Channel4_IRQHandler,Default_Handler
	
	.weak	DMA2_Channel5_IRQHandler
	.thumb_set DMA2_Channel5_IRQHandler,Default_Handler
	
	.weak	COMP_IRQHandler
	.thumb_set COMP_IRQHandler,Default_Handler
	
	.weak	LPTIM1_IRQHandler
	.thumb_set LPTIM1_IRQHandler,Default_Handler
	
	.weak	LPTIM2_IRQHandler
	.thumb_set LPTIM2_IRQHandler,Default_Handler
	
	.weak	DMA2_Channel6_IRQHandler
	.thumb_set DMA2_Channel6_IRQHandler,Default_Handler
	
	.weak	DMA2_Channel7_IRQHandler
	.thumb_set DMA2_Channel7_IRQHandler,Default_Handler
	
	.weak	I2C3_EV_IRQHandler
	.thumb_set I2C3_EV_IRQHandler,Default_Handler
	
	.weak	I2C3_ER_IRQHandler
	.thumb_set I2C3_ER_IRQHandler,Default_Handler
	
	.weak	SAI1_IRQHandler
	.thumb_set SAI1_IRQHandler,Default_Handler
	
	.weak	SWPMI1_IRQHandler
	.thumb_set SWPMI1_IRQHandler,Default_Handler
	
	.weak	TSC_IRQHandler
	.thumb_set TSC_IRQHandler,Default_Handler
	
	.weak	LCD_IRQHandler
	.thumb_set LCD_IRQHandler,Default_Handler
	
	.weak	RNG_IRQHandler
	.thumb_set RNG_IRQHandler,Default_Handler
	
	.weak	CRS_IRQHandler
	.thumb_set CRS_IRQHandler,Default_Handler
	
	.weak	FPU_IRQHandler
	.thumb_set FPU_IRQHandler,Default_Handler
	
	.weak	SystemInit

/************************ (C) COPYRIGHT Ac6 *****END OF FILE****/

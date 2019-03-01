/*****< halcfg.h >*************************************************************/
/*      Copyright 2012 - 2014 Stonestreet One.                                */
/*      All Rights Reserved.                                                  */
/*                                                                            */
/*  HALCFG - Hardware Abstraction Layer Configuration parameters.             */
/*                                                                            */
/*  Author:  Marcus Funk                                                      */
/*                                                                            */
/*** MODIFICATION HISTORY *****************************************************/
/*                                                                            */
/*   mm/dd/yy  F. Lastname    Description of Modification                     */
/*   --------  -----------    ------------------------------------------------*/
/*   11/08/12  M. Funk        Initial creation.                               */
/******************************************************************************/
#ifndef __HCITRCFGH__
#define __HCITRCFGH__

#include "BTAPITyp.h"            /* Bluetooth API Type Definitions.           */

#include "stm32f4xx.h"           /* STM32F register definitions.              */
#include "stm32f4xx_gpio.h"      /* STM32F GPIO control functions.            */
#include "stm32f4xx_usart.h"     /* STM32F USART control functions.           */
#include "stm32f4xx_rcc.h"       /* STM32F Clock control functions.           */
#include "stm32f4xx_exti.h"      /* STM32F Ext interrupt definitions.         */
#include "stm32f4xx_syscfg.h"    /* STM32F system config definitions.         */

   /* The following definitions define the UART/USART to be used by the */
   /* HCI transport and the pins that will be used by the UART.  Please */
   /* consult the processor's documentation to determine what pins are  */
   /* available for the desired UART.                                   */
   /* * NOTE * The TXD and RXD pins MUST be map-able to the selected    */
   /*          UART.  Additionally, if hardware flow control is desired,*/
   /*          the RTS and CTS pins must also be map-able to the        */
   /*          selected UART.  If software managed flow is used, RTS may*/
   /*          be any available GPIO but CTS must be a GPIO that can be */
   /*          mapped to an available EXTI line.  The RESET pin may be  */
   /*          any available GPIO.                                      */
#define CONSOLE_UART      2

#define CONSOLE_TXD_PORT  A
#define CONSOLE_TXD_PIN   2

#define CONSOLE_RXD_PORT  A
#define CONSOLE_RXD_PIN   3

#define HAL_LED_PORT      D
#define HAL_LED_PIN       12

 /* Define the baud rate that will be used for communication with the */
   /* Bluetooth chip.  The value of this rate will be configured in the */
   /* Bluetooth transport.                                              */
#define VENDOR_BAUD_RATE                                    921600L

/************************************************************************/
/* !!!DO NOT MODIFY PAST THIS POINT!!!                                  */
/************************************************************************/

   /* The following section builds the macros that can be used with the */
   /* STM32F standard peripheral libraries based on the above           */
   /* configuration.                                                    */

/* Standard C style concatenation macros                             */
#define DEF_CONCAT2(_x_, _y_)          __DEF_CONCAT2__(_x_, _y_)
#define __DEF_CONCAT2__(_x_, _y_)      _x_ ## _y_

#define DEF_CONCAT3(_x_, _y_, _z_)     __DEF_CONCAT3__(_x_, _y_, _z_)
#define __DEF_CONCAT3__(_x_, _y_, _z_) _x_ ## _y_ ## _z_


   /* Determine the Peripheral bus that is used by the UART.            */
#if ((CONSOLE_UART == 1) ||(CONSOLE_UART == 6))
   #define CONSOLE_UART_APB              2
#else
   #define CONSOLE_UART_APB              1
#endif

   /* Determine the type of UART.                                       */
#if ((CONSOLE_UART == 1) || (CONSOLE_UART == 2) || (CONSOLE_UART == 3) || (CONSOLE_UART == 6))

   #define CONSOLE_UART_TYPE             USART

#elif ((HCITR_UART == 4) || (HCITR_UART == 5))

   #define CONSOLE_UART_TYPE             UART

#else

   #error Unknown CONSOLE_UART

#endif

   /* The following section builds the macro names that can be used with*/
   /* the STM32F standard peripheral libraries.                         */

   /* UART control mapping.                                             */
#define CONSOLE_UART_BASE                (DEF_CONCAT2(CONSOLE_UART_TYPE, CONSOLE_UART))
#define CONSOLE_UART_IRQ                 (DEF_CONCAT3(CONSOLE_UART_TYPE, CONSOLE_UART, _IRQn))
#define CONSOLE_UART_IRQ_HANDLER         (DEF_CONCAT3(CONSOLE_UART_TYPE, CONSOLE_UART, _IRQHandler))

#define CONSOLE_UART_RCC_PERIPH_CLK_CMD  (DEF_CONCAT3(RCC_APB, CONSOLE_UART_APB, PeriphClockCmd))
#define CONSOLE_UART_RCC_PERIPH_CLK_BIT  (DEF_CONCAT3(DEF_CONCAT3(RCC_APB, CONSOLE_UART_APB, Periph_), CONSOLE_UART_TYPE, CONSOLE_UART))
#define CONSOLE_UART_GPIO_AF             (DEF_CONCAT3(GPIO_AF_, CONSOLE_UART_TYPE, CONSOLE_UART))

   /* GPIO mapping.                                                     */
#define CONSOLE_TXD_GPIO_PORT            (DEF_CONCAT2(GPIO, CONSOLE_TXD_PORT))
#define CONSOLE_RXD_GPIO_PORT            (DEF_CONCAT2(GPIO, CONSOLE_RXD_PORT))
#define HAL_LED_GPIO_PORT                (DEF_CONCAT2(GPIO, HAL_LED_PORT))

#define CONSOLE_TXD_GPIO_AHB_BIT         (DEF_CONCAT2(RCC_AHB1Periph_GPIO, CONSOLE_TXD_PORT))
#define CONSOLE_RXD_GPIO_AHB_BIT         (DEF_CONCAT2(RCC_AHB1Periph_GPIO, CONSOLE_RXD_PORT))
#define HAL_LED_GPIO_AHB_BIT             (DEF_CONCAT2(RCC_AHB1Periph_GPIO, HAL_LED_PORT))

#endif


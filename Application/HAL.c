/*****< HAL.c >***************************************************************/
/*      Copyright 2012 - 2014 Stonestreet One.                               */
/*      All Rights Reserved.                                                 */
/*                                                                           */
/*      Copyright 2015 Texas Instruments Incorporated.                       */
/*      All Rights Reserved.                                                 */
/*                                                                           */
/*  HAL - Hardware Abstraction function for ST STM3240G-EVAL Board           */
/*                                                                           */
/*  Author:  Marcus Funk                                                     */
/*                                                                           */
/*** MODIFICATION HISTORY ****************************************************/
/*                                                                           */
/*   mm/dd/yy  F. Lastname    Description of Modification                    */
/*   --------  -----------    -----------------------------------------------*/
/*   07/05/12  M. Funk        Initial creation.                              */
/*   11/24/14  R. Malovany    Update.                                        */
/*****************************************************************************/

/* Library includes. */
#include "HAL.h"                 /* Function for Hardware Abstraction.       */
#include "HALCFG.h"
#include "BTPSKRNL.h"            /* BTPS Kernel Header.                      */

   /* The following defines the Buffer sizes that will be used for the  */
   /* console UART.                                                     */
   /* * NOTE * As the HAL_ConsoleWrite operation does not block, the    */
   /*          output buffer MUST be large enough to store the longest  */
   /*          printed message.                                         */
#define HAL_OUTPUT_BUFFER_SIZE            3000
#define HAL_INPUT_BUFFER_SIZE             64

#define EnableConsoleUartPeriphClock()    CONSOLE_UART_RCC_PERIPH_CLK_CMD(CONSOLE_UART_RCC_PERIPH_CLK_BIT, ENABLE)
#define DisableConsoleUartPeriphClock()   CONSOLE_UART_RCC_PERIPH_CLK_CMD(CONSOLE_UART_RCC_PERIPH_CLK_BIT, DISABLE)

#define DisableInterrupts()               __set_PRIMASK(1)
#define EnableInterrupts()                __set_PRIMASK(0)

/* The following structure contains the buffers for the Console UART.   */
typedef struct _tagHAL_UartContext_t
{
   USART_TypeDef *Base;

   unsigned char  RxBuffer[HAL_INPUT_BUFFER_SIZE];
   unsigned int          RxBufferSize;
   volatile unsigned int RxBytesFree;
   unsigned int          RxInIndex;
   unsigned int          RxOutIndex;

   unsigned char  TxBuffer[HAL_OUTPUT_BUFFER_SIZE];
   unsigned int          TxBufferSize;
   volatile unsigned int TxBytesFree;
   unsigned int          TxInIndex;
   unsigned int          TxOutIndex;
} HAL_UartContext_t;

   /* Default UART config.                                              */
static USART_InitTypeDef ConsoleUartConfig = {115200, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No, USART_Mode_Rx | USART_Mode_Tx, USART_HardwareFlowControl_None};

static BTPSCONST GPIO_InitTypeDef HAL_TXDGpioConfiguration  =  {(1 << CONSOLE_TXD_PIN), GPIO_Mode_AF,  GPIO_Speed_25MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL};
static BTPSCONST GPIO_InitTypeDef HAL_RXDGpioConfiguration  =  {(1 << CONSOLE_RXD_PIN), GPIO_Mode_AF,  GPIO_Speed_25MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL};
static BTPSCONST GPIO_InitTypeDef HAL_LEDGpioConfiguration  =  {(1 << HAL_LED_PIN),     GPIO_Mode_OUT, GPIO_Speed_25MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL};

HAL_UartContext_t        HAL_UartContext;

static void HAL_RxInterrupt(void);
static void HAL_TxInterrupt(void);

   /* The following function is the Interrupt Service Routine for the   */
   /* UART RX interrupt.  The function is passed the Context of the UART*/
   /* that is to be serviced.                                           */
static void HAL_RxInterrupt(void)
{
   /* read the first byte from the port.                                */
   while((HAL_UartContext.RxBytesFree) && (USART_GetFlagStatus(HAL_UartContext.Base, USART_FLAG_RXNE) == SET))
   {
      /* Read a character from the port into the receive buffer         */
      HAL_UartContext.RxBuffer[HAL_UartContext.RxInIndex] = (unsigned char)USART_ReceiveData(HAL_UartContext.Base);

      HAL_UartContext.RxBytesFree --;
      HAL_UartContext.RxInIndex ++;

      /* See if we need to roll the RxInIndex back to 0.                */
      if(HAL_UartContext.RxInIndex == HAL_UartContext.RxBufferSize)
         HAL_UartContext.RxInIndex = 0;
   }

   /* if the buffer is full, read in the remaining data from the port.  */
   if(!HAL_UartContext.RxBytesFree)
   {
      while(USART_GetFlagStatus(HAL_UartContext.Base, USART_FLAG_RXNE) == SET)
         USART_ReceiveData(HAL_UartContext.Base);
   }
}

   /* The following function is the FIFO Primer and Interrupt Service   */
   /* Routine for the UART TX interrupt.  The function is passed the    */
   /* Context of the UART that is to be serviced.                       */
static void HAL_TxInterrupt(void)
{
   /* The interrupt was caused by the THR becoming empty.  Are there any*/
   /* more characters to transmit?                                      */
   while((HAL_UartContext.TxBytesFree != HAL_UartContext.TxBufferSize) && (HAL_UartContext.Base->SR & USART_FLAG_TXE))
   {
      /* Place the next character into the output buffer.               */
      USART_SendData(HAL_UartContext.Base, HAL_UartContext.TxBuffer[HAL_UartContext.TxOutIndex]);

      /* Adjust the character counts and check to see if the index needs*/
      /* to be wrapped.                                                 */
      HAL_UartContext.TxBytesFree ++;
      HAL_UartContext.TxOutIndex ++;
      if(HAL_UartContext.TxOutIndex == HAL_UartContext.TxBufferSize)
         HAL_UartContext.TxOutIndex = 0;
   }

   if(HAL_UartContext.TxBytesFree == HAL_UartContext.TxBufferSize)
   {
      /* No more data to send, inhibit transmit interrupts.             */
      USART_ITConfig(HAL_UartContext.Base, USART_IT_TXE, DISABLE);
   }
}

   /* The following function handles the UART interrupts for the        */
   /* console.                                                          */
void CONSOLE_UART_IRQ_HANDLER(void)
{
   unsigned int Flags;
   unsigned int Control;

   Flags   = HAL_UartContext.Base->SR;
   Control = HAL_UartContext.Base->CR1;
   /* Check to see if data is available in the Receive Buffer.          */
   if((Flags & (USART_FLAG_RXNE | USART_FLAG_ORE)) && (Control & (1 << (USART_IT_RXNE & 0x1F))))
      HAL_RxInterrupt();
   else
   {
      /* Check to see if the transmit buffers are ready for more data.  */
      if((Flags & USART_FLAG_TXE) && (Control & (1 << (USART_IT_TXE & 0x1F))))
         HAL_TxInterrupt();
   }
}


/* The following function configures the hardware as required for the   */
/* sample applications.                                                 */
void HAL_ConfigureHardware(void)
{
   BTPS_MemInitialize(&HAL_UartContext, 0, sizeof(HAL_UartContext_t));

   HAL_UartContext.Base         = CONSOLE_UART_BASE;
   HAL_UartContext.RxBufferSize = HAL_INPUT_BUFFER_SIZE;
   HAL_UartContext.RxBytesFree  = HAL_INPUT_BUFFER_SIZE;
   HAL_UartContext.TxBufferSize = HAL_OUTPUT_BUFFER_SIZE;
   HAL_UartContext.TxBytesFree  = HAL_OUTPUT_BUFFER_SIZE;

   NVIC_SetPriorityGrouping(3);

   /* Enable the peripheral clocks for the UART.                        */
   EnableConsoleUartPeriphClock();

   /* Configure used GPIO                                               */
   RCC_AHB1PeriphClockCmd(CONSOLE_TXD_GPIO_AHB_BIT, ENABLE);
   GPIO_Init(CONSOLE_TXD_GPIO_PORT, (GPIO_InitTypeDef *)&HAL_TXDGpioConfiguration);
   GPIO_PinAFConfig(CONSOLE_TXD_GPIO_PORT, CONSOLE_TXD_PIN, CONSOLE_UART_GPIO_AF);

   RCC_AHB1PeriphClockCmd(CONSOLE_RXD_GPIO_AHB_BIT, ENABLE);
   GPIO_Init(CONSOLE_RXD_GPIO_PORT, (GPIO_InitTypeDef *)&HAL_RXDGpioConfiguration);
   GPIO_PinAFConfig(CONSOLE_RXD_GPIO_PORT, CONSOLE_RXD_PIN, CONSOLE_UART_GPIO_AF);

   RCC_AHB1PeriphClockCmd(HAL_LED_GPIO_AHB_BIT, ENABLE);
   GPIO_Init(HAL_LED_GPIO_PORT, (GPIO_InitTypeDef *)&HAL_LEDGpioConfiguration);

   /* Initialize the UART.                                              */
   USART_Init(HAL_UartContext.Base, &ConsoleUartConfig);
   USART_ITConfig(HAL_UartContext.Base, USART_IT_RXNE, ENABLE);
   USART_Cmd(HAL_UartContext.Base, ENABLE);

   NVIC_SetPriority(CONSOLE_UART_IRQ, 0xF);
   NVIC_EnableIRQ(CONSOLE_UART_IRQ);
}

   /* The following function is used to illuminate an LED.  The number  */
   /* of LEDs on a board is board specific.  If the LED_ID provided does*/
   /* not exist on the hardware platform then nothing is done.          */
void HAL_LedOn(int LED_ID)
{
   if(LED_ID == 0)
      GPIO_SetBits(HAL_LED_GPIO_PORT, (1 << HAL_LED_PIN));
}

   /* The following function is used to extinguish an LED.  The number  */
   /* of LEDs on a board is board specific.  If the LED_ID provided does*/
   /* not exist on the hardware platform then nothing is done.          */
void HAL_LedOff(int LED_ID)
{
   if(LED_ID == 0)
      GPIO_ResetBits(HAL_LED_GPIO_PORT, (1 << HAL_LED_PIN));
}

   /* The following function is used to toggle the state of an LED.  The*/
   /* number of LEDs on a board is board specific.  If the LED_ID       */
   /* provided does not exist on the hardware platform then nothing is  */
   /* done.                                                             */
void HAL_LedToggle(int LED_ID)
{
   if(LED_ID == 0)
   {
      if(GPIO_ReadOutputDataBit(HAL_LED_GPIO_PORT, (1 << HAL_LED_PIN)) == Bit_SET)
         HAL_LedOff(LED_ID);
      else
         HAL_LedOn(LED_ID);
   }
}

   /* The following function is used to retrieve data from the UART     */
   /* input queue.  the function receives a pointer to a buffer that    */
   /* will receive the UART characters a the length of the buffer.  The */
   /* function will return the number of characters that were returned  */
   /* in Buffer.                                                        */
int HAL_ConsoleRead(int Length, char *Buffer)
{
   int ret_val;

   if((Length) && (Buffer))
   {
      /* Set the size to be copied equal to the smaller of the length   */
      /* and the bytes in the receive buffer.                           */
      ret_val = HAL_UartContext.RxBufferSize - HAL_UartContext.RxBytesFree;
      ret_val = (ret_val < Length) ? ret_val : Length;

      if(ret_val > (HAL_UartContext.RxBufferSize - HAL_UartContext.RxOutIndex))
      {
         /* The data wraps around the end of the buffer, so copy it in  */
         /* two steps.                                                  */
         Length = (HAL_UartContext.RxBufferSize - HAL_UartContext.RxOutIndex);
         BTPS_MemCopy(Buffer, &HAL_UartContext.RxBuffer[HAL_UartContext.RxOutIndex], Length);
         BTPS_MemCopy((Buffer + Length), HAL_UartContext.RxBuffer, (ret_val - Length));

         HAL_UartContext.RxOutIndex = ret_val - Length;
      }
      else
      {
         BTPS_MemCopy(Buffer, &HAL_UartContext.RxBuffer[HAL_UartContext.RxOutIndex], ret_val);

         HAL_UartContext.RxOutIndex += ret_val;

         if(HAL_UartContext.RxOutIndex == HAL_UartContext.RxBufferSize)
            HAL_UartContext.RxOutIndex = 0;
      }

      HAL_UartContext.RxBytesFree += ret_val;
   }
   else
      ret_val = 0;

   return(ret_val);
}


   /* The following function is used to send data to the UART output    */
   /* queue.  the function receives a pointer to a buffer that will     */
   /* contains the data to send and the length of the data.  The        */
   /* function will return the number of characters that were           */
   /* successfully saved in the output buffer.                          */
int HAL_ConsoleWrite(int Length, char *Buffer)
{
   int ret_val;
   int Count;
   int BytesFree;

   if((Length) && (Buffer))
   {
      ret_val = 0;

      while(Length)
      {
         /* Wait for space to be availale in the buffer.                */
         while(!HAL_UartContext.TxBytesFree)
            BTPS_Delay(1);

         /* The data may have to be copied in 2 phases.  Calculate the  */
         /* number of character that can be placed in the buffer before */
         /* the buffer must be wrapped.                                 */
         BytesFree = HAL_UartContext.TxBytesFree;
         Count = Length;
         Count = (BytesFree < Count) ? BytesFree : Count;
         Count = ((HAL_UartContext.TxBufferSize - HAL_UartContext.TxInIndex) < Count) ? (HAL_UartContext.TxBufferSize - HAL_UartContext.TxInIndex) : Count;

         BTPS_MemCopy(&(HAL_UartContext.TxBuffer[HAL_UartContext.TxInIndex]), Buffer, Count);

         /* Adjust the counts and index.                                */
         Buffer                      += Count;
         Length                      -= Count;
         ret_val                     += Count;
         HAL_UartContext.TxInIndex   += Count;
         if(HAL_UartContext.TxInIndex == HAL_UartContext.TxBufferSize)
            HAL_UartContext.TxInIndex = 0;

         DisableInterrupts();
         HAL_UartContext.TxBytesFree -= Count;
         USART_ITConfig(HAL_UartContext.Base, USART_IT_TXE, ENABLE);
         EnableInterrupts();
      }
   }
   else
      ret_val = 0;

   return(ret_val);
}

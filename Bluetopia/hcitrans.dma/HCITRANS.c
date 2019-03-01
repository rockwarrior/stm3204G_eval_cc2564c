/*****< hcitrans.c >***********************************************************/
/*      Copyright 2012 - 2013 Stonestreet One.                                */
/*      All Rights Reserved.                                                  */
/*                                                                            */
/*  HCITRANS - HCI Transport Layer for use with Bluetopia.                    */
/*                                                                            */
/*  Author:  Marcus Funk                                                      */
/*                                                                            */
/*** MODIFICATION HISTORY *****************************************************/
/*                                                                            */
/*   mm/dd/yy  F. Lastname    Description of Modification                     */
/*   --------  -----------    ------------------------------------------------*/
/*   11/08/12  M. Funk        Initial creation.                               */
/******************************************************************************/

#include "BTPSKRNL.h"       /* Bluetooth Kernel Prototypes/Constants.         */
#include "HCITRANS.h"       /* HCI Transport Prototypes/Constants.            */
#include "HCITRCFG.h"       /* HCI Transport configuration.                   */

   /* The following define the size of the buffers used by HCITRANS.    */
#define INPUT_BUFFER_SIZE        1536
#define OUTPUT_BUFFER_SIZE       1056

   /* The following defines the DMA length used for reading data from   */
   /* the UART. Larger values will require less processor intervention  */
   /* but will also result in more wasted space in the buffer (as the   */
   /* buffer will wrap when the space at the end is less than           */
   /* INPUT_DMA_SIZE) and possibly slower reaction times (as the        */
   /* interrupt will not occur until half of INPUT_DMA_SIZE bytes have  */
   /* been received).                                                   */
#define INPUT_DMA_SIZE           128

   /* The following defines the maximum DMA length used for writing data*/
   /* to the UART. Larger values will require less processor            */
   /* intervention but may result in longer blocking times in           */
   /* HCITR_COMWrite as the data being written by a DMA transfer will   */
   /* not be credited back to the buffer until the DMA transaction      */
   /* is completed.                                                     */
#define OUTPUT_MAXIMUM_DMA_SIZE  32

#define ClearReset()             GPIO_SetBits(HCITR_RESET_GPIO_PORT, (1 << HCITR_RESET_PIN))
#define SetReset()               GPIO_ResetBits(HCITR_RESET_GPIO_PORT, (1 << HCITR_RESET_PIN))

#define EnableUartPeriphClock()  HCITR_UART_RCC_PERIPH_CLK_CMD(HCITR_UART_RCC_PERIPH_CLK_BIT, ENABLE)
#define DisableUartPeriphClock() HCITR_UART_RCC_PERIPH_CLK_CMD(HCITR_UART_RCC_PERIPH_CLK_BIT, DISABLE)

#define DisableInterrupts()      __set_PRIMASK(1)
#define EnableInterrupts()       __set_PRIMASK(0)

#define INTERRUPT_PRIORITY       5

#define TRANSPORT_ID             1

#define DEBUG_PRINT              BTPS_OutputMessage

typedef enum
{
   hssNormal,
   hssSuspendWait,
   hssSuspendWaitInterrupted,
   hssSuspended
} SuspendState_t;

typedef struct _tagUartContext_t
{
#ifdef HCITR_ENABLE_DEBUG_LOGGING

   Boolean_t                DebugEnabled;

#endif

   SuspendState_t           SuspendState;

   HCITR_COMDataCallback_t  COMDataCallbackFunction;
   unsigned long            COMDataCallbackParameter;

   unsigned short           RxInIndex;
   unsigned short           RxOutIndex;
   volatile unsigned short  RxBufferSize;
   volatile unsigned short  RxBytesUsed;
   volatile Boolean_t       RxDMAInProgress;
   unsigned char            RxBuffer[INPUT_BUFFER_SIZE];

   unsigned short           TxInIndex;
   unsigned short           TxOutIndex;
   volatile unsigned short  TxBytesFree;
   volatile unsigned short  TxPreviousDMALength;
   unsigned char            TxBuffer[OUTPUT_BUFFER_SIZE];
} UartContext_t;

   /* Internal Variables to this Module (Remember that all variables    */
   /* declared static are initialized to 0 automatically by the         */
   /* compiler as part of standard C/C++).                              */

   /* Default UART config.                                              */
   static BTPSCONST USART_InitTypeDef UartConfiguration   = {115200, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No, USART_Mode_Rx | USART_Mode_Tx, USART_HardwareFlowControl_RTS_CTS};

#ifdef SUPPORT_TRANSPORT_SUSPEND

   /* CTS external interrupt handler.                                   */
static BTPSCONST EXTI_InitTypeDef CTS_ExtIntConfiguration = {HCITR_CTS_IRQ_LINE, EXTI_Mode_Interrupt, EXTI_Trigger_Falling, ENABLE};

#endif

static UartContext_t              UartContext;
static int                        HCITransportOpen        = 0;

   /* Define the configuration arrays for the DMAs.      */
static BTPSCONST DMA_InitTypeDef  RXDDMAConfiguration =
{
   HCITR_RXD_DMA_CHANNEL,
   HCITR_UART_DR_REGISTER_ADDRESS,
   0,
   DMA_DIR_PeripheralToMemory,
   INPUT_DMA_SIZE,
   DMA_PeripheralInc_Disable,
   DMA_MemoryInc_Enable,
   DMA_PeripheralDataSize_Byte,
   DMA_MemoryDataSize_Byte,
   DMA_Mode_Normal,
   DMA_Priority_VeryHigh,
   DMA_FIFOMode_Disable,
   DMA_FIFOThreshold_1QuarterFull,
   DMA_MemoryBurst_Single,
   DMA_PeripheralBurst_Single
};

static BTPSCONST DMA_InitTypeDef  TXDDMAConfiguration =
{
   HCITR_TXD_DMA_CHANNEL,
   HCITR_UART_DR_REGISTER_ADDRESS,
   0,
   DMA_DIR_MemoryToPeripheral,
   0,
   DMA_PeripheralInc_Disable,
   DMA_MemoryInc_Enable,
   DMA_PeripheralDataSize_Byte,
   DMA_MemoryDataSize_Byte,
   DMA_Mode_Normal,
   DMA_Priority_VeryHigh,
   DMA_FIFOMode_Disable,
   DMA_FIFOThreshold_Full,
   DMA_MemoryBurst_Single,
   DMA_PeripheralBurst_Single
};

   /* Local Function Prototypes.                                        */
static void SetBaudRate(USART_TypeDef *UartBase, unsigned int BaudRate);
static void ConfigureGPIO(GPIO_TypeDef *Port, unsigned int Pin, GPIOMode_TypeDef Mode);
static void SetSuspendGPIO(Boolean_t Suspend);
static void StartTxDMATransfer(void);
static void StartRxDMATransfer(void);
static Boolean_t ProcessRxDMA(void);

   /* The following function will reconfigure the BAUD rate without     */
   /* reconfiguring the entire port.  This function is also potentially */
   /* more accurate than the method used in the ST standard peripheral  */
   /* libraries.                                                        */
static void SetBaudRate(USART_TypeDef *UartBase, unsigned int BaudRate)
{
   RCC_ClocksTypeDef RCC_ClocksStatus;
   unsigned int      SourceFrequency;
   unsigned int      Divider;
   unsigned int      TempDiv;

   RCC_GetClocksFreq(&RCC_ClocksStatus);

#if((HCITR_UART == 1) || (HCITR_UART == 6))

   SourceFrequency = RCC_ClocksStatus.PCLK2_Frequency;

#else

   SourceFrequency = RCC_ClocksStatus.PCLK1_Frequency;

#endif

   /* The following calculation will yield the integer divider          */
   /* concatenated with the fractional divider.  If 16-bit oversampling */
   /* is used, the least significant 4 bits will be the fraction divider*/
   /* and if 8-bit oversampling is used, the least significant 3 bits   */
   /* will be the fraction divider.                                     */
   Divider = (((SourceFrequency * 2) / BaudRate) + 1) / 2;

   /* The integer divider must be between 1 and 4095 so if the divider  */
   /* value is less than 16, 8-bit oversampling must be used.           */
   if(Divider < 16)
   {
      /* When 8-bit oversampling is used, bits [2..0] of the BRR        */
      /* register represent the fractional divider and bits [15..4]     */
      /* represent the integer divider.  Because of this, the integer   */
      /* divider needs to be shifted left one bit before the baud rate  */
      /* control register is updated.                                   */
      TempDiv = Divider & 0x7;
      Divider = (Divider & ~0x07) << 1;
      Divider |= TempDiv;

      /* Disable the UART while updating the baud rate.                 */
      UartBase->CR1 &= ~USART_CR1_UE;
      UartBase->CR1 |= USART_CR1_OVER8;
   }
   else
   {
      /* For lower frequencies, 16bit oversampling may be used.         */

      /* Disable the UART while updating the baud rate.                 */
      UartBase->CR1 &= ~USART_CR1_UE;
      UartBase->CR1 &= ~USART_CR1_OVER8;
   }

   UartBase->BRR = Divider;
   UartBase->CR1 |= USART_CR1_UE;
}

   /* The following function is a utility function to set the           */
   /* configuration of a specified GPIO.  This function accepts as its  */
   /* parameters the GPIO port, pin and mode of operation.              */
static void ConfigureGPIO(GPIO_TypeDef *Port, unsigned int Pin, GPIOMode_TypeDef Mode)
{
   GPIO_InitTypeDef GpioConfiguration;

   /* Setup the configuration structure.                                */
   GpioConfiguration.GPIO_Pin   = 1 << Pin;
   GpioConfiguration.GPIO_Mode  = Mode;
   GpioConfiguration.GPIO_Speed = GPIO_Speed_25MHz;
   GpioConfiguration.GPIO_OType = GPIO_OType_PP;
   GpioConfiguration.GPIO_PuPd  = GPIO_PuPd_NOPULL;

   /* Configure the GPIO.                                               */
   GPIO_Init(Port, &GpioConfiguration);
}

   /* The following function is responsible for setting the CTS and RTS */
   /* GPIO to the correct state when entering and leaving suspend mode. */
   /* The first parameter specifies if the GPIO should be set for the   */
   /* suspend mode (TRUE) or not-suspended mode (FALSE).                */
static void SetSuspendGPIO(Boolean_t Suspend)
{
#ifdef SUPPORT_TRANSPORT_SUSPEND

   EXTI_InitTypeDef ExtIntConfiguration;

   if(Suspend)
   {
      /* Set CTS as a GPIO input.                                       */
      ConfigureGPIO(HCITR_CTS_GPIO_PORT, HCITR_CTS_PIN, GPIO_Mode_IN);

      /* Enable the external interrupt on the CTS pin.                  */
      EXTI_Init((EXTI_InitTypeDef *)&CTS_ExtIntConfiguration);
      NVIC_SetPriority(HCITR_CTS_IRQ, INTERRUPT_PRIORITY);
      NVIC_EnableIRQ(HCITR_CTS_IRQ);

      /* Set RTS as an output and pull high.                            */
      ConfigureGPIO(HCITR_RTS_GPIO_PORT, HCITR_RTS_PIN, GPIO_Mode_OUT);
   }
   else
   {
      /* Disable the CTS interrupt.                                     */
      BTPS_MemCopy(&ExtIntConfiguration, &CTS_ExtIntConfiguration, sizeof(EXTI_InitTypeDef));
      ExtIntConfiguration.EXTI_LineCmd = DISABLE;
      EXTI_Init(&ExtIntConfiguration);

      /* Set the CTS and RTS pins back to the default functionality.    */
      ConfigureGPIO(HCITR_CTS_GPIO_PORT, HCITR_CTS_PIN, GPIO_Mode_AF);
      ConfigureGPIO(HCITR_RTS_GPIO_PORT, HCITR_RTS_PIN, GPIO_Mode_AF);
   }

#endif
}

   /* The following function starts a TX DMA transfer if one is not     */
   /* already in progress.                                              */
   /* * NOTE * Interrupts Must be disabled when calling this function as*/
   /*          it is not re-entrant.                                    */
static void StartTxDMATransfer(void)
{
   /* Confirm that a DMA is not currently in progress (true if the      */
   /* previous DMA length is 0) and that there is data waiting to be    */
   /* sent.                                                             */
   if((!(UartContext.TxPreviousDMALength)) && ((UartContext.TxPreviousDMALength = (OUTPUT_BUFFER_SIZE - UartContext.TxBytesFree)) != 0))
   {
      /* Determine the size of the transfer as the minimum of the data  */
      /* to be sent, the maximum DMA length and the amount at the end of*/
      /* the buffer.                                                    */
      if(UartContext.TxPreviousDMALength > OUTPUT_MAXIMUM_DMA_SIZE)
         UartContext.TxPreviousDMALength = OUTPUT_MAXIMUM_DMA_SIZE;

      if(UartContext.TxPreviousDMALength > (OUTPUT_BUFFER_SIZE - UartContext.TxOutIndex))
         UartContext.TxPreviousDMALength = OUTPUT_BUFFER_SIZE - UartContext.TxOutIndex;

      /* Set the address and length of the transfer.                    */
      HCITR_TXD_DMA_STREAM->M0AR = (unsigned int)&(UartContext.TxBuffer[UartContext.TxOutIndex]);
      HCITR_TXD_DMA_STREAM->NDTR = UartContext.TxPreviousDMALength;

      /* Start the DMA.                                                 */
      USART_DMACmd(HCITR_UART_BASE, USART_DMAReq_Tx, ENABLE);
      USART_ClearFlag(HCITR_UART_BASE, USART_FLAG_TC);
      DMA_Cmd(HCITR_TXD_DMA_STREAM, ENABLE);

      /* Adjust the indexes.                                            */
      UartContext.TxOutIndex  += UartContext.TxPreviousDMALength;
      if(UartContext.TxOutIndex == OUTPUT_BUFFER_SIZE)
         UartContext.TxOutIndex = 0;
   }
}

   /* The following function starts a RX DMA transfer if one is not     */
   /* already in progress.                                              */
   /* * NOTE * Interrupts Must be disabled when calling this function as*/
   /*          it is not re-entrant.                                    */
static void StartRxDMATransfer(void)
{
   unsigned BufferEnd;

   /* Determine the end of the availabe region to use for DMA as either */
   /* the output index (if the current data in the buffer wraps the end)*/
   /* or the size of the buffer.                                        */
   if(UartContext.RxOutIndex > UartContext.RxInIndex)
      BufferEnd = UartContext.RxOutIndex;
   else
      BufferEnd = INPUT_BUFFER_SIZE;

   /* Confirm that a Rx DMA is not curently in progress and there is    */
   /* space in the buffer for another DMA transfer.                     */
   if((!(UartContext.RxDMAInProgress)) && (UartContext.RxBytesUsed != UartContext.RxBufferSize) && ((BufferEnd - UartContext.RxInIndex) >= INPUT_DMA_SIZE))
   {
      HCITR_RXD_DMA_STREAM->M0AR  = (unsigned int)(&(UartContext.RxBuffer[UartContext.RxInIndex]));
      HCITR_RXD_DMA_STREAM->NDTR  = INPUT_DMA_SIZE;

      DMA_Cmd(HCITR_RXD_DMA_STREAM, ENABLE);
      USART_DMACmd(HCITR_UART_BASE, USART_DMAReq_Rx, ENABLE);

      UartContext.RxDMAInProgress = TRUE;
   }
}

   /* The following function processes data on the Rx DMA stream. Before*/
   /* processing starts, the DMA will be disabled and re-enabled when   */
   /* processing is complete. This function returns TRUE if data was    */
   /* read from the DMA or FALSE otherwise.                             */
   /* * NOTE * Interrupts MUST be disabled when this function is called */
   /*          to prevent clashes with the ISR.                         */
static Boolean_t ProcessRxDMA(void)
{
   Boolean_t    ret_val;
   unsigned int Length;

   /* Check if any data is available in the DMA buffer.                 */
   if(HCITR_RXD_DMA_STREAM->NDTR != INPUT_DMA_SIZE)
   {
      /* Make sure the DMA is disabled and clear the interrupt flags.   */
      USART_DMACmd(HCITR_UART_BASE, USART_DMAReq_Rx, DISABLE);
      DMA_Cmd(HCITR_RXD_DMA_STREAM, DISABLE);
      while(DMA_GetCmdStatus(HCITR_RXD_DMA_STREAM) == ENABLE);

      DMA_ClearFlag(HCITR_RXD_DMA_STREAM, HCITR_RXD_DMA_FLAG_TCIF | HCITR_RXD_DMA_FLAG_HTIF);

      UartContext.RxDMAInProgress = FALSE;

      /* Note how much data is in the buffer and adjust the indexes.    */
      Length = INPUT_DMA_SIZE - (unsigned short)(HCITR_RXD_DMA_STREAM->NDTR);
      UartContext.RxBytesUsed += Length;
      UartContext.RxInIndex   += Length;

      /* Determine if the next DMA transfer will wrap the end of the    */
      /* buffer. If this is the case then set the size of the buffer to */
      /* the current index.                                             */
      if(UartContext.RxInIndex >= (INPUT_BUFFER_SIZE - INPUT_DMA_SIZE))
      {
         UartContext.RxBufferSize = UartContext.RxInIndex;
         UartContext.RxInIndex    = 0;
      }

      /* Re-start the DMA transfer.                                     */
      StartRxDMATransfer();
      ret_val = TRUE;
   }
   else
      ret_val = FALSE;

   return(ret_val);
}

   /* The following function is the Interrupt Service Routine for the   */
   /* UART TXD DMA.                                                     */
   /* * NOTE * This function is mapped to the appropriate DMA in        */
   /*          HCITRCFG.h.                                              */
void HCITR_TXD_IRQHandler(void)
{
   DMA_ClearFlag(HCITR_TXD_DMA_STREAM, HCITR_TXD_DMA_FLAG_TCIF);

   UartContext.TxBytesFree         += UartContext.TxPreviousDMALength;
   UartContext.TxPreviousDMALength  = 0;

   /* Start the next DMA transfer.                                      */
   StartTxDMATransfer();
}

   /* The following function is the Interrupt Service Routine for the   */
   /* UART RXD DMA.                                                     */
   /* * NOTE * This function is mapped to the appropriate DMA in        */
   /*          HCITRCFG.h.                                              */
void HCITR_RXD_IRQHandler(void)
{
   ProcessRxDMA();
}

#ifdef SUPPORT_TRANSPORT_SUSPEND

   /* The following function handles the GPIO interrupt from the        */
   /* Bluetooth CTS line.  will make sure that the transmit interrupt is*/
   /* enabled when the flow control line is low.                        */
void HCITR_CTS_IRQ_HANDLER(void)
{
   if(UartContext.SuspendState == hssSuspended)
   {
      /* Resume the UART.                                               */
      EnableUartPeriphClock();
      SetSuspendGPIO(FALSE);
      UartContext.SuspendState = hssNormal;
   }
   else
   {
      if(UartContext.SuspendState == hssSuspendWait)
      {
         /* Indicate the suspend is interrupted.                        */
         UartContext.SuspendState = hssSuspendWaitInterrupted;
      }
   }

   /* Enable the UART transmit interrupt if there is data in the buffer.*/
   if(UartContext.TxBytesFree != OUTPUT_BUFFER_SIZE)
      USART_ITConfig(HCITR_UART_BASE, USART_IT_TXE, ENABLE);

   EXTI_ClearFlag(CTS_ExtIntConfiguration.EXTI_Line);
}

#endif

   /* The following function is responsible for opening the HCI         */
   /* Transport layer that will be used by Bluetopia to send and receive*/
   /* COM (Serial) data.  This function must be successfully issued in  */
   /* order for Bluetopia to function.  This function accepts as its    */
   /* parameter the HCI COM Transport COM Information that is to be used*/
   /* to open the port.  The final two parameters specify the HCI       */
   /* Transport Data Callback and Callback Parameter (respectively) that*/
   /* is to be called when data is received from the UART.  A successful*/
   /* call to this function will return a non-zero, positive value which*/
   /* specifies the HCITransportID that is used with the remaining      */
   /* transport functions in this module.  This function returns a      */
   /* negative return value to signify an error.                        */
int BTPSAPI HCITR_COMOpen(HCI_COMMDriverInformation_t *COMMDriverInformation, HCITR_COMDataCallback_t COMDataCallback, unsigned long CallbackParameter)
{
   int ret_val;

   /* First, make sure that the port is not already open and make sure  */
   /* that valid COMM Driver Information was specified.                 */
   if((!HCITransportOpen) && (COMMDriverInformation) && (COMDataCallback))
   {
      /* Initialize the return value for success.                       */
      ret_val               = TRANSPORT_ID;

      /* Initialize the context structure.                              */
      BTPS_MemInitialize(&UartContext, 0, sizeof(UartContext_t));

      UartContext.COMDataCallbackFunction  = COMDataCallback;
      UartContext.COMDataCallbackParameter = CallbackParameter;
      UartContext.TxBytesFree              = OUTPUT_BUFFER_SIZE;
      UartContext.RxBufferSize             = INPUT_BUFFER_SIZE;
      UartContext.SuspendState             = hssNormal;

      /* Flag that the HCI Transport is open.                           */
      HCITransportOpen = 1;

      /* Enable the peripheral clocks for the UART and its GPIO.        */
      EnableUartPeriphClock();
      RCC_AHB1PeriphClockCmd(HCITR_TXD_GPIO_AHB_BIT | HCITR_RXD_GPIO_AHB_BIT | HCITR_RTS_GPIO_AHB_BIT | HCITR_CTS_GPIO_AHB_BIT | HCITR_RESET_GPIO_AHB_BIT | HCITR_TXD_DMA_AHB_BIT | HCITR_RXD_DMA_AHB_BIT, ENABLE);

      /* Configure the GPIO.                                            */
      ConfigureGPIO(HCITR_RESET_GPIO_PORT, HCITR_RESET_PIN, GPIO_Mode_OUT);
      SetReset();

      ConfigureGPIO(HCITR_TXD_GPIO_PORT, HCITR_TXD_PIN, GPIO_Mode_AF);
      GPIO_PinAFConfig(HCITR_TXD_GPIO_PORT, HCITR_TXD_PIN, HCITR_UART_GPIO_AF);

      ConfigureGPIO(HCITR_RXD_GPIO_PORT, HCITR_RXD_PIN, GPIO_Mode_AF);
      GPIO_PinAFConfig(HCITR_RXD_GPIO_PORT, HCITR_RXD_PIN, HCITR_UART_GPIO_AF);

      ConfigureGPIO(HCITR_RTS_GPIO_PORT, HCITR_RTS_PIN, GPIO_Mode_AF);
      GPIO_PinAFConfig(HCITR_RTS_GPIO_PORT, HCITR_RTS_PIN, HCITR_UART_GPIO_AF);
      GPIO_SetBits(HCITR_RTS_GPIO_PORT, (1 << HCITR_RTS_PIN));

      ConfigureGPIO(HCITR_CTS_GPIO_PORT, HCITR_CTS_PIN, GPIO_Mode_AF);
      GPIO_PinAFConfig(HCITR_CTS_GPIO_PORT, HCITR_CTS_PIN, HCITR_UART_GPIO_AF);

      /* Initialize the UART.                                           */
      USART_Init(HCITR_UART_BASE, (USART_InitTypeDef *)&UartConfiguration);

      /* Reconfigure the baud rate to make sure it is as accurate as    */
      /* possible.                                                      */
      SetBaudRate(HCITR_UART_BASE, COMMDriverInformation->BaudRate);

      /* Initialze the DMA.                                             */
      DMA_Init(HCITR_TXD_DMA_STREAM, (DMA_InitTypeDef *)&TXDDMAConfiguration);
      DMA_Init(HCITR_RXD_DMA_STREAM, (DMA_InitTypeDef *)&RXDDMAConfiguration);

      DMA_ITConfig(HCITR_TXD_DMA_STREAM, DMA_IT_TC, ENABLE);
      DMA_ITConfig(HCITR_RXD_DMA_STREAM, DMA_IT_HT, ENABLE);
      StartRxDMATransfer();

      NVIC_SetPriority(HCITR_TXD_IRQ, INTERRUPT_PRIORITY);
      NVIC_SetPriority(HCITR_RXD_IRQ, INTERRUPT_PRIORITY);
      NVIC_EnableIRQ(HCITR_TXD_IRQ);
      NVIC_EnableIRQ(HCITR_RXD_IRQ);

      /* Enable the UART.                                               */
      USART_Cmd(HCITR_UART_BASE, ENABLE);

#ifdef SUPPORT_TRANSPORT_SUSPEND

      /* Initialize the external interrupt for the CTS line             */
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
      SYSCFG_EXTILineConfig(HCITR_CTS_EXTI_PORT, HCITR_CTS_PIN);
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, DISABLE);

#endif

      /* Clear the reset.                                               */
      BTPS_Delay(10);
      ClearReset();
      BTPS_Delay(250);
   }
   else
      ret_val = HCITR_ERROR_UNABLE_TO_OPEN_TRANSPORT;

   return(ret_val);
}

   /* The following function is responsible for closing the specific HCI*/
   /* Transport layer that was opened via a successful call to the      */
   /* HCITR_COMOpen() function (specified by the first parameter).      */
   /* Bluetopia makes a call to this function whenever an either        */
   /* Bluetopia is closed, or an error occurs during initialization and */
   /* the driver has been opened (and ONLY in this case).  Once this    */
   /* function completes, the transport layer that was closed will no   */
   /* longer process received data until the transport layer is         */
   /* Re-Opened by calling the HCITR_COMOpen() function.                */
   /* * NOTE * This function *MUST* close the specified COM Port.  This */
   /*          module will then call the registered COM Data Callback   */
   /*          function with zero as the data length and NULL as the    */
   /*          data pointer.  This will signify to the HCI Driver that  */
   /*          this module is completely finished with the port and     */
   /*          information and (more importantly) that NO further data  */
   /*          callbacks will be issued.  In other words the very last  */
   /*          data callback that is issued from this module *MUST* be a*/
   /*          data callback specifying zero and NULL for the data      */
   /*          length and data buffer (respectively).                   */
void BTPSAPI HCITR_COMClose(unsigned int HCITransportID)
{
   HCITR_COMDataCallback_t COMDataCallback;

#ifdef SUPPORT_TRANSPORT_SUSPEND

   EXTI_InitTypeDef ExtIntConfiguration;

#endif

   /* Check to make sure that the specified Transport ID is valid.      */
   if((HCITransportID == TRANSPORT_ID) && (HCITransportOpen))
   {
      /* Flag that the HCI Transport is no longer open.                 */
      HCITransportOpen = 0;

#ifdef SUPPORT_TRANSPORT_SUSPEND

      /* Disable external interrupt for the CTS line                    */
      NVIC_DisableIRQ(HCITR_CTS_IRQ);
      BTPS_MemCopy(&ExtIntConfiguration, &CTS_ExtIntConfiguration, sizeof(EXTI_InitTypeDef));
      ExtIntConfiguration.EXTI_LineCmd = DISABLE;
      EXTI_Init(&ExtIntConfiguration);

#endif

      NVIC_DisableIRQ(HCITR_TXD_IRQ);
      NVIC_DisableIRQ(HCITR_RXD_IRQ);

      /* Appears to be valid, go ahead and close the port.              */
      USART_DMACmd(HCITR_UART_BASE, USART_DMAReq_Rx, DISABLE);
      DMA_Cmd(HCITR_RXD_DMA_STREAM, DISABLE);
      while(DMA_GetCmdStatus(HCITR_RXD_DMA_STREAM) == ENABLE);

      USART_DMACmd(HCITR_UART_BASE, USART_DMAReq_Tx, DISABLE);
      DMA_Cmd(HCITR_TXD_DMA_STREAM, DISABLE);
      while(DMA_GetCmdStatus(HCITR_TXD_DMA_STREAM) == ENABLE);

      DMA_ClearFlag(HCITR_RXD_DMA_STREAM, HCITR_RXD_DMA_FLAG_TCIF | HCITR_RXD_DMA_FLAG_HTIF);
      DMA_ClearFlag(HCITR_TXD_DMA_STREAM, HCITR_TXD_DMA_FLAG_TCIF);

      /* Place the Bluetooth Device in Reset.                           */
      SetReset();

      /* Disable the peripheral clock for the UART.                     */
      DisableUartPeriphClock();

      /* Note the Callback information.                                 */
      COMDataCallback   = UartContext.COMDataCallbackFunction;

      UartContext.COMDataCallbackFunction = NULL;

      /* All finished, perform the callback to let the upper layer know */
      /* that this module will no longer issue data callbacks and is    */
      /* completely cleaned up.                                         */
      if(COMDataCallback)
         (*COMDataCallback)(HCITransportID, 0, NULL, UartContext.COMDataCallbackParameter);

      UartContext.COMDataCallbackParameter = 0;
   }
}

   /* The following function is responsible for instructing the         */
   /* specified HCI Transport layer (first parameter) that was opened   */
   /* via a successful call to the HCITR_COMOpen() function to          */
   /* reconfigure itself with the specified information.                */
   /* * NOTE * This function does not close the HCI Transport specified */
   /*          by HCI Transport ID, it merely reconfigures the          */
   /*          transport.  This means that the HCI Transport specified  */
   /*          by HCI Transport ID is still valid until it is closed via*/
   /*          the HCI_COMClose() function.                             */
void BTPSAPI HCITR_COMReconfigure(unsigned int HCITransportID, HCI_Driver_Reconfigure_Data_t *DriverReconfigureData)
{
   HCI_COMMReconfigureInformation_t *ReconfigureInformation;

   /* Check to make sure that the specified Transport ID is valid.      */
   if((HCITransportID == TRANSPORT_ID) && (HCITransportOpen) && (DriverReconfigureData))
   {
      if((DriverReconfigureData->ReconfigureCommand == HCI_COMM_DRIVER_RECONFIGURE_DATA_COMMAND_CHANGE_COMM_PARAMETERS) && (DriverReconfigureData->ReconfigureData))
      {
         ReconfigureInformation = (HCI_COMMReconfigureInformation_t *)(DriverReconfigureData->ReconfigureData);

         /* Check if the baud rate needs to change.                     */
         if(ReconfigureInformation->ReconfigureFlags & HCI_COMM_RECONFIGURE_INFORMATION_RECONFIGURE_FLAGS_CHANGE_BAUDRATE)
         {
            DisableInterrupts();
            SetBaudRate(HCITR_UART_BASE, ReconfigureInformation->BaudRate);
            EnableInterrupts();
         }
      }
   }
}

   /* The following function is provided to allow a mechanism for       */
   /* modules to force the processing of incoming COM Data.             */
   /* * NOTE * This function is only applicable in device stacks that   */
   /*          are non-threaded.  This function has no effect for device*/
   /*          stacks that are operating in threaded environments.      */
void BTPSAPI HCITR_COMProcess(unsigned int HCITransportID)
{
   unsigned int MaxLength;
   unsigned int TotalLength;

#ifdef HCITR_ENABLE_DEBUG_LOGGING

   unsigned int Index;

#endif

   /* Check to make sure that the specified Transport ID is valid.      */
   if((HCITransportID == TRANSPORT_ID) && (HCITransportOpen))
   {
      if(!UartContext.RxBytesUsed)
      {
         /* No data has been received, Process the DMA registers to see */
         /* if any partial packets are available.                       */
         DisableInterrupts();
         ProcessRxDMA();
         EnableInterrupts();
      }

      /* Loop until the receive buffer is empty.                        */
      while((TotalLength = UartContext.RxBytesUsed) != 0)
      {
         /* Determine the maximum number of characters that we can send */
         /* before we reach the end of the buffer.  We need to process  */
         /* the smaller of the max characters or the number of          */
         /* characters that are in the buffer.                          */
         MaxLength = (UartContext.RxBufferSize - UartContext.RxOutIndex);
         if(TotalLength > MaxLength)
            TotalLength = MaxLength;

#ifdef HCITR_ENABLE_DEBUG_LOGGING

         if(UartContext.DebugEnabled)
         {
            DEBUG_PRINT(">");

            for(Index = 0; Index < TotalLength; Index ++)
               DEBUG_PRINT(" %02X", UartContext.RxBuffer[UartContext.RxOutIndex + Index]);

            DEBUG_PRINT("\r\n");
         }

#endif

         /* Call the upper layer back with the data.                    */
         if(UartContext.COMDataCallbackFunction)
            (*UartContext.COMDataCallbackFunction)(TRANSPORT_ID, TotalLength, &(UartContext.RxBuffer[UartContext.RxOutIndex]), UartContext.COMDataCallbackParameter);

         DisableInterrupts();

         /* Adjust the Out Index and handle any looping.                */
         UartContext.RxBytesUsed -= TotalLength;
         UartContext.RxOutIndex += TotalLength;
         if(UartContext.RxOutIndex == UartContext.RxBufferSize)
         {
            UartContext.RxOutIndex = 0;
            UartContext.RxBufferSize = INPUT_BUFFER_SIZE;
         }

         /* Make sure the receive DMA is running.                          */
         StartRxDMATransfer();
         EnableInterrupts();
      }
   }
}

   /* The following function is responsible for actually sending data   */
   /* through the opened HCI Transport layer (specified by the first    */
   /* parameter).  Bluetopia uses this function to send formatted HCI   */
   /* packets to the attached Bluetooth Device.  The second parameter to*/
   /* this function specifies the number of bytes pointed to by the     */
   /* third parameter that are to be sent to the Bluetooth Device.  This*/
   /* function returns a zero if the all data was transferred           */
   /* successfully or a negative value if an error occurred.  This      */
   /* function MUST NOT return until all of the data is sent (or an     */
   /* error condition occurs).  Bluetopia WILL NOT attempt to call this */
   /* function repeatedly if data fails to be delivered.  This function */
   /* will block until it has either buffered the specified data or sent*/
   /* all of the specified data to the Bluetooth Device.                */
   /* * NOTE * The type of data (Command, ACL, SCO, etc.) is NOT passed */
   /*          to this function because it is assumed that this         */
   /*          information is contained in the Data Stream being passed */
   /*          to this function.                                        */
int BTPSAPI HCITR_COMWrite(unsigned int HCITransportID, unsigned int Length, unsigned char *Buffer)
{
   int ret_val;
   int Count;
   int BytesFree;

#ifdef HCITR_ENABLE_DEBUG_LOGGING

   unsigned int Index;

#endif

   /* Check to make sure that the specified Transport ID is valid and   */
   /* the output buffer appears to be valid as well.                    */
   if((HCITransportID == TRANSPORT_ID) && (HCITransportOpen) && (Length) && (Buffer))
   {
      /* If the UART is suspended, resume it.                           */
      if(UartContext.SuspendState == hssSuspended)
      {
         DisableInterrupts();

         EnableUartPeriphClock();
         SetSuspendGPIO(FALSE);
         UartContext.SuspendState = hssNormal;

         EnableInterrupts();
      }

#ifdef HCITR_ENABLE_DEBUG_LOGGING

      if(UartContext.DebugEnabled)
      {
         DEBUG_PRINT("<");

         for(Index = 0; Index < Length; Index ++)
            DEBUG_PRINT(" %02X", Buffer[Index]);

         DEBUG_PRINT("\r\n");
      }

#endif

      /* Process all of the data.                                       */
      while(Length)
      {
         /* Wait for space in the transmit buffer.                      */
         while(!UartContext.TxBytesFree)
            BTPS_Delay(1);

         /* The data may have to be copied in 2 phases.  Calculate the  */
         /* number of character that can be placed in the buffer before */
         /* the buffer must be wrapped.                                 */
         BytesFree = UartContext.TxBytesFree;
         Count = Length;
         Count = (BytesFree < Count) ? BytesFree : Count;
         Count = ((OUTPUT_BUFFER_SIZE - UartContext.TxInIndex) < Count) ? (OUTPUT_BUFFER_SIZE - UartContext.TxInIndex) : Count;

         BTPS_MemCopy(&(UartContext.TxBuffer[UartContext.TxInIndex]), Buffer, Count);

         /* Update the number of free bytes in the buffer.  Since this  */
         /* count can also be updated in the interrupt routine, we will */
         /* have to update this with interrupts disabled.               */

         /* Adjust the index values.                                    */
         Buffer                  += Count;
         Length                  -= Count;
         UartContext.TxInIndex   += Count;
         if(UartContext.TxInIndex == OUTPUT_BUFFER_SIZE)
            UartContext.TxInIndex = 0;

         /* Update the bytes free and make sure the transmit interrupt  */
         /* is enabled.                                                 */
         DisableInterrupts();
         UartContext.TxBytesFree -= Count;
         StartTxDMATransfer();
         EnableInterrupts();
      }

      ret_val = 0;
   }
   else
      ret_val = HCITR_ERROR_WRITING_TO_PORT;

   return(ret_val);
}

   /* The following function is responsible for suspending the HCI COM  */
   /* transport.  It will block until the transmit buffers are empty and*/
   /* all data has been sent then put the transport in a suspended      */
   /* state.  This function will return a value of 0 if the suspend was */
   /* successful or a negative value if there is an error.              */
   /* * NOTE * An error will occur if the suspend operation was         */
   /*          interrupted by another event, such as data being received*/
   /*          before the transmit buffer was empty.                    */
   /* * NOTE * The calling function must lock the Bluetooth stack with a*/
   /*          call to BSC_LockBluetoothStack() before this function is */
   /*          called.                                                  */
   /* * NOTE * This function should only be called when the baseband    */
   /*          low-power protocol in use has indicated that it is safe  */
   /*          to sleep.  Also, this function must be called            */
   /*          successfully before any clocks necessary for the         */
   /*          transport to operate are disabled.                       */
int BTPSAPI HCITR_COMSuspend(unsigned int HCITransportID)
{
   int ret_val;

#ifdef SUPPORT_TRANSPORT_SUSPEND

   if(HCITransportID == TRANSPORT_ID)
   {
      /* Signal that we are waiting for a suspend operation to complete.*/
      UartContext.SuspendState = hssSuspendWait;

      /* Set the CTS interrupt.                                         */
      SetSuspendGPIO(TRUE);

      /* Wait for the UART transmit buffer and FIFO to be empty.        */
      while(((UartContext.TxBytesFree != OUTPUT_BUFFER_SIZE) || (UartContext.TxPreviousDMALength) || (USART_GetFlagStatus(HCITR_UART_BASE, USART_FLAG_TC) != SET)) && (UartContext.SuspendState == hssSuspendWait))
         BTPS_Delay(1);

      /* Confirm that no data was received in this time and suspend the */
      /* UART.                                                          */
      DisableInterrupts();

      if(UartContext.SuspendState == hssSuspendWait)
      {
         UartContext.SuspendState = hssSuspended;

         /* Disable the UART clock.                                     */
         DisableUartPeriphClock();

         ret_val = 0;
      }
      else
      {
         /* Data was received, abort suspending the UART.               */
         SetSuspendGPIO(FALSE);

         ret_val = HCITR_ERROR_SUSPEND_ABORTED;
      }

      EnableInterrupts();

   }
   else
      ret_val = HCITR_ERROR_INVALID_PARAMETER;

#else

   ret_val = HCITR_ERROR_INVALID_PARAMETER;

#endif

   return(ret_val);
}

   /* The following function is used to enable or disable debug logging */
   /* within HCITRANS.  The function accepts as its parameter a flag    */
   /* which indicates if debugging should be enabled.  It returns zero  */
   /* if successful or a negative value if there was an error.          */
int BTPSAPI HCITR_EnableDebugLogging(Boolean_t Enable)
{
   int ret_val;

#ifdef HCITR_ENABLE_DEBUG_LOGGING

   UartContext.DebugEnabled = Enable;

   ret_val = 0;

#else

   ret_val = HCITR_ERROR_INVALID_PARAMETER;

#endif

   return(ret_val);
}

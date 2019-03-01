/*****< AUDIO.c >*************************************************************/
/*      Copyright 2013 Stonestreet One.                                      */
/*      All Rights Reserved.                                                 */
/*                                                                           */
/*      Copyright 2015 Texas Instruments Incorporated.                       */
/*      All Rights Reserved.                                                 */
/*                                                                           */
/*  AUDIO - I2C and I2S interface for audio on the STM3240G-EVAL, includes   */
/*          PCM interface configuratopn on the CC256x Bluetooth chip         */
/*                                                                           */
/*  Author:  Marcus Funk                                                     */
/*                                                                           */
/*** MODIFICATION HISTORY ****************************************************/
/*                                                                           */
/*   mm/dd/yy  F. Lastname    Description of Modification                    */
/*   --------  -----------    -----------------------------------------------*/
/*   10/01/13  M. Funk        Initial creation.                              */
/*   21/01/15  Doron Keren    Bug fixes.                                     */
/*   03/03/15  Doron Keren    Change the STM32f I2S configuration, add calls */
/*                            to the CS43L22 DAC interface, external clock   */
/*                            for the I2S in 8/16KHz and MIC ADC sampling.   */
/*****************************************************************************/

/* Library includes. */
#include "BTPSKRNL.h"
#include "SS1BTVS.h"       /* Vendor Specific Prototypes/Constants.*/
#include "AUDIO.h"
#include "AUDIOCFG.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_adc.h"
#include "stm324xg_eval_ioe.h"
#include "dac_cs43l22.h"

#define AUDIO_INTERRUPT_PRIORITY      (1)

#define I2S_CLK_FREQ_IN_FS_8KHZ           256
#define I2S_CLK_FREQ_IN_FS_16KHZ          512
#define I2S_CLK_FREQ_IN_FS_48KHZ          1536
#define I2S_CLK_FREQ_IN_FS_44_1KHZ        1411

#define ADC_MIC_ZERO_SAMPLE               0x0790
/* The value for the Noise-Gate. When the sample value is between */
/* +/-DC_REMOVAL_THRESHOLD the sample is cleared */
#define DC_REMOVAL_THRESHOLD              800
/* For DC removal, each 256 ADC samples are averaged */
#define AVERAGE_NUMBER_FOR_DC_REMOVAL            256
/* For DC removal, take 16 values from the last 256 averages and make moving average */
#define MOVING_AVERAGE_NUMBER_FOR_DC_REMOVAL     16

 /* The following is used as a printf() replacement.        */
#define Display(_x) do { BTPS_OutputMessage _x; } while(0)

typedef enum
{
   psPlaying,
   psPaused
} PlaybackState_t;

typedef struct _tagAudio_Context_t
{
   Boolean_t        Initialized;
   PlaybackState_t  PlaybackState;
   unsigned short   CurrentVolume;
   Boolean_t        hfpAudio;
} AUDIO_Context_t;

static AUDIO_Context_t AUDIO_Context;

/* Configure the ADC Microphone input pin(PF10) */
static BTPSCONST GPIO_InitTypeDef ADC3_Input_GpioConfiguration = {(1 << ADC3_MIC_PIN),  
	GPIO_Mode_AN, GPIO_Speed_25MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL};
/* Configure MCO2 pin(PC9) */
static BTPSCONST GPIO_InitTypeDef MCO2_GpioConfiguration	= {(1 << MCO2_OUT_PIN),  
	GPIO_Mode_AF, GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_UP};
/* Configure the I2S pins */
static BTPSCONST GPIO_InitTypeDef I2S_WS_GpioConfiguration  = {(1 << AUDIO_I2S_WS_PIN),  
	GPIO_Mode_AF, GPIO_Speed_25MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL};
static BTPSCONST GPIO_InitTypeDef I2S_SCK_GpioConfiguration = {(1 << AUDIO_I2S_SCK_PIN), 
	GPIO_Mode_AF, GPIO_Speed_25MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL};
static BTPSCONST GPIO_InitTypeDef I2S_SDO_GpioConfiguration = {(1 << AUDIO_I2S_SDO_PIN), 
	GPIO_Mode_AF, GPIO_Speed_25MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL}; 
static BTPSCONST GPIO_InitTypeDef I2S_MCK_GpioConfiguration = {(1 << AUDIO_I2S_MCK_PIN), 
	GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL};
/* Set the I2S configuration structure */
static BTPSCONST I2S_InitTypeDef  I2S_Configuration = {I2S_Mode_MasterTx, I2S_Standard_Phillips, 
	I2S_DataFormat_16b, I2S_MCLKOutput_Disable, I2S_AudioFreq_8k, I2S_CPOL_Low};

#ifdef DEBUG_ADC_SAMPLING_TIME
static BTPSCONST GPIO_InitTypeDef Audio_DBG_GpioConfiguration = {(1 << AUDIO_DBG_PIN), 
	GPIO_Mode_OUT, GPIO_Speed_25MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL};
#endif // DEBUG_ADC_SAMPLING_TIME

static int SetVolume(unsigned int Volume);

/* The following function Cpnfigure the ADC for Microphone voice sampling */
static void ADC_Configuration(void)
{
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
 
  /* Put everything back to power-on defaults */
  ADC_DeInit();
  /* Enable ADC3 and GPIO F the MIC input clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
  /* ADC3 Channel 8 -> PF10 */
  GPIO_Init(GPIOF, (GPIO_InitTypeDef *)&ADC3_Input_GpioConfiguration);
  /* ADC Common Init */
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);
  
  /* ADC3 Init */
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC3, &ADC_InitStructure);

  /* Enable ADC3 */
  ADC_Cmd(ADC3, ENABLE);
}

/* The following function reads ADC sample and returns the value  */
static unsigned short readADC3(unsigned char channel)
{
  /* Enable ADC3 */
  ADC_Cmd(ADC3, ENABLE);
  
  ADC_RegularChannelConfig(ADC3, ADC_Channel_8, 1, ADC_SampleTime_3Cycles);
  /* Start the conversion */
  ADC_SoftwareStartConv(ADC3);
  /* Wait until conversion completion */
  while(ADC_GetFlagStatus(ADC3, ADC_FLAG_EOC) == RESET);
  /* Return the conversion value */
  return ADC_GetConversionValue(ADC3);
}

#ifdef A3DP_SRC_PLAY_SIN
unsigned short sin_array8k400hz[120] = {
    0x7ff, 0x86a, 0x8d5, 0x93f, 0x9a9, 0xa11, 0xa78, 0xadd, 0xb40, 0xba1,
    0xbff, 0xc5a, 0xcb2, 0xd08, 0xd59, 0xda7, 0xdf1, 0xe36, 0xe77, 0xeb4,
    0xeec, 0xf1f, 0xf4d, 0xf77, 0xf9a, 0xfb9, 0xfd2, 0xfe5, 0xff3, 0xffc,
    0xfff, 0xffc, 0xff3, 0xfe5, 0xfd2, 0xfb9, 0xf9a, 0xf77, 0xf4d, 0xf1f,
    0xeec, 0xeb4, 0xe77, 0xe36, 0xdf1, 0xda7, 0xd59, 0xd08, 0xcb2, 0xc5a,
    0xbff, 0xba1, 0xb40, 0xadd, 0xa78, 0xa11, 0x9a9, 0x93f, 0x8d5, 0x86a,
    0x7ff, 0x794, 0x729, 0x6bf, 0x655, 0x5ed, 0x586, 0x521, 0x4be, 0x45d,
    0x3ff, 0x3a4, 0x34c, 0x2f6, 0x2a5, 0x257, 0x20d, 0x1c8, 0x187, 0x14a,
    0x112, 0xdf, 0xb1, 0x87, 0x64, 0x45, 0x2c, 0x19, 0xb, 0x2,
    0x0, 0x2, 0xb, 0x19, 0x2c, 0x45, 0x64, 0x87, 0xb1, 0xdf,
    0x112, 0x14a, 0x187, 0x1c8, 0x20d, 0x257, 0x2a5, 0x2f6, 0x34c, 0x3a4,
    0x3ff, 0x45d, 0x4be, 0x521, 0x586, 0x5ed, 0x655, 0x6bf, 0x729, 0x794
  };
#endif /* A3DP_SRC_PLAY_SIN */

/* The following function is the interrupt request handler for the   */
/* I2S interface.                                                    */
void AUDIO_I2S_IRQ_HANDLER(void)
{
#ifdef A3DP_SRC_PLAY_SIN
   static int  sin_index = 0;
#endif
   static unsigned short  in_sample = 0;
   static unsigned short  last_in_samples[4];
   static unsigned short  last_in_samples_point = 0;
   static unsigned short  last_samples_average[256];
   static short           samples_array_pointer = 0;
   static unsigned long   samples_average = ADC_MIC_ZERO_SAMPLE;
   static unsigned long   averages_array[16];
   static unsigned long   averages_array_pointer = 0;
   static unsigned char   start_dc_removal = 0;
   static unsigned char   Data_Status;
   static unsigned short  right_sample = 0;
   /* Write the next word onto the transmit buffer. */
   if(SPI_I2S_GetFlagStatus(AUDIO_I2S_BASE, SPI_I2S_FLAG_TXE ))
   {
   		/* Check what channel, L/R */
   		if(SPI_I2S_GetFlagStatus(AUDIO_I2S_BASE, I2S_FLAG_CHSIDE))
   		{   /* Channel R */
#ifdef A3DP_SRC_PLAY_SIN
            SPI_I2S_SendData(SPI2, in_sample);
#else 
			SPI_I2S_SendData(SPI2, 0x00);
            if(AUDIO_Context.hfpAudio)
            {
                right_sample = readADC3(8);
            }
#endif  
   		}
		else
		{   /* Channel L */
            int i2s_16b_val = 0;
#ifdef DEBUG_ADC_SAMPLING_TIME
			GPIO_SetBits(AUDIO_DBG_GPIO_PORT, (1 << AUDIO_DBG_PIN));
#endif
            if(AUDIO_Context.hfpAudio)
            {
                in_sample = readADC3(8);
                in_sample = ((right_sample + in_sample) >> 1);
                /* Save the sample in integer value for rescale to (+/-)2^15 */
                i2s_16b_val = (int) in_sample;
                /* Save the reading for average and DC removal */
                last_samples_average[samples_array_pointer] = in_sample;
                samples_array_pointer++;
                /* Check if we have new 256 readings, for moving average */
                if(samples_array_pointer >= 256) {
                    int i;
                    unsigned long averaging = 0;
                    /* Clear the pointer for next 256 readings */
                    samples_array_pointer = 0;
                    /* Sum the samples -> Max(12bits)= 0x3FFF = 16,383 -> 16,383*256=4,194,048 which smaller then 2^32 ( U long ) */
                    for(i=0; i<256; i++) {
                        averaging += last_samples_average[i];
                    }
                    /* Devide by 256=2^8 */
                    averaging = averaging >> 8;
                    /* Save in the averages array */
                    averages_array[averages_array_pointer++] = averaging;
                    /* Check if clear the round moving pointer */
                    if(averages_array_pointer >= 16) {
                        averages_array_pointer = 0;
                        /* Set the flag to start the DC removal by changing the average */
                        if(!start_dc_removal) {
                            start_dc_removal = 1;
                        }
                    }
                    /* Check if start DC removal after 256*16=2048 samples arrived -> in 16KHz - 256msec */
                    if(start_dc_removal) {
                        averaging = 0;
                        /* Make 16 average on the averages moving array */
                        for(i=0; i<16; i++) {
                            averaging += averages_array[i];
                        }
                        /* Devide by 16 - 2^4 */
                        averaging = averaging >> 4;
                        /* Set new DC value */
                        samples_average = averaging;
                    }
                }
                
                i2s_16b_val -= samples_average; 
                i2s_16b_val <<= 3;
                
                /* Remove DC noise */
                if((i2s_16b_val < DC_REMOVAL_THRESHOLD) && (i2s_16b_val > (-DC_REMOVAL_THRESHOLD)))
                {
                     i2s_16b_val = 0;
                }
                
                in_sample = (unsigned short) i2s_16b_val;
                /* Make simple Filter to remove circuit noises */
                last_in_samples[last_in_samples_point] = in_sample;
                last_in_samples_point++;
                if(last_in_samples_point >= 4)
                {
                    last_in_samples_point = 0;
                    Data_Status = 1;
                }
            
                if(Data_Status)
                {
                    unsigned short a0_sample, a1_sample, a2_sample, a3_sample;
                
                    if(0 ==  last_in_samples_point)
                    {
                        a0_sample = last_in_samples[3];
                        a1_sample = last_in_samples[2];
                        a2_sample = last_in_samples[1];
                        a3_sample = last_in_samples[0];
                    }
                    else
                    {
                        a0_sample = last_in_samples[(last_in_samples_point-1)];
                        a3_sample = last_in_samples[(last_in_samples_point)];
                        a2_sample = last_in_samples[((last_in_samples_point+1)%4)];
                        a1_sample = last_in_samples[((last_in_samples_point+2)%4)];
                    }
                
                    /*  Simple FIR coefficients: 1/2=a0  1/4=a1  1/8=a2  1/8=a3  */
                    in_sample = (a0_sample >> 1) + (a1_sample >> 2) + (a2_sample >> 3) + (a3_sample >> 3);
                SPI_I2S_SendData(SPI2, in_sample);
            }
            else
            {
                    /* Start send the sampled data to the Bluetooth after the first 4 samples to average */
                    SPI_I2S_SendData(SPI2, 0x00);
                }
            }
            else
            {
#ifdef A3DP_SRC_PLAY_SIN
                /* For Sin() */
                in_sample = (sin_array8k400hz[sin_index] << 3);
				SPI_I2S_SendData(SPI2, in_sample);
				sin_index++;
				if(sin_index >= 120)
				{
					sin_index = 0;
				}
#else  
                SPI_I2S_SendData(SPI2, 0x00);
#endif 
            }
#ifdef DEBUG_ADC_SAMPLING_TIME
			GPIO_ResetBits(AUDIO_DBG_GPIO_PORT, (1 << AUDIO_DBG_PIN));
#endif
		}
   }
}

/* The following function sets the volume of the codec to the        */
/* specified value.  This function returns zero if successful or a   */
/* negative value if there was an error.                             */
static int SetVolume(unsigned int Volume)
{
   int           ret_val;
   unsigned char RegisterValue;

   /* Determine the value that will be written to the CODEC's registers     */
   /* for the specified volume. Register=165 + Volumex6 for 3dB increments. */
   /* The value is between 155 to (155+15x6)= 245 = Gain of -5dB to -50dB   */
   RegisterValue = (Volume * 6) + 155;
   /* Write the value to the DAC registers. */
   Display(("\r\n Volume changed to %d, register value = 0x%x \r\n", Volume, RegisterValue));
   ret_val = setVolumeDAC(RegisterValue);
   /* Update the context information if there wasn't an error. */
   if(!ret_val)
   {
      AUDIO_Context.CurrentVolume = Volume;
   }

   return(ret_val);
}

/* The following function is used to initailize the I2S interface. */
int initializeAudio(unsigned int BluetoothStackID, unsigned long Frequency)
{
	int              ret_val = 0;
	int              timeout_counter = 0;
	I2S_InitTypeDef  I2SConfig;
	unsigned long    register_rcc_plli2scfgr;
	unsigned short   register_spi_i2spr;
	
    if(TRUE == AUDIO_Context.Initialized)
    {
		Display(("\r\n Audio already initialized... \r\n"));
		return 0;
    }
   
    Display(("\r\n I2S configuration start f = %d \r\n", Frequency));
    /* Configure the I2C in the STM3240G-EVAL board */
    IOE_Config();
    /* Hold the DAC, CS43L22, in RESET. */
	IOE_WriteIOPin(AUDIO_RESET_PIN, BitReset);
	/* Configure the BT PCM lines */
	switch(Frequency)
	{
		/* The I2S clock frequency = 16bits x 2ch x Frequency, in KHz, Frequency in Hz */
		case 8000:
			ret_val = VS_PCM_Codec_Config_Slave_I2S(BluetoothStackID, 
				I2S_CLK_FREQ_IN_FS_8KHZ, Frequency);
			register_rcc_plli2scfgr = (2 << 28) | (192 << 6);
			register_spi_i2spr = (1 << 8) | 187;
			break;
		case 16000:
			ret_val = VS_PCM_Codec_Config_Slave_I2S(BluetoothStackID, 
				I2S_CLK_FREQ_IN_FS_16KHZ, Frequency);
			register_rcc_plli2scfgr = (3 << 28) | (192 << 6);
			register_spi_i2spr = (1 << 8) | 62;
			break;
		case 48000:
			ret_val = VS_PCM_Codec_Config_Slave_I2S(BluetoothStackID, 
				I2S_CLK_FREQ_IN_FS_48KHZ, Frequency);
			register_rcc_plli2scfgr = (3 << 28) | (258 << 6);
			register_spi_i2spr = SPI_I2SPR_MCKOE | (1 << 8) | 3;
			break;
		case 44100:
			ret_val = VS_PCM_Codec_Config_Slave_I2S(BluetoothStackID, 
				I2S_CLK_FREQ_IN_FS_44_1KHZ, Frequency);
			register_rcc_plli2scfgr = (2 << 28) | (271 << 6);
			register_spi_i2spr = SPI_I2SPR_MCKOE | (0 << 8) | 6;
			break;
		default:
			Display(("\r\n Unsupported I2S Frequency !!! \r\n"));
			ret_val = -1; 
			return ret_val;
	}
	
	if(ret_val)
	{
		Display(("\r\n Failed to configure the BT PCM Interface !!! \r\n"));
		return ret_val;
	}
    
    /* Enable the peripheral clocks. */
    RCC_APB1PeriphClockCmd(AUDIO_I2C_RCC_PERIPH_CLK_BIT | AUDIO_I2S_RCC_PERIPH_CLK_BIT,
    						ENABLE);
    RCC_AHB1PeriphClockCmd(AUDIO_I2C_SDA_GPIO_AHB_BIT | AUDIO_I2C_SCL_GPIO_AHB_BIT | 
    	AUDIO_I2S_WS_GPIO_AHB_BIT | AUDIO_I2S_SCK_GPIO_AHB_BIT | AUDIO_I2S_SDO_GPIO_AHB_BIT | 
    	AUDIO_I2S_MCK_GPIO_AHB_BIT, ENABLE);
    if(Frequency < 32000)
    {
	    /* Output PLLI2S clock on MCO2 pin(PC9) */ 
	    /* Enable the GPIOC peripheral          */ 
	    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	    /* Configure MCO2 pin(PC9) in alternate function */
		GPIO_Init(GPIOC,  (GPIO_InitTypeDef *)&MCO2_GpioConfiguration);
        AUDIO_Context.hfpAudio = 1;
    }
    else
    {
        AUDIO_Context.hfpAudio = 0;
    }
    /* Set the I2S I/O configuration for each pin */
    GPIO_Init(AUDIO_I2S_WS_GPIO_PORT,  (GPIO_InitTypeDef *)&I2S_WS_GpioConfiguration);
    GPIO_Init(AUDIO_I2S_SCK_GPIO_PORT, (GPIO_InitTypeDef *)&I2S_SCK_GpioConfiguration);
    GPIO_Init(AUDIO_I2S_MCK_GPIO_PORT, (GPIO_InitTypeDef *)&I2S_MCK_GpioConfiguration);
    /* I2S SDO */
    GPIO_Init(AUDIO_I2S_SDO_GPIO_PORT, (GPIO_InitTypeDef *)&I2S_SDO_GpioConfiguration);
 
#ifdef DEBUG_ADC_SAMPLING_TIME
    /* For ADC time measurments */
    GPIO_Init(AUDIO_DBG_GPIO_PORT, (GPIO_InitTypeDef *)&Audio_DBG_GpioConfiguration);
#endif
    /* Set alternate function for the I2S used pins */
    GPIO_PinAFConfig(AUDIO_I2S_WS_GPIO_PORT,  AUDIO_I2S_WS_PIN,  AUDIO_I2S_GPIO_AF);
    GPIO_PinAFConfig(AUDIO_I2S_SCK_GPIO_PORT, AUDIO_I2S_SCK_PIN, AUDIO_I2S_GPIO_AF);
    GPIO_PinAFConfig(AUDIO_I2S_SDO_GPIO_PORT, AUDIO_I2S_SDO_PIN, AUDIO_I2S_GPIO_AF);
	GPIO_PinAFConfig(AUDIO_I2S_MCK_GPIO_PORT, AUDIO_I2S_MCK_PIN, AUDIO_I2S_GPIO_AF);
	if(Frequency < 32000)
    {
		/* Configure the ADC for MIC input sampling */
	    ADC_Configuration();
	    /* PLLI2SCLK clock selected to output on MCO2 pin(PC9) */
	    RCC_MCO2Config(RCC_MCO2Source_PLLCLK, RCC_MCO2Div_4);
	}
	/* Configure the I2S peripheral, AUDIO_I2S_BASE is 2 for I2S2 */
    I2S_Cmd(AUDIO_I2S_BASE, DISABLE);
   
    /* Configure the initial PLL values for the sake of I2S_Init.  */
    RCC->CFGR       |= RCC_CFGR_I2SSRC;  // Set bit 23 (means that the I2S clock source is external, change afterward)
    RCC->CR         &= ~((uint32_t)RCC_CR_PLLI2SON); // Clear PLL for I2S bit 26
    
    RCC->CFGR       &= ~RCC_CFGR_I2SSRC; // Clear bit 23 (means that the I2S clock is PLLI2S)
    // I2S clk=VCO clk/PLLI2SR, the value of PLLI2SR(shift 6) and PLLI2SN(28) is configured here
    RCC->PLLI2SCFGR  = register_rcc_plli2scfgr; // (2 << 28) | (192 << 6); // (2 << 28) | (344 << 6); // (5 << 28) | (256 << 6); // FrequencyConfiguration[0].PLL_I2S; 
    RCC->CR         |= ((uint32_t)RCC_CR_PLLI2SON);  // Set PLL for I2S bit 26
    while(!(RCC->CR & RCC_CR_PLLI2SRDY))
	{
		BTPS_Delay(1);
		timeout_counter++;
		/* Check if we are waiting more then 1sec return error */
		if(timeout_counter > 1000)
		{
			/* Set error return value */
			ret_val = (-1);
			return ret_val;
		}
	};

    /* Configure the pre-scaler.*/
    AUDIO_I2S_BASE->I2SPR = register_spi_i2spr;

	/* Inialize memory for the I2S configuration structure to build */
	BTPS_MemCopy(&I2SConfig, &I2S_Configuration, sizeof(I2S_InitTypeDef)); 
	/* Set the I2S frequency */
    I2SConfig.I2S_AudioFreq = Frequency;
	/* Initialize the I2S interface with the built structure */              
    I2S_Init(AUDIO_I2S_BASE, (I2S_InitTypeDef *)&I2S_Configuration);
	/* Configure the pre-scaler */
    AUDIO_I2S_BASE->I2SPR = register_spi_i2spr;
    /* Start the I2S peripheral */
    I2S_Cmd(AUDIO_I2S_BASE, ENABLE);
    ret_val = startDAC(Frequency);
    if(ret_val)
    {
    	Display(("\r\n !!! Error in startDAC() !!! \r\n"));
		return ret_val;
    }
    /* Set I2S interupt priority */
    NVIC_SetPriority(AUDIO_I2S_IRQ, AUDIO_INTERRUPT_PRIORITY);
    /* Set the status before any interrupt */
    AUDIO_Context.Initialized = TRUE;
    AUDIO_Context.PlaybackState = psPlaying;
    // if(Frequency < 32000)
    // {
        /* Enable I2S interupts */
        SPI_I2S_ITConfig(AUDIO_I2S_BASE, SPI_I2S_IT_TXE, ENABLE);  
        NVIC_EnableIRQ(AUDIO_I2S_IRQ);
    // }
    Display(("\r\n I2S configuration finished \r\n"));
    return ret_val;
}

   /* The following function un-initilizes the codec and disables       */
   /* playback and recording.  This function will return zero if        */
   /* successful or a negative value if there was an error.             */
int uninitializeAUDIO(void)
{
   int ret_val = 0;
   
   if(TRUE == AUDIO_Context.Initialized)
   {
   	   /* Mute the CODEC first */
	   ret_val = muteDAC(TRUE);
	   if(ret_val)
	   {
	       Display(("\r\n Mute the CODEC, returned: %d\r\n", ret_val));
	   }
	   /* Put the CODEC in power save mode */
	   ret_val = powerSaveDAC(TRUE);
	   if(ret_val)
	   {
	       Display(("\r\n Power off the CODEC, returned: %d\r\n", ret_val));
	   }
	   /* Hold the DAC, CS43L22, in RESET. */
       IOE_WriteIOPin(AUDIO_RESET_PIN, BitReset);
	   I2S_Cmd(AUDIO_I2S_BASE, DISABLE);
	   SPI_I2S_ITConfig(AUDIO_I2S_BASE, SPI_I2S_IT_TXE, DISABLE);
	   SPI_I2S_DeInit(AUDIO_I2S_BASE);
	   /* For inialize back in PLAY after PAUSE */
	   AUDIO_Context.Initialized = FALSE;
	   Display(("\r\n uninitializeAUDIO finished...\r\n"));
       BTPS_Delay(50);	       
	   return(ret_val);
   }
   else
   {
	   Display(("\r\n Audio already uninitialized... \r\n"));
	   return(0);
   }
}
 
   /* The following function will set the volume of the audio output.   */
   /* It accepts as its parameter the new volume as a percentage between*/
   /* zero (off) and 100.  This function return zero if successful or a */
   /* negative value if there was an error.                             */
int AUDIO_Set_Volume(unsigned int NewVolume)
{
   int ret_val;

   if(NewVolume <= 16)
      ret_val = SetVolume(NewVolume);
   else
      ret_val = AUDIO_ERROR_INVALID_PARAMETER;

   return(ret_val);
}

   /* The following function returns the current level of the audio     */
   /* volume as a percentage between zero (muted) and 100.  This        */
   /* function returns the volume if successful or a negative value if  */
   /* there was an error.                                               */
int AUDIO_Get_Volume(void)
{
   return(AUDIO_Context.CurrentVolume);
}

   /* The following function configures the DAC, CS43L22, and hold it   */
   /* in HW reset */
void AUDIO_Reset_CODEC(void)
{
   int ret_val = startDAC(16000);
   Display(("Mute and enter the DAC, CS43L22, into power save mode with reset \r\n"));
   /* Mute the CODEC  */
   ret_val = muteDAC(TRUE);
   AUDIO_Context.PlaybackState = psPaused;
   /* Power save mode */
   ret_val = powerSaveDAC(TRUE);
   /* Hold the DAC, CS43L22, in RESET. */
   IOE_WriteIOPin(AUDIO_RESET_PIN, BitReset);
}

   /* The following function Mutes/Un-Mutes the HW CODEC and puts       */
   /* the CODEC in Power Save mode. This function will return zero if   */
   /* successful or a negative value if there was an error.             */
int pauseResumeAudio(void)
{
    int ret_val = 0;
    if(psPaused == AUDIO_Context.PlaybackState)
    {
        Display(("Unmute and exit CODEC power save mode\r\n"));
        /* Just Un-mute the CODEC Headphones output */
		ret_val = powerSaveDAC(FALSE);
		ret_val = muteDAC(FALSE);
        AUDIO_Context.PlaybackState = psPlaying;
    }
    else if(psPlaying == AUDIO_Context.PlaybackState)
    {
       Display(("Mute and enter CODEC power save mode\r\n"));
       /* Just mute the CODEC  */
	   ret_val = muteDAC(TRUE);
       AUDIO_Context.PlaybackState = psPaused;
	   /* Power save mode */
	   ret_val = powerSaveDAC(TRUE);
    }
    else
    {
       Display(("\r\nError!!! pauseResumeA3dpAudio(), AUDIO in unknown state, PlaybackStat=%d \r\n", 
	   			AUDIO_Context.PlaybackState));
    }
    
    return ret_val;
}
   


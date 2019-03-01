/*****< dac_cs43l22.c >*******************************************************/
/*      Copyright 2015 Texas Instruments Incorporated.                       */
/*      All Rights Reserved.                                                 */
/*                                                                           */
/*  dac_cs43l22.c CS43L22 DAC interface                                      */
/*                                                                           */
/*  Author:  Doron Keren                                                     */
/*                                                                           */
/*** MODIFICATION HISTORY ****************************************************/
/*                                                                           */
/*   mm/dd/yy  F. Lastname    Description of Modification                    */
/*   --------  -----------    -----------------------------------------------*/
/*   03/03/15  Doron Keren    Initial creation.                              */
/*****************************************************************************/

/* Library includes. */
#include "BTPSKRNL.h"
#include "AUDIOCFG.h"
#include "dac_cs43l22.h"
#include "stm324xg_eval_ioe.h"

/* CS43L22 registers addresses */
#define POWER_CTRL1_ADDR                  0x02
#define POWER_CTRL2_ADDR                  0x04
#define CLOCK_CTRL_ADDR                   0x05
#define INT_CTRL1_ADDR                    0x06
#define PCM_A_VOL_ADDR                    0x1A
#define PCM_B_VOL_ADDR                    0x1B
#define MASTER_VOL_A_CTRL_ADDR            0x20
#define MASTER_VOL_B_CTRL_ADDR            0x21
#define HP_VOL_A_CTRL_ADDR                0x22
#define HP_VOL_B_CTRL_ADDR                0x23
/* CS43L22 registers values */
#define POWER_CTRL1_POWER_UP              0x9E
#define POWER_CTRL1_POWER_DOWN            0x01
/* 0xBF: Headphone channel B on=10, A off=11, SPK A and B off=1111 */
#define POWER_CTRL2_VAL                   0xAF   
#define POWER_CTRL2_ALL_OFF               0xFF
/* Auto detect disabled=0, Quarter-speed mode=11, 8/16/32KHz=1, No 27MHz video Clock, Internal MCLK/LRCK=125=01, MCLKDIV2=1 */
#define CLOCK_CTRL_VAL_8K                 0x73
/* Auto detect disabled=0, Quarter-speed mode=10, 8/16/32KHz=1, No 27MHz video Clock, Internal MCLK/LRCK=125=01, MCLKDIV2=1 */
#define CLOCK_CTRL_VAL_16K                0x53   
/* Auto detect disabled=0, Quarter-speed mode=01, 8/16/32KHz=0, No 27MHz video Clock, Internal MCLK/LRCK=125=00, MCLKDIV2=0 */
#define CLOCK_CTRL_VAL_44_1K              0x20   
/* Auto detect disabled=0, Quarter-speed mode=01, 8/16/32KHz=0, No 27MHz video Clock, Internal MCLK/LRCK=125=00, MCLKDIV2=0 */
#define CLOCK_CTRL_VAL_48K                0x20
/* Slave mode=0, No invert=0, Reserved=0, DSP mode disable=0, Left Justified=00, 16b data=11 */
#define INT_CTRL1_SLAVE_VAL               0x03   
/* Master mode=1, No invert=0, Reserved=0, DSP mode disable=0, Left Justified=00, 16b data=11 */
#define INT_CTRL1_MASTER_VAL              0x83   
/*  Mute = MSB bit=1, 7 LSBs for gain, 0x00=0dB, 0xff=-0.5dB, MAX=0x18=12dB, 0x19=-51.5dB, 0.5dB delta */
#define PCM_VOL                           0x00
#define PCM_VOL_MUTE                      0x80   
/* 8 bits for gain, 0x00=0dB, 0xff=-0.5dB, Mute=0x01, 0x34-0x3F=-96dB, 0.5dB delta */
#define MASTER_VOL_MUTE                   0x19
#define MASTER_DEFAULT_VOLUME             0xC0
/* 8 bits for gain, MAX=0x00=0dB, MAX=0x18=12dB, 0x19-0x34=-102dB, 0.5dB delta */
#define HP_VOL_MUTE                       0x01
#define HP_VOL_DEFAULT                    0xFE   

#define CS43L22_I2C_ADDRESS           		0x94
#define CS43L22_I2C_SHORT_RETRY_COUNT       0x00001000
#define CS43L22_I2C_LONG_RETRY_COUNT        0x0012C000
#define CS43L22_I2C_FREQUENCY               100000
#define CS43L22_I2C_DAC_SHIFTED_ADDRESS     0x33
/* From AUDIO.h, to avoid including all the file */
#define AUDIO_ERROR_I2C_OPERATION_FAILED  (-3001)

static BTPSCONST I2C_InitTypeDef  I2C_Configuration = {CS43L22_I2C_FREQUENCY, I2C_Mode_I2C, I2C_DutyCycle_2, 
	CS43L22_I2C_DAC_SHIFTED_ADDRESS, I2C_Ack_Enable, I2C_AcknowledgedAddress_7bit};

static int I2C_WaitEvent(unsigned int I2CEvent, ErrorStatus Value, unsigned int RetryCount);
static int I2C_WaitFlag(unsigned int I2CFlag, FlagStatus Value, unsigned int RetryCount);
static int I2C_StartTransaction(Boolean_t Transmit);
static int WriteRegister(unsigned char Address, unsigned char Value);


/********************** STM32 I2C interface *****************************/

   /* The following function waits while an event is not in the         */
   /* specified state.  It accepts as its parameters the I2C event the  */
   /* function will monitor, the state of the event that indicates the  */
   /* wait is complete and the number of retries before the wait times  */
   /* out.  This function will return zero if succesful or a negative   */
   /* value if there was an error.                                      */
static int I2C_WaitEvent(unsigned int I2CEvent, ErrorStatus Value, unsigned int RetryCount)
{
   int ret_val;

   while((I2C_CheckEvent(AUDIO_I2C_BASE, I2CEvent) != Value) && (RetryCount))
      RetryCount --;

   /* If the retry count reached zero, indicate a failure.              */
   if(RetryCount)
      ret_val = 0;
   else
      ret_val = AUDIO_ERROR_I2C_OPERATION_FAILED;

   return(ret_val);
}

   /* The following function waits while a flag is not in the specified */
   /* state.  It accepts as its parameters the I2C flag the function    */
   /* will monitor, the state of the flag that indicates the wait is    */
   /* complete, and the number of retries before the wait times out.    */
   /* This function will return zero if succesful or a negative value if*/
   /* there was an error.                                               */
static int I2C_WaitFlag(unsigned int I2CFlag, FlagStatus Value, unsigned int RetryCount)
{
   int ret_val;

   while((I2C_GetFlagStatus(AUDIO_I2C_BASE, I2CFlag) != Value) && (RetryCount))
      RetryCount --;

   /* If the retry count reached zero, indicate a failure.*/
   if(RetryCount)
      ret_val = 0;
   else
      ret_val = AUDIO_ERROR_I2C_OPERATION_FAILED;

   return(ret_val);
}


   /* The following function starts an I2C transaction by sending the   */
   /* start operation and address.  It accepts as its paraemter a flag  */
   /* indicating if this is a transmit (TRUE) or receive (FALSE)        */
   /* operation being started.  This function returns zero if successful*/
   /* or a negative value if there was an error.                        */
static int I2C_StartTransaction(Boolean_t Transmit)
{
   int ret_val;
   /* Start the configuration sequence. */
   I2C_GenerateSTART(AUDIO_I2C_BASE, ENABLE);

   if((ret_val = I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT, SUCCESS, CS43L22_I2C_SHORT_RETRY_COUNT)) == 0)
   {
      /* Transmit the slave address and enable writing operation */
      I2C_Send7bitAddress(AUDIO_I2C_BASE, CS43L22_I2C_ADDRESS, (Transmit) ? I2C_Direction_Transmitter : I2C_Direction_Receiver);
   }
   else
      ret_val = AUDIO_ERROR_I2C_OPERATION_FAILED;

   return(ret_val);
}

   /* The following function writes a value to the CODECs register.  It */
   /* accepts as its parameter the address of the CODEC's register and  */
   /* the value to be written.  This function returns zero if successful*/
   /* or a negative value if there was an error.                        */
static int WriteRegister(unsigned char Address, unsigned char Value)
{
   int ret_val;

   /* Wait while the I2C is busy. */
   if((ret_val = I2C_WaitFlag(I2C_FLAG_BUSY, RESET, CS43L22_I2C_LONG_RETRY_COUNT)) == 0)
   {
      if((ret_val = I2C_StartTransaction(TRUE)) == 0)
      {
         if((ret_val = I2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, SUCCESS, CS43L22_I2C_SHORT_RETRY_COUNT)) == 0)
         {
            /* Transmit the register's address. */
            I2C_SendData(AUDIO_I2C_BASE, Address);

            if((ret_val = I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTING, SUCCESS, CS43L22_I2C_SHORT_RETRY_COUNT)) == 0)
            {
               /* Send the register's value. */
               I2C_SendData(AUDIO_I2C_BASE, Value);

               if((ret_val = I2C_WaitFlag(I2C_FLAG_BTF, SET, CS43L22_I2C_LONG_RETRY_COUNT)) == 0)
               {
                  /* Send the stop sequence. */
                  I2C_GenerateSTOP(AUDIO_I2C_BASE, ENABLE);
               }
            }
         }
      }
   }

   /* Return the byte read from Codec */
   return ret_val;
}


/* 
	The following function is used to initailize the DAC.
    Input: Frequency in Hertz.
    Output: return value = 0 on succes and negative otherwize
*/
int startDAC(unsigned long Frequency)
{
	int    ret_val;
    unsigned char dac_clocking_ctrl_val = 0;
	/* Reset the CODEC.                                               */
	IOE_WriteIOPin(AUDIO_RESET_PIN, BitReset);
	BTPS_Delay(50);
	IOE_WriteIOPin(AUDIO_RESET_PIN, BitSet);

	/* Initialize the I2C Interface.                                  */
	I2C_Cmd(AUDIO_I2C_BASE, ENABLE);
	I2C_Init(AUDIO_I2C_BASE, (I2C_InitTypeDef *)&I2C_Configuration);

	switch(Frequency)
	{
		/* The I2S clock frequency = 16bits x 2ch x Frequency, in KHz, Frequency in Hz */
		case 8000:
			dac_clocking_ctrl_val = CLOCK_CTRL_VAL_8K;
			break;
		case 16000:
			dac_clocking_ctrl_val = CLOCK_CTRL_VAL_16K;
			break;
		case 48000:
			dac_clocking_ctrl_val = CLOCK_CTRL_VAL_44_1K;
			break;
		case 44100:
			dac_clocking_ctrl_val = CLOCK_CTRL_VAL_48K;
			break;
	}

	/* Keep Codec powered OFF                                         */
	if((ret_val = WriteRegister(POWER_CTRL1_ADDR, POWER_CTRL1_POWER_DOWN)) == 0)
	{
		/* Set output to Headphone -> Headphone channel is always ON  */
		/* Speaker channel is always OFF */
		if((ret_val = WriteRegister(POWER_CTRL2_ADDR, POWER_CTRL2_VAL)) == 0)
		{
			/* Clock control configuration with No Auto detection */
			if((ret_val = WriteRegister(CLOCK_CTRL_ADDR, dac_clocking_ctrl_val)) == 0)
			{
				/* Set the Slave Mode (MSB=0) and the audio standard to Phillips. WriteRegister(0x06, 0x04) */
				/* Slave, no invert-> INV_SCLK=0, Set Left justified 16bit */
				if((ret_val = WriteRegister(INT_CTRL1_ADDR, INT_CTRL1_SLAVE_VAL)) == 0)
				{
					/* Set the Master and hedphones volume */
					if((ret_val = setVolumeDAC(MASTER_DEFAULT_VOLUME)) == 0)
					{ 
                       /* Set the PCM A data (from SDIN) digital volume */
                       if((ret_val = WriteRegister(PCM_B_VOL_ADDR, PCM_VOL)) == 0)
                       {
                            /* Set the PCM B data (from SDIN) digital volume */
                            if((ret_val = WriteRegister(PCM_A_VOL_ADDR, PCM_VOL)) == 0)
                            {
                                /* Power on the Codec */
                                ret_val = WriteRegister(POWER_CTRL1_ADDR, POWER_CTRL1_POWER_UP);
                            }
                       }
					}
				}
			}
		}
	}
  
	return ret_val;
}

/*
	Description: The following function mutes/unmutes all the DAC outputs.
    Inputs:  TRUE/FALSE for mute/unmute respectively.
    Outputs: Return value = 0 for succes and negative otherwize
*/
int muteDAC(Boolean_t val)
{
	int    ret_val;

	if(val)
	{
	   /* Mute all DAC outputs */
	   ret_val = WriteRegister(POWER_CTRL2_ADDR, POWER_CTRL2_ALL_OFF);
	}
	else
	{
		/* Unmute the DAC outputs */
		ret_val = WriteRegister(POWER_CTRL2_ADDR, POWER_CTRL2_VAL);
	}
	return ret_val;
}

/*
	Description: The following function puts the DAC in Power Save mode or release from power save.
    Inputs:  TRUE/FALSE for power save/power on the DAC respectively.
    Outputs: Return value = 0 for succes and negative otherwize
*/
int powerSaveDAC(Boolean_t val)
{
	int    ret_val;
	if(val)
	{
		/* Enter the DAC Power save mode */
    	ret_val =  WriteRegister(POWER_CTRL1_ADDR, POWER_CTRL1_POWER_DOWN);
	}
	else
	{
        /* Exit the DAC Power save mode */
    	ret_val =  WriteRegister(POWER_CTRL1_ADDR, POWER_CTRL1_POWER_UP);
	}
	return ret_val;
}

/* 
	The following function sets the DAC volume for the channel Master
	volume and the Headphone channel output volume.
    Input: Volume value 0-255.
    Output: return value = 0 on succes and negative otherwize
*/
int setVolumeDAC(unsigned char volume)
{
	int    ret_val;
	
    /* Write first to the Channel B Master Volume Register 0x21. */
    if((ret_val = WriteRegister(MASTER_VOL_B_CTRL_ADDR, volume)) == 0)
    {
		/* Write to the Channel B Headphones Volume Register 0x23. */
		ret_val = WriteRegister(HP_VOL_B_CTRL_ADDR, HP_VOL_DEFAULT);
    }
    
    /* Write to the Channel A Master Volume Register 0x20. */
    if((ret_val = WriteRegister(MASTER_VOL_A_CTRL_ADDR, volume)) == 0)
    {
		/* Write to the Channel A Headphones Volume Register 0x22. */
		ret_val = WriteRegister(HP_VOL_A_CTRL_ADDR, HP_VOL_DEFAULT);
    }
    
	return ret_val;
}
   

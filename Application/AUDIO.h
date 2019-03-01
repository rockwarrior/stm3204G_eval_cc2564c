/*****< audio.h >*********************************************************/
/*      Copyright 2013 Stonestreet One.                                      */
/*      All Rights Reserved.                                                 */
/*                                                                           */
/*      Copyright 2015 Texas Instruments Incorporated.                       */
/*      All Rights Reserved.                                                 */
/*																			 */
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
#ifndef AUDIO_H_
#define AUDIO_H_

#define AUDIO_ERROR_INVALID_PARAMETER     (-3000)
#define AUDIO_ERROR_I2C_OPERATION_FAILED  (-3001)

   /* The following function initilizes the codec and enables           */
   /* the I2S as master.  This function will return zero if             */
   /* successful or a negative value if there was an error.             */
int initializeAudio(unsigned int BluetoothStackID, unsigned long Frequency);

   /* The following function un-initilizes the codec and disables       */
   /* the I2S.  This function will return zero if                       */
   /* successful or a negative value if there was an error.             */
int uninitializeAUDIO(void);
 
   /* The following function will set the volume of the audio output.   */
   /* It accepts as its parameter the new volume as a percentage between*/
   /* zero (off) and 100.  This function return zero if successful or a */
   /* negative value if there was an error.                             */
int AUDIO_Set_Volume(unsigned int NewVolume);

   /* The following function returns the current level of the audio     */
   /* volume as a percentage between zero (muted) and 100.  This        */
   /* function returns the volume if successful or a negative value if  */
   /* there was an error.                                               */
int AUDIO_Get_Volume(void);

   /* The following function configures the DAC, CS43L22, and hold it   */
   /* in HW reset */
void AUDIO_Reset_CODEC(void);

   /* The following function Mutes/Un-Mutes the HW CODEC and puts       */
   /* the CODEC in Power Save mode. This function will return zero if   */
   /* successful or a negative value if there was an error.             */
int pauseResumeAudio(void);

#endif

/*****< dac_cs43l22.h >*******************************************************/
/*      Copyright 2015 Texas Instruments Incorporated.                       */
/*      All Rights Reserved.                                                 */
/*                                                                           */
/*  dac_cs43l22.h CS43L22 DAC interface                                      */
/*                                                                           */
/*  Author:  Doron Keren                                                     */
/*                                                                           */
/*** MODIFICATION HISTORY ****************************************************/
/*                                                                           */
/*   mm/dd/yy  F. Lastname    Description of Modification                    */
/*   --------  -----------    -----------------------------------------------*/
/*   03/03/15  Doron Keren    Initial creation.                              */
/*****************************************************************************/
#ifndef DAC_CS43L22_H_
#define DAC_CS43L22_H_

int startDAC(unsigned long Frequency);
int muteDAC(Boolean_t val);
int powerSaveDAC(Boolean_t val);
int setVolumeDAC(unsigned char volume);

#endif

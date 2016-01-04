/** ###################################################################
 **     Filename    : DAC0_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the DAC0 module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void DAC0_Init(void);
 **
 **     Copyright : 1997 - 2014 Freescale Semiconductor, Inc.
 **     All Rights Reserved.
 **
 **     Redistribution and use in source and binary forms, with or without modification,
 **     are permitted provided that the following conditions are met:
 **
 **     o Redistributions of source code must retain the above copyright notice, this list
 **       of conditions and the following disclaimer.
 **
 **     o Redistributions in binary form must reproduce the above copyright notice, this
 **       list of conditions and the following disclaimer in the documentation and/or
 **       other materials provided with the distribution.
 **
 **     o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 **       contributors may be used to endorse or promote products derived from this
 **       software without specific prior written permission.
 **
 **     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 **     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 **     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 **     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 **     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 **     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 **     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 **     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 **     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 **     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **
 **     http: www.freescale.com
 **     mail: support@freescale.com
 ** ###################################################################*/

/*!
 * @file DAC0_Init.c
 * @brief This file implements the DAC0 module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE DAC0_Init. */

#include "DAC0_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void DAC0_Init(void) {

  /* Register 'DAC0_C0' initialization */
  #ifdef DAC0_C0_VALUE_1
  DAC0_C0 = DAC0_C0_VALUE_1;
  #endif

  /* Register 'DAC0_SR' initialization */
  #ifdef DAC0_SR_VALUE
  DAC0_SR = DAC0_SR_VALUE;
  #endif

  /* Register 'DAC0_DAT0L' initialization */
  #ifdef DAC0_DAT0L_VALUE
  DAC0_DAT0L = DAC0_DAT0L_VALUE;
  #endif

  /* Register 'DAC0_DAT0H' initialization */
  #if DAC0_DAT0H_MASK
    #if DAC0_DAT0H_MASK == 0xFF
  DAC0_DAT0H = DAC0_DAT0H_VALUE;
    #elif DAC0_DAT0H_MASK == DAC0_DAT0H_VALUE
  DAC0_DAT0H |= DAC0_DAT0H_VALUE;
    #elif DAC0_DAT0H_VALUE == 0
  DAC0_DAT0H &= ~DAC0_DAT0H_MASK;
    #else
  DAC0_DAT0H = (DAC0_DAT0H & (~DAC0_DAT0H_MASK)) | DAC0_DAT0H_VALUE;
    #endif
  #elif defined(DAC0_DAT0H_VALUE)
  DAC0_DAT0H = DAC0_DAT0H_VALUE;
  #endif

  /* Register 'DAC0_DAT1L' initialization */
  #ifdef DAC0_DAT1L_VALUE
  DAC0_DAT1L = DAC0_DAT1L_VALUE;
  #endif

  /* Register 'DAC0_DAT1H' initialization */
  #ifdef DAC0_DAT1H_VALUE
  DAC0_DAT1H = DAC0_DAT1H_VALUE;
  #endif

  /* Register 'DAC0_DAT2L' initialization */
  #ifdef DAC0_DAT2L_VALUE
  DAC0_DAT2L = DAC0_DAT2L_VALUE;
  #endif

  /* Register 'DAC0_DAT2H' initialization */
  #ifdef DAC0_DAT2H_VALUE
  DAC0_DAT2H = DAC0_DAT2H_VALUE;
  #endif

  /* Register 'DAC0_DAT3L' initialization */
  #ifdef DAC0_DAT3L_VALUE
  DAC0_DAT3L = DAC0_DAT3L_VALUE;
  #endif

  /* Register 'DAC0_DAT3H' initialization */
  #ifdef DAC0_DAT3H_VALUE
  DAC0_DAT3H = DAC0_DAT3H_VALUE;
  #endif

  /* Register 'DAC0_DAT4L' initialization */
  #ifdef DAC0_DAT4L_VALUE
  DAC0_DAT4L = DAC0_DAT4L_VALUE;
  #endif

  /* Register 'DAC0_DAT4H' initialization */
  #ifdef DAC0_DAT4H_VALUE
  DAC0_DAT4H = DAC0_DAT4H_VALUE;
  #endif

  /* Register 'DAC0_DAT5L' initialization */
  #ifdef DAC0_DAT5L_VALUE
  DAC0_DAT5L = DAC0_DAT5L_VALUE;
  #endif

  /* Register 'DAC0_DAT5H' initialization */
  #ifdef DAC0_DAT5H_VALUE
  DAC0_DAT5H = DAC0_DAT5H_VALUE;
  #endif

  /* Register 'DAC0_DAT6L' initialization */
  #ifdef DAC0_DAT6L_VALUE
  DAC0_DAT6L = DAC0_DAT6L_VALUE;
  #endif

  /* Register 'DAC0_DAT6H' initialization */
  #ifdef DAC0_DAT6H_VALUE
  DAC0_DAT6H = DAC0_DAT6H_VALUE;
  #endif

  /* Register 'DAC0_DAT7L' initialization */
  #ifdef DAC0_DAT7L_VALUE
  DAC0_DAT7L = DAC0_DAT7L_VALUE;
  #endif

  /* Register 'DAC0_DAT7H' initialization */
  #ifdef DAC0_DAT7H_VALUE
  DAC0_DAT7H = DAC0_DAT7H_VALUE;
  #endif

  /* Register 'DAC0_DAT8L' initialization */
  #ifdef DAC0_DAT8L_VALUE
  DAC0_DAT8L = DAC0_DAT8L_VALUE;
  #endif

  /* Register 'DAC0_DAT8H' initialization */
  #ifdef DAC0_DAT8H_VALUE
  DAC0_DAT8H = DAC0_DAT8H_VALUE;
  #endif

  /* Register 'DAC0_DAT9L' initialization */
  #ifdef DAC0_DAT9L_VALUE
  DAC0_DAT9L = DAC0_DAT9L_VALUE;
  #endif

  /* Register 'DAC0_DAT9H' initialization */
  #ifdef DAC0_DAT9H_VALUE
  DAC0_DAT9H = DAC0_DAT9H_VALUE;
  #endif

  /* Register 'DAC0_DAT10L' initialization */
  #ifdef DAC0_DAT10L_VALUE
  DAC0_DAT10L = DAC0_DAT10L_VALUE;
  #endif

  /* Register 'DAC0_DAT10H' initialization */
  #ifdef DAC0_DAT10H_VALUE
  DAC0_DAT10H = DAC0_DAT10H_VALUE;
  #endif

  /* Register 'DAC0_DAT11L' initialization */
  #ifdef DAC0_DAT11L_VALUE
  DAC0_DAT11L = DAC0_DAT11L_VALUE;
  #endif

  /* Register 'DAC0_DAT11H' initialization */
  #ifdef DAC0_DAT11H_VALUE
  DAC0_DAT11H = DAC0_DAT11H_VALUE;
  #endif

  /* Register 'DAC0_DAT12L' initialization */
  #ifdef DAC0_DAT12L_VALUE
  DAC0_DAT12L = DAC0_DAT12L_VALUE;
  #endif

  /* Register 'DAC0_DAT12H' initialization */
  #ifdef DAC0_DAT12H_VALUE
  DAC0_DAT12H = DAC0_DAT12H_VALUE;
  #endif

  /* Register 'DAC0_DAT13L' initialization */
  #ifdef DAC0_DAT13L_VALUE
  DAC0_DAT13L = DAC0_DAT13L_VALUE;
  #endif

  /* Register 'DAC0_DAT13H' initialization */
  #ifdef DAC0_DAT13H_VALUE
  DAC0_DAT13H = DAC0_DAT13H_VALUE;
  #endif

  /* Register 'DAC0_DAT14L' initialization */
  #ifdef DAC0_DAT14L_VALUE
  DAC0_DAT14L = DAC0_DAT14L_VALUE;
  #endif

  /* Register 'DAC0_DAT14H' initialization */
  #ifdef DAC0_DAT14H_VALUE
  DAC0_DAT14H = DAC0_DAT14H_VALUE;
  #endif

  /* Register 'DAC0_DAT15L' initialization */
  #ifdef DAC0_DAT15L_VALUE
  DAC0_DAT15L = DAC0_DAT15L_VALUE;
  #endif

  /* Register 'DAC0_DAT15H' initialization */
  #ifdef DAC0_DAT15H_VALUE
  DAC0_DAT15H = DAC0_DAT15H_VALUE;
  #endif

  /* Register 'DAC0_C2' initialization */
  #ifdef DAC0_C2_VALUE
  DAC0_C2 = DAC0_C2_VALUE;
  #endif

  /* Register 'DAC0_C1' initialization */
  #if DAC0_C1_MASK
    #if DAC0_C1_MASK == 0xFF
  DAC0_C1 = DAC0_C1_VALUE;
    #elif DAC0_C1_MASK == DAC0_C1_VALUE
  DAC0_C1 |= DAC0_C1_VALUE;
    #elif DAC0_C1_VALUE == 0
  DAC0_C1 &= ~DAC0_C1_MASK;
    #else
  DAC0_C1 = (DAC0_C1 & (~DAC0_C1_MASK)) | DAC0_C1_VALUE;
    #endif
  #elif defined(DAC0_C1_VALUE)
  DAC0_C1 = DAC0_C1_VALUE;
  #endif

  /* Register 'DAC0_C0' initialization */
  #ifdef DAC0_C0_VALUE_2
  DAC0_C0 = DAC0_C0_VALUE_2;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END DAC0_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

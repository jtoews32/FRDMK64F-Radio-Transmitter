/** ###################################################################
 **     Filename    : DMAMUX_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the DMAMUX module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void DMAMUX_Init(void);
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
 * @file DMAMUX_Init.c
 * @brief This file implements the DMAMUX module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE DMAMUX_Init. */

#include "DMAMUX_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void DMAMUX_Init(void) {

  /* Register 'DMAMUX_CHCFG0' initialization */
  #ifdef DMAMUX_CHCFG0_VALUE
  DMAMUX_CHCFG0 = DMAMUX_CHCFG0_VALUE;
  #endif

  /* Register 'DMAMUX_CHCFG1' initialization */
  #ifdef DMAMUX_CHCFG1_VALUE
  DMAMUX_CHCFG1 = DMAMUX_CHCFG1_VALUE;
  #endif

  /* Register 'DMAMUX_CHCFG2' initialization */
  #ifdef DMAMUX_CHCFG2_VALUE
  DMAMUX_CHCFG2 = DMAMUX_CHCFG2_VALUE;
  #endif

  /* Register 'DMAMUX_CHCFG3' initialization */
  #ifdef DMAMUX_CHCFG3_VALUE
  DMAMUX_CHCFG3 = DMAMUX_CHCFG3_VALUE;
  #endif

  /* Register 'DMAMUX_CHCFG4' initialization */
  #if DMAMUX_CHCFG4_MASK
    #if DMAMUX_CHCFG4_MASK == 0xFF
  DMAMUX_CHCFG4 = DMAMUX_CHCFG4_VALUE;
    #elif DMAMUX_CHCFG4_MASK == DMAMUX_CHCFG4_VALUE
  DMAMUX_CHCFG4 |= DMAMUX_CHCFG4_VALUE;
    #elif DMAMUX_CHCFG4_VALUE == 0
  DMAMUX_CHCFG4 &= ~DMAMUX_CHCFG4_MASK;
    #else
  DMAMUX_CHCFG4 = (DMAMUX_CHCFG4 & (~DMAMUX_CHCFG4_MASK)) | DMAMUX_CHCFG4_VALUE;
    #endif
  #elif defined(DMAMUX_CHCFG4_VALUE)
  DMAMUX_CHCFG4 = DMAMUX_CHCFG4_VALUE;
  #endif

  /* Register 'DMAMUX_CHCFG5' initialization */
  #if DMAMUX_CHCFG5_MASK
    #if DMAMUX_CHCFG5_MASK == 0xFF
  DMAMUX_CHCFG5 = DMAMUX_CHCFG5_VALUE;
    #elif DMAMUX_CHCFG5_MASK == DMAMUX_CHCFG5_VALUE
  DMAMUX_CHCFG5 |= DMAMUX_CHCFG5_VALUE;
    #elif DMAMUX_CHCFG5_VALUE == 0
  DMAMUX_CHCFG5 &= ~DMAMUX_CHCFG5_MASK;
    #else
  DMAMUX_CHCFG5 = (DMAMUX_CHCFG5 & (~DMAMUX_CHCFG5_MASK)) | DMAMUX_CHCFG5_VALUE;
    #endif
  #elif defined(DMAMUX_CHCFG5_VALUE)
  DMAMUX_CHCFG5 = DMAMUX_CHCFG5_VALUE;
  #endif

  /* Register 'DMAMUX_CHCFG6' initialization */
  #if DMAMUX_CHCFG6_MASK
    #if DMAMUX_CHCFG6_MASK == 0xFF
  DMAMUX_CHCFG6 = DMAMUX_CHCFG6_VALUE;
    #elif DMAMUX_CHCFG6_MASK == DMAMUX_CHCFG6_VALUE
  DMAMUX_CHCFG6 |= DMAMUX_CHCFG6_VALUE;
    #elif DMAMUX_CHCFG6_VALUE == 0
  DMAMUX_CHCFG6 &= ~DMAMUX_CHCFG6_MASK;
    #else
  DMAMUX_CHCFG6 = (DMAMUX_CHCFG6 & (~DMAMUX_CHCFG6_MASK)) | DMAMUX_CHCFG6_VALUE;
    #endif
  #elif defined(DMAMUX_CHCFG6_VALUE)
  DMAMUX_CHCFG6 = DMAMUX_CHCFG6_VALUE;
  #endif

  /* Register 'DMAMUX_CHCFG7' initialization */
  #if DMAMUX_CHCFG7_MASK
    #if DMAMUX_CHCFG7_MASK == 0xFF
  DMAMUX_CHCFG7 = DMAMUX_CHCFG7_VALUE;
    #elif DMAMUX_CHCFG7_MASK == DMAMUX_CHCFG7_VALUE
  DMAMUX_CHCFG7 |= DMAMUX_CHCFG7_VALUE;
    #elif DMAMUX_CHCFG7_VALUE == 0
  DMAMUX_CHCFG7 &= ~DMAMUX_CHCFG7_MASK;
    #else
  DMAMUX_CHCFG7 = (DMAMUX_CHCFG7 & (~DMAMUX_CHCFG7_MASK)) | DMAMUX_CHCFG7_VALUE;
    #endif
  #elif defined(DMAMUX_CHCFG7_VALUE)
  DMAMUX_CHCFG7 = DMAMUX_CHCFG7_VALUE;
  #endif

  /* Register 'DMAMUX_CHCFG8' initialization */
  #if DMAMUX_CHCFG8_MASK
    #if DMAMUX_CHCFG8_MASK == 0xFF
  DMAMUX_CHCFG8 = DMAMUX_CHCFG8_VALUE;
    #elif DMAMUX_CHCFG8_MASK == DMAMUX_CHCFG8_VALUE
  DMAMUX_CHCFG8 |= DMAMUX_CHCFG8_VALUE;
    #elif DMAMUX_CHCFG8_VALUE == 0
  DMAMUX_CHCFG8 &= ~DMAMUX_CHCFG8_MASK;
    #else
  DMAMUX_CHCFG8 = (DMAMUX_CHCFG8 & (~DMAMUX_CHCFG8_MASK)) | DMAMUX_CHCFG8_VALUE;
    #endif
  #elif defined(DMAMUX_CHCFG8_VALUE)
  DMAMUX_CHCFG8 = DMAMUX_CHCFG8_VALUE;
  #endif

  /* Register 'DMAMUX_CHCFG9' initialization */
  #if DMAMUX_CHCFG9_MASK
    #if DMAMUX_CHCFG9_MASK == 0xFF
  DMAMUX_CHCFG9 = DMAMUX_CHCFG9_VALUE;
    #elif DMAMUX_CHCFG9_MASK == DMAMUX_CHCFG9_VALUE
  DMAMUX_CHCFG9 |= DMAMUX_CHCFG9_VALUE;
    #elif DMAMUX_CHCFG9_VALUE == 0
  DMAMUX_CHCFG9 &= ~DMAMUX_CHCFG9_MASK;
    #else
  DMAMUX_CHCFG9 = (DMAMUX_CHCFG9 & (~DMAMUX_CHCFG9_MASK)) | DMAMUX_CHCFG9_VALUE;
    #endif
  #elif defined(DMAMUX_CHCFG9_VALUE)
  DMAMUX_CHCFG9 = DMAMUX_CHCFG9_VALUE;
  #endif

  /* Register 'DMAMUX_CHCFG10' initialization */
  #if DMAMUX_CHCFG10_MASK
    #if DMAMUX_CHCFG10_MASK == 0xFF
  DMAMUX_CHCFG10 = DMAMUX_CHCFG10_VALUE;
    #elif DMAMUX_CHCFG10_MASK == DMAMUX_CHCFG10_VALUE
  DMAMUX_CHCFG10 |= DMAMUX_CHCFG10_VALUE;
    #elif DMAMUX_CHCFG10_VALUE == 0
  DMAMUX_CHCFG10 &= ~DMAMUX_CHCFG10_MASK;
    #else
  DMAMUX_CHCFG10 = (DMAMUX_CHCFG10 & (~DMAMUX_CHCFG10_MASK)) | DMAMUX_CHCFG10_VALUE;
    #endif
  #elif defined(DMAMUX_CHCFG10_VALUE)
  DMAMUX_CHCFG10 = DMAMUX_CHCFG10_VALUE;
  #endif

  /* Register 'DMAMUX_CHCFG11' initialization */
  #if DMAMUX_CHCFG11_MASK
    #if DMAMUX_CHCFG11_MASK == 0xFF
  DMAMUX_CHCFG11 = DMAMUX_CHCFG11_VALUE;
    #elif DMAMUX_CHCFG11_MASK == DMAMUX_CHCFG11_VALUE
  DMAMUX_CHCFG11 |= DMAMUX_CHCFG11_VALUE;
    #elif DMAMUX_CHCFG11_VALUE == 0
  DMAMUX_CHCFG11 &= ~DMAMUX_CHCFG11_MASK;
    #else
  DMAMUX_CHCFG11 = (DMAMUX_CHCFG11 & (~DMAMUX_CHCFG11_MASK)) | DMAMUX_CHCFG11_VALUE;
    #endif
  #elif defined(DMAMUX_CHCFG11_VALUE)
  DMAMUX_CHCFG11 = DMAMUX_CHCFG11_VALUE;
  #endif

  /* Register 'DMAMUX_CHCFG12' initialization */
  #if DMAMUX_CHCFG12_MASK
    #if DMAMUX_CHCFG12_MASK == 0xFF
  DMAMUX_CHCFG12 = DMAMUX_CHCFG12_VALUE;
    #elif DMAMUX_CHCFG12_MASK == DMAMUX_CHCFG12_VALUE
  DMAMUX_CHCFG12 |= DMAMUX_CHCFG12_VALUE;
    #elif DMAMUX_CHCFG12_VALUE == 0
  DMAMUX_CHCFG12 &= ~DMAMUX_CHCFG12_MASK;
    #else
  DMAMUX_CHCFG12 = (DMAMUX_CHCFG12 & (~DMAMUX_CHCFG12_MASK)) | DMAMUX_CHCFG12_VALUE;
    #endif
  #elif defined(DMAMUX_CHCFG12_VALUE)
  DMAMUX_CHCFG12 = DMAMUX_CHCFG12_VALUE;
  #endif

  /* Register 'DMAMUX_CHCFG13' initialization */
  #if DMAMUX_CHCFG13_MASK
    #if DMAMUX_CHCFG13_MASK == 0xFF
  DMAMUX_CHCFG13 = DMAMUX_CHCFG13_VALUE;
    #elif DMAMUX_CHCFG13_MASK == DMAMUX_CHCFG13_VALUE
  DMAMUX_CHCFG13 |= DMAMUX_CHCFG13_VALUE;
    #elif DMAMUX_CHCFG13_VALUE == 0
  DMAMUX_CHCFG13 &= ~DMAMUX_CHCFG13_MASK;
    #else
  DMAMUX_CHCFG13 = (DMAMUX_CHCFG13 & (~DMAMUX_CHCFG13_MASK)) | DMAMUX_CHCFG13_VALUE;
    #endif
  #elif defined(DMAMUX_CHCFG13_VALUE)
  DMAMUX_CHCFG13 = DMAMUX_CHCFG13_VALUE;
  #endif

  /* Register 'DMAMUX_CHCFG14' initialization */
  #if DMAMUX_CHCFG14_MASK
    #if DMAMUX_CHCFG14_MASK == 0xFF
  DMAMUX_CHCFG14 = DMAMUX_CHCFG14_VALUE;
    #elif DMAMUX_CHCFG14_MASK == DMAMUX_CHCFG14_VALUE
  DMAMUX_CHCFG14 |= DMAMUX_CHCFG14_VALUE;
    #elif DMAMUX_CHCFG14_VALUE == 0
  DMAMUX_CHCFG14 &= ~DMAMUX_CHCFG14_MASK;
    #else
  DMAMUX_CHCFG14 = (DMAMUX_CHCFG14 & (~DMAMUX_CHCFG14_MASK)) | DMAMUX_CHCFG14_VALUE;
    #endif
  #elif defined(DMAMUX_CHCFG14_VALUE)
  DMAMUX_CHCFG14 = DMAMUX_CHCFG14_VALUE;
  #endif

  /* Register 'DMAMUX_CHCFG15' initialization */
  #if DMAMUX_CHCFG15_MASK
    #if DMAMUX_CHCFG15_MASK == 0xFF
  DMAMUX_CHCFG15 = DMAMUX_CHCFG15_VALUE;
    #elif DMAMUX_CHCFG15_MASK == DMAMUX_CHCFG15_VALUE
  DMAMUX_CHCFG15 |= DMAMUX_CHCFG15_VALUE;
    #elif DMAMUX_CHCFG15_VALUE == 0
  DMAMUX_CHCFG15 &= ~DMAMUX_CHCFG15_MASK;
    #else
  DMAMUX_CHCFG15 = (DMAMUX_CHCFG15 & (~DMAMUX_CHCFG15_MASK)) | DMAMUX_CHCFG15_VALUE;
    #endif
  #elif defined(DMAMUX_CHCFG15_VALUE)
  DMAMUX_CHCFG15 = DMAMUX_CHCFG15_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END DMAMUX_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

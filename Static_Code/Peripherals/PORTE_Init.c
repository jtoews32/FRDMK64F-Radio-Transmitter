/** ###################################################################
 **     Filename    : PORTE_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the PORTE module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void PORTE_Init(void);
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
 * @file PORTE_Init.c
 * @brief This file implements the PORTE module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE PORTE_Init. */

#include "PORTE_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void PORTE_Init(void) {

  /* Register 'PORTE_PCR0' initialization */
  #if PORTE_PCR0_MASK_1
    #if PORTE_PCR0_MASK_1 == 0xFFFFFFFF
  PORTE_PCR0 = PORTE_PCR0_VALUE_1;
    #elif PORTE_PCR0_MASK_1 == PORTE_PCR0_VALUE_1
  PORTE_PCR0 |= PORTE_PCR0_VALUE_1;
    #elif PORTE_PCR0_VALUE_1 == 0
  PORTE_PCR0 &= ~PORTE_PCR0_MASK_1;
    #else
  PORTE_PCR0 = (PORTE_PCR0 & (~PORTE_PCR0_MASK_1)) | PORTE_PCR0_VALUE_1;
    #endif
  #elif defined(PORTE_PCR0_VALUE_1)
  PORTE_PCR0 = PORTE_PCR0_VALUE_1;
  #endif

  /* Register 'PORTE_PCR1' initialization */
  #if PORTE_PCR1_MASK_1
    #if PORTE_PCR1_MASK_1 == 0xFFFFFFFF
  PORTE_PCR1 = PORTE_PCR1_VALUE_1;
    #elif PORTE_PCR1_MASK_1 == PORTE_PCR1_VALUE_1
  PORTE_PCR1 |= PORTE_PCR1_VALUE_1;
    #elif PORTE_PCR1_VALUE_1 == 0
  PORTE_PCR1 &= ~PORTE_PCR1_MASK_1;
    #else
  PORTE_PCR1 = (PORTE_PCR1 & (~PORTE_PCR1_MASK_1)) | PORTE_PCR1_VALUE_1;
    #endif
  #elif defined(PORTE_PCR1_VALUE_1)
  PORTE_PCR1 = PORTE_PCR1_VALUE_1;
  #endif

  /* Register 'PORTE_PCR2' initialization */
  #if PORTE_PCR2_MASK_1
    #if PORTE_PCR2_MASK_1 == 0xFFFFFFFF
  PORTE_PCR2 = PORTE_PCR2_VALUE_1;
    #elif PORTE_PCR2_MASK_1 == PORTE_PCR2_VALUE_1
  PORTE_PCR2 |= PORTE_PCR2_VALUE_1;
    #elif PORTE_PCR2_VALUE_1 == 0
  PORTE_PCR2 &= ~PORTE_PCR2_MASK_1;
    #else
  PORTE_PCR2 = (PORTE_PCR2 & (~PORTE_PCR2_MASK_1)) | PORTE_PCR2_VALUE_1;
    #endif
  #elif defined(PORTE_PCR2_VALUE_1)
  PORTE_PCR2 = PORTE_PCR2_VALUE_1;
  #endif

  /* Register 'PORTE_PCR3' initialization */
  #if PORTE_PCR3_MASK_1
    #if PORTE_PCR3_MASK_1 == 0xFFFFFFFF
  PORTE_PCR3 = PORTE_PCR3_VALUE_1;
    #elif PORTE_PCR3_MASK_1 == PORTE_PCR3_VALUE_1
  PORTE_PCR3 |= PORTE_PCR3_VALUE_1;
    #elif PORTE_PCR3_VALUE_1 == 0
  PORTE_PCR3 &= ~PORTE_PCR3_MASK_1;
    #else
  PORTE_PCR3 = (PORTE_PCR3 & (~PORTE_PCR3_MASK_1)) | PORTE_PCR3_VALUE_1;
    #endif
  #elif defined(PORTE_PCR3_VALUE_1)
  PORTE_PCR3 = PORTE_PCR3_VALUE_1;
  #endif

  /* Register 'PORTE_PCR4' initialization */
  #if PORTE_PCR4_MASK_1
    #if PORTE_PCR4_MASK_1 == 0xFFFFFFFF
  PORTE_PCR4 = PORTE_PCR4_VALUE_1;
    #elif PORTE_PCR4_MASK_1 == PORTE_PCR4_VALUE_1
  PORTE_PCR4 |= PORTE_PCR4_VALUE_1;
    #elif PORTE_PCR4_VALUE_1 == 0
  PORTE_PCR4 &= ~PORTE_PCR4_MASK_1;
    #else
  PORTE_PCR4 = (PORTE_PCR4 & (~PORTE_PCR4_MASK_1)) | PORTE_PCR4_VALUE_1;
    #endif
  #elif defined(PORTE_PCR4_VALUE_1)
  PORTE_PCR4 = PORTE_PCR4_VALUE_1;
  #endif

  /* Register 'PORTE_PCR5' initialization */
  #if PORTE_PCR5_MASK_1
    #if PORTE_PCR5_MASK_1 == 0xFFFFFFFF
  PORTE_PCR5 = PORTE_PCR5_VALUE_1;
    #elif PORTE_PCR5_MASK_1 == PORTE_PCR5_VALUE_1
  PORTE_PCR5 |= PORTE_PCR5_VALUE_1;
    #elif PORTE_PCR5_VALUE_1 == 0
  PORTE_PCR5 &= ~PORTE_PCR5_MASK_1;
    #else
  PORTE_PCR5 = (PORTE_PCR5 & (~PORTE_PCR5_MASK_1)) | PORTE_PCR5_VALUE_1;
    #endif
  #elif defined(PORTE_PCR5_VALUE_1)
  PORTE_PCR5 = PORTE_PCR5_VALUE_1;
  #endif

  /* Register 'PORTE_PCR6' initialization */
  #if PORTE_PCR6_MASK_1
    #if PORTE_PCR6_MASK_1 == 0xFFFFFFFF
  PORTE_PCR6 = PORTE_PCR6_VALUE_1;
    #elif PORTE_PCR6_MASK_1 == PORTE_PCR6_VALUE_1
  PORTE_PCR6 |= PORTE_PCR6_VALUE_1;
    #elif PORTE_PCR6_VALUE_1 == 0
  PORTE_PCR6 &= ~PORTE_PCR6_MASK_1;
    #else
  PORTE_PCR6 = (PORTE_PCR6 & (~PORTE_PCR6_MASK_1)) | PORTE_PCR6_VALUE_1;
    #endif
  #elif defined(PORTE_PCR6_VALUE_1)
  PORTE_PCR6 = PORTE_PCR6_VALUE_1;
  #endif

  /* Register 'PORTE_PCR24' initialization */
  #if PORTE_PCR24_MASK_1
    #if PORTE_PCR24_MASK_1 == 0xFFFFFFFF
  PORTE_PCR24 = PORTE_PCR24_VALUE_1;
    #elif PORTE_PCR24_MASK_1 == PORTE_PCR24_VALUE_1
  PORTE_PCR24 |= PORTE_PCR24_VALUE_1;
    #elif PORTE_PCR24_VALUE_1 == 0
  PORTE_PCR24 &= ~PORTE_PCR24_MASK_1;
    #else
  PORTE_PCR24 = (PORTE_PCR24 & (~PORTE_PCR24_MASK_1)) | PORTE_PCR24_VALUE_1;
    #endif
  #elif defined(PORTE_PCR24_VALUE_1)
  PORTE_PCR24 = PORTE_PCR24_VALUE_1;
  #endif

  /* Register 'PORTE_PCR25' initialization */
  #if PORTE_PCR25_MASK_1
    #if PORTE_PCR25_MASK_1 == 0xFFFFFFFF
  PORTE_PCR25 = PORTE_PCR25_VALUE_1;
    #elif PORTE_PCR25_MASK_1 == PORTE_PCR25_VALUE_1
  PORTE_PCR25 |= PORTE_PCR25_VALUE_1;
    #elif PORTE_PCR25_VALUE_1 == 0
  PORTE_PCR25 &= ~PORTE_PCR25_MASK_1;
    #else
  PORTE_PCR25 = (PORTE_PCR25 & (~PORTE_PCR25_MASK_1)) | PORTE_PCR25_VALUE_1;
    #endif
  #elif defined(PORTE_PCR25_VALUE_1)
  PORTE_PCR25 = PORTE_PCR25_VALUE_1;
  #endif

  /* Register 'PORTE_PCR26' initialization */
  #if PORTE_PCR26_MASK_1
    #if PORTE_PCR26_MASK_1 == 0xFFFFFFFF
  PORTE_PCR26 = PORTE_PCR26_VALUE_1;
    #elif PORTE_PCR26_MASK_1 == PORTE_PCR26_VALUE_1
  PORTE_PCR26 |= PORTE_PCR26_VALUE_1;
    #elif PORTE_PCR26_VALUE_1 == 0
  PORTE_PCR26 &= ~PORTE_PCR26_MASK_1;
    #else
  PORTE_PCR26 = (PORTE_PCR26 & (~PORTE_PCR26_MASK_1)) | PORTE_PCR26_VALUE_1;
    #endif
  #elif defined(PORTE_PCR26_VALUE_1)
  PORTE_PCR26 = PORTE_PCR26_VALUE_1;
  #endif

  /* Register 'PORTE_PCR0' initialization */
  #if PORTE_PCR0_MASK_2
    #if PORTE_PCR0_MASK_2 == 0xFFFFFFFF
  PORTE_PCR0 = PORTE_PCR0_VALUE_2;
    #elif PORTE_PCR0_MASK_2 == PORTE_PCR0_VALUE_2
  PORTE_PCR0 |= PORTE_PCR0_VALUE_2;
    #elif PORTE_PCR0_VALUE_2 == 0
  PORTE_PCR0 &= ~PORTE_PCR0_MASK_2;
    #else
  PORTE_PCR0 = (PORTE_PCR0 & (~PORTE_PCR0_MASK_2)) | PORTE_PCR0_VALUE_2;
    #endif
  #elif defined(PORTE_PCR0_VALUE_2)
  PORTE_PCR0 = PORTE_PCR0_VALUE_2;
  #endif

  /* Register 'PORTE_PCR1' initialization */
  #if PORTE_PCR1_MASK_2
    #if PORTE_PCR1_MASK_2 == 0xFFFFFFFF
  PORTE_PCR1 = PORTE_PCR1_VALUE_2;
    #elif PORTE_PCR1_MASK_2 == PORTE_PCR1_VALUE_2
  PORTE_PCR1 |= PORTE_PCR1_VALUE_2;
    #elif PORTE_PCR1_VALUE_2 == 0
  PORTE_PCR1 &= ~PORTE_PCR1_MASK_2;
    #else
  PORTE_PCR1 = (PORTE_PCR1 & (~PORTE_PCR1_MASK_2)) | PORTE_PCR1_VALUE_2;
    #endif
  #elif defined(PORTE_PCR1_VALUE_2)
  PORTE_PCR1 = PORTE_PCR1_VALUE_2;
  #endif

  /* Register 'PORTE_PCR2' initialization */
  #if PORTE_PCR2_MASK_2
    #if PORTE_PCR2_MASK_2 == 0xFFFFFFFF
  PORTE_PCR2 = PORTE_PCR2_VALUE_2;
    #elif PORTE_PCR2_MASK_2 == PORTE_PCR2_VALUE_2
  PORTE_PCR2 |= PORTE_PCR2_VALUE_2;
    #elif PORTE_PCR2_VALUE_2 == 0
  PORTE_PCR2 &= ~PORTE_PCR2_MASK_2;
    #else
  PORTE_PCR2 = (PORTE_PCR2 & (~PORTE_PCR2_MASK_2)) | PORTE_PCR2_VALUE_2;
    #endif
  #elif defined(PORTE_PCR2_VALUE_2)
  PORTE_PCR2 = PORTE_PCR2_VALUE_2;
  #endif

  /* Register 'PORTE_PCR3' initialization */
  #if PORTE_PCR3_MASK_2
    #if PORTE_PCR3_MASK_2 == 0xFFFFFFFF
  PORTE_PCR3 = PORTE_PCR3_VALUE_2;
    #elif PORTE_PCR3_MASK_2 == PORTE_PCR3_VALUE_2
  PORTE_PCR3 |= PORTE_PCR3_VALUE_2;
    #elif PORTE_PCR3_VALUE_2 == 0
  PORTE_PCR3 &= ~PORTE_PCR3_MASK_2;
    #else
  PORTE_PCR3 = (PORTE_PCR3 & (~PORTE_PCR3_MASK_2)) | PORTE_PCR3_VALUE_2;
    #endif
  #elif defined(PORTE_PCR3_VALUE_2)
  PORTE_PCR3 = PORTE_PCR3_VALUE_2;
  #endif

  /* Register 'PORTE_PCR4' initialization */
  #if PORTE_PCR4_MASK_2
    #if PORTE_PCR4_MASK_2 == 0xFFFFFFFF
  PORTE_PCR4 = PORTE_PCR4_VALUE_2;
    #elif PORTE_PCR4_MASK_2 == PORTE_PCR4_VALUE_2
  PORTE_PCR4 |= PORTE_PCR4_VALUE_2;
    #elif PORTE_PCR4_VALUE_2 == 0
  PORTE_PCR4 &= ~PORTE_PCR4_MASK_2;
    #else
  PORTE_PCR4 = (PORTE_PCR4 & (~PORTE_PCR4_MASK_2)) | PORTE_PCR4_VALUE_2;
    #endif
  #elif defined(PORTE_PCR4_VALUE_2)
  PORTE_PCR4 = PORTE_PCR4_VALUE_2;
  #endif

  /* Register 'PORTE_PCR5' initialization */
  #if PORTE_PCR5_MASK_2
    #if PORTE_PCR5_MASK_2 == 0xFFFFFFFF
  PORTE_PCR5 = PORTE_PCR5_VALUE_2;
    #elif PORTE_PCR5_MASK_2 == PORTE_PCR5_VALUE_2
  PORTE_PCR5 |= PORTE_PCR5_VALUE_2;
    #elif PORTE_PCR5_VALUE_2 == 0
  PORTE_PCR5 &= ~PORTE_PCR5_MASK_2;
    #else
  PORTE_PCR5 = (PORTE_PCR5 & (~PORTE_PCR5_MASK_2)) | PORTE_PCR5_VALUE_2;
    #endif
  #elif defined(PORTE_PCR5_VALUE_2)
  PORTE_PCR5 = PORTE_PCR5_VALUE_2;
  #endif

  /* Register 'PORTE_PCR6' initialization */
  #if PORTE_PCR6_MASK_2
    #if PORTE_PCR6_MASK_2 == 0xFFFFFFFF
  PORTE_PCR6 = PORTE_PCR6_VALUE_2;
    #elif PORTE_PCR6_MASK_2 == PORTE_PCR6_VALUE_2
  PORTE_PCR6 |= PORTE_PCR6_VALUE_2;
    #elif PORTE_PCR6_VALUE_2 == 0
  PORTE_PCR6 &= ~PORTE_PCR6_MASK_2;
    #else
  PORTE_PCR6 = (PORTE_PCR6 & (~PORTE_PCR6_MASK_2)) | PORTE_PCR6_VALUE_2;
    #endif
  #elif defined(PORTE_PCR6_VALUE_2)
  PORTE_PCR6 = PORTE_PCR6_VALUE_2;
  #endif

  /* Register 'PORTE_PCR24' initialization */
  #if PORTE_PCR24_MASK_2
    #if PORTE_PCR24_MASK_2 == 0xFFFFFFFF
  PORTE_PCR24 = PORTE_PCR24_VALUE_2;
    #elif PORTE_PCR24_MASK_2 == PORTE_PCR24_VALUE_2
  PORTE_PCR24 |= PORTE_PCR24_VALUE_2;
    #elif PORTE_PCR24_VALUE_2 == 0
  PORTE_PCR24 &= ~PORTE_PCR24_MASK_2;
    #else
  PORTE_PCR24 = (PORTE_PCR24 & (~PORTE_PCR24_MASK_2)) | PORTE_PCR24_VALUE_2;
    #endif
  #elif defined(PORTE_PCR24_VALUE_2)
  PORTE_PCR24 = PORTE_PCR24_VALUE_2;
  #endif

  /* Register 'PORTE_PCR25' initialization */
  #if PORTE_PCR25_MASK_2
    #if PORTE_PCR25_MASK_2 == 0xFFFFFFFF
  PORTE_PCR25 = PORTE_PCR25_VALUE_2;
    #elif PORTE_PCR25_MASK_2 == PORTE_PCR25_VALUE_2
  PORTE_PCR25 |= PORTE_PCR25_VALUE_2;
    #elif PORTE_PCR25_VALUE_2 == 0
  PORTE_PCR25 &= ~PORTE_PCR25_MASK_2;
    #else
  PORTE_PCR25 = (PORTE_PCR25 & (~PORTE_PCR25_MASK_2)) | PORTE_PCR25_VALUE_2;
    #endif
  #elif defined(PORTE_PCR25_VALUE_2)
  PORTE_PCR25 = PORTE_PCR25_VALUE_2;
  #endif

  /* Register 'PORTE_PCR26' initialization */
  #if PORTE_PCR26_MASK_2
    #if PORTE_PCR26_MASK_2 == 0xFFFFFFFF
  PORTE_PCR26 = PORTE_PCR26_VALUE_2;
    #elif PORTE_PCR26_MASK_2 == PORTE_PCR26_VALUE_2
  PORTE_PCR26 |= PORTE_PCR26_VALUE_2;
    #elif PORTE_PCR26_VALUE_2 == 0
  PORTE_PCR26 &= ~PORTE_PCR26_MASK_2;
    #else
  PORTE_PCR26 = (PORTE_PCR26 & (~PORTE_PCR26_MASK_2)) | PORTE_PCR26_VALUE_2;
    #endif
  #elif defined(PORTE_PCR26_VALUE_2)
  PORTE_PCR26 = PORTE_PCR26_VALUE_2;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END PORTE_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

/** ###################################################################
 **     Filename    : PORTA_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the PORTA module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void PORTA_Init(void);
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
 * @file PORTA_Init.c
 * @brief This file implements the PORTA module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE PORTA_Init. */

#include "PORTA_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void PORTA_Init(void) {

  /* Register 'PORTA_PCR0' initialization */
  #if PORTA_PCR0_MASK_1
    #if PORTA_PCR0_MASK_1 == 0xFFFFFFFF
  PORTA_PCR0 = PORTA_PCR0_VALUE_1;
    #elif PORTA_PCR0_MASK_1 == PORTA_PCR0_VALUE_1
  PORTA_PCR0 |= PORTA_PCR0_VALUE_1;
    #elif PORTA_PCR0_VALUE_1 == 0
  PORTA_PCR0 &= ~PORTA_PCR0_MASK_1;
    #else
  PORTA_PCR0 = (PORTA_PCR0 & (~PORTA_PCR0_MASK_1)) | PORTA_PCR0_VALUE_1;
    #endif
  #elif defined(PORTA_PCR0_VALUE_1)
  PORTA_PCR0 = PORTA_PCR0_VALUE_1;
  #endif

  /* Register 'PORTA_PCR1' initialization */
  #if PORTA_PCR1_MASK_1
    #if PORTA_PCR1_MASK_1 == 0xFFFFFFFF
  PORTA_PCR1 = PORTA_PCR1_VALUE_1;
    #elif PORTA_PCR1_MASK_1 == PORTA_PCR1_VALUE_1
  PORTA_PCR1 |= PORTA_PCR1_VALUE_1;
    #elif PORTA_PCR1_VALUE_1 == 0
  PORTA_PCR1 &= ~PORTA_PCR1_MASK_1;
    #else
  PORTA_PCR1 = (PORTA_PCR1 & (~PORTA_PCR1_MASK_1)) | PORTA_PCR1_VALUE_1;
    #endif
  #elif defined(PORTA_PCR1_VALUE_1)
  PORTA_PCR1 = PORTA_PCR1_VALUE_1;
  #endif

  /* Register 'PORTA_PCR2' initialization */
  #if PORTA_PCR2_MASK_1
    #if PORTA_PCR2_MASK_1 == 0xFFFFFFFF
  PORTA_PCR2 = PORTA_PCR2_VALUE_1;
    #elif PORTA_PCR2_MASK_1 == PORTA_PCR2_VALUE_1
  PORTA_PCR2 |= PORTA_PCR2_VALUE_1;
    #elif PORTA_PCR2_VALUE_1 == 0
  PORTA_PCR2 &= ~PORTA_PCR2_MASK_1;
    #else
  PORTA_PCR2 = (PORTA_PCR2 & (~PORTA_PCR2_MASK_1)) | PORTA_PCR2_VALUE_1;
    #endif
  #elif defined(PORTA_PCR2_VALUE_1)
  PORTA_PCR2 = PORTA_PCR2_VALUE_1;
  #endif

  /* Register 'PORTA_PCR3' initialization */
  #if PORTA_PCR3_MASK_1
    #if PORTA_PCR3_MASK_1 == 0xFFFFFFFF
  PORTA_PCR3 = PORTA_PCR3_VALUE_1;
    #elif PORTA_PCR3_MASK_1 == PORTA_PCR3_VALUE_1
  PORTA_PCR3 |= PORTA_PCR3_VALUE_1;
    #elif PORTA_PCR3_VALUE_1 == 0
  PORTA_PCR3 &= ~PORTA_PCR3_MASK_1;
    #else
  PORTA_PCR3 = (PORTA_PCR3 & (~PORTA_PCR3_MASK_1)) | PORTA_PCR3_VALUE_1;
    #endif
  #elif defined(PORTA_PCR3_VALUE_1)
  PORTA_PCR3 = PORTA_PCR3_VALUE_1;
  #endif

  /* Register 'PORTA_PCR4' initialization */
  #if PORTA_PCR4_MASK_1
    #if PORTA_PCR4_MASK_1 == 0xFFFFFFFF
  PORTA_PCR4 = PORTA_PCR4_VALUE_1;
    #elif PORTA_PCR4_MASK_1 == PORTA_PCR4_VALUE_1
  PORTA_PCR4 |= PORTA_PCR4_VALUE_1;
    #elif PORTA_PCR4_VALUE_1 == 0
  PORTA_PCR4 &= ~PORTA_PCR4_MASK_1;
    #else
  PORTA_PCR4 = (PORTA_PCR4 & (~PORTA_PCR4_MASK_1)) | PORTA_PCR4_VALUE_1;
    #endif
  #elif defined(PORTA_PCR4_VALUE_1)
  PORTA_PCR4 = PORTA_PCR4_VALUE_1;
  #endif

  /* Register 'PORTA_PCR5' initialization */
  #if PORTA_PCR5_MASK_1
    #if PORTA_PCR5_MASK_1 == 0xFFFFFFFF
  PORTA_PCR5 = PORTA_PCR5_VALUE_1;
    #elif PORTA_PCR5_MASK_1 == PORTA_PCR5_VALUE_1
  PORTA_PCR5 |= PORTA_PCR5_VALUE_1;
    #elif PORTA_PCR5_VALUE_1 == 0
  PORTA_PCR5 &= ~PORTA_PCR5_MASK_1;
    #else
  PORTA_PCR5 = (PORTA_PCR5 & (~PORTA_PCR5_MASK_1)) | PORTA_PCR5_VALUE_1;
    #endif
  #elif defined(PORTA_PCR5_VALUE_1)
  PORTA_PCR5 = PORTA_PCR5_VALUE_1;
  #endif

  /* Register 'PORTA_PCR12' initialization */
  #if PORTA_PCR12_MASK_1
    #if PORTA_PCR12_MASK_1 == 0xFFFFFFFF
  PORTA_PCR12 = PORTA_PCR12_VALUE_1;
    #elif PORTA_PCR12_MASK_1 == PORTA_PCR12_VALUE_1
  PORTA_PCR12 |= PORTA_PCR12_VALUE_1;
    #elif PORTA_PCR12_VALUE_1 == 0
  PORTA_PCR12 &= ~PORTA_PCR12_MASK_1;
    #else
  PORTA_PCR12 = (PORTA_PCR12 & (~PORTA_PCR12_MASK_1)) | PORTA_PCR12_VALUE_1;
    #endif
  #elif defined(PORTA_PCR12_VALUE_1)
  PORTA_PCR12 = PORTA_PCR12_VALUE_1;
  #endif

  /* Register 'PORTA_PCR13' initialization */
  #if PORTA_PCR13_MASK_1
    #if PORTA_PCR13_MASK_1 == 0xFFFFFFFF
  PORTA_PCR13 = PORTA_PCR13_VALUE_1;
    #elif PORTA_PCR13_MASK_1 == PORTA_PCR13_VALUE_1
  PORTA_PCR13 |= PORTA_PCR13_VALUE_1;
    #elif PORTA_PCR13_VALUE_1 == 0
  PORTA_PCR13 &= ~PORTA_PCR13_MASK_1;
    #else
  PORTA_PCR13 = (PORTA_PCR13 & (~PORTA_PCR13_MASK_1)) | PORTA_PCR13_VALUE_1;
    #endif
  #elif defined(PORTA_PCR13_VALUE_1)
  PORTA_PCR13 = PORTA_PCR13_VALUE_1;
  #endif

  /* Register 'PORTA_PCR14' initialization */
  #if PORTA_PCR14_MASK_1
    #if PORTA_PCR14_MASK_1 == 0xFFFFFFFF
  PORTA_PCR14 = PORTA_PCR14_VALUE_1;
    #elif PORTA_PCR14_MASK_1 == PORTA_PCR14_VALUE_1
  PORTA_PCR14 |= PORTA_PCR14_VALUE_1;
    #elif PORTA_PCR14_VALUE_1 == 0
  PORTA_PCR14 &= ~PORTA_PCR14_MASK_1;
    #else
  PORTA_PCR14 = (PORTA_PCR14 & (~PORTA_PCR14_MASK_1)) | PORTA_PCR14_VALUE_1;
    #endif
  #elif defined(PORTA_PCR14_VALUE_1)
  PORTA_PCR14 = PORTA_PCR14_VALUE_1;
  #endif

  /* Register 'PORTA_PCR15' initialization */
  #if PORTA_PCR15_MASK_1
    #if PORTA_PCR15_MASK_1 == 0xFFFFFFFF
  PORTA_PCR15 = PORTA_PCR15_VALUE_1;
    #elif PORTA_PCR15_MASK_1 == PORTA_PCR15_VALUE_1
  PORTA_PCR15 |= PORTA_PCR15_VALUE_1;
    #elif PORTA_PCR15_VALUE_1 == 0
  PORTA_PCR15 &= ~PORTA_PCR15_MASK_1;
    #else
  PORTA_PCR15 = (PORTA_PCR15 & (~PORTA_PCR15_MASK_1)) | PORTA_PCR15_VALUE_1;
    #endif
  #elif defined(PORTA_PCR15_VALUE_1)
  PORTA_PCR15 = PORTA_PCR15_VALUE_1;
  #endif

  /* Register 'PORTA_PCR16' initialization */
  #if PORTA_PCR16_MASK_1
    #if PORTA_PCR16_MASK_1 == 0xFFFFFFFF
  PORTA_PCR16 = PORTA_PCR16_VALUE_1;
    #elif PORTA_PCR16_MASK_1 == PORTA_PCR16_VALUE_1
  PORTA_PCR16 |= PORTA_PCR16_VALUE_1;
    #elif PORTA_PCR16_VALUE_1 == 0
  PORTA_PCR16 &= ~PORTA_PCR16_MASK_1;
    #else
  PORTA_PCR16 = (PORTA_PCR16 & (~PORTA_PCR16_MASK_1)) | PORTA_PCR16_VALUE_1;
    #endif
  #elif defined(PORTA_PCR16_VALUE_1)
  PORTA_PCR16 = PORTA_PCR16_VALUE_1;
  #endif

  /* Register 'PORTA_PCR17' initialization */
  #if PORTA_PCR17_MASK_1
    #if PORTA_PCR17_MASK_1 == 0xFFFFFFFF
  PORTA_PCR17 = PORTA_PCR17_VALUE_1;
    #elif PORTA_PCR17_MASK_1 == PORTA_PCR17_VALUE_1
  PORTA_PCR17 |= PORTA_PCR17_VALUE_1;
    #elif PORTA_PCR17_VALUE_1 == 0
  PORTA_PCR17 &= ~PORTA_PCR17_MASK_1;
    #else
  PORTA_PCR17 = (PORTA_PCR17 & (~PORTA_PCR17_MASK_1)) | PORTA_PCR17_VALUE_1;
    #endif
  #elif defined(PORTA_PCR17_VALUE_1)
  PORTA_PCR17 = PORTA_PCR17_VALUE_1;
  #endif

  /* Register 'PORTA_PCR18' initialization */
  #if PORTA_PCR18_MASK_1
    #if PORTA_PCR18_MASK_1 == 0xFFFFFFFF
  PORTA_PCR18 = PORTA_PCR18_VALUE_1;
    #elif PORTA_PCR18_MASK_1 == PORTA_PCR18_VALUE_1
  PORTA_PCR18 |= PORTA_PCR18_VALUE_1;
    #elif PORTA_PCR18_VALUE_1 == 0
  PORTA_PCR18 &= ~PORTA_PCR18_MASK_1;
    #else
  PORTA_PCR18 = (PORTA_PCR18 & (~PORTA_PCR18_MASK_1)) | PORTA_PCR18_VALUE_1;
    #endif
  #elif defined(PORTA_PCR18_VALUE_1)
  PORTA_PCR18 = PORTA_PCR18_VALUE_1;
  #endif

  /* Register 'PORTA_PCR19' initialization */
  #if PORTA_PCR19_MASK_1
    #if PORTA_PCR19_MASK_1 == 0xFFFFFFFF
  PORTA_PCR19 = PORTA_PCR19_VALUE_1;
    #elif PORTA_PCR19_MASK_1 == PORTA_PCR19_VALUE_1
  PORTA_PCR19 |= PORTA_PCR19_VALUE_1;
    #elif PORTA_PCR19_VALUE_1 == 0
  PORTA_PCR19 &= ~PORTA_PCR19_MASK_1;
    #else
  PORTA_PCR19 = (PORTA_PCR19 & (~PORTA_PCR19_MASK_1)) | PORTA_PCR19_VALUE_1;
    #endif
  #elif defined(PORTA_PCR19_VALUE_1)
  PORTA_PCR19 = PORTA_PCR19_VALUE_1;
  #endif

  /* Register 'PORTA_PCR0' initialization */
  #if PORTA_PCR0_MASK_2
    #if PORTA_PCR0_MASK_2 == 0xFFFFFFFF
  PORTA_PCR0 = PORTA_PCR0_VALUE_2;
    #elif PORTA_PCR0_MASK_2 == PORTA_PCR0_VALUE_2
  PORTA_PCR0 |= PORTA_PCR0_VALUE_2;
    #elif PORTA_PCR0_VALUE_2 == 0
  PORTA_PCR0 &= ~PORTA_PCR0_MASK_2;
    #else
  PORTA_PCR0 = (PORTA_PCR0 & (~PORTA_PCR0_MASK_2)) | PORTA_PCR0_VALUE_2;
    #endif
  #elif defined(PORTA_PCR0_VALUE_2)
  PORTA_PCR0 = PORTA_PCR0_VALUE_2;
  #endif

  /* Register 'PORTA_PCR1' initialization */
  #if PORTA_PCR1_MASK_2
    #if PORTA_PCR1_MASK_2 == 0xFFFFFFFF
  PORTA_PCR1 = PORTA_PCR1_VALUE_2;
    #elif PORTA_PCR1_MASK_2 == PORTA_PCR1_VALUE_2
  PORTA_PCR1 |= PORTA_PCR1_VALUE_2;
    #elif PORTA_PCR1_VALUE_2 == 0
  PORTA_PCR1 &= ~PORTA_PCR1_MASK_2;
    #else
  PORTA_PCR1 = (PORTA_PCR1 & (~PORTA_PCR1_MASK_2)) | PORTA_PCR1_VALUE_2;
    #endif
  #elif defined(PORTA_PCR1_VALUE_2)
  PORTA_PCR1 = PORTA_PCR1_VALUE_2;
  #endif

  /* Register 'PORTA_PCR2' initialization */
  #if PORTA_PCR2_MASK_2
    #if PORTA_PCR2_MASK_2 == 0xFFFFFFFF
  PORTA_PCR2 = PORTA_PCR2_VALUE_2;
    #elif PORTA_PCR2_MASK_2 == PORTA_PCR2_VALUE_2
  PORTA_PCR2 |= PORTA_PCR2_VALUE_2;
    #elif PORTA_PCR2_VALUE_2 == 0
  PORTA_PCR2 &= ~PORTA_PCR2_MASK_2;
    #else
  PORTA_PCR2 = (PORTA_PCR2 & (~PORTA_PCR2_MASK_2)) | PORTA_PCR2_VALUE_2;
    #endif
  #elif defined(PORTA_PCR2_VALUE_2)
  PORTA_PCR2 = PORTA_PCR2_VALUE_2;
  #endif

  /* Register 'PORTA_PCR3' initialization */
  #if PORTA_PCR3_MASK_2
    #if PORTA_PCR3_MASK_2 == 0xFFFFFFFF
  PORTA_PCR3 = PORTA_PCR3_VALUE_2;
    #elif PORTA_PCR3_MASK_2 == PORTA_PCR3_VALUE_2
  PORTA_PCR3 |= PORTA_PCR3_VALUE_2;
    #elif PORTA_PCR3_VALUE_2 == 0
  PORTA_PCR3 &= ~PORTA_PCR3_MASK_2;
    #else
  PORTA_PCR3 = (PORTA_PCR3 & (~PORTA_PCR3_MASK_2)) | PORTA_PCR3_VALUE_2;
    #endif
  #elif defined(PORTA_PCR3_VALUE_2)
  PORTA_PCR3 = PORTA_PCR3_VALUE_2;
  #endif

  /* Register 'PORTA_PCR4' initialization */
  #if PORTA_PCR4_MASK_2
    #if PORTA_PCR4_MASK_2 == 0xFFFFFFFF
  PORTA_PCR4 = PORTA_PCR4_VALUE_2;
    #elif PORTA_PCR4_MASK_2 == PORTA_PCR4_VALUE_2
  PORTA_PCR4 |= PORTA_PCR4_VALUE_2;
    #elif PORTA_PCR4_VALUE_2 == 0
  PORTA_PCR4 &= ~PORTA_PCR4_MASK_2;
    #else
  PORTA_PCR4 = (PORTA_PCR4 & (~PORTA_PCR4_MASK_2)) | PORTA_PCR4_VALUE_2;
    #endif
  #elif defined(PORTA_PCR4_VALUE_2)
  PORTA_PCR4 = PORTA_PCR4_VALUE_2;
  #endif

  /* Register 'PORTA_PCR5' initialization */
  #if PORTA_PCR5_MASK_2
    #if PORTA_PCR5_MASK_2 == 0xFFFFFFFF
  PORTA_PCR5 = PORTA_PCR5_VALUE_2;
    #elif PORTA_PCR5_MASK_2 == PORTA_PCR5_VALUE_2
  PORTA_PCR5 |= PORTA_PCR5_VALUE_2;
    #elif PORTA_PCR5_VALUE_2 == 0
  PORTA_PCR5 &= ~PORTA_PCR5_MASK_2;
    #else
  PORTA_PCR5 = (PORTA_PCR5 & (~PORTA_PCR5_MASK_2)) | PORTA_PCR5_VALUE_2;
    #endif
  #elif defined(PORTA_PCR5_VALUE_2)
  PORTA_PCR5 = PORTA_PCR5_VALUE_2;
  #endif

  /* Register 'PORTA_PCR12' initialization */
  #if PORTA_PCR12_MASK_2
    #if PORTA_PCR12_MASK_2 == 0xFFFFFFFF
  PORTA_PCR12 = PORTA_PCR12_VALUE_2;
    #elif PORTA_PCR12_MASK_2 == PORTA_PCR12_VALUE_2
  PORTA_PCR12 |= PORTA_PCR12_VALUE_2;
    #elif PORTA_PCR12_VALUE_2 == 0
  PORTA_PCR12 &= ~PORTA_PCR12_MASK_2;
    #else
  PORTA_PCR12 = (PORTA_PCR12 & (~PORTA_PCR12_MASK_2)) | PORTA_PCR12_VALUE_2;
    #endif
  #elif defined(PORTA_PCR12_VALUE_2)
  PORTA_PCR12 = PORTA_PCR12_VALUE_2;
  #endif

  /* Register 'PORTA_PCR13' initialization */
  #if PORTA_PCR13_MASK_2
    #if PORTA_PCR13_MASK_2 == 0xFFFFFFFF
  PORTA_PCR13 = PORTA_PCR13_VALUE_2;
    #elif PORTA_PCR13_MASK_2 == PORTA_PCR13_VALUE_2
  PORTA_PCR13 |= PORTA_PCR13_VALUE_2;
    #elif PORTA_PCR13_VALUE_2 == 0
  PORTA_PCR13 &= ~PORTA_PCR13_MASK_2;
    #else
  PORTA_PCR13 = (PORTA_PCR13 & (~PORTA_PCR13_MASK_2)) | PORTA_PCR13_VALUE_2;
    #endif
  #elif defined(PORTA_PCR13_VALUE_2)
  PORTA_PCR13 = PORTA_PCR13_VALUE_2;
  #endif

  /* Register 'PORTA_PCR14' initialization */
  #if PORTA_PCR14_MASK_2
    #if PORTA_PCR14_MASK_2 == 0xFFFFFFFF
  PORTA_PCR14 = PORTA_PCR14_VALUE_2;
    #elif PORTA_PCR14_MASK_2 == PORTA_PCR14_VALUE_2
  PORTA_PCR14 |= PORTA_PCR14_VALUE_2;
    #elif PORTA_PCR14_VALUE_2 == 0
  PORTA_PCR14 &= ~PORTA_PCR14_MASK_2;
    #else
  PORTA_PCR14 = (PORTA_PCR14 & (~PORTA_PCR14_MASK_2)) | PORTA_PCR14_VALUE_2;
    #endif
  #elif defined(PORTA_PCR14_VALUE_2)
  PORTA_PCR14 = PORTA_PCR14_VALUE_2;
  #endif

  /* Register 'PORTA_PCR15' initialization */
  #if PORTA_PCR15_MASK_2
    #if PORTA_PCR15_MASK_2 == 0xFFFFFFFF
  PORTA_PCR15 = PORTA_PCR15_VALUE_2;
    #elif PORTA_PCR15_MASK_2 == PORTA_PCR15_VALUE_2
  PORTA_PCR15 |= PORTA_PCR15_VALUE_2;
    #elif PORTA_PCR15_VALUE_2 == 0
  PORTA_PCR15 &= ~PORTA_PCR15_MASK_2;
    #else
  PORTA_PCR15 = (PORTA_PCR15 & (~PORTA_PCR15_MASK_2)) | PORTA_PCR15_VALUE_2;
    #endif
  #elif defined(PORTA_PCR15_VALUE_2)
  PORTA_PCR15 = PORTA_PCR15_VALUE_2;
  #endif

  /* Register 'PORTA_PCR16' initialization */
  #if PORTA_PCR16_MASK_2
    #if PORTA_PCR16_MASK_2 == 0xFFFFFFFF
  PORTA_PCR16 = PORTA_PCR16_VALUE_2;
    #elif PORTA_PCR16_MASK_2 == PORTA_PCR16_VALUE_2
  PORTA_PCR16 |= PORTA_PCR16_VALUE_2;
    #elif PORTA_PCR16_VALUE_2 == 0
  PORTA_PCR16 &= ~PORTA_PCR16_MASK_2;
    #else
  PORTA_PCR16 = (PORTA_PCR16 & (~PORTA_PCR16_MASK_2)) | PORTA_PCR16_VALUE_2;
    #endif
  #elif defined(PORTA_PCR16_VALUE_2)
  PORTA_PCR16 = PORTA_PCR16_VALUE_2;
  #endif

  /* Register 'PORTA_PCR17' initialization */
  #if PORTA_PCR17_MASK_2
    #if PORTA_PCR17_MASK_2 == 0xFFFFFFFF
  PORTA_PCR17 = PORTA_PCR17_VALUE_2;
    #elif PORTA_PCR17_MASK_2 == PORTA_PCR17_VALUE_2
  PORTA_PCR17 |= PORTA_PCR17_VALUE_2;
    #elif PORTA_PCR17_VALUE_2 == 0
  PORTA_PCR17 &= ~PORTA_PCR17_MASK_2;
    #else
  PORTA_PCR17 = (PORTA_PCR17 & (~PORTA_PCR17_MASK_2)) | PORTA_PCR17_VALUE_2;
    #endif
  #elif defined(PORTA_PCR17_VALUE_2)
  PORTA_PCR17 = PORTA_PCR17_VALUE_2;
  #endif

  /* Register 'PORTA_PCR18' initialization */
  #if PORTA_PCR18_MASK_2
    #if PORTA_PCR18_MASK_2 == 0xFFFFFFFF
  PORTA_PCR18 = PORTA_PCR18_VALUE_2;
    #elif PORTA_PCR18_MASK_2 == PORTA_PCR18_VALUE_2
  PORTA_PCR18 |= PORTA_PCR18_VALUE_2;
    #elif PORTA_PCR18_VALUE_2 == 0
  PORTA_PCR18 &= ~PORTA_PCR18_MASK_2;
    #else
  PORTA_PCR18 = (PORTA_PCR18 & (~PORTA_PCR18_MASK_2)) | PORTA_PCR18_VALUE_2;
    #endif
  #elif defined(PORTA_PCR18_VALUE_2)
  PORTA_PCR18 = PORTA_PCR18_VALUE_2;
  #endif

  /* Register 'PORTA_PCR19' initialization */
  #if PORTA_PCR19_MASK_2
    #if PORTA_PCR19_MASK_2 == 0xFFFFFFFF
  PORTA_PCR19 = PORTA_PCR19_VALUE_2;
    #elif PORTA_PCR19_MASK_2 == PORTA_PCR19_VALUE_2
  PORTA_PCR19 |= PORTA_PCR19_VALUE_2;
    #elif PORTA_PCR19_VALUE_2 == 0
  PORTA_PCR19 &= ~PORTA_PCR19_MASK_2;
    #else
  PORTA_PCR19 = (PORTA_PCR19 & (~PORTA_PCR19_MASK_2)) | PORTA_PCR19_VALUE_2;
    #endif
  #elif defined(PORTA_PCR19_VALUE_2)
  PORTA_PCR19 = PORTA_PCR19_VALUE_2;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END PORTA_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

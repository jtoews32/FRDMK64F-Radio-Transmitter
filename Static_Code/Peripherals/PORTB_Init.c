/** ###################################################################
 **     Filename    : PORTB_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the PORTB module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void PORTB_Init(void);
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
 * @file PORTB_Init.c
 * @brief This file implements the PORTB module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE PORTB_Init. */

#include "PORTB_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void PORTB_Init(void) {

  /* Register 'PORTB_PCR0' initialization */
  #if PORTB_PCR0_MASK_1
    #if PORTB_PCR0_MASK_1 == 0xFFFFFFFF
  PORTB_PCR0 = PORTB_PCR0_VALUE_1;
    #elif PORTB_PCR0_MASK_1 == PORTB_PCR0_VALUE_1
  PORTB_PCR0 |= PORTB_PCR0_VALUE_1;
    #elif PORTB_PCR0_VALUE_1 == 0
  PORTB_PCR0 &= ~PORTB_PCR0_MASK_1;
    #else
  PORTB_PCR0 = (PORTB_PCR0 & (~PORTB_PCR0_MASK_1)) | PORTB_PCR0_VALUE_1;
    #endif
  #elif defined(PORTB_PCR0_VALUE_1)
  PORTB_PCR0 = PORTB_PCR0_VALUE_1;
  #endif

  /* Register 'PORTB_PCR1' initialization */
  #if PORTB_PCR1_MASK_1
    #if PORTB_PCR1_MASK_1 == 0xFFFFFFFF
  PORTB_PCR1 = PORTB_PCR1_VALUE_1;
    #elif PORTB_PCR1_MASK_1 == PORTB_PCR1_VALUE_1
  PORTB_PCR1 |= PORTB_PCR1_VALUE_1;
    #elif PORTB_PCR1_VALUE_1 == 0
  PORTB_PCR1 &= ~PORTB_PCR1_MASK_1;
    #else
  PORTB_PCR1 = (PORTB_PCR1 & (~PORTB_PCR1_MASK_1)) | PORTB_PCR1_VALUE_1;
    #endif
  #elif defined(PORTB_PCR1_VALUE_1)
  PORTB_PCR1 = PORTB_PCR1_VALUE_1;
  #endif

  /* Register 'PORTB_PCR2' initialization */
  #if PORTB_PCR2_MASK_1
    #if PORTB_PCR2_MASK_1 == 0xFFFFFFFF
  PORTB_PCR2 = PORTB_PCR2_VALUE_1;
    #elif PORTB_PCR2_MASK_1 == PORTB_PCR2_VALUE_1
  PORTB_PCR2 |= PORTB_PCR2_VALUE_1;
    #elif PORTB_PCR2_VALUE_1 == 0
  PORTB_PCR2 &= ~PORTB_PCR2_MASK_1;
    #else
  PORTB_PCR2 = (PORTB_PCR2 & (~PORTB_PCR2_MASK_1)) | PORTB_PCR2_VALUE_1;
    #endif
  #elif defined(PORTB_PCR2_VALUE_1)
  PORTB_PCR2 = PORTB_PCR2_VALUE_1;
  #endif

  /* Register 'PORTB_PCR3' initialization */
  #if PORTB_PCR3_MASK_1
    #if PORTB_PCR3_MASK_1 == 0xFFFFFFFF
  PORTB_PCR3 = PORTB_PCR3_VALUE_1;
    #elif PORTB_PCR3_MASK_1 == PORTB_PCR3_VALUE_1
  PORTB_PCR3 |= PORTB_PCR3_VALUE_1;
    #elif PORTB_PCR3_VALUE_1 == 0
  PORTB_PCR3 &= ~PORTB_PCR3_MASK_1;
    #else
  PORTB_PCR3 = (PORTB_PCR3 & (~PORTB_PCR3_MASK_1)) | PORTB_PCR3_VALUE_1;
    #endif
  #elif defined(PORTB_PCR3_VALUE_1)
  PORTB_PCR3 = PORTB_PCR3_VALUE_1;
  #endif

  /* Register 'PORTB_PCR9' initialization */
  #if PORTB_PCR9_MASK_1
    #if PORTB_PCR9_MASK_1 == 0xFFFFFFFF
  PORTB_PCR9 = PORTB_PCR9_VALUE_1;
    #elif PORTB_PCR9_MASK_1 == PORTB_PCR9_VALUE_1
  PORTB_PCR9 |= PORTB_PCR9_VALUE_1;
    #elif PORTB_PCR9_VALUE_1 == 0
  PORTB_PCR9 &= ~PORTB_PCR9_MASK_1;
    #else
  PORTB_PCR9 = (PORTB_PCR9 & (~PORTB_PCR9_MASK_1)) | PORTB_PCR9_VALUE_1;
    #endif
  #elif defined(PORTB_PCR9_VALUE_1)
  PORTB_PCR9 = PORTB_PCR9_VALUE_1;
  #endif

  /* Register 'PORTB_PCR10' initialization */
  #if PORTB_PCR10_MASK_1
    #if PORTB_PCR10_MASK_1 == 0xFFFFFFFF
  PORTB_PCR10 = PORTB_PCR10_VALUE_1;
    #elif PORTB_PCR10_MASK_1 == PORTB_PCR10_VALUE_1
  PORTB_PCR10 |= PORTB_PCR10_VALUE_1;
    #elif PORTB_PCR10_VALUE_1 == 0
  PORTB_PCR10 &= ~PORTB_PCR10_MASK_1;
    #else
  PORTB_PCR10 = (PORTB_PCR10 & (~PORTB_PCR10_MASK_1)) | PORTB_PCR10_VALUE_1;
    #endif
  #elif defined(PORTB_PCR10_VALUE_1)
  PORTB_PCR10 = PORTB_PCR10_VALUE_1;
  #endif

  /* Register 'PORTB_PCR11' initialization */
  #if PORTB_PCR11_MASK_1
    #if PORTB_PCR11_MASK_1 == 0xFFFFFFFF
  PORTB_PCR11 = PORTB_PCR11_VALUE_1;
    #elif PORTB_PCR11_MASK_1 == PORTB_PCR11_VALUE_1
  PORTB_PCR11 |= PORTB_PCR11_VALUE_1;
    #elif PORTB_PCR11_VALUE_1 == 0
  PORTB_PCR11 &= ~PORTB_PCR11_MASK_1;
    #else
  PORTB_PCR11 = (PORTB_PCR11 & (~PORTB_PCR11_MASK_1)) | PORTB_PCR11_VALUE_1;
    #endif
  #elif defined(PORTB_PCR11_VALUE_1)
  PORTB_PCR11 = PORTB_PCR11_VALUE_1;
  #endif

  /* Register 'PORTB_PCR16' initialization */
  #if PORTB_PCR16_MASK_1
    #if PORTB_PCR16_MASK_1 == 0xFFFFFFFF
  PORTB_PCR16 = PORTB_PCR16_VALUE_1;
    #elif PORTB_PCR16_MASK_1 == PORTB_PCR16_VALUE_1
  PORTB_PCR16 |= PORTB_PCR16_VALUE_1;
    #elif PORTB_PCR16_VALUE_1 == 0
  PORTB_PCR16 &= ~PORTB_PCR16_MASK_1;
    #else
  PORTB_PCR16 = (PORTB_PCR16 & (~PORTB_PCR16_MASK_1)) | PORTB_PCR16_VALUE_1;
    #endif
  #elif defined(PORTB_PCR16_VALUE_1)
  PORTB_PCR16 = PORTB_PCR16_VALUE_1;
  #endif

  /* Register 'PORTB_PCR17' initialization */
  #if PORTB_PCR17_MASK_1
    #if PORTB_PCR17_MASK_1 == 0xFFFFFFFF
  PORTB_PCR17 = PORTB_PCR17_VALUE_1;
    #elif PORTB_PCR17_MASK_1 == PORTB_PCR17_VALUE_1
  PORTB_PCR17 |= PORTB_PCR17_VALUE_1;
    #elif PORTB_PCR17_VALUE_1 == 0
  PORTB_PCR17 &= ~PORTB_PCR17_MASK_1;
    #else
  PORTB_PCR17 = (PORTB_PCR17 & (~PORTB_PCR17_MASK_1)) | PORTB_PCR17_VALUE_1;
    #endif
  #elif defined(PORTB_PCR17_VALUE_1)
  PORTB_PCR17 = PORTB_PCR17_VALUE_1;
  #endif

  /* Register 'PORTB_PCR18' initialization */
  #if PORTB_PCR18_MASK_1
    #if PORTB_PCR18_MASK_1 == 0xFFFFFFFF
  PORTB_PCR18 = PORTB_PCR18_VALUE_1;
    #elif PORTB_PCR18_MASK_1 == PORTB_PCR18_VALUE_1
  PORTB_PCR18 |= PORTB_PCR18_VALUE_1;
    #elif PORTB_PCR18_VALUE_1 == 0
  PORTB_PCR18 &= ~PORTB_PCR18_MASK_1;
    #else
  PORTB_PCR18 = (PORTB_PCR18 & (~PORTB_PCR18_MASK_1)) | PORTB_PCR18_VALUE_1;
    #endif
  #elif defined(PORTB_PCR18_VALUE_1)
  PORTB_PCR18 = PORTB_PCR18_VALUE_1;
  #endif

  /* Register 'PORTB_PCR19' initialization */
  #if PORTB_PCR19_MASK_1
    #if PORTB_PCR19_MASK_1 == 0xFFFFFFFF
  PORTB_PCR19 = PORTB_PCR19_VALUE_1;
    #elif PORTB_PCR19_MASK_1 == PORTB_PCR19_VALUE_1
  PORTB_PCR19 |= PORTB_PCR19_VALUE_1;
    #elif PORTB_PCR19_VALUE_1 == 0
  PORTB_PCR19 &= ~PORTB_PCR19_MASK_1;
    #else
  PORTB_PCR19 = (PORTB_PCR19 & (~PORTB_PCR19_MASK_1)) | PORTB_PCR19_VALUE_1;
    #endif
  #elif defined(PORTB_PCR19_VALUE_1)
  PORTB_PCR19 = PORTB_PCR19_VALUE_1;
  #endif

  /* Register 'PORTB_PCR20' initialization */
  #if PORTB_PCR20_MASK_1
    #if PORTB_PCR20_MASK_1 == 0xFFFFFFFF
  PORTB_PCR20 = PORTB_PCR20_VALUE_1;
    #elif PORTB_PCR20_MASK_1 == PORTB_PCR20_VALUE_1
  PORTB_PCR20 |= PORTB_PCR20_VALUE_1;
    #elif PORTB_PCR20_VALUE_1 == 0
  PORTB_PCR20 &= ~PORTB_PCR20_MASK_1;
    #else
  PORTB_PCR20 = (PORTB_PCR20 & (~PORTB_PCR20_MASK_1)) | PORTB_PCR20_VALUE_1;
    #endif
  #elif defined(PORTB_PCR20_VALUE_1)
  PORTB_PCR20 = PORTB_PCR20_VALUE_1;
  #endif

  /* Register 'PORTB_PCR21' initialization */
  #if PORTB_PCR21_MASK_1
    #if PORTB_PCR21_MASK_1 == 0xFFFFFFFF
  PORTB_PCR21 = PORTB_PCR21_VALUE_1;
    #elif PORTB_PCR21_MASK_1 == PORTB_PCR21_VALUE_1
  PORTB_PCR21 |= PORTB_PCR21_VALUE_1;
    #elif PORTB_PCR21_VALUE_1 == 0
  PORTB_PCR21 &= ~PORTB_PCR21_MASK_1;
    #else
  PORTB_PCR21 = (PORTB_PCR21 & (~PORTB_PCR21_MASK_1)) | PORTB_PCR21_VALUE_1;
    #endif
  #elif defined(PORTB_PCR21_VALUE_1)
  PORTB_PCR21 = PORTB_PCR21_VALUE_1;
  #endif

  /* Register 'PORTB_PCR22' initialization */
  #if PORTB_PCR22_MASK_1
    #if PORTB_PCR22_MASK_1 == 0xFFFFFFFF
  PORTB_PCR22 = PORTB_PCR22_VALUE_1;
    #elif PORTB_PCR22_MASK_1 == PORTB_PCR22_VALUE_1
  PORTB_PCR22 |= PORTB_PCR22_VALUE_1;
    #elif PORTB_PCR22_VALUE_1 == 0
  PORTB_PCR22 &= ~PORTB_PCR22_MASK_1;
    #else
  PORTB_PCR22 = (PORTB_PCR22 & (~PORTB_PCR22_MASK_1)) | PORTB_PCR22_VALUE_1;
    #endif
  #elif defined(PORTB_PCR22_VALUE_1)
  PORTB_PCR22 = PORTB_PCR22_VALUE_1;
  #endif

  /* Register 'PORTB_PCR23' initialization */
  #if PORTB_PCR23_MASK_1
    #if PORTB_PCR23_MASK_1 == 0xFFFFFFFF
  PORTB_PCR23 = PORTB_PCR23_VALUE_1;
    #elif PORTB_PCR23_MASK_1 == PORTB_PCR23_VALUE_1
  PORTB_PCR23 |= PORTB_PCR23_VALUE_1;
    #elif PORTB_PCR23_VALUE_1 == 0
  PORTB_PCR23 &= ~PORTB_PCR23_MASK_1;
    #else
  PORTB_PCR23 = (PORTB_PCR23 & (~PORTB_PCR23_MASK_1)) | PORTB_PCR23_VALUE_1;
    #endif
  #elif defined(PORTB_PCR23_VALUE_1)
  PORTB_PCR23 = PORTB_PCR23_VALUE_1;
  #endif

  /* Register 'PORTB_PCR0' initialization */
  #if PORTB_PCR0_MASK_2
    #if PORTB_PCR0_MASK_2 == 0xFFFFFFFF
  PORTB_PCR0 = PORTB_PCR0_VALUE_2;
    #elif PORTB_PCR0_MASK_2 == PORTB_PCR0_VALUE_2
  PORTB_PCR0 |= PORTB_PCR0_VALUE_2;
    #elif PORTB_PCR0_VALUE_2 == 0
  PORTB_PCR0 &= ~PORTB_PCR0_MASK_2;
    #else
  PORTB_PCR0 = (PORTB_PCR0 & (~PORTB_PCR0_MASK_2)) | PORTB_PCR0_VALUE_2;
    #endif
  #elif defined(PORTB_PCR0_VALUE_2)
  PORTB_PCR0 = PORTB_PCR0_VALUE_2;
  #endif

  /* Register 'PORTB_PCR1' initialization */
  #if PORTB_PCR1_MASK_2
    #if PORTB_PCR1_MASK_2 == 0xFFFFFFFF
  PORTB_PCR1 = PORTB_PCR1_VALUE_2;
    #elif PORTB_PCR1_MASK_2 == PORTB_PCR1_VALUE_2
  PORTB_PCR1 |= PORTB_PCR1_VALUE_2;
    #elif PORTB_PCR1_VALUE_2 == 0
  PORTB_PCR1 &= ~PORTB_PCR1_MASK_2;
    #else
  PORTB_PCR1 = (PORTB_PCR1 & (~PORTB_PCR1_MASK_2)) | PORTB_PCR1_VALUE_2;
    #endif
  #elif defined(PORTB_PCR1_VALUE_2)
  PORTB_PCR1 = PORTB_PCR1_VALUE_2;
  #endif

  /* Register 'PORTB_PCR2' initialization */
  #if PORTB_PCR2_MASK_2
    #if PORTB_PCR2_MASK_2 == 0xFFFFFFFF
  PORTB_PCR2 = PORTB_PCR2_VALUE_2;
    #elif PORTB_PCR2_MASK_2 == PORTB_PCR2_VALUE_2
  PORTB_PCR2 |= PORTB_PCR2_VALUE_2;
    #elif PORTB_PCR2_VALUE_2 == 0
  PORTB_PCR2 &= ~PORTB_PCR2_MASK_2;
    #else
  PORTB_PCR2 = (PORTB_PCR2 & (~PORTB_PCR2_MASK_2)) | PORTB_PCR2_VALUE_2;
    #endif
  #elif defined(PORTB_PCR2_VALUE_2)
  PORTB_PCR2 = PORTB_PCR2_VALUE_2;
  #endif

  /* Register 'PORTB_PCR3' initialization */
  #if PORTB_PCR3_MASK_2
    #if PORTB_PCR3_MASK_2 == 0xFFFFFFFF
  PORTB_PCR3 = PORTB_PCR3_VALUE_2;
    #elif PORTB_PCR3_MASK_2 == PORTB_PCR3_VALUE_2
  PORTB_PCR3 |= PORTB_PCR3_VALUE_2;
    #elif PORTB_PCR3_VALUE_2 == 0
  PORTB_PCR3 &= ~PORTB_PCR3_MASK_2;
    #else
  PORTB_PCR3 = (PORTB_PCR3 & (~PORTB_PCR3_MASK_2)) | PORTB_PCR3_VALUE_2;
    #endif
  #elif defined(PORTB_PCR3_VALUE_2)
  PORTB_PCR3 = PORTB_PCR3_VALUE_2;
  #endif

  /* Register 'PORTB_PCR9' initialization */
  #if PORTB_PCR9_MASK_2
    #if PORTB_PCR9_MASK_2 == 0xFFFFFFFF
  PORTB_PCR9 = PORTB_PCR9_VALUE_2;
    #elif PORTB_PCR9_MASK_2 == PORTB_PCR9_VALUE_2
  PORTB_PCR9 |= PORTB_PCR9_VALUE_2;
    #elif PORTB_PCR9_VALUE_2 == 0
  PORTB_PCR9 &= ~PORTB_PCR9_MASK_2;
    #else
  PORTB_PCR9 = (PORTB_PCR9 & (~PORTB_PCR9_MASK_2)) | PORTB_PCR9_VALUE_2;
    #endif
  #elif defined(PORTB_PCR9_VALUE_2)
  PORTB_PCR9 = PORTB_PCR9_VALUE_2;
  #endif

  /* Register 'PORTB_PCR10' initialization */
  #if PORTB_PCR10_MASK_2
    #if PORTB_PCR10_MASK_2 == 0xFFFFFFFF
  PORTB_PCR10 = PORTB_PCR10_VALUE_2;
    #elif PORTB_PCR10_MASK_2 == PORTB_PCR10_VALUE_2
  PORTB_PCR10 |= PORTB_PCR10_VALUE_2;
    #elif PORTB_PCR10_VALUE_2 == 0
  PORTB_PCR10 &= ~PORTB_PCR10_MASK_2;
    #else
  PORTB_PCR10 = (PORTB_PCR10 & (~PORTB_PCR10_MASK_2)) | PORTB_PCR10_VALUE_2;
    #endif
  #elif defined(PORTB_PCR10_VALUE_2)
  PORTB_PCR10 = PORTB_PCR10_VALUE_2;
  #endif

  /* Register 'PORTB_PCR11' initialization */
  #if PORTB_PCR11_MASK_2
    #if PORTB_PCR11_MASK_2 == 0xFFFFFFFF
  PORTB_PCR11 = PORTB_PCR11_VALUE_2;
    #elif PORTB_PCR11_MASK_2 == PORTB_PCR11_VALUE_2
  PORTB_PCR11 |= PORTB_PCR11_VALUE_2;
    #elif PORTB_PCR11_VALUE_2 == 0
  PORTB_PCR11 &= ~PORTB_PCR11_MASK_2;
    #else
  PORTB_PCR11 = (PORTB_PCR11 & (~PORTB_PCR11_MASK_2)) | PORTB_PCR11_VALUE_2;
    #endif
  #elif defined(PORTB_PCR11_VALUE_2)
  PORTB_PCR11 = PORTB_PCR11_VALUE_2;
  #endif

  /* Register 'PORTB_PCR16' initialization */
  #if PORTB_PCR16_MASK_2
    #if PORTB_PCR16_MASK_2 == 0xFFFFFFFF
  PORTB_PCR16 = PORTB_PCR16_VALUE_2;
    #elif PORTB_PCR16_MASK_2 == PORTB_PCR16_VALUE_2
  PORTB_PCR16 |= PORTB_PCR16_VALUE_2;
    #elif PORTB_PCR16_VALUE_2 == 0
  PORTB_PCR16 &= ~PORTB_PCR16_MASK_2;
    #else
  PORTB_PCR16 = (PORTB_PCR16 & (~PORTB_PCR16_MASK_2)) | PORTB_PCR16_VALUE_2;
    #endif
  #elif defined(PORTB_PCR16_VALUE_2)
  PORTB_PCR16 = PORTB_PCR16_VALUE_2;
  #endif

  /* Register 'PORTB_PCR17' initialization */
  #if PORTB_PCR17_MASK_2
    #if PORTB_PCR17_MASK_2 == 0xFFFFFFFF
  PORTB_PCR17 = PORTB_PCR17_VALUE_2;
    #elif PORTB_PCR17_MASK_2 == PORTB_PCR17_VALUE_2
  PORTB_PCR17 |= PORTB_PCR17_VALUE_2;
    #elif PORTB_PCR17_VALUE_2 == 0
  PORTB_PCR17 &= ~PORTB_PCR17_MASK_2;
    #else
  PORTB_PCR17 = (PORTB_PCR17 & (~PORTB_PCR17_MASK_2)) | PORTB_PCR17_VALUE_2;
    #endif
  #elif defined(PORTB_PCR17_VALUE_2)
  PORTB_PCR17 = PORTB_PCR17_VALUE_2;
  #endif

  /* Register 'PORTB_PCR18' initialization */
  #if PORTB_PCR18_MASK_2
    #if PORTB_PCR18_MASK_2 == 0xFFFFFFFF
  PORTB_PCR18 = PORTB_PCR18_VALUE_2;
    #elif PORTB_PCR18_MASK_2 == PORTB_PCR18_VALUE_2
  PORTB_PCR18 |= PORTB_PCR18_VALUE_2;
    #elif PORTB_PCR18_VALUE_2 == 0
  PORTB_PCR18 &= ~PORTB_PCR18_MASK_2;
    #else
  PORTB_PCR18 = (PORTB_PCR18 & (~PORTB_PCR18_MASK_2)) | PORTB_PCR18_VALUE_2;
    #endif
  #elif defined(PORTB_PCR18_VALUE_2)
  PORTB_PCR18 = PORTB_PCR18_VALUE_2;
  #endif

  /* Register 'PORTB_PCR19' initialization */
  #if PORTB_PCR19_MASK_2
    #if PORTB_PCR19_MASK_2 == 0xFFFFFFFF
  PORTB_PCR19 = PORTB_PCR19_VALUE_2;
    #elif PORTB_PCR19_MASK_2 == PORTB_PCR19_VALUE_2
  PORTB_PCR19 |= PORTB_PCR19_VALUE_2;
    #elif PORTB_PCR19_VALUE_2 == 0
  PORTB_PCR19 &= ~PORTB_PCR19_MASK_2;
    #else
  PORTB_PCR19 = (PORTB_PCR19 & (~PORTB_PCR19_MASK_2)) | PORTB_PCR19_VALUE_2;
    #endif
  #elif defined(PORTB_PCR19_VALUE_2)
  PORTB_PCR19 = PORTB_PCR19_VALUE_2;
  #endif

  /* Register 'PORTB_PCR20' initialization */
  #if PORTB_PCR20_MASK_2
    #if PORTB_PCR20_MASK_2 == 0xFFFFFFFF
  PORTB_PCR20 = PORTB_PCR20_VALUE_2;
    #elif PORTB_PCR20_MASK_2 == PORTB_PCR20_VALUE_2
  PORTB_PCR20 |= PORTB_PCR20_VALUE_2;
    #elif PORTB_PCR20_VALUE_2 == 0
  PORTB_PCR20 &= ~PORTB_PCR20_MASK_2;
    #else
  PORTB_PCR20 = (PORTB_PCR20 & (~PORTB_PCR20_MASK_2)) | PORTB_PCR20_VALUE_2;
    #endif
  #elif defined(PORTB_PCR20_VALUE_2)
  PORTB_PCR20 = PORTB_PCR20_VALUE_2;
  #endif

  /* Register 'PORTB_PCR21' initialization */
  #if PORTB_PCR21_MASK_2
    #if PORTB_PCR21_MASK_2 == 0xFFFFFFFF
  PORTB_PCR21 = PORTB_PCR21_VALUE_2;
    #elif PORTB_PCR21_MASK_2 == PORTB_PCR21_VALUE_2
  PORTB_PCR21 |= PORTB_PCR21_VALUE_2;
    #elif PORTB_PCR21_VALUE_2 == 0
  PORTB_PCR21 &= ~PORTB_PCR21_MASK_2;
    #else
  PORTB_PCR21 = (PORTB_PCR21 & (~PORTB_PCR21_MASK_2)) | PORTB_PCR21_VALUE_2;
    #endif
  #elif defined(PORTB_PCR21_VALUE_2)
  PORTB_PCR21 = PORTB_PCR21_VALUE_2;
  #endif

  /* Register 'PORTB_PCR22' initialization */
  #if PORTB_PCR22_MASK_2
    #if PORTB_PCR22_MASK_2 == 0xFFFFFFFF
  PORTB_PCR22 = PORTB_PCR22_VALUE_2;
    #elif PORTB_PCR22_MASK_2 == PORTB_PCR22_VALUE_2
  PORTB_PCR22 |= PORTB_PCR22_VALUE_2;
    #elif PORTB_PCR22_VALUE_2 == 0
  PORTB_PCR22 &= ~PORTB_PCR22_MASK_2;
    #else
  PORTB_PCR22 = (PORTB_PCR22 & (~PORTB_PCR22_MASK_2)) | PORTB_PCR22_VALUE_2;
    #endif
  #elif defined(PORTB_PCR22_VALUE_2)
  PORTB_PCR22 = PORTB_PCR22_VALUE_2;
  #endif

  /* Register 'PORTB_PCR23' initialization */
  #if PORTB_PCR23_MASK_2
    #if PORTB_PCR23_MASK_2 == 0xFFFFFFFF
  PORTB_PCR23 = PORTB_PCR23_VALUE_2;
    #elif PORTB_PCR23_MASK_2 == PORTB_PCR23_VALUE_2
  PORTB_PCR23 |= PORTB_PCR23_VALUE_2;
    #elif PORTB_PCR23_VALUE_2 == 0
  PORTB_PCR23 &= ~PORTB_PCR23_MASK_2;
    #else
  PORTB_PCR23 = (PORTB_PCR23 & (~PORTB_PCR23_MASK_2)) | PORTB_PCR23_VALUE_2;
    #endif
  #elif defined(PORTB_PCR23_VALUE_2)
  PORTB_PCR23 = PORTB_PCR23_VALUE_2;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END PORTB_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

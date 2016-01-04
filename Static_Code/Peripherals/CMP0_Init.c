/** ###################################################################
 **     Filename    : CMP0_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the CMP0 module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void CMP0_Init(void);
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
 * @file CMP0_Init.c
 * @brief This file implements the CMP0 module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE CMP0_Init. */

#include "CMP0_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void CMP0_Init(void) {

  CMP0_CR1 = 0;

  CMP0_CR0 = 0;

  CMP0_MUXCR = 0;

  /* Register 'CMP0_FPR' initialization */
  #ifdef CMP0_FPR_VALUE
  CMP0_FPR = CMP0_FPR_VALUE;
  #endif

  /* Register 'CMP0_CR1' initialization */
  #ifdef CMP0_CR1_VALUE_1
  CMP0_CR1 = CMP0_CR1_VALUE_1;
  #endif

  /* Register 'CMP0_SCR' initialization */
  #ifdef CMP0_SCR_VALUE
  CMP0_SCR = CMP0_SCR_VALUE;
  #endif

  /* Register 'CMP0_CR0' initialization */
  #ifdef CMP0_CR0_VALUE
  CMP0_CR0 = CMP0_CR0_VALUE;
  #endif

  /* Register 'CMP0_DACCR' initialization */
  #if CMP0_DACCR_MASK
    #if CMP0_DACCR_MASK == 0xFF
  CMP0_DACCR = CMP0_DACCR_VALUE;
    #elif CMP0_DACCR_MASK == CMP0_DACCR_VALUE
  CMP0_DACCR |= CMP0_DACCR_VALUE;
    #elif CMP0_DACCR_VALUE == 0
  CMP0_DACCR &= ~CMP0_DACCR_MASK;
    #else
  CMP0_DACCR = (CMP0_DACCR & (~CMP0_DACCR_MASK)) | CMP0_DACCR_VALUE;
    #endif
  #elif defined(CMP0_DACCR_VALUE)
  CMP0_DACCR = CMP0_DACCR_VALUE;
  #endif

  /* Register 'CMP0_MUXCR' initialization */
  #ifdef CMP0_MUXCR_VALUE
  CMP0_MUXCR = CMP0_MUXCR_VALUE;
  #endif

  /* Register 'CMP0_CR1' initialization */
  #if CMP0_CR1_MASK_2
    #if CMP0_CR1_MASK_2 == 0xFF
  CMP0_CR1 = CMP0_CR1_VALUE_2;
    #elif CMP0_CR1_MASK_2 == CMP0_CR1_VALUE_2
  CMP0_CR1 |= CMP0_CR1_VALUE_2;
    #elif CMP0_CR1_VALUE_2 == 0
  CMP0_CR1 &= ~CMP0_CR1_MASK_2;
    #else
  CMP0_CR1 = (CMP0_CR1 & (~CMP0_CR1_MASK_2)) | CMP0_CR1_VALUE_2;
    #endif
  #elif defined(CMP0_CR1_VALUE_2)
  CMP0_CR1 = CMP0_CR1_VALUE_2;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END CMP0_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

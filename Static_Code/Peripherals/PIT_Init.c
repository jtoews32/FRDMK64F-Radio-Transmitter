/** ###################################################################
 **     Filename    : PIT_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the PIT module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void PIT_Init(void);
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
 * @file PIT_Init.c
 * @brief This file implements the PIT module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE PIT_Init. */

#include "PIT_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void PIT_Init(void) {

  /* Register 'PIT_MCR' initialization */
  #ifdef PIT_MCR_VALUE_1
  PIT_MCR = PIT_MCR_VALUE_1;
  #endif

  /* Register 'PIT_TFLG0' initialization */
  #ifdef PIT_TFLG0_VALUE
  PIT_TFLG0 = PIT_TFLG0_VALUE;
  #endif

  /* Register 'PIT_TFLG1' initialization */
  #ifdef PIT_TFLG1_VALUE
  PIT_TFLG1 = PIT_TFLG1_VALUE;
  #endif

  /* Register 'PIT_TFLG2' initialization */
  #ifdef PIT_TFLG2_VALUE
  PIT_TFLG2 = PIT_TFLG2_VALUE;
  #endif

  /* Register 'PIT_TFLG3' initialization */
  #ifdef PIT_TFLG3_VALUE
  PIT_TFLG3 = PIT_TFLG3_VALUE;
  #endif

  /* Register 'PIT_TCTRL0' initialization */
  #if PIT_TCTRL0_MASK_1
    #if PIT_TCTRL0_MASK_1 == 0xFFFFFFFF
  PIT_TCTRL0 = PIT_TCTRL0_VALUE_1;
    #elif PIT_TCTRL0_MASK_1 == PIT_TCTRL0_VALUE_1
  PIT_TCTRL0 |= PIT_TCTRL0_VALUE_1;
    #elif PIT_TCTRL0_VALUE_1 == 0
  PIT_TCTRL0 &= ~PIT_TCTRL0_MASK_1;
    #else
  PIT_TCTRL0 = (PIT_TCTRL0 & (~PIT_TCTRL0_MASK_1)) | PIT_TCTRL0_VALUE_1;
    #endif
  #elif defined(PIT_TCTRL0_VALUE_1)
  PIT_TCTRL0 = PIT_TCTRL0_VALUE_1;
  #endif

  /* Register 'PIT_TCTRL1' initialization */
  #ifdef PIT_TCTRL1_VALUE_1
  PIT_TCTRL1 = PIT_TCTRL1_VALUE_1;
  #endif

  /* Register 'PIT_TCTRL2' initialization */
  #ifdef PIT_TCTRL2_VALUE_1
  PIT_TCTRL2 = PIT_TCTRL2_VALUE_1;
  #endif

  /* Register 'PIT_TCTRL3' initialization */
  #ifdef PIT_TCTRL3_VALUE_1
  PIT_TCTRL3 = PIT_TCTRL3_VALUE_1;
  #endif

  /* Register 'PIT_LDVAL0' initialization */
  #ifdef PIT_LDVAL0_VALUE
  PIT_LDVAL0 = PIT_LDVAL0_VALUE;
  #endif

  /* Register 'PIT_LDVAL1' initialization */
  #ifdef PIT_LDVAL1_VALUE
  PIT_LDVAL1 = PIT_LDVAL1_VALUE;
  #endif

  /* Register 'PIT_LDVAL2' initialization */
  #ifdef PIT_LDVAL2_VALUE
  PIT_LDVAL2 = PIT_LDVAL2_VALUE;
  #endif

  /* Register 'PIT_LDVAL3' initialization */
  #ifdef PIT_LDVAL3_VALUE
  PIT_LDVAL3 = PIT_LDVAL3_VALUE;
  #endif

  /* Register 'PIT_TCTRL0' initialization */
  #if PIT_TCTRL0_MASK_2
    #if PIT_TCTRL0_MASK_2 == 0xFFFFFFFF
  PIT_TCTRL0 = PIT_TCTRL0_VALUE_2;
    #elif PIT_TCTRL0_MASK_2 == PIT_TCTRL0_VALUE_2
  PIT_TCTRL0 |= PIT_TCTRL0_VALUE_2;
    #elif PIT_TCTRL0_VALUE_2 == 0
  PIT_TCTRL0 &= ~PIT_TCTRL0_MASK_2;
    #else
  PIT_TCTRL0 = (PIT_TCTRL0 & (~PIT_TCTRL0_MASK_2)) | PIT_TCTRL0_VALUE_2;
    #endif
  #elif defined(PIT_TCTRL0_VALUE_2)
  PIT_TCTRL0 = PIT_TCTRL0_VALUE_2;
  #endif

  /* Register 'PIT_TCTRL1' initialization */
  #if PIT_TCTRL1_MASK_2
    #if PIT_TCTRL1_MASK_2 == 0xFFFFFFFF
  PIT_TCTRL1 = PIT_TCTRL1_VALUE_2;
    #elif PIT_TCTRL1_MASK_2 == PIT_TCTRL1_VALUE_2
  PIT_TCTRL1 |= PIT_TCTRL1_VALUE_2;
    #elif PIT_TCTRL1_VALUE_2 == 0
  PIT_TCTRL1 &= ~PIT_TCTRL1_MASK_2;
    #else
  PIT_TCTRL1 = (PIT_TCTRL1 & (~PIT_TCTRL1_MASK_2)) | PIT_TCTRL1_VALUE_2;
    #endif
  #elif defined(PIT_TCTRL1_VALUE_2)
  PIT_TCTRL1 = PIT_TCTRL1_VALUE_2;
  #endif

  /* Register 'PIT_TCTRL2' initialization */
  #if PIT_TCTRL2_MASK_2
    #if PIT_TCTRL2_MASK_2 == 0xFFFFFFFF
  PIT_TCTRL2 = PIT_TCTRL2_VALUE_2;
    #elif PIT_TCTRL2_MASK_2 == PIT_TCTRL2_VALUE_2
  PIT_TCTRL2 |= PIT_TCTRL2_VALUE_2;
    #elif PIT_TCTRL2_VALUE_2 == 0
  PIT_TCTRL2 &= ~PIT_TCTRL2_MASK_2;
    #else
  PIT_TCTRL2 = (PIT_TCTRL2 & (~PIT_TCTRL2_MASK_2)) | PIT_TCTRL2_VALUE_2;
    #endif
  #elif defined(PIT_TCTRL2_VALUE_2)
  PIT_TCTRL2 = PIT_TCTRL2_VALUE_2;
  #endif

  /* Register 'PIT_TCTRL3' initialization */
  #if PIT_TCTRL3_MASK_2
    #if PIT_TCTRL3_MASK_2 == 0xFFFFFFFF
  PIT_TCTRL3 = PIT_TCTRL3_VALUE_2;
    #elif PIT_TCTRL3_MASK_2 == PIT_TCTRL3_VALUE_2
  PIT_TCTRL3 |= PIT_TCTRL3_VALUE_2;
    #elif PIT_TCTRL3_VALUE_2 == 0
  PIT_TCTRL3 &= ~PIT_TCTRL3_MASK_2;
    #else
  PIT_TCTRL3 = (PIT_TCTRL3 & (~PIT_TCTRL3_MASK_2)) | PIT_TCTRL3_VALUE_2;
    #endif
  #elif defined(PIT_TCTRL3_VALUE_2)
  PIT_TCTRL3 = PIT_TCTRL3_VALUE_2;
  #endif

  /* Register 'PIT_MCR' initialization */
  #if PIT_MCR_MASK_2
    #if PIT_MCR_MASK_2 == 0xFFFFFFFF
  PIT_MCR = PIT_MCR_VALUE_2;
    #elif PIT_MCR_MASK_2 == PIT_MCR_VALUE_2
  PIT_MCR |= PIT_MCR_VALUE_2;
    #elif PIT_MCR_VALUE_2 == 0
  PIT_MCR &= ~PIT_MCR_MASK_2;
    #else
  PIT_MCR = (PIT_MCR & (~PIT_MCR_MASK_2)) | PIT_MCR_VALUE_2;
    #endif
  #elif defined(PIT_MCR_VALUE_2)
  PIT_MCR = PIT_MCR_VALUE_2;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END PIT_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

/** ###################################################################
 **     Filename    : SystemControl_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the SystemControl module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void SystemControl_Init(void);
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
 * @file SystemControl_Init.c
 * @brief This file implements the SystemControl module initialization according
 *        to the Peripheral Initialization settings, and defines interrupt
 *        service routines prototypes.
 */


/* MODULE SystemControl_Init. */

#include "SystemControl_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void SystemControl_Init(void) {

  /* Register 'SCB_ACTLR' initialization */
  #if SCB_ACTLR_MASK_1
    #if SCB_ACTLR_MASK_1 == 0xFFFFFFFF
  SCB_ACTLR = SCB_ACTLR_VALUE_1;
    #elif SCB_ACTLR_MASK_1 == SCB_ACTLR_VALUE_1
  SCB_ACTLR |= SCB_ACTLR_VALUE_1;
    #elif SCB_ACTLR_VALUE_1 == 0
  SCB_ACTLR &= ~SCB_ACTLR_MASK_1;
    #else
  SCB_ACTLR = (SCB_ACTLR & (~SCB_ACTLR_MASK_1)) | SCB_ACTLR_VALUE_1;
    #endif
  #elif defined(SCB_ACTLR_VALUE_1)
  SCB_ACTLR = SCB_ACTLR_VALUE_1;
  #endif

  /* Register 'SCB_CCR' initialization */
  #if SCB_CCR_MASK
    #if SCB_CCR_MASK == 0xFFFFFFFF
  SCB_CCR = SCB_CCR_VALUE;
    #elif SCB_CCR_MASK == SCB_CCR_VALUE
  SCB_CCR |= SCB_CCR_VALUE;
    #elif SCB_CCR_VALUE == 0
  SCB_CCR &= ~SCB_CCR_MASK;
    #else
  SCB_CCR = (SCB_CCR & (~SCB_CCR_MASK)) | SCB_CCR_VALUE;
    #endif
  #elif defined(SCB_CCR_VALUE)
  SCB_CCR = SCB_CCR_VALUE;
  #endif

  /* Register 'SCB_SCR' initialization */
  #if SCB_SCR_MASK
    #if SCB_SCR_MASK == 0xFFFFFFFF
  SCB_SCR = SCB_SCR_VALUE;
    #elif SCB_SCR_MASK == SCB_SCR_VALUE
  SCB_SCR |= SCB_SCR_VALUE;
    #elif SCB_SCR_VALUE == 0
  SCB_SCR &= ~SCB_SCR_MASK;
    #else
  SCB_SCR = (SCB_SCR & (~SCB_SCR_MASK)) | SCB_SCR_VALUE;
    #endif
  #elif defined(SCB_SCR_VALUE)
  SCB_SCR = SCB_SCR_VALUE;
  #endif

  /* Register 'SCB_VTOR' initialization */
  #if SCB_VTOR_MASK
    #if SCB_VTOR_MASK == 0xFFFFFFFF
  SCB_VTOR = SCB_VTOR_VALUE;
    #elif SCB_VTOR_MASK == SCB_VTOR_VALUE
  SCB_VTOR |= SCB_VTOR_VALUE;
    #elif SCB_VTOR_VALUE == 0
  SCB_VTOR &= ~SCB_VTOR_MASK;
    #else
  SCB_VTOR = (SCB_VTOR & (~SCB_VTOR_MASK)) | SCB_VTOR_VALUE;
    #endif
  #elif defined(SCB_VTOR_VALUE)
  SCB_VTOR = SCB_VTOR_VALUE;
  #endif

  /* Register 'SCB_ACTLR' initialization */
  #if SCB_ACTLR_MASK_2
    #if SCB_ACTLR_MASK_2 == 0xFFFFFFFF
  SCB_ACTLR = SCB_ACTLR_VALUE_2;
    #elif SCB_ACTLR_MASK_2 == SCB_ACTLR_VALUE_2
  SCB_ACTLR |= SCB_ACTLR_VALUE_2;
    #elif SCB_ACTLR_VALUE_2 == 0
  SCB_ACTLR &= ~SCB_ACTLR_MASK_2;
    #else
  SCB_ACTLR = (SCB_ACTLR & (~SCB_ACTLR_MASK_2)) | SCB_ACTLR_VALUE_2;
    #endif
  #elif defined(SCB_ACTLR_VALUE_2)
  SCB_ACTLR = SCB_ACTLR_VALUE_2;
  #endif

  /* Register 'SCB_AIRCR' initialization */
  #ifdef SCB_AIRCR_VALUE
  SCB_AIRCR = SCB_AIRCR_VALUE;
  #endif

  /* Register 'SCB_SHPR1' initialization */
  #if SCB_SHPR1_MASK
    #if SCB_SHPR1_MASK == 0xFFFFFFFF
  SCB_SHPR1 = SCB_SHPR1_VALUE;
    #elif SCB_SHPR1_MASK == SCB_SHPR1_VALUE
  SCB_SHPR1 |= SCB_SHPR1_VALUE;
    #elif SCB_SHPR1_VALUE == 0
  SCB_SHPR1 &= ~SCB_SHPR1_MASK;
    #else
  SCB_SHPR1 = (SCB_SHPR1 & (~SCB_SHPR1_MASK)) | SCB_SHPR1_VALUE;
    #endif
  #elif defined(SCB_SHPR1_VALUE)
  SCB_SHPR1 = SCB_SHPR1_VALUE;
  #endif

  /* Register 'SCB_SHPR2' initialization */
  #if SCB_SHPR2_MASK
    #if SCB_SHPR2_MASK == 0xFFFFFFFF
  SCB_SHPR2 = SCB_SHPR2_VALUE;
    #elif SCB_SHPR2_MASK == SCB_SHPR2_VALUE
  SCB_SHPR2 |= SCB_SHPR2_VALUE;
    #elif SCB_SHPR2_VALUE == 0
  SCB_SHPR2 &= ~SCB_SHPR2_MASK;
    #else
  SCB_SHPR2 = (SCB_SHPR2 & (~SCB_SHPR2_MASK)) | SCB_SHPR2_VALUE;
    #endif
  #elif defined(SCB_SHPR2_VALUE)
  SCB_SHPR2 = SCB_SHPR2_VALUE;
  #endif

  /* Register 'SCB_SHPR3' initialization */
  #if SCB_SHPR3_MASK
    #if SCB_SHPR3_MASK == 0xFFFFFFFF
  SCB_SHPR3 = SCB_SHPR3_VALUE;
    #elif SCB_SHPR3_MASK == SCB_SHPR3_VALUE
  SCB_SHPR3 |= SCB_SHPR3_VALUE;
    #elif SCB_SHPR3_VALUE == 0
  SCB_SHPR3 &= ~SCB_SHPR3_MASK;
    #else
  SCB_SHPR3 = (SCB_SHPR3 & (~SCB_SHPR3_MASK)) | SCB_SHPR3_VALUE;
    #endif
  #elif defined(SCB_SHPR3_VALUE)
  SCB_SHPR3 = SCB_SHPR3_VALUE;
  #endif

  /* Register 'SCB_ICSR' initialization */
  #if SCB_ICSR_MASK
    #if SCB_ICSR_MASK == 0xFFFFFFFF
  SCB_ICSR = SCB_ICSR_VALUE;
    #elif SCB_ICSR_MASK == SCB_ICSR_VALUE
  SCB_ICSR |= SCB_ICSR_VALUE;
    #elif SCB_ICSR_VALUE == 0
  SCB_ICSR &= ~SCB_ICSR_MASK;
    #else
  SCB_ICSR = (SCB_ICSR & (~SCB_ICSR_MASK)) | SCB_ICSR_VALUE;
    #endif
  #elif defined(SCB_ICSR_VALUE)
  SCB_ICSR = SCB_ICSR_VALUE;
  #endif

  /* Register 'SCB_SHCSR' initialization */
  #if SCB_SHCSR_MASK
    #if SCB_SHCSR_MASK == 0xFFFFFFFF
  SCB_SHCSR = SCB_SHCSR_VALUE;
    #elif SCB_SHCSR_MASK == SCB_SHCSR_VALUE
  SCB_SHCSR |= SCB_SHCSR_VALUE;
    #elif SCB_SHCSR_VALUE == 0
  SCB_SHCSR &= ~SCB_SHCSR_MASK;
    #else
  SCB_SHCSR = (SCB_SHCSR & (~SCB_SHCSR_MASK)) | SCB_SHCSR_VALUE;
    #endif
  #elif defined(SCB_SHCSR_VALUE)
  SCB_SHCSR = SCB_SHCSR_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END SystemControl_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

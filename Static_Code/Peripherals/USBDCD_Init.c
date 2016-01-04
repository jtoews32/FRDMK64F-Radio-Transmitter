/** ###################################################################
 **     Filename    : USBDCD_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the USBDCD module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void USBDCD_Init(void);
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
 * @file USBDCD_Init.c
 * @brief This file implements the USBDCD module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE USBDCD_Init. */

#include "USBDCD_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void USBDCD_Init(void) {

  /* Register 'USBDCD_CONTROL' initialization */
  #if USBDCD_CONTROL_MASK_1
    #if USBDCD_CONTROL_MASK_1 == 0xFFFFFFFF
  USBDCD_CONTROL = USBDCD_CONTROL_VALUE_1;
    #elif USBDCD_CONTROL_MASK_1 == USBDCD_CONTROL_VALUE_1
  USBDCD_CONTROL |= USBDCD_CONTROL_VALUE_1;
    #elif USBDCD_CONTROL_VALUE_1 == 0
  USBDCD_CONTROL &= ~USBDCD_CONTROL_MASK_1;
    #else
  USBDCD_CONTROL = (USBDCD_CONTROL & (~USBDCD_CONTROL_MASK_1)) | USBDCD_CONTROL_VALUE_1;
    #endif
  #elif defined(USBDCD_CONTROL_VALUE_1)
  USBDCD_CONTROL = USBDCD_CONTROL_VALUE_1;
  #endif

  /* Register 'USBDCD_CONTROL' initialization */
  #if USBDCD_CONTROL_MASK_2
    #if USBDCD_CONTROL_MASK_2 == 0xFFFFFFFF
  USBDCD_CONTROL = USBDCD_CONTROL_VALUE_2;
    #elif USBDCD_CONTROL_MASK_2 == USBDCD_CONTROL_VALUE_2
  USBDCD_CONTROL |= USBDCD_CONTROL_VALUE_2;
    #elif USBDCD_CONTROL_VALUE_2 == 0
  USBDCD_CONTROL &= ~USBDCD_CONTROL_MASK_2;
    #else
  USBDCD_CONTROL = (USBDCD_CONTROL & (~USBDCD_CONTROL_MASK_2)) | USBDCD_CONTROL_VALUE_2;
    #endif
  #elif defined(USBDCD_CONTROL_VALUE_2)
  USBDCD_CONTROL = USBDCD_CONTROL_VALUE_2;
  #endif

  /* Register 'USBDCD_CLOCK' initialization */
  #ifdef USBDCD_CLOCK_VALUE
  USBDCD_CLOCK = USBDCD_CLOCK_VALUE;
  #endif

  /* Register 'USBDCD_TIMER0' initialization */
  #ifdef USBDCD_TIMER0_VALUE
  USBDCD_TIMER0 = USBDCD_TIMER0_VALUE;
  #endif

  /* Register 'USBDCD_TIMER1' initialization */
  #ifdef USBDCD_TIMER1_VALUE
  USBDCD_TIMER1 = USBDCD_TIMER1_VALUE;
  #endif

  /* Register 'USBDCD_TIMER2_BC11' initialization */
  #if USBDCD_TIMER2_BC11_MASK
    #if USBDCD_TIMER2_BC11_MASK == 0xFFFFFFFF
  USBDCD_TIMER2_BC11 = USBDCD_TIMER2_BC11_VALUE;
    #elif USBDCD_TIMER2_BC11_MASK == USBDCD_TIMER2_BC11_VALUE
  USBDCD_TIMER2_BC11 |= USBDCD_TIMER2_BC11_VALUE;
    #elif USBDCD_TIMER2_BC11_VALUE == 0
  USBDCD_TIMER2_BC11 &= ~USBDCD_TIMER2_BC11_MASK;
    #else
  USBDCD_TIMER2_BC11 = (USBDCD_TIMER2_BC11 & (~USBDCD_TIMER2_BC11_MASK)) | USBDCD_TIMER2_BC11_VALUE;
    #endif
  #elif defined(USBDCD_TIMER2_BC11_VALUE)
  USBDCD_TIMER2_BC11 = USBDCD_TIMER2_BC11_VALUE;
  #endif

  /* Register 'USBDCD_TIMER2_BC12' initialization */
  #if USBDCD_TIMER2_BC12_MASK
    #if USBDCD_TIMER2_BC12_MASK == 0xFFFFFFFF
  USBDCD_TIMER2_BC12 = USBDCD_TIMER2_BC12_VALUE;
    #elif USBDCD_TIMER2_BC12_MASK == USBDCD_TIMER2_BC12_VALUE
  USBDCD_TIMER2_BC12 |= USBDCD_TIMER2_BC12_VALUE;
    #elif USBDCD_TIMER2_BC12_VALUE == 0
  USBDCD_TIMER2_BC12 &= ~USBDCD_TIMER2_BC12_MASK;
    #else
  USBDCD_TIMER2_BC12 = (USBDCD_TIMER2_BC12 & (~USBDCD_TIMER2_BC12_MASK)) | USBDCD_TIMER2_BC12_VALUE;
    #endif
  #elif defined(USBDCD_TIMER2_BC12_VALUE)
  USBDCD_TIMER2_BC12 = USBDCD_TIMER2_BC12_VALUE;
  #endif

  /* Register 'USBDCD_CONTROL' initialization */
  #if USBDCD_CONTROL_MASK_3
    #if USBDCD_CONTROL_MASK_3 == 0xFFFFFFFF
  USBDCD_CONTROL = USBDCD_CONTROL_VALUE_3;
    #elif USBDCD_CONTROL_MASK_3 == USBDCD_CONTROL_VALUE_3
  USBDCD_CONTROL |= USBDCD_CONTROL_VALUE_3;
    #elif USBDCD_CONTROL_VALUE_3 == 0
  USBDCD_CONTROL &= ~USBDCD_CONTROL_MASK_3;
    #else
  USBDCD_CONTROL = (USBDCD_CONTROL & (~USBDCD_CONTROL_MASK_3)) | USBDCD_CONTROL_VALUE_3;
    #endif
  #elif defined(USBDCD_CONTROL_VALUE_3)
  USBDCD_CONTROL = USBDCD_CONTROL_VALUE_3;
  #endif

  /* Register 'USBDCD_CONTROL' initialization */
  #if USBDCD_CONTROL_MASK_4
    #if USBDCD_CONTROL_MASK_4 == 0xFFFFFFFF
  USBDCD_CONTROL = USBDCD_CONTROL_VALUE_4;
    #elif USBDCD_CONTROL_MASK_4 == USBDCD_CONTROL_VALUE_4
  USBDCD_CONTROL |= USBDCD_CONTROL_VALUE_4;
    #elif USBDCD_CONTROL_VALUE_4 == 0
  USBDCD_CONTROL &= ~USBDCD_CONTROL_MASK_4;
    #else
  USBDCD_CONTROL = (USBDCD_CONTROL & (~USBDCD_CONTROL_MASK_4)) | USBDCD_CONTROL_VALUE_4;
    #endif
  #elif defined(USBDCD_CONTROL_VALUE_4)
  USBDCD_CONTROL = USBDCD_CONTROL_VALUE_4;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END USBDCD_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

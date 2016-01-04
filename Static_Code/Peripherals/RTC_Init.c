/** ###################################################################
 **     Filename    : RTC_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the RTC module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void RTC_Init(void);
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
 * @file RTC_Init.c
 * @brief This file implements the RTC module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE RTC_Init. */

#include "RTC_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void RTC_Init(void) {

  RTC_SR = 0;

  /* Register 'RTC_CR' initialization */
  #ifdef RTC_CR_VALUE
  RTC_CR = RTC_CR_VALUE;
  #endif

  /* Register 'RTC_TPR' initialization */
  #ifdef RTC_TPR_VALUE
  RTC_TPR = RTC_TPR_VALUE;
  #endif

  /* Register 'RTC_TSR' initialization */
  #ifdef RTC_TSR_VALUE
  RTC_TSR = RTC_TSR_VALUE;
  #endif

  /* Register 'RTC_TAR' initialization */
  #ifdef RTC_TAR_VALUE
  RTC_TAR = RTC_TAR_VALUE;
  #endif

  /* Register 'RTC_TCR' initialization */
  #ifdef RTC_TCR_VALUE
  RTC_TCR = RTC_TCR_VALUE;
  #endif

  /* Register 'RTC_IER' initialization */
  #if RTC_IER_MASK
    #if RTC_IER_MASK == 0xFFFFFFFF
  RTC_IER = RTC_IER_VALUE;
    #elif RTC_IER_MASK == RTC_IER_VALUE
  RTC_IER |= RTC_IER_VALUE;
    #elif RTC_IER_VALUE == 0
  RTC_IER &= ~RTC_IER_MASK;
    #else
  RTC_IER = (RTC_IER & (~RTC_IER_MASK)) | RTC_IER_VALUE;
    #endif
  #elif defined(RTC_IER_VALUE)
  RTC_IER = RTC_IER_VALUE;
  #endif

  /* Register 'RTC_SR' initialization */
  #ifdef RTC_SR_VALUE
  RTC_SR = RTC_SR_VALUE;
  #endif

  /* Register 'RTC_RAR' initialization */
  #ifdef RTC_RAR_VALUE
  RTC_RAR = RTC_RAR_VALUE;
  #endif

  /* Register 'RTC_WAR' initialization */
  #ifdef RTC_WAR_VALUE
  RTC_WAR = RTC_WAR_VALUE;
  #endif

  /* Register 'RTC_LR' initialization */
  #ifdef RTC_LR_VALUE
  RTC_LR = RTC_LR_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END RTC_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

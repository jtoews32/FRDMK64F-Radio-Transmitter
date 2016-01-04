/** ###################################################################
 **     Filename    : WDOG_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the WDOG module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void WDOG_Init(void);
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
 * @file WDOG_Init.c
 * @brief This file implements the WDOG module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE WDOG_Init. */

#include "WDOG_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void WDOG_Init(void) {

  WDOG_UNLOCK = 0xC520;

  WDOG_UNLOCK = 0xD928;

  /* Register 'WDOG_STCTRLL' initialization */
  #if WDOG_STCTRLL_MASK
    #if WDOG_STCTRLL_MASK == 0xFFFF
  WDOG_STCTRLL = WDOG_STCTRLL_VALUE;
    #elif WDOG_STCTRLL_MASK == WDOG_STCTRLL_VALUE
  WDOG_STCTRLL |= WDOG_STCTRLL_VALUE;
    #elif WDOG_STCTRLL_VALUE == 0
  WDOG_STCTRLL &= ~WDOG_STCTRLL_MASK;
    #else
  WDOG_STCTRLL = (WDOG_STCTRLL & (~WDOG_STCTRLL_MASK)) | WDOG_STCTRLL_VALUE;
    #endif
  #elif defined(WDOG_STCTRLL_VALUE)
  WDOG_STCTRLL = WDOG_STCTRLL_VALUE;
  #endif

  /* Register 'WDOG_WINL' initialization */
  #ifdef WDOG_WINL_VALUE
  WDOG_WINL = WDOG_WINL_VALUE;
  #endif

  /* Register 'WDOG_WINH' initialization */
  #ifdef WDOG_WINH_VALUE
  WDOG_WINH = WDOG_WINH_VALUE;
  #endif

  /* Register 'WDOG_TOVALL' initialization */
  #ifdef WDOG_TOVALL_VALUE
  WDOG_TOVALL = WDOG_TOVALL_VALUE;
  #endif

  /* Register 'WDOG_TOVALH' initialization */
  #ifdef WDOG_TOVALH_VALUE
  WDOG_TOVALH = WDOG_TOVALH_VALUE;
  #endif

  /* Register 'WDOG_PRESC' initialization */
  #ifdef WDOG_PRESC_VALUE
  WDOG_PRESC = WDOG_PRESC_VALUE;
  #endif

  /* Register 'WDOG_STCTRLH' initialization */
  #ifdef WDOG_STCTRLH_VALUE
  WDOG_STCTRLH = WDOG_STCTRLH_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END WDOG_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

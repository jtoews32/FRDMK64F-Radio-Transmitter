/** ###################################################################
 **     Filename    : RCM_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the RCM module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void RCM_Init(void);
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
 * @file RCM_Init.c
 * @brief This file implements the RCM module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE RCM_Init. */

#include "RCM_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void RCM_Init(void) {

  /* Register 'RCM_RPFW' initialization */
  #ifdef RCM_RPFW_VALUE
  RCM_RPFW = RCM_RPFW_VALUE;
  #endif

  /* Register 'RCM_RPFC' initialization */
  #if RCM_RPFC_MASK
    #if RCM_RPFC_MASK == 0xFF
  RCM_RPFC = RCM_RPFC_VALUE;
    #elif RCM_RPFC_MASK == RCM_RPFC_VALUE
  RCM_RPFC |= RCM_RPFC_VALUE;
    #elif RCM_RPFC_VALUE == 0
  RCM_RPFC &= ~RCM_RPFC_MASK;
    #else
  RCM_RPFC = (RCM_RPFC & (~RCM_RPFC_MASK)) | RCM_RPFC_VALUE;
    #endif
  #elif defined(RCM_RPFC_VALUE)
  RCM_RPFC = RCM_RPFC_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END RCM_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

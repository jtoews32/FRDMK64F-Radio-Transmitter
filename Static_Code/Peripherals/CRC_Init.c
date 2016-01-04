/** ###################################################################
 **     Filename    : CRC_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the CRC module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void CRC_Init(void);
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
 * @file CRC_Init.c
 * @brief This file implements the CRC module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE CRC_Init. */

#include "CRC_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void CRC_Init(void) {

  /* Register 'CRC_CTRL' initialization */
  #ifdef CRC_CTRL_VALUE_1
  CRC_CTRL = CRC_CTRL_VALUE_1;
  #endif

  /* Register 'CRC_GPOLY' initialization */
  #ifdef CRC_GPOLY_VALUE
  CRC_GPOLY = CRC_GPOLY_VALUE;
  #endif

  /* Register 'CRC_CTRL' initialization */
  #if CRC_CTRL_MASK_2
    #if CRC_CTRL_MASK_2 == 0xFFFFFFFF
  CRC_CTRL = CRC_CTRL_VALUE_2;
    #elif CRC_CTRL_MASK_2 == CRC_CTRL_VALUE_2
  CRC_CTRL |= CRC_CTRL_VALUE_2;
    #elif CRC_CTRL_VALUE_2 == 0
  CRC_CTRL &= ~CRC_CTRL_MASK_2;
    #else
  CRC_CTRL = (CRC_CTRL & (~CRC_CTRL_MASK_2)) | CRC_CTRL_VALUE_2;
    #endif
  #elif defined(CRC_CTRL_VALUE_2)
  CRC_CTRL = CRC_CTRL_VALUE_2;
  #endif

  /* Register 'CRC_DATA' initialization */
  #ifdef CRC_DATA_VALUE
  CRC_DATA = CRC_DATA_VALUE;
  #endif

  /* Register 'CRC_CTRL' initialization */
  #if CRC_CTRL_MASK_3
    #if CRC_CTRL_MASK_3 == 0xFFFFFFFF
  CRC_CTRL = CRC_CTRL_VALUE_3;
    #elif CRC_CTRL_MASK_3 == CRC_CTRL_VALUE_3
  CRC_CTRL |= CRC_CTRL_VALUE_3;
    #elif CRC_CTRL_VALUE_3 == 0
  CRC_CTRL &= ~CRC_CTRL_MASK_3;
    #else
  CRC_CTRL = (CRC_CTRL & (~CRC_CTRL_MASK_3)) | CRC_CTRL_VALUE_3;
    #endif
  #elif defined(CRC_CTRL_VALUE_3)
  CRC_CTRL = CRC_CTRL_VALUE_3;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END CRC_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

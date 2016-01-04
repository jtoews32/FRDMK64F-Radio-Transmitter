/** ###################################################################
 **     Filename    : AXBS_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the AXBS module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void AXBS_Init(void);
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
 * @file AXBS_Init.c
 * @brief This file implements the AXBS module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE AXBS_Init. */

#include "AXBS_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void AXBS_Init(void) {

  /* Register 'AXBS_PRS0' initialization */
  #ifdef AXBS_PRS0_VALUE
  AXBS_PRS0 = AXBS_PRS0_VALUE;
  #endif

  /* Register 'AXBS_PRS1' initialization */
  #ifdef AXBS_PRS1_VALUE
  AXBS_PRS1 = AXBS_PRS1_VALUE;
  #endif

  /* Register 'AXBS_PRS2' initialization */
  #ifdef AXBS_PRS2_VALUE
  AXBS_PRS2 = AXBS_PRS2_VALUE;
  #endif

  /* Register 'AXBS_PRS3' initialization */
  #ifdef AXBS_PRS3_VALUE
  AXBS_PRS3 = AXBS_PRS3_VALUE;
  #endif

  /* Register 'AXBS_PRS4' initialization */
  #ifdef AXBS_PRS4_VALUE
  AXBS_PRS4 = AXBS_PRS4_VALUE;
  #endif

  /* Register 'AXBS_CRS0' initialization */
  #ifdef AXBS_CRS0_VALUE
  AXBS_CRS0 = AXBS_CRS0_VALUE;
  #endif

  /* Register 'AXBS_CRS1' initialization */
  #ifdef AXBS_CRS1_VALUE
  AXBS_CRS1 = AXBS_CRS1_VALUE;
  #endif

  /* Register 'AXBS_CRS2' initialization */
  #ifdef AXBS_CRS2_VALUE
  AXBS_CRS2 = AXBS_CRS2_VALUE;
  #endif

  /* Register 'AXBS_CRS3' initialization */
  #ifdef AXBS_CRS3_VALUE
  AXBS_CRS3 = AXBS_CRS3_VALUE;
  #endif

  /* Register 'AXBS_CRS4' initialization */
  #ifdef AXBS_CRS4_VALUE
  AXBS_CRS4 = AXBS_CRS4_VALUE;
  #endif

  /* Register 'AXBS_MGPCR0' initialization */
  #ifdef AXBS_MGPCR0_VALUE
  AXBS_MGPCR0 = AXBS_MGPCR0_VALUE;
  #endif

  /* Register 'AXBS_MGPCR1' initialization */
  #ifdef AXBS_MGPCR1_VALUE
  AXBS_MGPCR1 = AXBS_MGPCR1_VALUE;
  #endif

  /* Register 'AXBS_MGPCR2' initialization */
  #ifdef AXBS_MGPCR2_VALUE
  AXBS_MGPCR2 = AXBS_MGPCR2_VALUE;
  #endif

  /* Register 'AXBS_MGPCR3' initialization */
  #ifdef AXBS_MGPCR3_VALUE
  AXBS_MGPCR3 = AXBS_MGPCR3_VALUE;
  #endif

  /* Register 'AXBS_MGPCR4' initialization */
  #ifdef AXBS_MGPCR4_VALUE
  AXBS_MGPCR4 = AXBS_MGPCR4_VALUE;
  #endif

  /* Register 'AXBS_MGPCR5' initialization */
  #ifdef AXBS_MGPCR5_VALUE
  AXBS_MGPCR5 = AXBS_MGPCR5_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END AXBS_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

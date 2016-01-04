/** ###################################################################
 **     Filename    : CMT_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the CMT module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void CMT_Init(void);
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
 * @file CMT_Init.c
 * @brief This file implements the CMT module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE CMT_Init. */

#include "CMT_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void CMT_Init(void) {
    (void)CMT_MSC;            /* Dummy read of the CMT Modulator Status and Control register to clear the interrupt flag */
    (void)CMT_CMD2;           /* Dummy read of the CMT Modulator Data register to finish the clearing of the interrupt flag */

  /* Register 'CMT_OC' initialization */
  #ifdef CMT_OC_VALUE
  CMT_OC = CMT_OC_VALUE;
  #endif

  /* Register 'CMT_PPS' initialization */
  #ifdef CMT_PPS_VALUE
  CMT_PPS = CMT_PPS_VALUE;
  #endif

  /* Register 'CMT_CGH1' initialization */
  #ifdef CMT_CGH1_VALUE
  CMT_CGH1 = CMT_CGH1_VALUE;
  #endif

  /* Register 'CMT_CGL1' initialization */
  #ifdef CMT_CGL1_VALUE
  CMT_CGL1 = CMT_CGL1_VALUE;
  #endif

  /* Register 'CMT_CGH2' initialization */
  #ifdef CMT_CGH2_VALUE
  CMT_CGH2 = CMT_CGH2_VALUE;
  #endif

  /* Register 'CMT_CGL2' initialization */
  #ifdef CMT_CGL2_VALUE
  CMT_CGL2 = CMT_CGL2_VALUE;
  #endif

  /* Register 'CMT_CMD2' initialization */
  #ifdef CMT_CMD2_VALUE
  CMT_CMD2 = CMT_CMD2_VALUE;
  #endif

  /* Register 'CMT_CMD1' initialization */
  #ifdef CMT_CMD1_VALUE
  CMT_CMD1 = CMT_CMD1_VALUE;
  #endif

  /* Register 'CMT_CMD4' initialization */
  #ifdef CMT_CMD4_VALUE
  CMT_CMD4 = CMT_CMD4_VALUE;
  #endif

  /* Register 'CMT_CMD3' initialization */
  #ifdef CMT_CMD3_VALUE
  CMT_CMD3 = CMT_CMD3_VALUE;
  #endif

  /* Register 'CMT_DMA' initialization */
  #ifdef CMT_DMA_VALUE
  CMT_DMA = CMT_DMA_VALUE;
  #endif

  /* Register 'CMT_MSC' initialization */
  #ifdef CMT_MSC_VALUE
  CMT_MSC = CMT_MSC_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END CMT_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

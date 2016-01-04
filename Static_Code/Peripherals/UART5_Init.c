/** ###################################################################
 **     Filename    : UART5_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the UART5 module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void UART5_Init(void);
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
 * @file UART5_Init.c
 * @brief This file implements the UART5 module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE UART5_Init. */

#include "UART5_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void UART5_Init(void) {

  UART5_C2 &= ~0xC;

  /* Register 'UART5_BDH' initialization */
  #ifdef UART5_BDH_VALUE
  UART5_BDH = UART5_BDH_VALUE;
  #endif

  /* Register 'UART5_BDL' initialization */
  #ifdef UART5_BDL_VALUE
  UART5_BDL = UART5_BDL_VALUE;
  #endif

  /* Register 'UART5_C4' initialization */
  #ifdef UART5_C4_VALUE
  UART5_C4 = UART5_C4_VALUE;
  #endif

  /* Register 'UART5_MA1' initialization */
  #ifdef UART5_MA1_VALUE
  UART5_MA1 = UART5_MA1_VALUE;
  #endif

  /* Register 'UART5_MA2' initialization */
  #ifdef UART5_MA2_VALUE
  UART5_MA2 = UART5_MA2_VALUE;
  #endif

  /* Register 'UART5_C1' initialization */
  #ifdef UART5_C1_VALUE
  UART5_C1 = UART5_C1_VALUE;
  #endif

  /* Register 'UART5_S2' initialization */
  #ifdef UART5_S2_VALUE
  UART5_S2 = UART5_S2_VALUE;
  #endif

  /* Register 'UART5_MODEM' initialization */
  #ifdef UART5_MODEM_VALUE
  UART5_MODEM = UART5_MODEM_VALUE;
  #endif

  /* Register 'UART5_IR' initialization */
  #ifdef UART5_IR_VALUE
  UART5_IR = UART5_IR_VALUE;
  #endif

  /* Register 'UART5_TWFIFO' initialization */
  #ifdef UART5_TWFIFO_VALUE
  UART5_TWFIFO = UART5_TWFIFO_VALUE;
  #endif

  /* Register 'UART5_RWFIFO' initialization */
  #ifdef UART5_RWFIFO_VALUE
  UART5_RWFIFO = UART5_RWFIFO_VALUE;
  #endif

  /* Register 'UART5_SFIFO' initialization */
  #ifdef UART5_SFIFO_VALUE
  UART5_SFIFO = UART5_SFIFO_VALUE;
  #endif

  /* Register 'UART5_CFIFO' initialization */
  #if UART5_CFIFO_MASK
    #if UART5_CFIFO_MASK == 0xFF
  UART5_CFIFO = UART5_CFIFO_VALUE;
    #elif UART5_CFIFO_MASK == UART5_CFIFO_VALUE
  UART5_CFIFO |= UART5_CFIFO_VALUE;
    #elif UART5_CFIFO_VALUE == 0
  UART5_CFIFO &= ~UART5_CFIFO_MASK;
    #else
  UART5_CFIFO = (UART5_CFIFO & (~UART5_CFIFO_MASK)) | UART5_CFIFO_VALUE;
    #endif
  #elif defined(UART5_CFIFO_VALUE)
  UART5_CFIFO = UART5_CFIFO_VALUE;
  #endif

  /* Register 'UART5_PFIFO' initialization */
  #if UART5_PFIFO_MASK
    #if UART5_PFIFO_MASK == 0xFF
  UART5_PFIFO = UART5_PFIFO_VALUE;
    #elif UART5_PFIFO_MASK == UART5_PFIFO_VALUE
  UART5_PFIFO |= UART5_PFIFO_VALUE;
    #elif UART5_PFIFO_VALUE == 0
  UART5_PFIFO &= ~UART5_PFIFO_MASK;
    #else
  UART5_PFIFO = (UART5_PFIFO & (~UART5_PFIFO_MASK)) | UART5_PFIFO_VALUE;
    #endif
  #elif defined(UART5_PFIFO_VALUE)
  UART5_PFIFO = UART5_PFIFO_VALUE;
  #endif

  /* Clear flags */
  #ifdef UART5_CLEAR_FLAGS
  (void)UART5_S1;
  (void)UART5_D;
  #endif

  /* Register 'UART5_C5' initialization */
  #ifdef UART5_C5_VALUE
  UART5_C5 = UART5_C5_VALUE;
  #endif

  /* Register 'UART5_C3' initialization */
  #ifdef UART5_C3_VALUE
  UART5_C3 = UART5_C3_VALUE;
  #endif

  /* Register 'UART5_C2' initialization */
  #ifdef UART5_C2_VALUE
  UART5_C2 = UART5_C2_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END UART5_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

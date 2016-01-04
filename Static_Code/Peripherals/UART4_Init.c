/** ###################################################################
 **     Filename    : UART4_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the UART4 module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void UART4_Init(void);
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
 * @file UART4_Init.c
 * @brief This file implements the UART4 module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE UART4_Init. */

#include "UART4_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void UART4_Init(void) {

  UART4_C2 &= ~0xC;

  /* Register 'UART4_BDH' initialization */
  #ifdef UART4_BDH_VALUE
  UART4_BDH = UART4_BDH_VALUE;
  #endif

  /* Register 'UART4_BDL' initialization */
  #ifdef UART4_BDL_VALUE
  UART4_BDL = UART4_BDL_VALUE;
  #endif

  /* Register 'UART4_C4' initialization */
  #ifdef UART4_C4_VALUE
  UART4_C4 = UART4_C4_VALUE;
  #endif

  /* Register 'UART4_MA1' initialization */
  #ifdef UART4_MA1_VALUE
  UART4_MA1 = UART4_MA1_VALUE;
  #endif

  /* Register 'UART4_MA2' initialization */
  #ifdef UART4_MA2_VALUE
  UART4_MA2 = UART4_MA2_VALUE;
  #endif

  /* Register 'UART4_C1' initialization */
  #ifdef UART4_C1_VALUE
  UART4_C1 = UART4_C1_VALUE;
  #endif

  /* Register 'UART4_S2' initialization */
  #ifdef UART4_S2_VALUE
  UART4_S2 = UART4_S2_VALUE;
  #endif

  /* Register 'UART4_MODEM' initialization */
  #ifdef UART4_MODEM_VALUE
  UART4_MODEM = UART4_MODEM_VALUE;
  #endif

  /* Register 'UART4_IR' initialization */
  #ifdef UART4_IR_VALUE
  UART4_IR = UART4_IR_VALUE;
  #endif

  /* Register 'UART4_TWFIFO' initialization */
  #ifdef UART4_TWFIFO_VALUE
  UART4_TWFIFO = UART4_TWFIFO_VALUE;
  #endif

  /* Register 'UART4_RWFIFO' initialization */
  #ifdef UART4_RWFIFO_VALUE
  UART4_RWFIFO = UART4_RWFIFO_VALUE;
  #endif

  /* Register 'UART4_SFIFO' initialization */
  #ifdef UART4_SFIFO_VALUE
  UART4_SFIFO = UART4_SFIFO_VALUE;
  #endif

  /* Register 'UART4_CFIFO' initialization */
  #if UART4_CFIFO_MASK
    #if UART4_CFIFO_MASK == 0xFF
  UART4_CFIFO = UART4_CFIFO_VALUE;
    #elif UART4_CFIFO_MASK == UART4_CFIFO_VALUE
  UART4_CFIFO |= UART4_CFIFO_VALUE;
    #elif UART4_CFIFO_VALUE == 0
  UART4_CFIFO &= ~UART4_CFIFO_MASK;
    #else
  UART4_CFIFO = (UART4_CFIFO & (~UART4_CFIFO_MASK)) | UART4_CFIFO_VALUE;
    #endif
  #elif defined(UART4_CFIFO_VALUE)
  UART4_CFIFO = UART4_CFIFO_VALUE;
  #endif

  /* Register 'UART4_PFIFO' initialization */
  #if UART4_PFIFO_MASK
    #if UART4_PFIFO_MASK == 0xFF
  UART4_PFIFO = UART4_PFIFO_VALUE;
    #elif UART4_PFIFO_MASK == UART4_PFIFO_VALUE
  UART4_PFIFO |= UART4_PFIFO_VALUE;
    #elif UART4_PFIFO_VALUE == 0
  UART4_PFIFO &= ~UART4_PFIFO_MASK;
    #else
  UART4_PFIFO = (UART4_PFIFO & (~UART4_PFIFO_MASK)) | UART4_PFIFO_VALUE;
    #endif
  #elif defined(UART4_PFIFO_VALUE)
  UART4_PFIFO = UART4_PFIFO_VALUE;
  #endif

  /* Clear flags */
  #ifdef UART4_CLEAR_FLAGS
  (void)UART4_S1;
  (void)UART4_D;
  #endif

  /* Register 'UART4_C5' initialization */
  #ifdef UART4_C5_VALUE
  UART4_C5 = UART4_C5_VALUE;
  #endif

  /* Register 'UART4_C3' initialization */
  #ifdef UART4_C3_VALUE
  UART4_C3 = UART4_C3_VALUE;
  #endif

  /* Register 'UART4_C2' initialization */
  #ifdef UART4_C2_VALUE
  UART4_C2 = UART4_C2_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END UART4_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

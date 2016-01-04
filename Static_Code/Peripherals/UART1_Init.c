/** ###################################################################
 **     Filename    : UART1_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the UART1 module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void UART1_Init(void);
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
 * @file UART1_Init.c
 * @brief This file implements the UART1 module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE UART1_Init. */

#include "UART1_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void UART1_Init(void) {

  UART1_C2 &= ~0xC;

  /* Register 'UART1_BDH' initialization */
  #ifdef UART1_BDH_VALUE
  UART1_BDH = UART1_BDH_VALUE;
  #endif

  /* Register 'UART1_BDL' initialization */
  #ifdef UART1_BDL_VALUE
  UART1_BDL = UART1_BDL_VALUE;
  #endif

  /* Register 'UART1_C4' initialization */
  #ifdef UART1_C4_VALUE
  UART1_C4 = UART1_C4_VALUE;
  #endif

  /* Register 'UART1_MA1' initialization */
  #ifdef UART1_MA1_VALUE
  UART1_MA1 = UART1_MA1_VALUE;
  #endif

  /* Register 'UART1_MA2' initialization */
  #ifdef UART1_MA2_VALUE
  UART1_MA2 = UART1_MA2_VALUE;
  #endif

  /* Register 'UART1_C1' initialization */
  #ifdef UART1_C1_VALUE
  UART1_C1 = UART1_C1_VALUE;
  #endif

  /* Register 'UART1_S2' initialization */
  #ifdef UART1_S2_VALUE
  UART1_S2 = UART1_S2_VALUE;
  #endif

  /* Register 'UART1_MODEM' initialization */
  #ifdef UART1_MODEM_VALUE
  UART1_MODEM = UART1_MODEM_VALUE;
  #endif

  /* Register 'UART1_IR' initialization */
  #ifdef UART1_IR_VALUE
  UART1_IR = UART1_IR_VALUE;
  #endif

  /* Register 'UART1_TWFIFO' initialization */
  #ifdef UART1_TWFIFO_VALUE
  UART1_TWFIFO = UART1_TWFIFO_VALUE;
  #endif

  /* Register 'UART1_RWFIFO' initialization */
  #ifdef UART1_RWFIFO_VALUE
  UART1_RWFIFO = UART1_RWFIFO_VALUE;
  #endif

  /* Register 'UART1_SFIFO' initialization */
  #ifdef UART1_SFIFO_VALUE
  UART1_SFIFO = UART1_SFIFO_VALUE;
  #endif

  /* Register 'UART1_CFIFO' initialization */
  #if UART1_CFIFO_MASK
    #if UART1_CFIFO_MASK == 0xFF
  UART1_CFIFO = UART1_CFIFO_VALUE;
    #elif UART1_CFIFO_MASK == UART1_CFIFO_VALUE
  UART1_CFIFO |= UART1_CFIFO_VALUE;
    #elif UART1_CFIFO_VALUE == 0
  UART1_CFIFO &= ~UART1_CFIFO_MASK;
    #else
  UART1_CFIFO = (UART1_CFIFO & (~UART1_CFIFO_MASK)) | UART1_CFIFO_VALUE;
    #endif
  #elif defined(UART1_CFIFO_VALUE)
  UART1_CFIFO = UART1_CFIFO_VALUE;
  #endif

  /* Register 'UART1_PFIFO' initialization */
  #if UART1_PFIFO_MASK
    #if UART1_PFIFO_MASK == 0xFF
  UART1_PFIFO = UART1_PFIFO_VALUE;
    #elif UART1_PFIFO_MASK == UART1_PFIFO_VALUE
  UART1_PFIFO |= UART1_PFIFO_VALUE;
    #elif UART1_PFIFO_VALUE == 0
  UART1_PFIFO &= ~UART1_PFIFO_MASK;
    #else
  UART1_PFIFO = (UART1_PFIFO & (~UART1_PFIFO_MASK)) | UART1_PFIFO_VALUE;
    #endif
  #elif defined(UART1_PFIFO_VALUE)
  UART1_PFIFO = UART1_PFIFO_VALUE;
  #endif

  /* Clear flags */
  #ifdef UART1_CLEAR_FLAGS
  (void)UART1_S1;
  (void)UART1_D;
  #endif

  /* Register 'UART1_C5' initialization */
  #ifdef UART1_C5_VALUE
  UART1_C5 = UART1_C5_VALUE;
  #endif

  /* Register 'UART1_C3' initialization */
  #ifdef UART1_C3_VALUE
  UART1_C3 = UART1_C3_VALUE;
  #endif

  /* Register 'UART1_C2' initialization */
  #ifdef UART1_C2_VALUE
  UART1_C2 = UART1_C2_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END UART1_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

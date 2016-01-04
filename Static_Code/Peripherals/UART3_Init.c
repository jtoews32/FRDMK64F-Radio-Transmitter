/** ###################################################################
 **     Filename    : UART3_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the UART3 module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void UART3_Init(void);
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
 * @file UART3_Init.c
 * @brief This file implements the UART3 module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE UART3_Init. */

#include "UART3_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void UART3_Init(void) {

  UART3_C2 &= ~0xC;

  /* Register 'UART3_BDH' initialization */
  #ifdef UART3_BDH_VALUE
  UART3_BDH = UART3_BDH_VALUE;
  #endif

  /* Register 'UART3_BDL' initialization */
  #ifdef UART3_BDL_VALUE
  UART3_BDL = UART3_BDL_VALUE;
  #endif

  /* Register 'UART3_C4' initialization */
  #ifdef UART3_C4_VALUE
  UART3_C4 = UART3_C4_VALUE;
  #endif

  /* Register 'UART3_MA1' initialization */
  #ifdef UART3_MA1_VALUE
  UART3_MA1 = UART3_MA1_VALUE;
  #endif

  /* Register 'UART3_MA2' initialization */
  #ifdef UART3_MA2_VALUE
  UART3_MA2 = UART3_MA2_VALUE;
  #endif

  /* Register 'UART3_C1' initialization */
  #ifdef UART3_C1_VALUE
  UART3_C1 = UART3_C1_VALUE;
  #endif

  /* Register 'UART3_S2' initialization */
  #ifdef UART3_S2_VALUE
  UART3_S2 = UART3_S2_VALUE;
  #endif

  /* Register 'UART3_MODEM' initialization */
  #ifdef UART3_MODEM_VALUE
  UART3_MODEM = UART3_MODEM_VALUE;
  #endif

  /* Register 'UART3_IR' initialization */
  #ifdef UART3_IR_VALUE
  UART3_IR = UART3_IR_VALUE;
  #endif

  /* Register 'UART3_TWFIFO' initialization */
  #ifdef UART3_TWFIFO_VALUE
  UART3_TWFIFO = UART3_TWFIFO_VALUE;
  #endif

  /* Register 'UART3_RWFIFO' initialization */
  #ifdef UART3_RWFIFO_VALUE
  UART3_RWFIFO = UART3_RWFIFO_VALUE;
  #endif

  /* Register 'UART3_SFIFO' initialization */
  #ifdef UART3_SFIFO_VALUE
  UART3_SFIFO = UART3_SFIFO_VALUE;
  #endif

  /* Register 'UART3_CFIFO' initialization */
  #if UART3_CFIFO_MASK
    #if UART3_CFIFO_MASK == 0xFF
  UART3_CFIFO = UART3_CFIFO_VALUE;
    #elif UART3_CFIFO_MASK == UART3_CFIFO_VALUE
  UART3_CFIFO |= UART3_CFIFO_VALUE;
    #elif UART3_CFIFO_VALUE == 0
  UART3_CFIFO &= ~UART3_CFIFO_MASK;
    #else
  UART3_CFIFO = (UART3_CFIFO & (~UART3_CFIFO_MASK)) | UART3_CFIFO_VALUE;
    #endif
  #elif defined(UART3_CFIFO_VALUE)
  UART3_CFIFO = UART3_CFIFO_VALUE;
  #endif

  /* Register 'UART3_PFIFO' initialization */
  #if UART3_PFIFO_MASK
    #if UART3_PFIFO_MASK == 0xFF
  UART3_PFIFO = UART3_PFIFO_VALUE;
    #elif UART3_PFIFO_MASK == UART3_PFIFO_VALUE
  UART3_PFIFO |= UART3_PFIFO_VALUE;
    #elif UART3_PFIFO_VALUE == 0
  UART3_PFIFO &= ~UART3_PFIFO_MASK;
    #else
  UART3_PFIFO = (UART3_PFIFO & (~UART3_PFIFO_MASK)) | UART3_PFIFO_VALUE;
    #endif
  #elif defined(UART3_PFIFO_VALUE)
  UART3_PFIFO = UART3_PFIFO_VALUE;
  #endif

  /* Clear flags */
  #ifdef UART3_CLEAR_FLAGS
  (void)UART3_S1;
  (void)UART3_D;
  #endif

  /* Register 'UART3_C5' initialization */
  #ifdef UART3_C5_VALUE
  UART3_C5 = UART3_C5_VALUE;
  #endif

  /* Register 'UART3_C3' initialization */
  #ifdef UART3_C3_VALUE
  UART3_C3 = UART3_C3_VALUE;
  #endif

  /* Register 'UART3_C2' initialization */
  #ifdef UART3_C2_VALUE
  UART3_C2 = UART3_C2_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END UART3_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

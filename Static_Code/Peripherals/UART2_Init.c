/** ###################################################################
 **     Filename    : UART2_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the UART2 module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void UART2_Init(void);
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
 * @file UART2_Init.c
 * @brief This file implements the UART2 module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE UART2_Init. */

#include "UART2_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void UART2_Init(void) {

  UART2_C2 &= ~0xC;

  /* Register 'UART2_BDH' initialization */
  #ifdef UART2_BDH_VALUE
  UART2_BDH = UART2_BDH_VALUE;
  #endif

  /* Register 'UART2_BDL' initialization */
  #ifdef UART2_BDL_VALUE
  UART2_BDL = UART2_BDL_VALUE;
  #endif

  /* Register 'UART2_C4' initialization */
  #ifdef UART2_C4_VALUE
  UART2_C4 = UART2_C4_VALUE;
  #endif

  /* Register 'UART2_MA1' initialization */
  #ifdef UART2_MA1_VALUE
  UART2_MA1 = UART2_MA1_VALUE;
  #endif

  /* Register 'UART2_MA2' initialization */
  #ifdef UART2_MA2_VALUE
  UART2_MA2 = UART2_MA2_VALUE;
  #endif

  /* Register 'UART2_C1' initialization */
  #ifdef UART2_C1_VALUE
  UART2_C1 = UART2_C1_VALUE;
  #endif

  /* Register 'UART2_S2' initialization */
  #ifdef UART2_S2_VALUE
  UART2_S2 = UART2_S2_VALUE;
  #endif

  /* Register 'UART2_MODEM' initialization */
  #ifdef UART2_MODEM_VALUE
  UART2_MODEM = UART2_MODEM_VALUE;
  #endif

  /* Register 'UART2_IR' initialization */
  #ifdef UART2_IR_VALUE
  UART2_IR = UART2_IR_VALUE;
  #endif

  /* Register 'UART2_TWFIFO' initialization */
  #ifdef UART2_TWFIFO_VALUE
  UART2_TWFIFO = UART2_TWFIFO_VALUE;
  #endif

  /* Register 'UART2_RWFIFO' initialization */
  #ifdef UART2_RWFIFO_VALUE
  UART2_RWFIFO = UART2_RWFIFO_VALUE;
  #endif

  /* Register 'UART2_SFIFO' initialization */
  #ifdef UART2_SFIFO_VALUE
  UART2_SFIFO = UART2_SFIFO_VALUE;
  #endif

  /* Register 'UART2_CFIFO' initialization */
  #if UART2_CFIFO_MASK
    #if UART2_CFIFO_MASK == 0xFF
  UART2_CFIFO = UART2_CFIFO_VALUE;
    #elif UART2_CFIFO_MASK == UART2_CFIFO_VALUE
  UART2_CFIFO |= UART2_CFIFO_VALUE;
    #elif UART2_CFIFO_VALUE == 0
  UART2_CFIFO &= ~UART2_CFIFO_MASK;
    #else
  UART2_CFIFO = (UART2_CFIFO & (~UART2_CFIFO_MASK)) | UART2_CFIFO_VALUE;
    #endif
  #elif defined(UART2_CFIFO_VALUE)
  UART2_CFIFO = UART2_CFIFO_VALUE;
  #endif

  /* Register 'UART2_PFIFO' initialization */
  #if UART2_PFIFO_MASK
    #if UART2_PFIFO_MASK == 0xFF
  UART2_PFIFO = UART2_PFIFO_VALUE;
    #elif UART2_PFIFO_MASK == UART2_PFIFO_VALUE
  UART2_PFIFO |= UART2_PFIFO_VALUE;
    #elif UART2_PFIFO_VALUE == 0
  UART2_PFIFO &= ~UART2_PFIFO_MASK;
    #else
  UART2_PFIFO = (UART2_PFIFO & (~UART2_PFIFO_MASK)) | UART2_PFIFO_VALUE;
    #endif
  #elif defined(UART2_PFIFO_VALUE)
  UART2_PFIFO = UART2_PFIFO_VALUE;
  #endif

  /* Clear flags */
  #ifdef UART2_CLEAR_FLAGS
  (void)UART2_S1;
  (void)UART2_D;
  #endif

  /* Register 'UART2_C5' initialization */
  #ifdef UART2_C5_VALUE
  UART2_C5 = UART2_C5_VALUE;
  #endif

  /* Register 'UART2_C3' initialization */
  #ifdef UART2_C3_VALUE
  UART2_C3 = UART2_C3_VALUE;
  #endif

  /* Register 'UART2_C2' initialization */
  #ifdef UART2_C2_VALUE
  UART2_C2 = UART2_C2_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END UART2_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

/** ###################################################################
 **     Filename    : PDB0_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the PDB0 module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void PDB0_Init(void);
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
 * @file PDB0_Init.c
 * @brief This file implements the PDB0 module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE PDB0_Init. */

#include "PDB0_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void PDB0_Init(void) {

  /* Register 'PDB0_SC' initialization */
  #if PDB0_SC_MASK_1
    #if PDB0_SC_MASK_1 == 0xFFFFFFFF
  PDB0_SC = PDB0_SC_VALUE_1;
    #elif PDB0_SC_MASK_1 == PDB0_SC_VALUE_1
  PDB0_SC |= PDB0_SC_VALUE_1;
    #elif PDB0_SC_VALUE_1 == 0
  PDB0_SC &= ~PDB0_SC_MASK_1;
    #else
  PDB0_SC = (PDB0_SC & (~PDB0_SC_MASK_1)) | PDB0_SC_VALUE_1;
    #endif
  #elif defined(PDB0_SC_VALUE_1)
  PDB0_SC = PDB0_SC_VALUE_1;
  #endif

  /* Register 'PDB0_MOD' initialization */
  #ifdef PDB0_MOD_VALUE
  PDB0_MOD = PDB0_MOD_VALUE;
  #endif

  /* Register 'PDB0_IDLY' initialization */
  #ifdef PDB0_IDLY_VALUE
  PDB0_IDLY = PDB0_IDLY_VALUE;
  #endif

  /* Register 'PDB0_CH0C1' initialization */
  #if PDB0_CH0C1_MASK
    #if PDB0_CH0C1_MASK == 0xFFFFFFFF
  PDB0_CH0C1 = PDB0_CH0C1_VALUE;
    #elif PDB0_CH0C1_MASK == PDB0_CH0C1_VALUE
  PDB0_CH0C1 |= PDB0_CH0C1_VALUE;
    #elif PDB0_CH0C1_VALUE == 0
  PDB0_CH0C1 &= ~PDB0_CH0C1_MASK;
    #else
  PDB0_CH0C1 = (PDB0_CH0C1 & (~PDB0_CH0C1_MASK)) | PDB0_CH0C1_VALUE;
    #endif
  #elif defined(PDB0_CH0C1_VALUE)
  PDB0_CH0C1 = PDB0_CH0C1_VALUE;
  #endif

  /* Register 'PDB0_CH0S' initialization */
  #if PDB0_CH0S_MASK
    #if PDB0_CH0S_MASK == 0xFFFFFFFF
  PDB0_CH0S = PDB0_CH0S_VALUE;
    #elif PDB0_CH0S_MASK == PDB0_CH0S_VALUE
  PDB0_CH0S |= PDB0_CH0S_VALUE;
    #elif PDB0_CH0S_VALUE == 0
  PDB0_CH0S &= ~PDB0_CH0S_MASK;
    #else
  PDB0_CH0S = (PDB0_CH0S & (~PDB0_CH0S_MASK)) | PDB0_CH0S_VALUE;
    #endif
  #elif defined(PDB0_CH0S_VALUE)
  PDB0_CH0S = PDB0_CH0S_VALUE;
  #endif

  /* Register 'PDB0_CH0DLY0' initialization */
  #if PDB0_CH0DLY0_MASK
    #if PDB0_CH0DLY0_MASK == 0xFFFFFFFF
  PDB0_CH0DLY0 = PDB0_CH0DLY0_VALUE;
    #elif PDB0_CH0DLY0_MASK == PDB0_CH0DLY0_VALUE
  PDB0_CH0DLY0 |= PDB0_CH0DLY0_VALUE;
    #elif PDB0_CH0DLY0_VALUE == 0
  PDB0_CH0DLY0 &= ~PDB0_CH0DLY0_MASK;
    #else
  PDB0_CH0DLY0 = (PDB0_CH0DLY0 & (~PDB0_CH0DLY0_MASK)) | PDB0_CH0DLY0_VALUE;
    #endif
  #elif defined(PDB0_CH0DLY0_VALUE)
  PDB0_CH0DLY0 = PDB0_CH0DLY0_VALUE;
  #endif

  /* Register 'PDB0_CH0DLY1' initialization */
  #if PDB0_CH0DLY1_MASK
    #if PDB0_CH0DLY1_MASK == 0xFFFFFFFF
  PDB0_CH0DLY1 = PDB0_CH0DLY1_VALUE;
    #elif PDB0_CH0DLY1_MASK == PDB0_CH0DLY1_VALUE
  PDB0_CH0DLY1 |= PDB0_CH0DLY1_VALUE;
    #elif PDB0_CH0DLY1_VALUE == 0
  PDB0_CH0DLY1 &= ~PDB0_CH0DLY1_MASK;
    #else
  PDB0_CH0DLY1 = (PDB0_CH0DLY1 & (~PDB0_CH0DLY1_MASK)) | PDB0_CH0DLY1_VALUE;
    #endif
  #elif defined(PDB0_CH0DLY1_VALUE)
  PDB0_CH0DLY1 = PDB0_CH0DLY1_VALUE;
  #endif

  /* Register 'PDB0_CH1C1' initialization */
  #if PDB0_CH1C1_MASK
    #if PDB0_CH1C1_MASK == 0xFFFFFFFF
  PDB0_CH1C1 = PDB0_CH1C1_VALUE;
    #elif PDB0_CH1C1_MASK == PDB0_CH1C1_VALUE
  PDB0_CH1C1 |= PDB0_CH1C1_VALUE;
    #elif PDB0_CH1C1_VALUE == 0
  PDB0_CH1C1 &= ~PDB0_CH1C1_MASK;
    #else
  PDB0_CH1C1 = (PDB0_CH1C1 & (~PDB0_CH1C1_MASK)) | PDB0_CH1C1_VALUE;
    #endif
  #elif defined(PDB0_CH1C1_VALUE)
  PDB0_CH1C1 = PDB0_CH1C1_VALUE;
  #endif

  /* Register 'PDB0_CH1S' initialization */
  #if PDB0_CH1S_MASK
    #if PDB0_CH1S_MASK == 0xFFFFFFFF
  PDB0_CH1S = PDB0_CH1S_VALUE;
    #elif PDB0_CH1S_MASK == PDB0_CH1S_VALUE
  PDB0_CH1S |= PDB0_CH1S_VALUE;
    #elif PDB0_CH1S_VALUE == 0
  PDB0_CH1S &= ~PDB0_CH1S_MASK;
    #else
  PDB0_CH1S = (PDB0_CH1S & (~PDB0_CH1S_MASK)) | PDB0_CH1S_VALUE;
    #endif
  #elif defined(PDB0_CH1S_VALUE)
  PDB0_CH1S = PDB0_CH1S_VALUE;
  #endif

  /* Register 'PDB0_CH1DLY0' initialization */
  #if PDB0_CH1DLY0_MASK
    #if PDB0_CH1DLY0_MASK == 0xFFFFFFFF
  PDB0_CH1DLY0 = PDB0_CH1DLY0_VALUE;
    #elif PDB0_CH1DLY0_MASK == PDB0_CH1DLY0_VALUE
  PDB0_CH1DLY0 |= PDB0_CH1DLY0_VALUE;
    #elif PDB0_CH1DLY0_VALUE == 0
  PDB0_CH1DLY0 &= ~PDB0_CH1DLY0_MASK;
    #else
  PDB0_CH1DLY0 = (PDB0_CH1DLY0 & (~PDB0_CH1DLY0_MASK)) | PDB0_CH1DLY0_VALUE;
    #endif
  #elif defined(PDB0_CH1DLY0_VALUE)
  PDB0_CH1DLY0 = PDB0_CH1DLY0_VALUE;
  #endif

  /* Register 'PDB0_CH1DLY1' initialization */
  #if PDB0_CH1DLY1_MASK
    #if PDB0_CH1DLY1_MASK == 0xFFFFFFFF
  PDB0_CH1DLY1 = PDB0_CH1DLY1_VALUE;
    #elif PDB0_CH1DLY1_MASK == PDB0_CH1DLY1_VALUE
  PDB0_CH1DLY1 |= PDB0_CH1DLY1_VALUE;
    #elif PDB0_CH1DLY1_VALUE == 0
  PDB0_CH1DLY1 &= ~PDB0_CH1DLY1_MASK;
    #else
  PDB0_CH1DLY1 = (PDB0_CH1DLY1 & (~PDB0_CH1DLY1_MASK)) | PDB0_CH1DLY1_VALUE;
    #endif
  #elif defined(PDB0_CH1DLY1_VALUE)
  PDB0_CH1DLY1 = PDB0_CH1DLY1_VALUE;
  #endif

  /* Register 'PDB0_DACINT0' initialization */
  #ifdef PDB0_DACINT0_VALUE
  PDB0_DACINT0 = PDB0_DACINT0_VALUE;
  #endif

  /* Register 'PDB0_DACINT1' initialization */
  #ifdef PDB0_DACINT1_VALUE
  PDB0_DACINT1 = PDB0_DACINT1_VALUE;
  #endif

  /* Register 'PDB0_DACINTC0' initialization */
  #ifdef PDB0_DACINTC0_VALUE
  PDB0_DACINTC0 = PDB0_DACINTC0_VALUE;
  #endif

  /* Register 'PDB0_DACINTC1' initialization */
  #ifdef PDB0_DACINTC1_VALUE
  PDB0_DACINTC1 = PDB0_DACINTC1_VALUE;
  #endif

  /* Register 'PDB0_POEN' initialization */
  #if PDB0_POEN_MASK
    #if PDB0_POEN_MASK == 0xFFFFFFFF
  PDB0_POEN = PDB0_POEN_VALUE;
    #elif PDB0_POEN_MASK == PDB0_POEN_VALUE
  PDB0_POEN |= PDB0_POEN_VALUE;
    #elif PDB0_POEN_VALUE == 0
  PDB0_POEN &= ~PDB0_POEN_MASK;
    #else
  PDB0_POEN = (PDB0_POEN & (~PDB0_POEN_MASK)) | PDB0_POEN_VALUE;
    #endif
  #elif defined(PDB0_POEN_VALUE)
  PDB0_POEN = PDB0_POEN_VALUE;
  #endif

  /* Register 'PDB0_PO0DLY' initialization */
  #ifdef PDB0_PO0DLY_VALUE
  PDB0_PO0DLY = PDB0_PO0DLY_VALUE;
  #endif

  /* Register 'PDB0_PO1DLY' initialization */
  #ifdef PDB0_PO1DLY_VALUE
  PDB0_PO1DLY = PDB0_PO1DLY_VALUE;
  #endif

  /* Register 'PDB0_PO2DLY' initialization */
  #ifdef PDB0_PO2DLY_VALUE
  PDB0_PO2DLY = PDB0_PO2DLY_VALUE;
  #endif

  /* Register 'PDB0_SC' initialization */
  #if PDB0_SC_MASK_2
    #if PDB0_SC_MASK_2 == 0xFFFFFFFF
  PDB0_SC = PDB0_SC_VALUE_2;
    #elif PDB0_SC_MASK_2 == PDB0_SC_VALUE_2
  PDB0_SC |= PDB0_SC_VALUE_2;
    #elif PDB0_SC_VALUE_2 == 0
  PDB0_SC &= ~PDB0_SC_MASK_2;
    #else
  PDB0_SC = (PDB0_SC & (~PDB0_SC_MASK_2)) | PDB0_SC_VALUE_2;
    #endif
  #elif defined(PDB0_SC_VALUE_2)
  PDB0_SC = PDB0_SC_VALUE_2;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END PDB0_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

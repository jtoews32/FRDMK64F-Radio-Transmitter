/** ###################################################################
 **     Filename    : ADC1_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the ADC1 module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void ADC1_Init(void);
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
 * @file ADC1_Init.c
 * @brief This file implements the ADC1 module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE ADC1_Init. */

#include "ADC1_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void ADC1_Init(void) {

  /* Register 'ADC1_CFG1' initialization */
  #ifdef ADC1_CFG1_VALUE
  ADC1_CFG1 = ADC1_CFG1_VALUE;
  #endif

  /* Register 'ADC1_CFG2' initialization */
  #ifdef ADC1_CFG2_VALUE
  ADC1_CFG2 = ADC1_CFG2_VALUE;
  #endif

  /* Register 'ADC1_CV1' initialization */
  #ifdef ADC1_CV1_VALUE
  ADC1_CV1 = ADC1_CV1_VALUE;
  #endif

  /* Register 'ADC1_CV2' initialization */
  #ifdef ADC1_CV2_VALUE
  ADC1_CV2 = ADC1_CV2_VALUE;
  #endif

  /* Register 'ADC1_OFS' initialization */
  #ifdef ADC1_OFS_VALUE
  ADC1_OFS = ADC1_OFS_VALUE;
  #endif

  /* Register 'ADC1_SC2' initialization */
  #if ADC1_SC2_MASK
    #if ADC1_SC2_MASK == 0xFFFFFFFF
  ADC1_SC2 = ADC1_SC2_VALUE;
    #elif ADC1_SC2_MASK == ADC1_SC2_VALUE
  ADC1_SC2 |= ADC1_SC2_VALUE;
    #elif ADC1_SC2_VALUE == 0
  ADC1_SC2 &= ~ADC1_SC2_MASK;
    #else
  ADC1_SC2 = (ADC1_SC2 & (~ADC1_SC2_MASK)) | ADC1_SC2_VALUE;
    #endif
  #elif defined(ADC1_SC2_VALUE)
  ADC1_SC2 = ADC1_SC2_VALUE;
  #endif

  /* Register 'ADC1_SC3' initialization */
  #ifdef ADC1_SC3_VALUE
  ADC1_SC3 = ADC1_SC3_VALUE;
  #endif

  /* Register 'ADC1_SC1A' initialization */
  #ifdef ADC1_SC1A_VALUE
  ADC1_SC1A = ADC1_SC1A_VALUE;
  #endif

  /* Register 'ADC1_SC1B' initialization */
  #ifdef ADC1_SC1B_VALUE
  ADC1_SC1B = ADC1_SC1B_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END ADC1_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

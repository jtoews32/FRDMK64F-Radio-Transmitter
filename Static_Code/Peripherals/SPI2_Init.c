/** ###################################################################
 **     Filename    : SPI2_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the SPI2 module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void SPI2_Init(void);
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
 * @file SPI2_Init.c
 * @brief This file implements the SPI2 module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE SPI2_Init. */

#include "SPI2_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void SPI2_Init(void) {

  SPI2_MCR = 0x1;

  /* Register 'SPI2_MCR' initialization */
  #if SPI2_MCR_MASK_1
    #if SPI2_MCR_MASK_1 == 0xFFFFFFFF
  SPI2_MCR = SPI2_MCR_VALUE_1;
    #elif SPI2_MCR_MASK_1 == SPI2_MCR_VALUE_1
  SPI2_MCR |= SPI2_MCR_VALUE_1;
    #elif SPI2_MCR_VALUE_1 == 0
  SPI2_MCR &= ~SPI2_MCR_MASK_1;
    #else
  SPI2_MCR = (SPI2_MCR & (~SPI2_MCR_MASK_1)) | SPI2_MCR_VALUE_1;
    #endif
  #elif defined(SPI2_MCR_VALUE_1)
  SPI2_MCR = SPI2_MCR_VALUE_1;
  #endif

  /* Register 'SPI2_TCR' initialization */
  #ifdef SPI2_TCR_VALUE
  SPI2_TCR = SPI2_TCR_VALUE;
  #endif

  /* Register 'SPI2_CTAR0' initialization */
  #ifdef SPI2_CTAR0_VALUE
  SPI2_CTAR0 = SPI2_CTAR0_VALUE;
  #endif

  /* Register 'SPI2_CTAR1' initialization */
  #ifdef SPI2_CTAR1_VALUE
  SPI2_CTAR1 = SPI2_CTAR1_VALUE;
  #endif

  /* Register 'SPI2_CTAR0_SLAVE' initialization */
  #if SPI2_CTAR0_SLAVE_MASK
    #if SPI2_CTAR0_SLAVE_MASK == 0xFFFFFFFF
  SPI2_CTAR0_SLAVE = SPI2_CTAR0_SLAVE_VALUE;
    #elif SPI2_CTAR0_SLAVE_MASK == SPI2_CTAR0_SLAVE_VALUE
  SPI2_CTAR0_SLAVE |= SPI2_CTAR0_SLAVE_VALUE;
    #elif SPI2_CTAR0_SLAVE_VALUE == 0
  SPI2_CTAR0_SLAVE &= ~SPI2_CTAR0_SLAVE_MASK;
    #else
  SPI2_CTAR0_SLAVE = (SPI2_CTAR0_SLAVE & (~SPI2_CTAR0_SLAVE_MASK)) | SPI2_CTAR0_SLAVE_VALUE;
    #endif
  #elif defined(SPI2_CTAR0_SLAVE_VALUE)
  SPI2_CTAR0_SLAVE = SPI2_CTAR0_SLAVE_VALUE;
  #endif

  /* Register 'SPI2_SR' initialization */
  #ifdef SPI2_SR_VALUE
  SPI2_SR = SPI2_SR_VALUE;
  #endif

  /* Register 'SPI2_RSER' initialization */
  #ifdef SPI2_RSER_VALUE
  SPI2_RSER = SPI2_RSER_VALUE;
  #endif

  /* Register 'SPI2_MCR' initialization */
  #if SPI2_MCR_MASK_2
    #if SPI2_MCR_MASK_2 == 0xFFFFFFFF
  SPI2_MCR = SPI2_MCR_VALUE_2;
    #elif SPI2_MCR_MASK_2 == SPI2_MCR_VALUE_2
  SPI2_MCR |= SPI2_MCR_VALUE_2;
    #elif SPI2_MCR_VALUE_2 == 0
  SPI2_MCR &= ~SPI2_MCR_MASK_2;
    #else
  SPI2_MCR = (SPI2_MCR & (~SPI2_MCR_MASK_2)) | SPI2_MCR_VALUE_2;
    #endif
  #elif defined(SPI2_MCR_VALUE_2)
  SPI2_MCR = SPI2_MCR_VALUE_2;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END SPI2_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

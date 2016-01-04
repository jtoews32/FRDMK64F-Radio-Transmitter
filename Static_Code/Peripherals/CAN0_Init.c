/** ###################################################################
 **     Filename    : CAN0_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the CAN0 module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void CAN0_Init(void);
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
 * @file CAN0_Init.c
 * @brief This file implements the CAN0 module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE CAN0_Init. */

#include "CAN0_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void CAN0_Init(void) {

  CAN0_MCR |= 0x80000000;

  /* Register 'CAN0_CTRL1' initialization */
  #if CAN0_CTRL1_MASK_1
    #if CAN0_CTRL1_MASK_1 == 0xFFFFFFFF
  CAN0_CTRL1 = CAN0_CTRL1_VALUE_1;
    #elif CAN0_CTRL1_MASK_1 == CAN0_CTRL1_VALUE_1
  CAN0_CTRL1 |= CAN0_CTRL1_VALUE_1;
    #elif CAN0_CTRL1_VALUE_1 == 0
  CAN0_CTRL1 &= ~CAN0_CTRL1_MASK_1;
    #else
  CAN0_CTRL1 = (CAN0_CTRL1 & (~CAN0_CTRL1_MASK_1)) | CAN0_CTRL1_VALUE_1;
    #endif
  #elif defined(CAN0_CTRL1_VALUE_1)
  CAN0_CTRL1 = CAN0_CTRL1_VALUE_1;
  #endif

  /* Register 'CAN0_TIMER' initialization */
  #if CAN0_TIMER_MASK
    #if CAN0_TIMER_MASK == 0xFFFFFFFF
  CAN0_TIMER = CAN0_TIMER_VALUE;
    #elif CAN0_TIMER_MASK == CAN0_TIMER_VALUE
  CAN0_TIMER |= CAN0_TIMER_VALUE;
    #elif CAN0_TIMER_VALUE == 0
  CAN0_TIMER &= ~CAN0_TIMER_MASK;
    #else
  CAN0_TIMER = (CAN0_TIMER & (~CAN0_TIMER_MASK)) | CAN0_TIMER_VALUE;
    #endif
  #elif defined(CAN0_TIMER_VALUE)
  CAN0_TIMER = CAN0_TIMER_VALUE;
  #endif

  CAN0_MCR = (CAN0_MCR & (~0x82000000)) | 0x2000000;
  while(!(CAN0_MCR & CAN_MCR_FRZACK_MASK)){} /* Wait for entering the freeze mode */

  /* Register 'CAN0_CTRL1' initialization */
  #if CAN0_CTRL1_MASK_2
    #if CAN0_CTRL1_MASK_2 == 0xFFFFFFFF
  CAN0_CTRL1 = CAN0_CTRL1_VALUE_2;
    #elif CAN0_CTRL1_MASK_2 == CAN0_CTRL1_VALUE_2
  CAN0_CTRL1 |= CAN0_CTRL1_VALUE_2;
    #elif CAN0_CTRL1_VALUE_2 == 0
  CAN0_CTRL1 &= ~CAN0_CTRL1_MASK_2;
    #else
  CAN0_CTRL1 = (CAN0_CTRL1 & (~CAN0_CTRL1_MASK_2)) | CAN0_CTRL1_VALUE_2;
    #endif
  #elif defined(CAN0_CTRL1_VALUE_2)
  CAN0_CTRL1 = CAN0_CTRL1_VALUE_2;
  #endif

  /* Register 'CAN0_MCR' initialization */
  #if CAN0_MCR_MASK_2
    #if CAN0_MCR_MASK_2 == 0xFFFFFFFF
  CAN0_MCR = CAN0_MCR_VALUE_2;
    #elif CAN0_MCR_MASK_2 == CAN0_MCR_VALUE_2
  CAN0_MCR |= CAN0_MCR_VALUE_2;
    #elif CAN0_MCR_VALUE_2 == 0
  CAN0_MCR &= ~CAN0_MCR_MASK_2;
    #else
  CAN0_MCR = (CAN0_MCR & (~CAN0_MCR_MASK_2)) | CAN0_MCR_VALUE_2;
    #endif
  #elif defined(CAN0_MCR_VALUE_2)
  CAN0_MCR = CAN0_MCR_VALUE_2;
  #endif

  /* Register 'CAN0_CTRL2' initialization */
  #if CAN0_CTRL2_MASK
    #if CAN0_CTRL2_MASK == 0xFFFFFFFF
  CAN0_CTRL2 = CAN0_CTRL2_VALUE;
    #elif CAN0_CTRL2_MASK == CAN0_CTRL2_VALUE
  CAN0_CTRL2 |= CAN0_CTRL2_VALUE;
    #elif CAN0_CTRL2_VALUE == 0
  CAN0_CTRL2 &= ~CAN0_CTRL2_MASK;
    #else
  CAN0_CTRL2 = (CAN0_CTRL2 & (~CAN0_CTRL2_MASK)) | CAN0_CTRL2_VALUE;
    #endif
  #elif defined(CAN0_CTRL2_VALUE)
  CAN0_CTRL2 = CAN0_CTRL2_VALUE;
  #endif

  /* Register 'CAN0_RXMGMASK' initialization */
  #ifdef CAN0_RXMGMASK_VALUE
  CAN0_RXMGMASK = CAN0_RXMGMASK_VALUE;
  #endif

  /* Register 'CAN0_RX14MASK' initialization */
  #ifdef CAN0_RX14MASK_VALUE
  CAN0_RX14MASK = CAN0_RX14MASK_VALUE;
  #endif

  /* Register 'CAN0_RX15MASK' initialization */
  #ifdef CAN0_RX15MASK_VALUE
  CAN0_RX15MASK = CAN0_RX15MASK_VALUE;
  #endif

  /* Register 'CAN0_RXFGMASK' initialization */
  #ifdef CAN0_RXFGMASK_VALUE
  CAN0_RXFGMASK = CAN0_RXFGMASK_VALUE;
  #endif

  /* Register 'CAN0_RXIMR0' initialization */
  #ifdef CAN0_RXIMR0_VALUE
  CAN0_RXIMR0 = CAN0_RXIMR0_VALUE;
  #endif

  /* Register 'CAN0_RXIMR1' initialization */
  #ifdef CAN0_RXIMR1_VALUE
  CAN0_RXIMR1 = CAN0_RXIMR1_VALUE;
  #endif

  /* Register 'CAN0_RXIMR2' initialization */
  #ifdef CAN0_RXIMR2_VALUE
  CAN0_RXIMR2 = CAN0_RXIMR2_VALUE;
  #endif

  /* Register 'CAN0_RXIMR3' initialization */
  #ifdef CAN0_RXIMR3_VALUE
  CAN0_RXIMR3 = CAN0_RXIMR3_VALUE;
  #endif

  /* Register 'CAN0_RXIMR4' initialization */
  #ifdef CAN0_RXIMR4_VALUE
  CAN0_RXIMR4 = CAN0_RXIMR4_VALUE;
  #endif

  /* Register 'CAN0_RXIMR5' initialization */
  #ifdef CAN0_RXIMR5_VALUE
  CAN0_RXIMR5 = CAN0_RXIMR5_VALUE;
  #endif

  /* Register 'CAN0_RXIMR6' initialization */
  #ifdef CAN0_RXIMR6_VALUE
  CAN0_RXIMR6 = CAN0_RXIMR6_VALUE;
  #endif

  /* Register 'CAN0_RXIMR7' initialization */
  #ifdef CAN0_RXIMR7_VALUE
  CAN0_RXIMR7 = CAN0_RXIMR7_VALUE;
  #endif

  /* Register 'CAN0_RXIMR8' initialization */
  #ifdef CAN0_RXIMR8_VALUE
  CAN0_RXIMR8 = CAN0_RXIMR8_VALUE;
  #endif

  /* Register 'CAN0_RXIMR9' initialization */
  #ifdef CAN0_RXIMR9_VALUE
  CAN0_RXIMR9 = CAN0_RXIMR9_VALUE;
  #endif

  /* Register 'CAN0_RXIMR10' initialization */
  #ifdef CAN0_RXIMR10_VALUE
  CAN0_RXIMR10 = CAN0_RXIMR10_VALUE;
  #endif

  /* Register 'CAN0_RXIMR11' initialization */
  #ifdef CAN0_RXIMR11_VALUE
  CAN0_RXIMR11 = CAN0_RXIMR11_VALUE;
  #endif

  /* Register 'CAN0_RXIMR12' initialization */
  #ifdef CAN0_RXIMR12_VALUE
  CAN0_RXIMR12 = CAN0_RXIMR12_VALUE;
  #endif

  /* Register 'CAN0_RXIMR13' initialization */
  #ifdef CAN0_RXIMR13_VALUE
  CAN0_RXIMR13 = CAN0_RXIMR13_VALUE;
  #endif

  /* Register 'CAN0_RXIMR14' initialization */
  #ifdef CAN0_RXIMR14_VALUE
  CAN0_RXIMR14 = CAN0_RXIMR14_VALUE;
  #endif

  /* Register 'CAN0_RXIMR15' initialization */
  #ifdef CAN0_RXIMR15_VALUE
  CAN0_RXIMR15 = CAN0_RXIMR15_VALUE;
  #endif

  CAN0_ESR1 |= 0x30007;

  CAN0_IFLAG1 |= 0xFFFFFFFE;

  /* Register 'CAN0_CTRL1' initialization */
  #if CAN0_CTRL1_MASK_3
    #if CAN0_CTRL1_MASK_3 == 0xFFFFFFFF
  CAN0_CTRL1 = CAN0_CTRL1_VALUE_3;
    #elif CAN0_CTRL1_MASK_3 == CAN0_CTRL1_VALUE_3
  CAN0_CTRL1 |= CAN0_CTRL1_VALUE_3;
    #elif CAN0_CTRL1_VALUE_3 == 0
  CAN0_CTRL1 &= ~CAN0_CTRL1_MASK_3;
    #else
  CAN0_CTRL1 = (CAN0_CTRL1 & (~CAN0_CTRL1_MASK_3)) | CAN0_CTRL1_VALUE_3;
    #endif
  #elif defined(CAN0_CTRL1_VALUE_3)
  CAN0_CTRL1 = CAN0_CTRL1_VALUE_3;
  #endif

  /* Register 'CAN0_MCR' initialization */
  #if CAN0_MCR_MASK_3
    #if CAN0_MCR_MASK_3 == 0xFFFFFFFF
  CAN0_MCR = CAN0_MCR_VALUE_3;
    #elif CAN0_MCR_MASK_3 == CAN0_MCR_VALUE_3
  CAN0_MCR |= CAN0_MCR_VALUE_3;
    #elif CAN0_MCR_VALUE_3 == 0
  CAN0_MCR &= ~CAN0_MCR_MASK_3;
    #else
  CAN0_MCR = (CAN0_MCR & (~CAN0_MCR_MASK_3)) | CAN0_MCR_VALUE_3;
    #endif
  #elif defined(CAN0_MCR_VALUE_3)
  CAN0_MCR = CAN0_MCR_VALUE_3;
  #endif

  /* Register 'CAN0_IMASK1' initialization */
  #if CAN0_IMASK1_MASK
    #if CAN0_IMASK1_MASK == 0xFFFFFFFF
  CAN0_IMASK1 = CAN0_IMASK1_VALUE;
    #elif CAN0_IMASK1_MASK == CAN0_IMASK1_VALUE
  CAN0_IMASK1 |= CAN0_IMASK1_VALUE;
    #elif CAN0_IMASK1_VALUE == 0
  CAN0_IMASK1 &= ~CAN0_IMASK1_MASK;
    #else
  CAN0_IMASK1 = (CAN0_IMASK1 & (~CAN0_IMASK1_MASK)) | CAN0_IMASK1_VALUE;
    #endif
  #elif defined(CAN0_IMASK1_VALUE)
  CAN0_IMASK1 = CAN0_IMASK1_VALUE;
  #endif

  /* Register 'CAN0_MCR' initialization */
  #if CAN0_MCR_MASK_4
    #if CAN0_MCR_MASK_4 == 0xFFFFFFFF
  CAN0_MCR = CAN0_MCR_VALUE_4;
    #elif CAN0_MCR_MASK_4 == CAN0_MCR_VALUE_4
  CAN0_MCR |= CAN0_MCR_VALUE_4;
    #elif CAN0_MCR_VALUE_4 == 0
  CAN0_MCR &= ~CAN0_MCR_MASK_4;
    #else
  CAN0_MCR = (CAN0_MCR & (~CAN0_MCR_MASK_4)) | CAN0_MCR_VALUE_4;
    #endif
  #elif defined(CAN0_MCR_VALUE_4)
  CAN0_MCR = CAN0_MCR_VALUE_4;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END CAN0_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

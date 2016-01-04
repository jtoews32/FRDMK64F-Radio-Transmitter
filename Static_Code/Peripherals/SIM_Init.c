/** ###################################################################
 **     Filename    : SIM_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the SIM module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void SIM_Init(void);
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
 * @file SIM_Init.c
 * @brief This file implements the SIM module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE SIM_Init. */

#include "SIM_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void SIM_Init(void) {

  SIM_SOPT1CFG = 0x7000000;

  /* Register 'SIM_SOPT1' initialization */
  #if SIM_SOPT1_MASK
    #if SIM_SOPT1_MASK == 0xFFFFFFFF
  SIM_SOPT1 = SIM_SOPT1_VALUE;
    #elif SIM_SOPT1_MASK == SIM_SOPT1_VALUE
  SIM_SOPT1 |= SIM_SOPT1_VALUE;
    #elif SIM_SOPT1_VALUE == 0
  SIM_SOPT1 &= ~SIM_SOPT1_MASK;
    #else
  SIM_SOPT1 = (SIM_SOPT1 & (~SIM_SOPT1_MASK)) | SIM_SOPT1_VALUE;
    #endif
  #elif defined(SIM_SOPT1_VALUE)
  SIM_SOPT1 = SIM_SOPT1_VALUE;
  #endif

  /* Register 'SIM_SOPT2' initialization */
  #if SIM_SOPT2_MASK
    #if SIM_SOPT2_MASK == 0xFFFFFFFF
  SIM_SOPT2 = SIM_SOPT2_VALUE;
    #elif SIM_SOPT2_MASK == SIM_SOPT2_VALUE
  SIM_SOPT2 |= SIM_SOPT2_VALUE;
    #elif SIM_SOPT2_VALUE == 0
  SIM_SOPT2 &= ~SIM_SOPT2_MASK;
    #else
  SIM_SOPT2 = (SIM_SOPT2 & (~SIM_SOPT2_MASK)) | SIM_SOPT2_VALUE;
    #endif
  #elif defined(SIM_SOPT2_VALUE)
  SIM_SOPT2 = SIM_SOPT2_VALUE;
  #endif

  /* Register 'SIM_CLKDIV2' initialization */
  #if SIM_CLKDIV2_MASK
    #if SIM_CLKDIV2_MASK == 0xFFFFFFFF
  SIM_CLKDIV2 = SIM_CLKDIV2_VALUE;
    #elif SIM_CLKDIV2_MASK == SIM_CLKDIV2_VALUE
  SIM_CLKDIV2 |= SIM_CLKDIV2_VALUE;
    #elif SIM_CLKDIV2_VALUE == 0
  SIM_CLKDIV2 &= ~SIM_CLKDIV2_MASK;
    #else
  SIM_CLKDIV2 = (SIM_CLKDIV2 & (~SIM_CLKDIV2_MASK)) | SIM_CLKDIV2_VALUE;
    #endif
  #elif defined(SIM_CLKDIV2_VALUE)
  SIM_CLKDIV2 = SIM_CLKDIV2_VALUE;
  #endif

  /* Register 'SIM_SOPT4' initialization */
  #if SIM_SOPT4_MASK
    #if SIM_SOPT4_MASK == 0xFFFFFFFF
  SIM_SOPT4 = SIM_SOPT4_VALUE;
    #elif SIM_SOPT4_MASK == SIM_SOPT4_VALUE
  SIM_SOPT4 |= SIM_SOPT4_VALUE;
    #elif SIM_SOPT4_VALUE == 0
  SIM_SOPT4 &= ~SIM_SOPT4_MASK;
    #else
  SIM_SOPT4 = (SIM_SOPT4 & (~SIM_SOPT4_MASK)) | SIM_SOPT4_VALUE;
    #endif
  #elif defined(SIM_SOPT4_VALUE)
  SIM_SOPT4 = SIM_SOPT4_VALUE;
  #endif

  /* Register 'SIM_SOPT5' initialization */
  #if SIM_SOPT5_MASK
    #if SIM_SOPT5_MASK == 0xFFFFFFFF
  SIM_SOPT5 = SIM_SOPT5_VALUE;
    #elif SIM_SOPT5_MASK == SIM_SOPT5_VALUE
  SIM_SOPT5 |= SIM_SOPT5_VALUE;
    #elif SIM_SOPT5_VALUE == 0
  SIM_SOPT5 &= ~SIM_SOPT5_MASK;
    #else
  SIM_SOPT5 = (SIM_SOPT5 & (~SIM_SOPT5_MASK)) | SIM_SOPT5_VALUE;
    #endif
  #elif defined(SIM_SOPT5_VALUE)
  SIM_SOPT5 = SIM_SOPT5_VALUE;
  #endif

  /* Register 'SIM_SOPT7' initialization */
  #ifdef SIM_SOPT7_VALUE
  SIM_SOPT7 = SIM_SOPT7_VALUE;
  #endif

  /* Register 'SIM_SCGC1' initialization */
  #if SIM_SCGC1_MASK
    #if SIM_SCGC1_MASK == 0xFFFFFFFF
  SIM_SCGC1 = SIM_SCGC1_VALUE;
    #elif SIM_SCGC1_MASK == SIM_SCGC1_VALUE
  SIM_SCGC1 |= SIM_SCGC1_VALUE;
    #elif SIM_SCGC1_VALUE == 0
  SIM_SCGC1 &= ~SIM_SCGC1_MASK;
    #else
  SIM_SCGC1 = (SIM_SCGC1 & (~SIM_SCGC1_MASK)) | SIM_SCGC1_VALUE;
    #endif
  #elif defined(SIM_SCGC1_VALUE)
  SIM_SCGC1 = SIM_SCGC1_VALUE;
  #endif

  /* Register 'SIM_SCGC2' initialization */
  #if SIM_SCGC2_MASK
    #if SIM_SCGC2_MASK == 0xFFFFFFFF
  SIM_SCGC2 = SIM_SCGC2_VALUE;
    #elif SIM_SCGC2_MASK == SIM_SCGC2_VALUE
  SIM_SCGC2 |= SIM_SCGC2_VALUE;
    #elif SIM_SCGC2_VALUE == 0
  SIM_SCGC2 &= ~SIM_SCGC2_MASK;
    #else
  SIM_SCGC2 = (SIM_SCGC2 & (~SIM_SCGC2_MASK)) | SIM_SCGC2_VALUE;
    #endif
  #elif defined(SIM_SCGC2_VALUE)
  SIM_SCGC2 = SIM_SCGC2_VALUE;
  #endif

  /* Register 'SIM_SCGC3' initialization */
  #if SIM_SCGC3_MASK
    #if SIM_SCGC3_MASK == 0xFFFFFFFF
  SIM_SCGC3 = SIM_SCGC3_VALUE;
    #elif SIM_SCGC3_MASK == SIM_SCGC3_VALUE
  SIM_SCGC3 |= SIM_SCGC3_VALUE;
    #elif SIM_SCGC3_VALUE == 0
  SIM_SCGC3 &= ~SIM_SCGC3_MASK;
    #else
  SIM_SCGC3 = (SIM_SCGC3 & (~SIM_SCGC3_MASK)) | SIM_SCGC3_VALUE;
    #endif
  #elif defined(SIM_SCGC3_VALUE)
  SIM_SCGC3 = SIM_SCGC3_VALUE;
  #endif

  /* Register 'SIM_SCGC4' initialization */
  #if SIM_SCGC4_MASK
    #if SIM_SCGC4_MASK == 0xFFFFFFFF
  SIM_SCGC4 = SIM_SCGC4_VALUE;
    #elif SIM_SCGC4_MASK == SIM_SCGC4_VALUE
  SIM_SCGC4 |= SIM_SCGC4_VALUE;
    #elif SIM_SCGC4_VALUE == 0
  SIM_SCGC4 &= ~SIM_SCGC4_MASK;
    #else
  SIM_SCGC4 = (SIM_SCGC4 & (~SIM_SCGC4_MASK)) | SIM_SCGC4_VALUE;
    #endif
  #elif defined(SIM_SCGC4_VALUE)
  SIM_SCGC4 = SIM_SCGC4_VALUE;
  #endif

  /* Register 'SIM_SCGC5' initialization */
  #if SIM_SCGC5_MASK
    #if SIM_SCGC5_MASK == 0xFFFFFFFF
  SIM_SCGC5 = SIM_SCGC5_VALUE;
    #elif SIM_SCGC5_MASK == SIM_SCGC5_VALUE
  SIM_SCGC5 |= SIM_SCGC5_VALUE;
    #elif SIM_SCGC5_VALUE == 0
  SIM_SCGC5 &= ~SIM_SCGC5_MASK;
    #else
  SIM_SCGC5 = (SIM_SCGC5 & (~SIM_SCGC5_MASK)) | SIM_SCGC5_VALUE;
    #endif
  #elif defined(SIM_SCGC5_VALUE)
  SIM_SCGC5 = SIM_SCGC5_VALUE;
  #endif

  /* Register 'SIM_SCGC6' initialization */
  #if SIM_SCGC6_MASK
    #if SIM_SCGC6_MASK == 0xFFFFFFFF
  SIM_SCGC6 = SIM_SCGC6_VALUE;
    #elif SIM_SCGC6_MASK == SIM_SCGC6_VALUE
  SIM_SCGC6 |= SIM_SCGC6_VALUE;
    #elif SIM_SCGC6_VALUE == 0
  SIM_SCGC6 &= ~SIM_SCGC6_MASK;
    #else
  SIM_SCGC6 = (SIM_SCGC6 & (~SIM_SCGC6_MASK)) | SIM_SCGC6_VALUE;
    #endif
  #elif defined(SIM_SCGC6_VALUE)
  SIM_SCGC6 = SIM_SCGC6_VALUE;
  #endif

  /* Register 'SIM_SCGC7' initialization */
  #if SIM_SCGC7_MASK
    #if SIM_SCGC7_MASK == 0xFFFFFFFF
  SIM_SCGC7 = SIM_SCGC7_VALUE;
    #elif SIM_SCGC7_MASK == SIM_SCGC7_VALUE
  SIM_SCGC7 |= SIM_SCGC7_VALUE;
    #elif SIM_SCGC7_VALUE == 0
  SIM_SCGC7 &= ~SIM_SCGC7_MASK;
    #else
  SIM_SCGC7 = (SIM_SCGC7 & (~SIM_SCGC7_MASK)) | SIM_SCGC7_VALUE;
    #endif
  #elif defined(SIM_SCGC7_VALUE)
  SIM_SCGC7 = SIM_SCGC7_VALUE;
  #endif

  /* Register 'SIM_CLKDIV1' initialization */
  #if SIM_CLKDIV1_MASK
    #if SIM_CLKDIV1_MASK == 0xFFFFFFFF
  SIM_CLKDIV1 = SIM_CLKDIV1_VALUE;
    #elif SIM_CLKDIV1_MASK == SIM_CLKDIV1_VALUE
  SIM_CLKDIV1 |= SIM_CLKDIV1_VALUE;
    #elif SIM_CLKDIV1_VALUE == 0
  SIM_CLKDIV1 &= ~SIM_CLKDIV1_MASK;
    #else
  SIM_CLKDIV1 = (SIM_CLKDIV1 & (~SIM_CLKDIV1_MASK)) | SIM_CLKDIV1_VALUE;
    #endif
  #elif defined(SIM_CLKDIV1_VALUE)
  SIM_CLKDIV1 = SIM_CLKDIV1_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END SIM_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

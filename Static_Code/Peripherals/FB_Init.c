/** ###################################################################
 **     Filename    : FB_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the FB module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void FB_Init(void);
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
 * @file FB_Init.c
 * @brief This file implements the FB module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service routines
 *        prototypes.
 */


/* MODULE FB_Init. */

#include "FB_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void FB_Init(void) {

  /* Register 'FB_CSAR0' initialization */
  #ifdef FB_CSAR0_VALUE
  FB_CSAR0 = FB_CSAR0_VALUE;
  #endif

  /* Register 'FB_CSAR1' initialization */
  #ifdef FB_CSAR1_VALUE
  FB_CSAR1 = FB_CSAR1_VALUE;
  #endif

  /* Register 'FB_CSAR2' initialization */
  #ifdef FB_CSAR2_VALUE
  FB_CSAR2 = FB_CSAR2_VALUE;
  #endif

  /* Register 'FB_CSAR4' initialization */
  #ifdef FB_CSAR4_VALUE
  FB_CSAR4 = FB_CSAR4_VALUE;
  #endif

  /* Register 'FB_CSAR5' initialization */
  #ifdef FB_CSAR5_VALUE
  FB_CSAR5 = FB_CSAR5_VALUE;
  #endif

  /* Register 'FB_CSCR0' initialization */
  #if FB_CSCR0_MASK
    #if FB_CSCR0_MASK == 0xFFFFFFFF
  FB_CSCR0 = FB_CSCR0_VALUE;
    #elif FB_CSCR0_MASK == FB_CSCR0_VALUE
  FB_CSCR0 |= FB_CSCR0_VALUE;
    #elif FB_CSCR0_VALUE == 0
  FB_CSCR0 &= ~FB_CSCR0_MASK;
    #else
  FB_CSCR0 = (FB_CSCR0 & (~FB_CSCR0_MASK)) | FB_CSCR0_VALUE;
    #endif
  #elif defined(FB_CSCR0_VALUE)
  FB_CSCR0 = FB_CSCR0_VALUE;
  #endif

  /* Register 'FB_CSCR1' initialization */
  #if FB_CSCR1_MASK
    #if FB_CSCR1_MASK == 0xFFFFFFFF
  FB_CSCR1 = FB_CSCR1_VALUE;
    #elif FB_CSCR1_MASK == FB_CSCR1_VALUE
  FB_CSCR1 |= FB_CSCR1_VALUE;
    #elif FB_CSCR1_VALUE == 0
  FB_CSCR1 &= ~FB_CSCR1_MASK;
    #else
  FB_CSCR1 = (FB_CSCR1 & (~FB_CSCR1_MASK)) | FB_CSCR1_VALUE;
    #endif
  #elif defined(FB_CSCR1_VALUE)
  FB_CSCR1 = FB_CSCR1_VALUE;
  #endif

  /* Register 'FB_CSCR2' initialization */
  #if FB_CSCR2_MASK
    #if FB_CSCR2_MASK == 0xFFFFFFFF
  FB_CSCR2 = FB_CSCR2_VALUE;
    #elif FB_CSCR2_MASK == FB_CSCR2_VALUE
  FB_CSCR2 |= FB_CSCR2_VALUE;
    #elif FB_CSCR2_VALUE == 0
  FB_CSCR2 &= ~FB_CSCR2_MASK;
    #else
  FB_CSCR2 = (FB_CSCR2 & (~FB_CSCR2_MASK)) | FB_CSCR2_VALUE;
    #endif
  #elif defined(FB_CSCR2_VALUE)
  FB_CSCR2 = FB_CSCR2_VALUE;
  #endif

  /* Register 'FB_CSCR4' initialization */
  #if FB_CSCR4_MASK
    #if FB_CSCR4_MASK == 0xFFFFFFFF
  FB_CSCR4 = FB_CSCR4_VALUE;
    #elif FB_CSCR4_MASK == FB_CSCR4_VALUE
  FB_CSCR4 |= FB_CSCR4_VALUE;
    #elif FB_CSCR4_VALUE == 0
  FB_CSCR4 &= ~FB_CSCR4_MASK;
    #else
  FB_CSCR4 = (FB_CSCR4 & (~FB_CSCR4_MASK)) | FB_CSCR4_VALUE;
    #endif
  #elif defined(FB_CSCR4_VALUE)
  FB_CSCR4 = FB_CSCR4_VALUE;
  #endif

  /* Register 'FB_CSCR5' initialization */
  #if FB_CSCR5_MASK
    #if FB_CSCR5_MASK == 0xFFFFFFFF
  FB_CSCR5 = FB_CSCR5_VALUE;
    #elif FB_CSCR5_MASK == FB_CSCR5_VALUE
  FB_CSCR5 |= FB_CSCR5_VALUE;
    #elif FB_CSCR5_VALUE == 0
  FB_CSCR5 &= ~FB_CSCR5_MASK;
    #else
  FB_CSCR5 = (FB_CSCR5 & (~FB_CSCR5_MASK)) | FB_CSCR5_VALUE;
    #endif
  #elif defined(FB_CSCR5_VALUE)
  FB_CSCR5 = FB_CSCR5_VALUE;
  #endif

  /* Register 'FB_CSMR0' initialization */
  #if FB_CSMR0_MASK
    #if FB_CSMR0_MASK == 0xFFFFFFFF
  FB_CSMR0 = FB_CSMR0_VALUE;
    #elif FB_CSMR0_MASK == FB_CSMR0_VALUE
  FB_CSMR0 |= FB_CSMR0_VALUE;
    #elif FB_CSMR0_VALUE == 0
  FB_CSMR0 &= ~FB_CSMR0_MASK;
    #else
  FB_CSMR0 = (FB_CSMR0 & (~FB_CSMR0_MASK)) | FB_CSMR0_VALUE;
    #endif
  #elif defined(FB_CSMR0_VALUE)
  FB_CSMR0 = FB_CSMR0_VALUE;
  #endif

  /* Register 'FB_CSMR1' initialization */
  #if FB_CSMR1_MASK
    #if FB_CSMR1_MASK == 0xFFFFFFFF
  FB_CSMR1 = FB_CSMR1_VALUE;
    #elif FB_CSMR1_MASK == FB_CSMR1_VALUE
  FB_CSMR1 |= FB_CSMR1_VALUE;
    #elif FB_CSMR1_VALUE == 0
  FB_CSMR1 &= ~FB_CSMR1_MASK;
    #else
  FB_CSMR1 = (FB_CSMR1 & (~FB_CSMR1_MASK)) | FB_CSMR1_VALUE;
    #endif
  #elif defined(FB_CSMR1_VALUE)
  FB_CSMR1 = FB_CSMR1_VALUE;
  #endif

  /* Register 'FB_CSMR2' initialization */
  #if FB_CSMR2_MASK
    #if FB_CSMR2_MASK == 0xFFFFFFFF
  FB_CSMR2 = FB_CSMR2_VALUE;
    #elif FB_CSMR2_MASK == FB_CSMR2_VALUE
  FB_CSMR2 |= FB_CSMR2_VALUE;
    #elif FB_CSMR2_VALUE == 0
  FB_CSMR2 &= ~FB_CSMR2_MASK;
    #else
  FB_CSMR2 = (FB_CSMR2 & (~FB_CSMR2_MASK)) | FB_CSMR2_VALUE;
    #endif
  #elif defined(FB_CSMR2_VALUE)
  FB_CSMR2 = FB_CSMR2_VALUE;
  #endif

  /* Register 'FB_CSMR4' initialization */
  #if FB_CSMR4_MASK
    #if FB_CSMR4_MASK == 0xFFFFFFFF
  FB_CSMR4 = FB_CSMR4_VALUE;
    #elif FB_CSMR4_MASK == FB_CSMR4_VALUE
  FB_CSMR4 |= FB_CSMR4_VALUE;
    #elif FB_CSMR4_VALUE == 0
  FB_CSMR4 &= ~FB_CSMR4_MASK;
    #else
  FB_CSMR4 = (FB_CSMR4 & (~FB_CSMR4_MASK)) | FB_CSMR4_VALUE;
    #endif
  #elif defined(FB_CSMR4_VALUE)
  FB_CSMR4 = FB_CSMR4_VALUE;
  #endif

  /* Register 'FB_CSMR5' initialization */
  #if FB_CSMR5_MASK
    #if FB_CSMR5_MASK == 0xFFFFFFFF
  FB_CSMR5 = FB_CSMR5_VALUE;
    #elif FB_CSMR5_MASK == FB_CSMR5_VALUE
  FB_CSMR5 |= FB_CSMR5_VALUE;
    #elif FB_CSMR5_VALUE == 0
  FB_CSMR5 &= ~FB_CSMR5_MASK;
    #else
  FB_CSMR5 = (FB_CSMR5 & (~FB_CSMR5_MASK)) | FB_CSMR5_VALUE;
    #endif
  #elif defined(FB_CSMR5_VALUE)
  FB_CSMR5 = FB_CSMR5_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END FB_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

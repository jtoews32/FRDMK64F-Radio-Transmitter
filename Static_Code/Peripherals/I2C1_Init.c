/** ###################################################################
 **     Filename    : I2C1_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the I2C1 module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void I2C1_Init(void);
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
 * @file I2C1_Init.c
 * @brief This file implements the I2C1 module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE I2C1_Init. */

#include "I2C1_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void I2C1_Init(void) {

  /* Register 'I2C1_FLT' initialization */
  #ifdef I2C1_FLT_VALUE
  I2C1_FLT = I2C1_FLT_VALUE;
  #endif

  /* Register 'I2C1_A1' initialization */
  #ifdef I2C1_A1_VALUE
  I2C1_A1 = I2C1_A1_VALUE;
  #endif

  /* Register 'I2C1_C2' initialization */
  #ifdef I2C1_C2_VALUE
  I2C1_C2 = I2C1_C2_VALUE;
  #endif

  /* Register 'I2C1_RA' initialization */
  #ifdef I2C1_RA_VALUE
  I2C1_RA = I2C1_RA_VALUE;
  #endif

  /* Register 'I2C1_F' initialization */
  #ifdef I2C1_F_VALUE
  I2C1_F = I2C1_F_VALUE;
  #endif

  /* Register 'I2C1_A2' initialization */
  #ifdef I2C1_A2_VALUE
  I2C1_A2 = I2C1_A2_VALUE;
  #endif

  /* Register 'I2C1_SMB' initialization */
  #ifdef I2C1_SMB_VALUE
  I2C1_SMB = I2C1_SMB_VALUE;
  #endif

  /* Register 'I2C1_SLTL' initialization */
  #ifdef I2C1_SLTL_VALUE
  I2C1_SLTL = I2C1_SLTL_VALUE;
  #endif

  /* Register 'I2C1_SLTH' initialization */
  #ifdef I2C1_SLTH_VALUE
  I2C1_SLTH = I2C1_SLTH_VALUE;
  #endif

  /* Register 'I2C1_S' initialization */
  #ifdef I2C1_S_VALUE
  I2C1_S = I2C1_S_VALUE;
  #endif

  /* Register 'I2C1_C1' initialization */
  #ifdef I2C1_C1_VALUE_1
  I2C1_C1 = I2C1_C1_VALUE_1;
  #endif

  /* Register 'I2C1_C1' initialization */
  #if I2C1_C1_MASK_2
    #if I2C1_C1_MASK_2 == 0xFF
  I2C1_C1 = I2C1_C1_VALUE_2;
    #elif I2C1_C1_MASK_2 == I2C1_C1_VALUE_2
  I2C1_C1 |= I2C1_C1_VALUE_2;
    #elif I2C1_C1_VALUE_2 == 0
  I2C1_C1 &= ~I2C1_C1_MASK_2;
    #else
  I2C1_C1 = (I2C1_C1 & (~I2C1_C1_MASK_2)) | I2C1_C1_VALUE_2;
    #endif
  #elif defined(I2C1_C1_VALUE_2)
  I2C1_C1 = I2C1_C1_VALUE_2;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END I2C1_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

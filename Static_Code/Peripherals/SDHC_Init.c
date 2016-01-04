/** ###################################################################
 **     Filename    : SDHC_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the SDHC module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void SDHC_Init(void);
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
 * @file SDHC_Init.c
 * @brief This file implements the SDHC module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE SDHC_Init. */

#include "SDHC_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void SDHC_Init(void) {

  SDHC_SYSCTL |= 0x1000000;
    while(SDHC_SYSCTL & (uint32_t)(SDHC_SYSCTL_RSTA_MASK)){} /* Wait for RSTA = 0 */

  /* Register 'SDHC_SYSCTL' initialization */
  #ifdef SDHC_SYSCTL_VALUE_2
  SDHC_SYSCTL = SDHC_SYSCTL_VALUE_2;
  #endif

  /* Register 'SDHC_BLKATTR' initialization */
  #ifdef SDHC_BLKATTR_VALUE
  SDHC_BLKATTR = SDHC_BLKATTR_VALUE;
  #endif

  /* Register 'SDHC_WML' initialization */
  #ifdef SDHC_WML_VALUE
  SDHC_WML = SDHC_WML_VALUE;
  #endif
  #ifdef SDHC_ADMA_SYSTEM_ADDRESS
  SDHC_ADSADDR = (uint32_t)SDHC_ADMA_SYSTEM_ADDRESS;
  #endif
  #ifdef SDHC_DMA_SYSTEM_ADDRESS
  SDHC_DSADDR = (uint32_t)SDHC_DMA_SYSTEM_ADDRESS;
  #endif

  /* Register 'SDHC_VENDOR' initialization */
  #ifdef SDHC_VENDOR_VALUE
  SDHC_VENDOR = SDHC_VENDOR_VALUE;
  #endif

  /* Register 'SDHC_PROCTL' initialization */
  #ifdef SDHC_PROCTL_VALUE
  SDHC_PROCTL = SDHC_PROCTL_VALUE;
  #endif

  /* Register 'SDHC_IRQSTAT' initialization */
  #if SDHC_IRQSTAT_MASK
    #if SDHC_IRQSTAT_MASK == 0xFFFFFFFF
  SDHC_IRQSTAT = SDHC_IRQSTAT_VALUE;
    #elif SDHC_IRQSTAT_MASK == SDHC_IRQSTAT_VALUE
  SDHC_IRQSTAT |= SDHC_IRQSTAT_VALUE;
    #elif SDHC_IRQSTAT_VALUE == 0
  SDHC_IRQSTAT &= ~SDHC_IRQSTAT_MASK;
    #else
  SDHC_IRQSTAT = (SDHC_IRQSTAT & (~SDHC_IRQSTAT_MASK)) | SDHC_IRQSTAT_VALUE;
    #endif
  #elif defined(SDHC_IRQSTAT_VALUE)
  SDHC_IRQSTAT = SDHC_IRQSTAT_VALUE;
  #endif

  /* Register 'SDHC_IRQSIGEN' initialization */
  #ifdef SDHC_IRQSIGEN_VALUE
  SDHC_IRQSIGEN = SDHC_IRQSIGEN_VALUE;
  #endif

  /* Register 'SDHC_IRQSTATEN' initialization */
  #ifdef SDHC_IRQSTATEN_VALUE
  SDHC_IRQSTATEN = SDHC_IRQSTATEN_VALUE;
  #endif

  /* Register 'SDHC_MMCBOOT' initialization */
  #ifdef SDHC_MMCBOOT_VALUE
  SDHC_MMCBOOT = SDHC_MMCBOOT_VALUE;
  #endif

  /* Register 'SDHC_SYSCTL' initialization */
  #if SDHC_SYSCTL_MASK_3
    #if SDHC_SYSCTL_MASK_3 == 0xFFFFFFFF
  SDHC_SYSCTL = SDHC_SYSCTL_VALUE_3;
    #elif SDHC_SYSCTL_MASK_3 == SDHC_SYSCTL_VALUE_3
  SDHC_SYSCTL |= SDHC_SYSCTL_VALUE_3;
    #elif SDHC_SYSCTL_VALUE_3 == 0
  SDHC_SYSCTL &= ~SDHC_SYSCTL_MASK_3;
    #else
  SDHC_SYSCTL = (SDHC_SYSCTL & (~SDHC_SYSCTL_MASK_3)) | SDHC_SYSCTL_VALUE_3;
    #endif
  #elif defined(SDHC_SYSCTL_VALUE_3)
  SDHC_SYSCTL = SDHC_SYSCTL_VALUE_3;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END SDHC_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

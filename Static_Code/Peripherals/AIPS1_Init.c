/** ###################################################################
 **     Filename    : AIPS1_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the AIPS1 module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void AIPS1_Init(void);
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
 * @file AIPS1_Init.c
 * @brief This file implements the AIPS1 module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE AIPS1_Init. */

#include "AIPS1_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void AIPS1_Init(void) {

  /* Register 'AIPS1_MPRA' initialization */
  #if AIPS1_MPRA_MASK
    #if AIPS1_MPRA_MASK == 0xFFFFFFFF
  AIPS1_MPRA = AIPS1_MPRA_VALUE;
    #elif AIPS1_MPRA_MASK == AIPS1_MPRA_VALUE
  AIPS1_MPRA |= AIPS1_MPRA_VALUE;
    #elif AIPS1_MPRA_VALUE == 0
  AIPS1_MPRA &= ~AIPS1_MPRA_MASK;
    #else
  AIPS1_MPRA = (AIPS1_MPRA & (~AIPS1_MPRA_MASK)) | AIPS1_MPRA_VALUE;
    #endif
  #elif defined(AIPS1_MPRA_VALUE)
  AIPS1_MPRA = AIPS1_MPRA_VALUE;
  #endif

  /* Register 'AIPS1_PACRA' initialization */
  #if AIPS1_PACRA_MASK
    #if AIPS1_PACRA_MASK == 0xFFFFFFFF
  AIPS1_PACRA = AIPS1_PACRA_VALUE;
    #elif AIPS1_PACRA_MASK == AIPS1_PACRA_VALUE
  AIPS1_PACRA |= AIPS1_PACRA_VALUE;
    #elif AIPS1_PACRA_VALUE == 0
  AIPS1_PACRA &= ~AIPS1_PACRA_MASK;
    #else
  AIPS1_PACRA = (AIPS1_PACRA & (~AIPS1_PACRA_MASK)) | AIPS1_PACRA_VALUE;
    #endif
  #elif defined(AIPS1_PACRA_VALUE)
  AIPS1_PACRA = AIPS1_PACRA_VALUE;
  #endif

  /* Register 'AIPS1_PACRF' initialization */
  #if AIPS1_PACRF_MASK
    #if AIPS1_PACRF_MASK == 0xFFFFFFFF
  AIPS1_PACRF = AIPS1_PACRF_VALUE;
    #elif AIPS1_PACRF_MASK == AIPS1_PACRF_VALUE
  AIPS1_PACRF |= AIPS1_PACRF_VALUE;
    #elif AIPS1_PACRF_VALUE == 0
  AIPS1_PACRF &= ~AIPS1_PACRF_MASK;
    #else
  AIPS1_PACRF = (AIPS1_PACRF & (~AIPS1_PACRF_MASK)) | AIPS1_PACRF_VALUE;
    #endif
  #elif defined(AIPS1_PACRF_VALUE)
  AIPS1_PACRF = AIPS1_PACRF_VALUE;
  #endif

  /* Register 'AIPS1_PACRG' initialization */
  #if AIPS1_PACRG_MASK
    #if AIPS1_PACRG_MASK == 0xFFFFFFFF
  AIPS1_PACRG = AIPS1_PACRG_VALUE;
    #elif AIPS1_PACRG_MASK == AIPS1_PACRG_VALUE
  AIPS1_PACRG |= AIPS1_PACRG_VALUE;
    #elif AIPS1_PACRG_VALUE == 0
  AIPS1_PACRG &= ~AIPS1_PACRG_MASK;
    #else
  AIPS1_PACRG = (AIPS1_PACRG & (~AIPS1_PACRG_MASK)) | AIPS1_PACRG_VALUE;
    #endif
  #elif defined(AIPS1_PACRG_VALUE)
  AIPS1_PACRG = AIPS1_PACRG_VALUE;
  #endif

  /* Register 'AIPS1_PACRH' initialization */
  #if AIPS1_PACRH_MASK
    #if AIPS1_PACRH_MASK == 0xFFFFFFFF
  AIPS1_PACRH = AIPS1_PACRH_VALUE;
    #elif AIPS1_PACRH_MASK == AIPS1_PACRH_VALUE
  AIPS1_PACRH |= AIPS1_PACRH_VALUE;
    #elif AIPS1_PACRH_VALUE == 0
  AIPS1_PACRH &= ~AIPS1_PACRH_MASK;
    #else
  AIPS1_PACRH = (AIPS1_PACRH & (~AIPS1_PACRH_MASK)) | AIPS1_PACRH_VALUE;
    #endif
  #elif defined(AIPS1_PACRH_VALUE)
  AIPS1_PACRH = AIPS1_PACRH_VALUE;
  #endif

  /* Register 'AIPS1_PACRI' initialization */
  #if AIPS1_PACRI_MASK
    #if AIPS1_PACRI_MASK == 0xFFFFFFFF
  AIPS1_PACRI = AIPS1_PACRI_VALUE;
    #elif AIPS1_PACRI_MASK == AIPS1_PACRI_VALUE
  AIPS1_PACRI |= AIPS1_PACRI_VALUE;
    #elif AIPS1_PACRI_VALUE == 0
  AIPS1_PACRI &= ~AIPS1_PACRI_MASK;
    #else
  AIPS1_PACRI = (AIPS1_PACRI & (~AIPS1_PACRI_MASK)) | AIPS1_PACRI_VALUE;
    #endif
  #elif defined(AIPS1_PACRI_VALUE)
  AIPS1_PACRI = AIPS1_PACRI_VALUE;
  #endif

  /* Register 'AIPS1_PACRJ' initialization */
  #if AIPS1_PACRJ_MASK
    #if AIPS1_PACRJ_MASK == 0xFFFFFFFF
  AIPS1_PACRJ = AIPS1_PACRJ_VALUE;
    #elif AIPS1_PACRJ_MASK == AIPS1_PACRJ_VALUE
  AIPS1_PACRJ |= AIPS1_PACRJ_VALUE;
    #elif AIPS1_PACRJ_VALUE == 0
  AIPS1_PACRJ &= ~AIPS1_PACRJ_MASK;
    #else
  AIPS1_PACRJ = (AIPS1_PACRJ & (~AIPS1_PACRJ_MASK)) | AIPS1_PACRJ_VALUE;
    #endif
  #elif defined(AIPS1_PACRJ_VALUE)
  AIPS1_PACRJ = AIPS1_PACRJ_VALUE;
  #endif

  /* Register 'AIPS1_PACRM' initialization */
  #if AIPS1_PACRM_MASK
    #if AIPS1_PACRM_MASK == 0xFFFFFFFF
  AIPS1_PACRM = AIPS1_PACRM_VALUE;
    #elif AIPS1_PACRM_MASK == AIPS1_PACRM_VALUE
  AIPS1_PACRM |= AIPS1_PACRM_VALUE;
    #elif AIPS1_PACRM_VALUE == 0
  AIPS1_PACRM &= ~AIPS1_PACRM_MASK;
    #else
  AIPS1_PACRM = (AIPS1_PACRM & (~AIPS1_PACRM_MASK)) | AIPS1_PACRM_VALUE;
    #endif
  #elif defined(AIPS1_PACRM_VALUE)
  AIPS1_PACRM = AIPS1_PACRM_VALUE;
  #endif

  /* Register 'AIPS1_PACRN' initialization */
  #if AIPS1_PACRN_MASK
    #if AIPS1_PACRN_MASK == 0xFFFFFFFF
  AIPS1_PACRN = AIPS1_PACRN_VALUE;
    #elif AIPS1_PACRN_MASK == AIPS1_PACRN_VALUE
  AIPS1_PACRN |= AIPS1_PACRN_VALUE;
    #elif AIPS1_PACRN_VALUE == 0
  AIPS1_PACRN &= ~AIPS1_PACRN_MASK;
    #else
  AIPS1_PACRN = (AIPS1_PACRN & (~AIPS1_PACRN_MASK)) | AIPS1_PACRN_VALUE;
    #endif
  #elif defined(AIPS1_PACRN_VALUE)
  AIPS1_PACRN = AIPS1_PACRN_VALUE;
  #endif

  /* Register 'AIPS1_PACRU' initialization */
  #if AIPS1_PACRU_MASK
    #if AIPS1_PACRU_MASK == 0xFFFFFFFF
  AIPS1_PACRU = AIPS1_PACRU_VALUE;
    #elif AIPS1_PACRU_MASK == AIPS1_PACRU_VALUE
  AIPS1_PACRU |= AIPS1_PACRU_VALUE;
    #elif AIPS1_PACRU_VALUE == 0
  AIPS1_PACRU &= ~AIPS1_PACRU_MASK;
    #else
  AIPS1_PACRU = (AIPS1_PACRU & (~AIPS1_PACRU_MASK)) | AIPS1_PACRU_VALUE;
    #endif
  #elif defined(AIPS1_PACRU_VALUE)
  AIPS1_PACRU = AIPS1_PACRU_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END AIPS1_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

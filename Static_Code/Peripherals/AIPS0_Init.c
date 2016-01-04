/** ###################################################################
 **     Filename    : AIPS0_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the AIPS0 module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void AIPS0_Init(void);
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
 * @file AIPS0_Init.c
 * @brief This file implements the AIPS0 module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE AIPS0_Init. */

#include "AIPS0_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void AIPS0_Init(void) {

  /* Register 'AIPS0_MPRA' initialization */
  #if AIPS0_MPRA_MASK
    #if AIPS0_MPRA_MASK == 0xFFFFFFFF
  AIPS0_MPRA = AIPS0_MPRA_VALUE;
    #elif AIPS0_MPRA_MASK == AIPS0_MPRA_VALUE
  AIPS0_MPRA |= AIPS0_MPRA_VALUE;
    #elif AIPS0_MPRA_VALUE == 0
  AIPS0_MPRA &= ~AIPS0_MPRA_MASK;
    #else
  AIPS0_MPRA = (AIPS0_MPRA & (~AIPS0_MPRA_MASK)) | AIPS0_MPRA_VALUE;
    #endif
  #elif defined(AIPS0_MPRA_VALUE)
  AIPS0_MPRA = AIPS0_MPRA_VALUE;
  #endif

  /* Register 'AIPS0_PACRA' initialization */
  #if AIPS0_PACRA_MASK
    #if AIPS0_PACRA_MASK == 0xFFFFFFFF
  AIPS0_PACRA = AIPS0_PACRA_VALUE;
    #elif AIPS0_PACRA_MASK == AIPS0_PACRA_VALUE
  AIPS0_PACRA |= AIPS0_PACRA_VALUE;
    #elif AIPS0_PACRA_VALUE == 0
  AIPS0_PACRA &= ~AIPS0_PACRA_MASK;
    #else
  AIPS0_PACRA = (AIPS0_PACRA & (~AIPS0_PACRA_MASK)) | AIPS0_PACRA_VALUE;
    #endif
  #elif defined(AIPS0_PACRA_VALUE)
  AIPS0_PACRA = AIPS0_PACRA_VALUE;
  #endif

  /* Register 'AIPS0_PACRB' initialization */
  #if AIPS0_PACRB_MASK
    #if AIPS0_PACRB_MASK == 0xFFFFFFFF
  AIPS0_PACRB = AIPS0_PACRB_VALUE;
    #elif AIPS0_PACRB_MASK == AIPS0_PACRB_VALUE
  AIPS0_PACRB |= AIPS0_PACRB_VALUE;
    #elif AIPS0_PACRB_VALUE == 0
  AIPS0_PACRB &= ~AIPS0_PACRB_MASK;
    #else
  AIPS0_PACRB = (AIPS0_PACRB & (~AIPS0_PACRB_MASK)) | AIPS0_PACRB_VALUE;
    #endif
  #elif defined(AIPS0_PACRB_VALUE)
  AIPS0_PACRB = AIPS0_PACRB_VALUE;
  #endif

  /* Register 'AIPS0_PACRD' initialization */
  #if AIPS0_PACRD_MASK
    #if AIPS0_PACRD_MASK == 0xFFFFFFFF
  AIPS0_PACRD = AIPS0_PACRD_VALUE;
    #elif AIPS0_PACRD_MASK == AIPS0_PACRD_VALUE
  AIPS0_PACRD |= AIPS0_PACRD_VALUE;
    #elif AIPS0_PACRD_VALUE == 0
  AIPS0_PACRD &= ~AIPS0_PACRD_MASK;
    #else
  AIPS0_PACRD = (AIPS0_PACRD & (~AIPS0_PACRD_MASK)) | AIPS0_PACRD_VALUE;
    #endif
  #elif defined(AIPS0_PACRD_VALUE)
  AIPS0_PACRD = AIPS0_PACRD_VALUE;
  #endif

  /* Register 'AIPS0_PACRE' initialization */
  #if AIPS0_PACRE_MASK
    #if AIPS0_PACRE_MASK == 0xFFFFFFFF
  AIPS0_PACRE = AIPS0_PACRE_VALUE;
    #elif AIPS0_PACRE_MASK == AIPS0_PACRE_VALUE
  AIPS0_PACRE |= AIPS0_PACRE_VALUE;
    #elif AIPS0_PACRE_VALUE == 0
  AIPS0_PACRE &= ~AIPS0_PACRE_MASK;
    #else
  AIPS0_PACRE = (AIPS0_PACRE & (~AIPS0_PACRE_MASK)) | AIPS0_PACRE_VALUE;
    #endif
  #elif defined(AIPS0_PACRE_VALUE)
  AIPS0_PACRE = AIPS0_PACRE_VALUE;
  #endif

  /* Register 'AIPS0_PACRF' initialization */
  #if AIPS0_PACRF_MASK
    #if AIPS0_PACRF_MASK == 0xFFFFFFFF
  AIPS0_PACRF = AIPS0_PACRF_VALUE;
    #elif AIPS0_PACRF_MASK == AIPS0_PACRF_VALUE
  AIPS0_PACRF |= AIPS0_PACRF_VALUE;
    #elif AIPS0_PACRF_VALUE == 0
  AIPS0_PACRF &= ~AIPS0_PACRF_MASK;
    #else
  AIPS0_PACRF = (AIPS0_PACRF & (~AIPS0_PACRF_MASK)) | AIPS0_PACRF_VALUE;
    #endif
  #elif defined(AIPS0_PACRF_VALUE)
  AIPS0_PACRF = AIPS0_PACRF_VALUE;
  #endif

  /* Register 'AIPS0_PACRG' initialization */
  #if AIPS0_PACRG_MASK
    #if AIPS0_PACRG_MASK == 0xFFFFFFFF
  AIPS0_PACRG = AIPS0_PACRG_VALUE;
    #elif AIPS0_PACRG_MASK == AIPS0_PACRG_VALUE
  AIPS0_PACRG |= AIPS0_PACRG_VALUE;
    #elif AIPS0_PACRG_VALUE == 0
  AIPS0_PACRG &= ~AIPS0_PACRG_MASK;
    #else
  AIPS0_PACRG = (AIPS0_PACRG & (~AIPS0_PACRG_MASK)) | AIPS0_PACRG_VALUE;
    #endif
  #elif defined(AIPS0_PACRG_VALUE)
  AIPS0_PACRG = AIPS0_PACRG_VALUE;
  #endif

  /* Register 'AIPS0_PACRH' initialization */
  #if AIPS0_PACRH_MASK
    #if AIPS0_PACRH_MASK == 0xFFFFFFFF
  AIPS0_PACRH = AIPS0_PACRH_VALUE;
    #elif AIPS0_PACRH_MASK == AIPS0_PACRH_VALUE
  AIPS0_PACRH |= AIPS0_PACRH_VALUE;
    #elif AIPS0_PACRH_VALUE == 0
  AIPS0_PACRH &= ~AIPS0_PACRH_MASK;
    #else
  AIPS0_PACRH = (AIPS0_PACRH & (~AIPS0_PACRH_MASK)) | AIPS0_PACRH_VALUE;
    #endif
  #elif defined(AIPS0_PACRH_VALUE)
  AIPS0_PACRH = AIPS0_PACRH_VALUE;
  #endif

  /* Register 'AIPS0_PACRI' initialization */
  #if AIPS0_PACRI_MASK
    #if AIPS0_PACRI_MASK == 0xFFFFFFFF
  AIPS0_PACRI = AIPS0_PACRI_VALUE;
    #elif AIPS0_PACRI_MASK == AIPS0_PACRI_VALUE
  AIPS0_PACRI |= AIPS0_PACRI_VALUE;
    #elif AIPS0_PACRI_VALUE == 0
  AIPS0_PACRI &= ~AIPS0_PACRI_MASK;
    #else
  AIPS0_PACRI = (AIPS0_PACRI & (~AIPS0_PACRI_MASK)) | AIPS0_PACRI_VALUE;
    #endif
  #elif defined(AIPS0_PACRI_VALUE)
  AIPS0_PACRI = AIPS0_PACRI_VALUE;
  #endif

  /* Register 'AIPS0_PACRJ' initialization */
  #if AIPS0_PACRJ_MASK
    #if AIPS0_PACRJ_MASK == 0xFFFFFFFF
  AIPS0_PACRJ = AIPS0_PACRJ_VALUE;
    #elif AIPS0_PACRJ_MASK == AIPS0_PACRJ_VALUE
  AIPS0_PACRJ |= AIPS0_PACRJ_VALUE;
    #elif AIPS0_PACRJ_VALUE == 0
  AIPS0_PACRJ &= ~AIPS0_PACRJ_MASK;
    #else
  AIPS0_PACRJ = (AIPS0_PACRJ & (~AIPS0_PACRJ_MASK)) | AIPS0_PACRJ_VALUE;
    #endif
  #elif defined(AIPS0_PACRJ_VALUE)
  AIPS0_PACRJ = AIPS0_PACRJ_VALUE;
  #endif

  /* Register 'AIPS0_PACRK' initialization */
  #if AIPS0_PACRK_MASK
    #if AIPS0_PACRK_MASK == 0xFFFFFFFF
  AIPS0_PACRK = AIPS0_PACRK_VALUE;
    #elif AIPS0_PACRK_MASK == AIPS0_PACRK_VALUE
  AIPS0_PACRK |= AIPS0_PACRK_VALUE;
    #elif AIPS0_PACRK_VALUE == 0
  AIPS0_PACRK &= ~AIPS0_PACRK_MASK;
    #else
  AIPS0_PACRK = (AIPS0_PACRK & (~AIPS0_PACRK_MASK)) | AIPS0_PACRK_VALUE;
    #endif
  #elif defined(AIPS0_PACRK_VALUE)
  AIPS0_PACRK = AIPS0_PACRK_VALUE;
  #endif

  /* Register 'AIPS0_PACRM' initialization */
  #if AIPS0_PACRM_MASK
    #if AIPS0_PACRM_MASK == 0xFFFFFFFF
  AIPS0_PACRM = AIPS0_PACRM_VALUE;
    #elif AIPS0_PACRM_MASK == AIPS0_PACRM_VALUE
  AIPS0_PACRM |= AIPS0_PACRM_VALUE;
    #elif AIPS0_PACRM_VALUE == 0
  AIPS0_PACRM &= ~AIPS0_PACRM_MASK;
    #else
  AIPS0_PACRM = (AIPS0_PACRM & (~AIPS0_PACRM_MASK)) | AIPS0_PACRM_VALUE;
    #endif
  #elif defined(AIPS0_PACRM_VALUE)
  AIPS0_PACRM = AIPS0_PACRM_VALUE;
  #endif

  /* Register 'AIPS0_PACRN' initialization */
  #if AIPS0_PACRN_MASK
    #if AIPS0_PACRN_MASK == 0xFFFFFFFF
  AIPS0_PACRN = AIPS0_PACRN_VALUE;
    #elif AIPS0_PACRN_MASK == AIPS0_PACRN_VALUE
  AIPS0_PACRN |= AIPS0_PACRN_VALUE;
    #elif AIPS0_PACRN_VALUE == 0
  AIPS0_PACRN &= ~AIPS0_PACRN_MASK;
    #else
  AIPS0_PACRN = (AIPS0_PACRN & (~AIPS0_PACRN_MASK)) | AIPS0_PACRN_VALUE;
    #endif
  #elif defined(AIPS0_PACRN_VALUE)
  AIPS0_PACRN = AIPS0_PACRN_VALUE;
  #endif

  /* Register 'AIPS0_PACRO' initialization */
  #if AIPS0_PACRO_MASK
    #if AIPS0_PACRO_MASK == 0xFFFFFFFF
  AIPS0_PACRO = AIPS0_PACRO_VALUE;
    #elif AIPS0_PACRO_MASK == AIPS0_PACRO_VALUE
  AIPS0_PACRO |= AIPS0_PACRO_VALUE;
    #elif AIPS0_PACRO_VALUE == 0
  AIPS0_PACRO &= ~AIPS0_PACRO_MASK;
    #else
  AIPS0_PACRO = (AIPS0_PACRO & (~AIPS0_PACRO_MASK)) | AIPS0_PACRO_VALUE;
    #endif
  #elif defined(AIPS0_PACRO_VALUE)
  AIPS0_PACRO = AIPS0_PACRO_VALUE;
  #endif

  /* Register 'AIPS0_PACRP' initialization */
  #if AIPS0_PACRP_MASK
    #if AIPS0_PACRP_MASK == 0xFFFFFFFF
  AIPS0_PACRP = AIPS0_PACRP_VALUE;
    #elif AIPS0_PACRP_MASK == AIPS0_PACRP_VALUE
  AIPS0_PACRP |= AIPS0_PACRP_VALUE;
    #elif AIPS0_PACRP_VALUE == 0
  AIPS0_PACRP &= ~AIPS0_PACRP_MASK;
    #else
  AIPS0_PACRP = (AIPS0_PACRP & (~AIPS0_PACRP_MASK)) | AIPS0_PACRP_VALUE;
    #endif
  #elif defined(AIPS0_PACRP_VALUE)
  AIPS0_PACRP = AIPS0_PACRP_VALUE;
  #endif

  /* Register 'AIPS0_PACRU' initialization */
  #if AIPS0_PACRU_MASK
    #if AIPS0_PACRU_MASK == 0xFFFFFFFF
  AIPS0_PACRU = AIPS0_PACRU_VALUE;
    #elif AIPS0_PACRU_MASK == AIPS0_PACRU_VALUE
  AIPS0_PACRU |= AIPS0_PACRU_VALUE;
    #elif AIPS0_PACRU_VALUE == 0
  AIPS0_PACRU &= ~AIPS0_PACRU_MASK;
    #else
  AIPS0_PACRU = (AIPS0_PACRU & (~AIPS0_PACRU_MASK)) | AIPS0_PACRU_VALUE;
    #endif
  #elif defined(AIPS0_PACRU_VALUE)
  AIPS0_PACRU = AIPS0_PACRU_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END AIPS0_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

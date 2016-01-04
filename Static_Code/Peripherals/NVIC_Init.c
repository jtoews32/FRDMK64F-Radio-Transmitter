/** ###################################################################
 **     Filename    : NVIC_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the NVIC module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void NVIC_Init(void);
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
 * @file NVIC_Init.c
 * @brief This file implements the NVIC module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE NVIC_Init. */

#include "NVIC_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void NVIC_Init(void) {

  /* Register 'NVICIP0' initialization */
  #ifdef NVICIP0_VALUE
  NVICIP0 = NVICIP0_VALUE;
  #endif

  /* Register 'NVICIP1' initialization */
  #ifdef NVICIP1_VALUE
  NVICIP1 = NVICIP1_VALUE;
  #endif

  /* Register 'NVICIP2' initialization */
  #ifdef NVICIP2_VALUE
  NVICIP2 = NVICIP2_VALUE;
  #endif

  /* Register 'NVICIP3' initialization */
  #ifdef NVICIP3_VALUE
  NVICIP3 = NVICIP3_VALUE;
  #endif

  /* Register 'NVICIP4' initialization */
  #ifdef NVICIP4_VALUE
  NVICIP4 = NVICIP4_VALUE;
  #endif

  /* Register 'NVICIP5' initialization */
  #ifdef NVICIP5_VALUE
  NVICIP5 = NVICIP5_VALUE;
  #endif

  /* Register 'NVICIP6' initialization */
  #ifdef NVICIP6_VALUE
  NVICIP6 = NVICIP6_VALUE;
  #endif

  /* Register 'NVICIP7' initialization */
  #ifdef NVICIP7_VALUE
  NVICIP7 = NVICIP7_VALUE;
  #endif

  /* Register 'NVICIP8' initialization */
  #ifdef NVICIP8_VALUE
  NVICIP8 = NVICIP8_VALUE;
  #endif

  /* Register 'NVICIP9' initialization */
  #ifdef NVICIP9_VALUE
  NVICIP9 = NVICIP9_VALUE;
  #endif

  /* Register 'NVICIP10' initialization */
  #ifdef NVICIP10_VALUE
  NVICIP10 = NVICIP10_VALUE;
  #endif

  /* Register 'NVICIP11' initialization */
  #ifdef NVICIP11_VALUE
  NVICIP11 = NVICIP11_VALUE;
  #endif

  /* Register 'NVICIP12' initialization */
  #ifdef NVICIP12_VALUE
  NVICIP12 = NVICIP12_VALUE;
  #endif

  /* Register 'NVICIP13' initialization */
  #ifdef NVICIP13_VALUE
  NVICIP13 = NVICIP13_VALUE;
  #endif

  /* Register 'NVICIP14' initialization */
  #ifdef NVICIP14_VALUE
  NVICIP14 = NVICIP14_VALUE;
  #endif

  /* Register 'NVICIP15' initialization */
  #ifdef NVICIP15_VALUE
  NVICIP15 = NVICIP15_VALUE;
  #endif

  /* Register 'NVICIP16' initialization */
  #ifdef NVICIP16_VALUE
  NVICIP16 = NVICIP16_VALUE;
  #endif

  /* Register 'NVICIP17' initialization */
  #ifdef NVICIP17_VALUE
  NVICIP17 = NVICIP17_VALUE;
  #endif

  /* Register 'NVICIP18' initialization */
  #ifdef NVICIP18_VALUE
  NVICIP18 = NVICIP18_VALUE;
  #endif

  /* Register 'NVICIP19' initialization */
  #ifdef NVICIP19_VALUE
  NVICIP19 = NVICIP19_VALUE;
  #endif

  /* Register 'NVICIP20' initialization */
  #ifdef NVICIP20_VALUE
  NVICIP20 = NVICIP20_VALUE;
  #endif

  /* Register 'NVICIP21' initialization */
  #ifdef NVICIP21_VALUE
  NVICIP21 = NVICIP21_VALUE;
  #endif

  /* Register 'NVICIP22' initialization */
  #ifdef NVICIP22_VALUE
  NVICIP22 = NVICIP22_VALUE;
  #endif

  /* Register 'NVICIP23' initialization */
  #ifdef NVICIP23_VALUE
  NVICIP23 = NVICIP23_VALUE;
  #endif

  /* Register 'NVICIP24' initialization */
  #ifdef NVICIP24_VALUE
  NVICIP24 = NVICIP24_VALUE;
  #endif

  /* Register 'NVICIP25' initialization */
  #ifdef NVICIP25_VALUE
  NVICIP25 = NVICIP25_VALUE;
  #endif

  /* Register 'NVICIP26' initialization */
  #ifdef NVICIP26_VALUE
  NVICIP26 = NVICIP26_VALUE;
  #endif

  /* Register 'NVICIP27' initialization */
  #ifdef NVICIP27_VALUE
  NVICIP27 = NVICIP27_VALUE;
  #endif

  /* Register 'NVICIP28' initialization */
  #ifdef NVICIP28_VALUE
  NVICIP28 = NVICIP28_VALUE;
  #endif

  /* Register 'NVICIP29' initialization */
  #ifdef NVICIP29_VALUE
  NVICIP29 = NVICIP29_VALUE;
  #endif

  /* Register 'NVICIP30' initialization */
  #ifdef NVICIP30_VALUE
  NVICIP30 = NVICIP30_VALUE;
  #endif

  /* Register 'NVICIP31' initialization */
  #ifdef NVICIP31_VALUE
  NVICIP31 = NVICIP31_VALUE;
  #endif

  /* Register 'NVICIP32' initialization */
  #ifdef NVICIP32_VALUE
  NVICIP32 = NVICIP32_VALUE;
  #endif

  /* Register 'NVICIP33' initialization */
  #ifdef NVICIP33_VALUE
  NVICIP33 = NVICIP33_VALUE;
  #endif

  /* Register 'NVICIP34' initialization */
  #ifdef NVICIP34_VALUE
  NVICIP34 = NVICIP34_VALUE;
  #endif

  /* Register 'NVICIP35' initialization */
  #ifdef NVICIP35_VALUE
  NVICIP35 = NVICIP35_VALUE;
  #endif

  /* Register 'NVICIP36' initialization */
  #ifdef NVICIP36_VALUE
  NVICIP36 = NVICIP36_VALUE;
  #endif

  /* Register 'NVICIP37' initialization */
  #ifdef NVICIP37_VALUE
  NVICIP37 = NVICIP37_VALUE;
  #endif

  /* Register 'NVICIP38' initialization */
  #ifdef NVICIP38_VALUE
  NVICIP38 = NVICIP38_VALUE;
  #endif

  /* Register 'NVICIP39' initialization */
  #ifdef NVICIP39_VALUE
  NVICIP39 = NVICIP39_VALUE;
  #endif

  /* Register 'NVICIP40' initialization */
  #ifdef NVICIP40_VALUE
  NVICIP40 = NVICIP40_VALUE;
  #endif

  /* Register 'NVICIP41' initialization */
  #ifdef NVICIP41_VALUE
  NVICIP41 = NVICIP41_VALUE;
  #endif

  /* Register 'NVICIP42' initialization */
  #ifdef NVICIP42_VALUE
  NVICIP42 = NVICIP42_VALUE;
  #endif

  /* Register 'NVICIP43' initialization */
  #ifdef NVICIP43_VALUE
  NVICIP43 = NVICIP43_VALUE;
  #endif

  /* Register 'NVICIP44' initialization */
  #ifdef NVICIP44_VALUE
  NVICIP44 = NVICIP44_VALUE;
  #endif

  /* Register 'NVICIP45' initialization */
  #ifdef NVICIP45_VALUE
  NVICIP45 = NVICIP45_VALUE;
  #endif

  /* Register 'NVICIP46' initialization */
  #ifdef NVICIP46_VALUE
  NVICIP46 = NVICIP46_VALUE;
  #endif

  /* Register 'NVICIP47' initialization */
  #ifdef NVICIP47_VALUE
  NVICIP47 = NVICIP47_VALUE;
  #endif

  /* Register 'NVICIP48' initialization */
  #ifdef NVICIP48_VALUE
  NVICIP48 = NVICIP48_VALUE;
  #endif

  /* Register 'NVICIP49' initialization */
  #ifdef NVICIP49_VALUE
  NVICIP49 = NVICIP49_VALUE;
  #endif

  /* Register 'NVICIP50' initialization */
  #ifdef NVICIP50_VALUE
  NVICIP50 = NVICIP50_VALUE;
  #endif

  /* Register 'NVICIP51' initialization */
  #ifdef NVICIP51_VALUE
  NVICIP51 = NVICIP51_VALUE;
  #endif

  /* Register 'NVICIP52' initialization */
  #ifdef NVICIP52_VALUE
  NVICIP52 = NVICIP52_VALUE;
  #endif

  /* Register 'NVICIP53' initialization */
  #ifdef NVICIP53_VALUE
  NVICIP53 = NVICIP53_VALUE;
  #endif

  /* Register 'NVICIP54' initialization */
  #ifdef NVICIP54_VALUE
  NVICIP54 = NVICIP54_VALUE;
  #endif

  /* Register 'NVICIP55' initialization */
  #ifdef NVICIP55_VALUE
  NVICIP55 = NVICIP55_VALUE;
  #endif

  /* Register 'NVICIP56' initialization */
  #ifdef NVICIP56_VALUE
  NVICIP56 = NVICIP56_VALUE;
  #endif

  /* Register 'NVICIP57' initialization */
  #ifdef NVICIP57_VALUE
  NVICIP57 = NVICIP57_VALUE;
  #endif

  /* Register 'NVICIP58' initialization */
  #ifdef NVICIP58_VALUE
  NVICIP58 = NVICIP58_VALUE;
  #endif

  /* Register 'NVICIP59' initialization */
  #ifdef NVICIP59_VALUE
  NVICIP59 = NVICIP59_VALUE;
  #endif

  /* Register 'NVICIP60' initialization */
  #ifdef NVICIP60_VALUE
  NVICIP60 = NVICIP60_VALUE;
  #endif

  /* Register 'NVICIP61' initialization */
  #ifdef NVICIP61_VALUE
  NVICIP61 = NVICIP61_VALUE;
  #endif

  /* Register 'NVICIP62' initialization */
  #ifdef NVICIP62_VALUE
  NVICIP62 = NVICIP62_VALUE;
  #endif

  /* Register 'NVICIP63' initialization */
  #ifdef NVICIP63_VALUE
  NVICIP63 = NVICIP63_VALUE;
  #endif

  /* Register 'NVICIP64' initialization */
  #ifdef NVICIP64_VALUE
  NVICIP64 = NVICIP64_VALUE;
  #endif

  /* Register 'NVICIP65' initialization */
  #ifdef NVICIP65_VALUE
  NVICIP65 = NVICIP65_VALUE;
  #endif

  /* Register 'NVICIP66' initialization */
  #ifdef NVICIP66_VALUE
  NVICIP66 = NVICIP66_VALUE;
  #endif

  /* Register 'NVICIP67' initialization */
  #ifdef NVICIP67_VALUE
  NVICIP67 = NVICIP67_VALUE;
  #endif

  /* Register 'NVICIP68' initialization */
  #ifdef NVICIP68_VALUE
  NVICIP68 = NVICIP68_VALUE;
  #endif

  /* Register 'NVICIP69' initialization */
  #ifdef NVICIP69_VALUE
  NVICIP69 = NVICIP69_VALUE;
  #endif

  /* Register 'NVICIP70' initialization */
  #ifdef NVICIP70_VALUE
  NVICIP70 = NVICIP70_VALUE;
  #endif

  /* Register 'NVICIP71' initialization */
  #ifdef NVICIP71_VALUE
  NVICIP71 = NVICIP71_VALUE;
  #endif

  /* Register 'NVICIP73' initialization */
  #ifdef NVICIP73_VALUE
  NVICIP73 = NVICIP73_VALUE;
  #endif

  /* Register 'NVICIP74' initialization */
  #ifdef NVICIP74_VALUE
  NVICIP74 = NVICIP74_VALUE;
  #endif

  /* Register 'NVICIP75' initialization */
  #ifdef NVICIP75_VALUE
  NVICIP75 = NVICIP75_VALUE;
  #endif

  /* Register 'NVICIP76' initialization */
  #ifdef NVICIP76_VALUE
  NVICIP76 = NVICIP76_VALUE;
  #endif

  /* Register 'NVICIP77' initialization */
  #ifdef NVICIP77_VALUE
  NVICIP77 = NVICIP77_VALUE;
  #endif

  /* Register 'NVICIP78' initialization */
  #ifdef NVICIP78_VALUE
  NVICIP78 = NVICIP78_VALUE;
  #endif

  /* Register 'NVICIP79' initialization */
  #ifdef NVICIP79_VALUE
  NVICIP79 = NVICIP79_VALUE;
  #endif

  /* Register 'NVICIP80' initialization */
  #ifdef NVICIP80_VALUE
  NVICIP80 = NVICIP80_VALUE;
  #endif

  /* Register 'NVICIP81' initialization */
  #ifdef NVICIP81_VALUE
  NVICIP81 = NVICIP81_VALUE;
  #endif

  /* Register 'NVICIP82' initialization */
  #ifdef NVICIP82_VALUE
  NVICIP82 = NVICIP82_VALUE;
  #endif

  /* Register 'NVICIP83' initialization */
  #ifdef NVICIP83_VALUE
  NVICIP83 = NVICIP83_VALUE;
  #endif

  /* Register 'NVICIP84' initialization */
  #ifdef NVICIP84_VALUE
  NVICIP84 = NVICIP84_VALUE;
  #endif

  /* Register 'NVICIP85' initialization */
  #ifdef NVICIP85_VALUE
  NVICIP85 = NVICIP85_VALUE;
  #endif

  /* Register 'NVICICPR0' initialization */
  #ifdef NVICICPR0_VALUE
  NVICICPR0 = NVICICPR0_VALUE;
  #endif

  /* Register 'NVICICPR1' initialization */
  #ifdef NVICICPR1_VALUE
  NVICICPR1 = NVICICPR1_VALUE;
  #endif

  /* Register 'NVICICPR2' initialization */
  #if NVICICPR2_MASK
    #if NVICICPR2_MASK == 0xFFFFFFFF
  NVICICPR2 = NVICICPR2_VALUE;
    #elif NVICICPR2_MASK == NVICICPR2_VALUE
  NVICICPR2 |= NVICICPR2_VALUE;
    #elif NVICICPR2_VALUE == 0
  NVICICPR2 &= ~NVICICPR2_MASK;
    #else
  NVICICPR2 = (NVICICPR2 & (~NVICICPR2_MASK)) | NVICICPR2_VALUE;
    #endif
  #elif defined(NVICICPR2_VALUE)
  NVICICPR2 = NVICICPR2_VALUE;
  #endif

  /* Register 'NVICISPR0' initialization */
  #ifdef NVICISPR0_VALUE
  NVICISPR0 = NVICISPR0_VALUE;
  #endif

  /* Register 'NVICISPR1' initialization */
  #ifdef NVICISPR1_VALUE
  NVICISPR1 = NVICISPR1_VALUE;
  #endif

  /* Register 'NVICISPR2' initialization */
  #if NVICISPR2_MASK
    #if NVICISPR2_MASK == 0xFFFFFFFF
  NVICISPR2 = NVICISPR2_VALUE;
    #elif NVICISPR2_MASK == NVICISPR2_VALUE
  NVICISPR2 |= NVICISPR2_VALUE;
    #elif NVICISPR2_VALUE == 0
  NVICISPR2 &= ~NVICISPR2_MASK;
    #else
  NVICISPR2 = (NVICISPR2 & (~NVICISPR2_MASK)) | NVICISPR2_VALUE;
    #endif
  #elif defined(NVICISPR2_VALUE)
  NVICISPR2 = NVICISPR2_VALUE;
  #endif

  /* Register 'NVICISER0' initialization */
  #if NVICISER0_MASK
    #if NVICISER0_MASK == 0xFFFFFFFF
  NVICISER0 = NVICISER0_VALUE;
    #elif NVICISER0_MASK == NVICISER0_VALUE
  NVICISER0 |= NVICISER0_VALUE;
    #elif NVICISER0_VALUE == 0
  NVICISER0 &= ~NVICISER0_MASK;
    #else
  NVICISER0 = (NVICISER0 & (~NVICISER0_MASK)) | NVICISER0_VALUE;
    #endif
  #elif defined(NVICISER0_VALUE)
  NVICISER0 = NVICISER0_VALUE;
  #endif

  /* Register 'NVICISER1' initialization */
  #if NVICISER1_MASK
    #if NVICISER1_MASK == 0xFFFFFFFF
  NVICISER1 = NVICISER1_VALUE;
    #elif NVICISER1_MASK == NVICISER1_VALUE
  NVICISER1 |= NVICISER1_VALUE;
    #elif NVICISER1_VALUE == 0
  NVICISER1 &= ~NVICISER1_MASK;
    #else
  NVICISER1 = (NVICISER1 & (~NVICISER1_MASK)) | NVICISER1_VALUE;
    #endif
  #elif defined(NVICISER1_VALUE)
  NVICISER1 = NVICISER1_VALUE;
  #endif

  /* Register 'NVICISER2' initialization */
  #if NVICISER2_MASK
    #if NVICISER2_MASK == 0xFFFFFFFF
  NVICISER2 = NVICISER2_VALUE;
    #elif NVICISER2_MASK == NVICISER2_VALUE
  NVICISER2 |= NVICISER2_VALUE;
    #elif NVICISER2_VALUE == 0
  NVICISER2 &= ~NVICISER2_MASK;
    #else
  NVICISER2 = (NVICISER2 & (~NVICISER2_MASK)) | NVICISER2_VALUE;
    #endif
  #elif defined(NVICISER2_VALUE)
  NVICISER2 = NVICISER2_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END NVIC_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

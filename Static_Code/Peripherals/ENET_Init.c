/** ###################################################################
 **     Filename    : ENET_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the ENET module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void ENET_Init(void);
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
 * @file ENET_Init.c
 * @brief This file implements the ENET module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE ENET_Init. */

#include "ENET_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void ENET_Init(void) {

  ENET_ECR &= ~0x2;

  /* Register 'ENET_EIMR' initialization */
  #ifdef ENET_EIMR_VALUE
  ENET_EIMR = ENET_EIMR_VALUE;
  #endif

  /* Register 'ENET_EIR' initialization */
  #if ENET_EIR_MASK
    #if ENET_EIR_MASK == 0xFFFFFFFF
  ENET_EIR = ENET_EIR_VALUE;
    #elif ENET_EIR_MASK == ENET_EIR_VALUE
  ENET_EIR |= ENET_EIR_VALUE;
    #elif ENET_EIR_VALUE == 0
  ENET_EIR &= ~ENET_EIR_MASK;
    #else
  ENET_EIR = (ENET_EIR & (~ENET_EIR_MASK)) | ENET_EIR_VALUE;
    #endif
  #elif defined(ENET_EIR_VALUE)
  ENET_EIR = ENET_EIR_VALUE;
  #endif

  /* Register 'ENET_TFWR' initialization */
  #if ENET_TFWR_MASK
    #if ENET_TFWR_MASK == 0xFFFFFFFF
  ENET_TFWR = ENET_TFWR_VALUE;
    #elif ENET_TFWR_MASK == ENET_TFWR_VALUE
  ENET_TFWR |= ENET_TFWR_VALUE;
    #elif ENET_TFWR_VALUE == 0
  ENET_TFWR &= ~ENET_TFWR_MASK;
    #else
  ENET_TFWR = (ENET_TFWR & (~ENET_TFWR_MASK)) | ENET_TFWR_VALUE;
    #endif
  #elif defined(ENET_TFWR_VALUE)
  ENET_TFWR = ENET_TFWR_VALUE;
  #endif

  /* Register 'ENET_IAUR' initialization */
  #ifdef ENET_IAUR_VALUE
  ENET_IAUR = ENET_IAUR_VALUE;
  #endif

  /* Register 'ENET_IALR' initialization */
  #ifdef ENET_IALR_VALUE
  ENET_IALR = ENET_IALR_VALUE;
  #endif

  /* Register 'ENET_GAUR' initialization */
  #ifdef ENET_GAUR_VALUE
  ENET_GAUR = ENET_GAUR_VALUE;
  #endif

  /* Register 'ENET_GALR' initialization */
  #ifdef ENET_GALR_VALUE
  ENET_GALR = ENET_GALR_VALUE;
  #endif

  /* Register 'ENET_PAUR' initialization */
  #if ENET_PAUR_MASK
    #if ENET_PAUR_MASK == 0xFFFFFFFF
  ENET_PAUR = ENET_PAUR_VALUE;
    #elif ENET_PAUR_MASK == ENET_PAUR_VALUE
  ENET_PAUR |= ENET_PAUR_VALUE;
    #elif ENET_PAUR_VALUE == 0
  ENET_PAUR &= ~ENET_PAUR_MASK;
    #else
  ENET_PAUR = (ENET_PAUR & (~ENET_PAUR_MASK)) | ENET_PAUR_VALUE;
    #endif
  #elif defined(ENET_PAUR_VALUE)
  ENET_PAUR = ENET_PAUR_VALUE;
  #endif

  /* Register 'ENET_PALR' initialization */
  #ifdef ENET_PALR_VALUE
  ENET_PALR = ENET_PALR_VALUE;
  #endif

  /* Register 'ENET_OPD' initialization */
  #ifdef ENET_OPD_VALUE
  ENET_OPD = ENET_OPD_VALUE;
  #endif

  /* Register 'ENET_RCR' initialization */
  #if ENET_RCR_MASK
    #if ENET_RCR_MASK == 0xFFFFFFFF
  ENET_RCR = ENET_RCR_VALUE;
    #elif ENET_RCR_MASK == ENET_RCR_VALUE
  ENET_RCR |= ENET_RCR_VALUE;
    #elif ENET_RCR_VALUE == 0
  ENET_RCR &= ~ENET_RCR_MASK;
    #else
  ENET_RCR = (ENET_RCR & (~ENET_RCR_MASK)) | ENET_RCR_VALUE;
    #endif
  #elif defined(ENET_RCR_VALUE)
  ENET_RCR = ENET_RCR_VALUE;
  #endif

  /* Register 'ENET_TCR' initialization */
  #if ENET_TCR_MASK
    #if ENET_TCR_MASK == 0xFFFFFFFF
  ENET_TCR = ENET_TCR_VALUE;
    #elif ENET_TCR_MASK == ENET_TCR_VALUE
  ENET_TCR |= ENET_TCR_VALUE;
    #elif ENET_TCR_VALUE == 0
  ENET_TCR &= ~ENET_TCR_MASK;
    #else
  ENET_TCR = (ENET_TCR & (~ENET_TCR_MASK)) | ENET_TCR_VALUE;
    #endif
  #elif defined(ENET_TCR_VALUE)
  ENET_TCR = ENET_TCR_VALUE;
  #endif

  /* Register 'ENET_MSCR' initialization */
  #if ENET_MSCR_MASK
    #if ENET_MSCR_MASK == 0xFFFFFFFF
  ENET_MSCR = ENET_MSCR_VALUE;
    #elif ENET_MSCR_MASK == ENET_MSCR_VALUE
  ENET_MSCR |= ENET_MSCR_VALUE;
    #elif ENET_MSCR_VALUE == 0
  ENET_MSCR &= ~ENET_MSCR_MASK;
    #else
  ENET_MSCR = (ENET_MSCR & (~ENET_MSCR_MASK)) | ENET_MSCR_VALUE;
    #endif
  #elif defined(ENET_MSCR_VALUE)
  ENET_MSCR = ENET_MSCR_VALUE;
  #endif

  /* Register 'ENET_MIBC' initialization */
  #if ENET_MIBC_MASK
    #if ENET_MIBC_MASK == 0xFFFFFFFF
  ENET_MIBC = ENET_MIBC_VALUE;
    #elif ENET_MIBC_MASK == ENET_MIBC_VALUE
  ENET_MIBC |= ENET_MIBC_VALUE;
    #elif ENET_MIBC_VALUE == 0
  ENET_MIBC &= ~ENET_MIBC_MASK;
    #else
  ENET_MIBC = (ENET_MIBC & (~ENET_MIBC_MASK)) | ENET_MIBC_VALUE;
    #endif
  #elif defined(ENET_MIBC_VALUE)
  ENET_MIBC = ENET_MIBC_VALUE;
  #endif

  /* Register 'ENET_MRBR' initialization */
  #if ENET_MRBR_MASK
    #if ENET_MRBR_MASK == 0xFFFFFFFF
  ENET_MRBR = ENET_MRBR_VALUE;
    #elif ENET_MRBR_MASK == ENET_MRBR_VALUE
  ENET_MRBR |= ENET_MRBR_VALUE;
    #elif ENET_MRBR_VALUE == 0
  ENET_MRBR &= ~ENET_MRBR_MASK;
    #else
  ENET_MRBR = (ENET_MRBR & (~ENET_MRBR_MASK)) | ENET_MRBR_VALUE;
    #endif
  #elif defined(ENET_MRBR_VALUE)
  ENET_MRBR = ENET_MRBR_VALUE;
  #endif

  /* Register 'ENET_RSFL' initialization */
  #if ENET_RSFL_MASK
    #if ENET_RSFL_MASK == 0xFFFFFFFF
  ENET_RSFL = ENET_RSFL_VALUE;
    #elif ENET_RSFL_MASK == ENET_RSFL_VALUE
  ENET_RSFL |= ENET_RSFL_VALUE;
    #elif ENET_RSFL_VALUE == 0
  ENET_RSFL &= ~ENET_RSFL_MASK;
    #else
  ENET_RSFL = (ENET_RSFL & (~ENET_RSFL_MASK)) | ENET_RSFL_VALUE;
    #endif
  #elif defined(ENET_RSFL_VALUE)
  ENET_RSFL = ENET_RSFL_VALUE;
  #endif

  /* Register 'ENET_RSEM' initialization */
  #if ENET_RSEM_MASK
    #if ENET_RSEM_MASK == 0xFFFFFFFF
  ENET_RSEM = ENET_RSEM_VALUE;
    #elif ENET_RSEM_MASK == ENET_RSEM_VALUE
  ENET_RSEM |= ENET_RSEM_VALUE;
    #elif ENET_RSEM_VALUE == 0
  ENET_RSEM &= ~ENET_RSEM_MASK;
    #else
  ENET_RSEM = (ENET_RSEM & (~ENET_RSEM_MASK)) | ENET_RSEM_VALUE;
    #endif
  #elif defined(ENET_RSEM_VALUE)
  ENET_RSEM = ENET_RSEM_VALUE;
  #endif

  /* Register 'ENET_RAEM' initialization */
  #if ENET_RAEM_MASK
    #if ENET_RAEM_MASK == 0xFFFFFFFF
  ENET_RAEM = ENET_RAEM_VALUE;
    #elif ENET_RAEM_MASK == ENET_RAEM_VALUE
  ENET_RAEM |= ENET_RAEM_VALUE;
    #elif ENET_RAEM_VALUE == 0
  ENET_RAEM &= ~ENET_RAEM_MASK;
    #else
  ENET_RAEM = (ENET_RAEM & (~ENET_RAEM_MASK)) | ENET_RAEM_VALUE;
    #endif
  #elif defined(ENET_RAEM_VALUE)
  ENET_RAEM = ENET_RAEM_VALUE;
  #endif

  /* Register 'ENET_RAFL' initialization */
  #if ENET_RAFL_MASK
    #if ENET_RAFL_MASK == 0xFFFFFFFF
  ENET_RAFL = ENET_RAFL_VALUE;
    #elif ENET_RAFL_MASK == ENET_RAFL_VALUE
  ENET_RAFL |= ENET_RAFL_VALUE;
    #elif ENET_RAFL_VALUE == 0
  ENET_RAFL &= ~ENET_RAFL_MASK;
    #else
  ENET_RAFL = (ENET_RAFL & (~ENET_RAFL_MASK)) | ENET_RAFL_VALUE;
    #endif
  #elif defined(ENET_RAFL_VALUE)
  ENET_RAFL = ENET_RAFL_VALUE;
  #endif

  /* Register 'ENET_TSEM' initialization */
  #if ENET_TSEM_MASK
    #if ENET_TSEM_MASK == 0xFFFFFFFF
  ENET_TSEM = ENET_TSEM_VALUE;
    #elif ENET_TSEM_MASK == ENET_TSEM_VALUE
  ENET_TSEM |= ENET_TSEM_VALUE;
    #elif ENET_TSEM_VALUE == 0
  ENET_TSEM &= ~ENET_TSEM_MASK;
    #else
  ENET_TSEM = (ENET_TSEM & (~ENET_TSEM_MASK)) | ENET_TSEM_VALUE;
    #endif
  #elif defined(ENET_TSEM_VALUE)
  ENET_TSEM = ENET_TSEM_VALUE;
  #endif

  /* Register 'ENET_TAEM' initialization */
  #if ENET_TAEM_MASK
    #if ENET_TAEM_MASK == 0xFFFFFFFF
  ENET_TAEM = ENET_TAEM_VALUE;
    #elif ENET_TAEM_MASK == ENET_TAEM_VALUE
  ENET_TAEM |= ENET_TAEM_VALUE;
    #elif ENET_TAEM_VALUE == 0
  ENET_TAEM &= ~ENET_TAEM_MASK;
    #else
  ENET_TAEM = (ENET_TAEM & (~ENET_TAEM_MASK)) | ENET_TAEM_VALUE;
    #endif
  #elif defined(ENET_TAEM_VALUE)
  ENET_TAEM = ENET_TAEM_VALUE;
  #endif

  /* Register 'ENET_TAFL' initialization */
  #if ENET_TAFL_MASK
    #if ENET_TAFL_MASK == 0xFFFFFFFF
  ENET_TAFL = ENET_TAFL_VALUE;
    #elif ENET_TAFL_MASK == ENET_TAFL_VALUE
  ENET_TAFL |= ENET_TAFL_VALUE;
    #elif ENET_TAFL_VALUE == 0
  ENET_TAFL &= ~ENET_TAFL_MASK;
    #else
  ENET_TAFL = (ENET_TAFL & (~ENET_TAFL_MASK)) | ENET_TAFL_VALUE;
    #endif
  #elif defined(ENET_TAFL_VALUE)
  ENET_TAFL = ENET_TAFL_VALUE;
  #endif

  /* Register 'ENET_FTRL' initialization */
  #if ENET_FTRL_MASK
    #if ENET_FTRL_MASK == 0xFFFFFFFF
  ENET_FTRL = ENET_FTRL_VALUE;
    #elif ENET_FTRL_MASK == ENET_FTRL_VALUE
  ENET_FTRL |= ENET_FTRL_VALUE;
    #elif ENET_FTRL_VALUE == 0
  ENET_FTRL &= ~ENET_FTRL_MASK;
    #else
  ENET_FTRL = (ENET_FTRL & (~ENET_FTRL_MASK)) | ENET_FTRL_VALUE;
    #endif
  #elif defined(ENET_FTRL_VALUE)
  ENET_FTRL = ENET_FTRL_VALUE;
  #endif

  /* Register 'ENET_TIPG' initialization */
  #if ENET_TIPG_MASK
    #if ENET_TIPG_MASK == 0xFFFFFFFF
  ENET_TIPG = ENET_TIPG_VALUE;
    #elif ENET_TIPG_MASK == ENET_TIPG_VALUE
  ENET_TIPG |= ENET_TIPG_VALUE;
    #elif ENET_TIPG_VALUE == 0
  ENET_TIPG &= ~ENET_TIPG_MASK;
    #else
  ENET_TIPG = (ENET_TIPG & (~ENET_TIPG_MASK)) | ENET_TIPG_VALUE;
    #endif
  #elif defined(ENET_TIPG_VALUE)
  ENET_TIPG = ENET_TIPG_VALUE;
  #endif

  /* Register 'ENET_TACC' initialization */
  #ifdef ENET_TACC_VALUE
  ENET_TACC = ENET_TACC_VALUE;
  #endif

  /* Register 'ENET_RACC' initialization */
  #ifdef ENET_RACC_VALUE
  ENET_RACC = ENET_RACC_VALUE;
  #endif

  /* Register 'ENET_ATOFF' initialization */
  #ifdef ENET_ATOFF_VALUE
  ENET_ATOFF = ENET_ATOFF_VALUE;
  #endif

  /* Register 'ENET_ATPER' initialization */
  #ifdef ENET_ATPER_VALUE
  ENET_ATPER = ENET_ATPER_VALUE;
  #endif

  /* Register 'ENET_ATCOR' initialization */
  #if ENET_ATCOR_MASK
    #if ENET_ATCOR_MASK == 0xFFFFFFFF
  ENET_ATCOR = ENET_ATCOR_VALUE;
    #elif ENET_ATCOR_MASK == ENET_ATCOR_VALUE
  ENET_ATCOR |= ENET_ATCOR_VALUE;
    #elif ENET_ATCOR_VALUE == 0
  ENET_ATCOR &= ~ENET_ATCOR_MASK;
    #else
  ENET_ATCOR = (ENET_ATCOR & (~ENET_ATCOR_MASK)) | ENET_ATCOR_VALUE;
    #endif
  #elif defined(ENET_ATCOR_VALUE)
  ENET_ATCOR = ENET_ATCOR_VALUE;
  #endif

  /* Register 'ENET_ATINC' initialization */
  #if ENET_ATINC_MASK
    #if ENET_ATINC_MASK == 0xFFFFFFFF
  ENET_ATINC = ENET_ATINC_VALUE;
    #elif ENET_ATINC_MASK == ENET_ATINC_VALUE
  ENET_ATINC |= ENET_ATINC_VALUE;
    #elif ENET_ATINC_VALUE == 0
  ENET_ATINC &= ~ENET_ATINC_MASK;
    #else
  ENET_ATINC = (ENET_ATINC & (~ENET_ATINC_MASK)) | ENET_ATINC_VALUE;
    #endif
  #elif defined(ENET_ATINC_VALUE)
  ENET_ATINC = ENET_ATINC_VALUE;
  #endif

  /* Register 'ENET_ATVR' initialization */
  #ifdef ENET_ATVR_VALUE
  ENET_ATVR = ENET_ATVR_VALUE;
  #endif

  /* Register 'ENET_ATCR' initialization */
  #if ENET_ATCR_MASK
    #if ENET_ATCR_MASK == 0xFFFFFFFF
  ENET_ATCR = ENET_ATCR_VALUE;
    #elif ENET_ATCR_MASK == ENET_ATCR_VALUE
  ENET_ATCR |= ENET_ATCR_VALUE;
    #elif ENET_ATCR_VALUE == 0
  ENET_ATCR &= ~ENET_ATCR_MASK;
    #else
  ENET_ATCR = (ENET_ATCR & (~ENET_ATCR_MASK)) | ENET_ATCR_VALUE;
    #endif
  #elif defined(ENET_ATCR_VALUE)
  ENET_ATCR = ENET_ATCR_VALUE;
  #endif

  /* Register 'ENET_TCSR0' initialization */
  #if ENET_TCSR0_MASK
    #if ENET_TCSR0_MASK == 0xFFFFFFFF
  ENET_TCSR0 = ENET_TCSR0_VALUE;
    #elif ENET_TCSR0_MASK == ENET_TCSR0_VALUE
  ENET_TCSR0 |= ENET_TCSR0_VALUE;
    #elif ENET_TCSR0_VALUE == 0
  ENET_TCSR0 &= ~ENET_TCSR0_MASK;
    #else
  ENET_TCSR0 = (ENET_TCSR0 & (~ENET_TCSR0_MASK)) | ENET_TCSR0_VALUE;
    #endif
  #elif defined(ENET_TCSR0_VALUE)
  ENET_TCSR0 = ENET_TCSR0_VALUE;
  #endif

  /* Register 'ENET_TCSR1' initialization */
  #if ENET_TCSR1_MASK
    #if ENET_TCSR1_MASK == 0xFFFFFFFF
  ENET_TCSR1 = ENET_TCSR1_VALUE;
    #elif ENET_TCSR1_MASK == ENET_TCSR1_VALUE
  ENET_TCSR1 |= ENET_TCSR1_VALUE;
    #elif ENET_TCSR1_VALUE == 0
  ENET_TCSR1 &= ~ENET_TCSR1_MASK;
    #else
  ENET_TCSR1 = (ENET_TCSR1 & (~ENET_TCSR1_MASK)) | ENET_TCSR1_VALUE;
    #endif
  #elif defined(ENET_TCSR1_VALUE)
  ENET_TCSR1 = ENET_TCSR1_VALUE;
  #endif

  /* Register 'ENET_TCSR2' initialization */
  #if ENET_TCSR2_MASK
    #if ENET_TCSR2_MASK == 0xFFFFFFFF
  ENET_TCSR2 = ENET_TCSR2_VALUE;
    #elif ENET_TCSR2_MASK == ENET_TCSR2_VALUE
  ENET_TCSR2 |= ENET_TCSR2_VALUE;
    #elif ENET_TCSR2_VALUE == 0
  ENET_TCSR2 &= ~ENET_TCSR2_MASK;
    #else
  ENET_TCSR2 = (ENET_TCSR2 & (~ENET_TCSR2_MASK)) | ENET_TCSR2_VALUE;
    #endif
  #elif defined(ENET_TCSR2_VALUE)
  ENET_TCSR2 = ENET_TCSR2_VALUE;
  #endif

  /* Register 'ENET_TCSR3' initialization */
  #if ENET_TCSR3_MASK
    #if ENET_TCSR3_MASK == 0xFFFFFFFF
  ENET_TCSR3 = ENET_TCSR3_VALUE;
    #elif ENET_TCSR3_MASK == ENET_TCSR3_VALUE
  ENET_TCSR3 |= ENET_TCSR3_VALUE;
    #elif ENET_TCSR3_VALUE == 0
  ENET_TCSR3 &= ~ENET_TCSR3_MASK;
    #else
  ENET_TCSR3 = (ENET_TCSR3 & (~ENET_TCSR3_MASK)) | ENET_TCSR3_VALUE;
    #endif
  #elif defined(ENET_TCSR3_VALUE)
  ENET_TCSR3 = ENET_TCSR3_VALUE;
  #endif

  /* Register 'ENET_TCCR0' initialization */
  #ifdef ENET_TCCR0_VALUE
  ENET_TCCR0 = ENET_TCCR0_VALUE;
  #endif

  /* Register 'ENET_TCCR1' initialization */
  #ifdef ENET_TCCR1_VALUE
  ENET_TCCR1 = ENET_TCCR1_VALUE;
  #endif

  /* Register 'ENET_TCCR2' initialization */
  #ifdef ENET_TCCR2_VALUE
  ENET_TCCR2 = ENET_TCCR2_VALUE;
  #endif

  /* Register 'ENET_TCCR3' initialization */
  #ifdef ENET_TCCR3_VALUE
  ENET_TCCR3 = ENET_TCCR3_VALUE;
  #endif
  #ifdef ENET_RDSR_VALUE
  ENET_RDSR = ENET_RDSR_VALUE;
  #endif
  #ifdef ENET_TDSR_VALUE
  ENET_TDSR = ENET_TDSR_VALUE;
  #endif
  /* NOTE: User should initialize (empty) Transmit/Receive Descriptor
           ring(s) structures before the Ethernet MAC is "Enabled". */

  /* Register 'ENET_ECR' initialization */
  #if ENET_ECR_MASK
    #if ENET_ECR_MASK == 0xFFFFFFFF
  ENET_ECR = ENET_ECR_VALUE;
    #elif ENET_ECR_MASK == ENET_ECR_VALUE
  ENET_ECR |= ENET_ECR_VALUE;
    #elif ENET_ECR_VALUE == 0
  ENET_ECR &= ~ENET_ECR_MASK;
    #else
  ENET_ECR = (ENET_ECR & (~ENET_ECR_MASK)) | ENET_ECR_VALUE;
    #endif
  #elif defined(ENET_ECR_VALUE)
  ENET_ECR = ENET_ECR_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END ENET_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

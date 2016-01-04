/** ###################################################################
 **     Filename    : MCM_Init.c
 **     Processor   : MK64FN1M0LL12
 **     Abstract    :
 **          This file implements the MCM module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void MCM_Init(void);
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
 * @file MCM_Init.c
 * @brief This file implements the MCM module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE MCM_Init. */

#include "MCM_Init.h"
#include "MK64F12.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void MCM_Init(void) {

  /* Register 'MCM_ISCR' initialization */
  #if MCM_ISCR_MASK
    #if MCM_ISCR_MASK == 0xFFFFFFFF
  MCM_ISCR = MCM_ISCR_VALUE;
    #elif MCM_ISCR_MASK == MCM_ISCR_VALUE
  MCM_ISCR |= MCM_ISCR_VALUE;
    #elif MCM_ISCR_VALUE == 0
  MCM_ISCR &= ~MCM_ISCR_MASK;
    #else
  MCM_ISCR = (MCM_ISCR & (~MCM_ISCR_MASK)) | MCM_ISCR_VALUE;
    #endif
  #elif defined(MCM_ISCR_VALUE)
  MCM_ISCR = MCM_ISCR_VALUE;
  #endif

  /* Register 'MCM_ETBRL' initialization */
  #if MCM_ETBRL_MASK
    #if MCM_ETBRL_MASK == 0xFFFFFFFF
  MCM_ETBRL = MCM_ETBRL_VALUE;
    #elif MCM_ETBRL_MASK == MCM_ETBRL_VALUE
  MCM_ETBRL |= MCM_ETBRL_VALUE;
    #elif MCM_ETBRL_VALUE == 0
  MCM_ETBRL &= ~MCM_ETBRL_MASK;
    #else
  MCM_ETBRL = (MCM_ETBRL & (~MCM_ETBRL_MASK)) | MCM_ETBRL_VALUE;
    #endif
  #elif defined(MCM_ETBRL_VALUE)
  MCM_ETBRL = MCM_ETBRL_VALUE;
  #endif

  /* Register 'MCM_ETBCC' initialization */
  #ifdef MCM_ETBCC_VALUE
  MCM_ETBCC = MCM_ETBCC_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END MCM_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */

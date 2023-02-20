#ifndef X2CSCOPE_H
#define X2CSCOPE_H

// *****************************************************************************
/* MISRA C-2012 Rule 3.1, and 8.6 deviated below. Deviation record ID -  
    H3_MISRAC_2012_R_3_1_DR_1, H3_MISRAC_2012_R_8_6_DR_1 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunknown-pragmas"
#pragma coverity compliance block \
(deviate:2 "MISRA C-2012 Rule 3.1" "H3_MISRAC_2012_R_3_1_DR_1" )\
(deviate:2 "MISRA C-2012 Rule 8.6" "H3_MISRAC_2012_R_8_6_DR_1" )
/*
 * Copyright (c) 2013, Linz Center of Mechatronics GmbH (LCM) http://www.lcm.at/
 * All rights reserved.
 */
/*
 * This file is licensed according to the BSD 3-clause license as follows:
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the "Linz Center of Mechatronics GmbH" and "LCM" nor
 *       the names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL "Linz Center of Mechatronics GmbH" BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
 * This file is part of X2C. http://www.mechatronic-simulation.org/
 */


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "definitions.h"

void X2CScope_Init(void);
void X2CScope_Communicate(void);
void X2CScope_Update(void);

#ifdef __cplusplus
}
#endif

#pragma coverity compliance end_block "MISRA C-2012 Rule 3.1"
#pragma coverity compliance end_block "MISRA C-2012 Rule 8.6"
#pragma GCC diagnostic pop
/* MISRAC 2012 deviation block end */
#endif

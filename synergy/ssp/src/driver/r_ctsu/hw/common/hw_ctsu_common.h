/***********************************************************************************************************************
 * Copyright [2016] Renesas Electronics Corporation and/or its licensors. All Rights Reserved.
 *
 * This file is part of Renesas SynergyTM Software Package (SSP)
 *
 * The contents of this file (the "contents") are proprietary and confidential to Renesas Electronics Corporation
 * and/or its licensors ("Renesas") and subject to statutory and contractual protections.
 *
 * This file is subject to a Renesas SSP license agreement. Unless otherwise agreed in an SSP license agreement with
 * Renesas: 1) you may not use, copy, modify, distribute, display, or perform the contents; 2) you may not use any name
 * or mark of Renesas for advertising or publicity purposes or in connection with your use of the contents; 3) RENESAS
 * MAKES NO WARRANTY OR REPRESENTATIONS ABOUT THE SUITABILITY OF THE CONTENTS FOR ANY PURPOSE; THE CONTENTS ARE PROVIDED
 * "AS IS" WITHOUT ANY EXPRESS OR IMPLIED WARRANTY, INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, AND NON-INFRINGEMENT; AND 4) RENESAS SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, OR
 * CONSEQUENTIAL DAMAGES, INCLUDING DAMAGES RESULTING FROM LOSS OF USE, DATA, OR PROJECTS, WHETHER IN AN ACTION OF
 * CONTRACT OR TORT, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THE CONTENTS. Third-party contents
 * included in this file may be subject to different terms.
 **********************************************************************************************************************/

#ifndef HW_CTSU_COMMON_H_
#define HW_CTSU_COMMON_H_

/**********************************************************************************************************************
Includes
***********************************************************************************************************************/

#include "bsp_api.h"

/* Common macro for SSP header files. There is also a corresponding SSP_FOOTER macro at the end of this file. */
SSP_HEADER

/***********************************************************************************************************************
 Function prototypes
 **********************************************************************************************************************/

uint8_t HW_CTSU_CTSUCR1Get(void);
uint8_t HW_CTSU_CTSUSTGet(void);
void HW_CTSU_CTSUMCH1Set(uint8_t value);
uint8_t HW_CTSU_CTSUMCH1Get(void);
uint8_t HW_CTSU_CTSUCHAC0Get(void);
uint8_t HW_CTSU_CTSUCHAC1Get(void);
uint8_t HW_CTSU_CTSUCHAC2Get(void);
uint8_t HW_CTSU_CTSUCHAC3Get(void);
uint8_t HW_CTSU_CTSUCHAC4Get(void);
uint8_t HW_CTSU_CTSUCHTRC0Get(void);
uint8_t HW_CTSU_CTSUCHTRC1Get(void);
uint8_t HW_CTSU_CTSUCHTRC2Get(void);
uint8_t HW_CTSU_CTSUCHTRC3Get(void);
uint8_t HW_CTSU_CTSUCHTRC4Get(void);

void HW_CTSU_CTSUCR0Set(uint8_t value);
void HW_CTSU_CTSUCR1Set(uint8_t value);
void HW_CTSU_CTSUSDPRSSet(uint8_t value);
void HW_CTSU_CTSUSSTSet(uint8_t value);
void HW_CTSU_CTSUCHAC0Set(uint8_t value);
void HW_CTSU_CTSUCHAC1Set(uint8_t value);
void HW_CTSU_CTSUCHAC2Set(uint8_t value);
void HW_CTSU_CTSUCHAC3Set(uint8_t value);
void HW_CTSU_CTSUCHAC4Set(uint8_t value);
void HW_CTSU_CTSUCHTRC0Set(uint8_t value);
void HW_CTSU_CTSUCHTRC1Set(uint8_t value);
void HW_CTSU_CTSUCHTRC2Set(uint8_t value);
void HW_CTSU_CTSUCHTRC3Set(uint8_t value);
void HW_CTSU_CTSUCHTRC4Set(uint8_t value);
void HW_CTSU_CTSUDCLKCSet(uint8_t value);

uint32_t HW_CTSU_CTSUCR0GetBitCTSUSTRT (void);
uint32_t HW_CTSU_CTSUSTGetBitCTSUSTC (void);
uint32_t HW_CTSU_CTSUSTGetBitCTSUDTSR (void);
uint32_t HW_CTSU_CTSUSTGetBitCTSUSOVF (void);
uint32_t HW_CTSU_CTSUSTGetBitCTSUROVF (void);
uint32_t HW_CTSU_CTSUERRSGetBitCTSUICOMP (void);

void HW_CTSU_CTSUSTSetBitCTSUROVF(uint32_t value);
void HW_CTSU_CTSUCR0SetBitCTSUSTRT (uint32_t value);
void HW_CTSU_CTSUCR0SetBitCTSUSNZ (uint32_t value);
void HW_CTSU_CTSUCR0SetBitCTSUINIT (uint32_t value);
void HW_CTSU_CTSUSTSetBitCTSUSOVF (uint32_t value);
void HW_CTSU_PowerOn (void);
void HW_CTSU_PowerOff (void);

/* Common macro for SSP header files. There is also a corresponding SSP_HEADER macro at the top of this file. */
SSP_FOOTER

#endif /* HW_CTSU_COMMON_H_ */

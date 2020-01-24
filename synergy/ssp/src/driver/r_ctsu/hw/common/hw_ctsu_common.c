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

/**********************************************************************************************************************
Includes
***********************************************************************************************************************/

#include "hw_ctsu_common.h"

/***********************************************************************************************************************
 Functions
 **********************************************************************************************************************/

uint8_t HW_CTSU_CTSUCR1Get(void){
    return (uint8_t) (R_CTSU)->CTSUCR1;
}

uint8_t HW_CTSU_CTSUSTGet(void){
    return (uint8_t) (R_CTSU)->CTSUST;
}
 
void HW_CTSU_CTSUMCH1Set(uint8_t value){
    (R_CTSU)->CTSUMCH1 = value;
}

uint8_t HW_CTSU_CTSUMCH1Get(void){
    return (uint8_t) (R_CTSU)->CTSUMCH1;
}
 
uint8_t HW_CTSU_CTSUCHAC0Get(void) {
    return (uint8_t) (R_CTSU)->CTSUCHAC0;
}

uint8_t HW_CTSU_CTSUCHAC1Get(void) {
    return (uint8_t) (R_CTSU)->CTSUCHAC1;
}

uint8_t HW_CTSU_CTSUCHAC2Get(void) {
    return (uint8_t) (R_CTSU)->CTSUCHAC2;
}

uint8_t HW_CTSU_CTSUCHAC3Get(void) {
    return (uint8_t) (R_CTSU)->CTSUCHAC3;
}

uint8_t HW_CTSU_CTSUCHAC4Get(void) {
    return (uint8_t) (R_CTSU)->CTSUCHAC4;
}

uint8_t HW_CTSU_CTSUCHTRC0Get(void) {
    return (uint8_t) (R_CTSU)->CTSUCHTRC0;
}

uint8_t HW_CTSU_CTSUCHTRC1Get(void) {
    return (uint8_t) (R_CTSU)->CTSUCHTRC1;
}

uint8_t HW_CTSU_CTSUCHTRC2Get(void) {
    return (uint8_t) (R_CTSU)->CTSUCHTRC2;
}

uint8_t HW_CTSU_CTSUCHTRC3Get(void) {
    return (uint8_t) (R_CTSU)->CTSUCHTRC3;
}

uint8_t HW_CTSU_CTSUCHTRC4Get(void) {
    return (uint8_t) (R_CTSU)->CTSUCHTRC4;
}

void HW_CTSU_CTSUCR0Set(uint8_t value) {
    (R_CTSU)->CTSUCR0 = value;
}

void HW_CTSU_CTSUCR1Set(uint8_t value) {
    (R_CTSU)->CTSUCR1 = value;
}

void HW_CTSU_CTSUSDPRSSet(uint8_t value) {
    (R_CTSU)->CTSUSDPRS = value;
}

void HW_CTSU_CTSUSSTSet(uint8_t value) {
    (R_CTSU)->CTSUSST = value;
}

void HW_CTSU_CTSUCHAC0Set(uint8_t value) {
    (R_CTSU)->CTSUCHAC0 = value;
}

void HW_CTSU_CTSUCHAC1Set(uint8_t value) {
    (R_CTSU)->CTSUCHAC1 = value;
}

void HW_CTSU_CTSUCHAC2Set(uint8_t value) {
    (R_CTSU)->CTSUCHAC2 = value;
}

void HW_CTSU_CTSUCHAC3Set(uint8_t value) {
    (R_CTSU)->CTSUCHAC3 = value;
}

void HW_CTSU_CTSUCHAC4Set(uint8_t value) {
    (R_CTSU)->CTSUCHAC4 = value;
}

void HW_CTSU_CTSUCHTRC0Set(uint8_t value) {
    (R_CTSU)->CTSUCHTRC0 = value;
}

void HW_CTSU_CTSUCHTRC1Set(uint8_t value) {
    (R_CTSU)->CTSUCHTRC1 = value;
}

void HW_CTSU_CTSUCHTRC2Set(uint8_t value) {
    (R_CTSU)->CTSUCHTRC2 = value;
}

void HW_CTSU_CTSUCHTRC3Set(uint8_t value) {
    (R_CTSU)->CTSUCHTRC3 = value;
}

void HW_CTSU_CTSUCHTRC4Set(uint8_t value) {
    (R_CTSU)->CTSUCHTRC4 = value;
}

void HW_CTSU_CTSUDCLKCSet(uint8_t value) {
    (R_CTSU)->CTSUDCLKC = value;
}

uint32_t HW_CTSU_CTSUCR0GetBitCTSUSTRT(void) {
    return (R_CTSU)->CTSUCR0_b.CTSUSTRT;
}

uint32_t HW_CTSU_CTSUSTGetBitCTSUSTC(void) {
    return (R_CTSU)->CTSUST_b.CTSUSTC;
}

uint32_t HW_CTSU_CTSUSTGetBitCTSUDTSR(void) {
    return (R_CTSU)->CTSUST_b.CTSUDTSR;
}

uint32_t HW_CTSU_CTSUSTGetBitCTSUSOVF(void) {
    return (R_CTSU)->CTSUST_b.CTSUSOVF;
}

uint32_t HW_CTSU_CTSUSTGetBitCTSUROVF(void) {
    return (R_CTSU)->CTSUST_b.CTSUROVF;
}

uint32_t HW_CTSU_CTSUERRSGetBitCTSUICOMP(void) {
    return (R_CTSU)->CTSUERRS_b.CTSUICOMP;
}

void HW_CTSU_CTSUSTSetBitCTSUROVF(uint32_t value) {
    (R_CTSU)->CTSUST_b.CTSUROVF = (value&0x01u);
}

void HW_CTSU_CTSUCR0SetBitCTSUSTRT(uint32_t value) {
    (R_CTSU)->CTSUCR0_b.CTSUSTRT = (value&0x01u);
}

void HW_CTSU_CTSUCR0SetBitCTSUSNZ(uint32_t value) {
    (R_CTSU)->CTSUCR0_b.CTSUSNZ = (value&0x01u);
}

void HW_CTSU_CTSUCR0SetBitCTSUINIT(uint32_t value) {
    (R_CTSU)->CTSUCR0_b.CTSUINIT = (value&0x01u);
}

void HW_CTSU_CTSUSTSetBitCTSUSOVF(uint32_t value) {
    (R_CTSU)->CTSUST_b.CTSUSOVF = (value&0x01u);
}

void HW_CTSU_PowerOn(void) {

	(R_MSTP)->MSTPCRC_b.MSTPC3 = 0U;
}

void HW_CTSU_PowerOff(void) {

	(R_MSTP)->MSTPCRC_b.MSTPC3 = 1U;
}

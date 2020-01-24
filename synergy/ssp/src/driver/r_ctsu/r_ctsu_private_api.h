/***********************************************************************************************************************
 * Copyright [2015-2017] Renesas Electronics Corporation and/or its licensors. All Rights Reserved.
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

#ifndef R_CTSU_PRIVATE_API_H
#define R_CTSU_PRIVATE_API_H

/* Common macro for SSP header files. There is also a corresponding SSP_FOOTER macro at the end of this file. */
SSP_HEADER

/***********************************************************************************************************************
 * Private Instance API Functions. DO NOT USE! Use functions through Interface API structure instead.
 **********************************************************************************************************************/

ssp_err_t R_CTSU_Open(ctsu_ctrl_t * p_ctrl, ctsu_cfg_t * p_cfg);
ssp_err_t R_CTSU_Close(ctsu_ctrl_t * p_ctrl, ctsu_close_option_t opts);
ssp_err_t R_CTSU_Process(ctsu_ctrl_t * p_ctrl, ctsu_process_option_t opts);
ssp_err_t R_CTSU_Start_Scan(ctsu_ctrl_t * p_ctrl);
ssp_err_t R_CTSU_Update_Parameters(ctsu_ctrl_t * p_ctrl);
ssp_err_t R_CTSU_Read_Results(ctsu_ctrl_t * p_ctrl, void * p_dest, ctsu_read_t opts,
                              const ctsu_channel_pair_t * channels, const uint16_t count);
ssp_err_t R_CTSU_VersionGet(ssp_version_t * const p_version);
ssp_err_t R_CTSU_Control(ctsu_cmd_t cmd, void* p_data);

/* Common macro for SSP header files. There is also a corresponding SSP_HEADER macro at the top of this file. */
SSP_FOOTER

#endif /* R_CTSU_PRIVATE_API_H */

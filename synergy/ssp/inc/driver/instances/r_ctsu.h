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

/**********************************************************************************************************************
 * File Name    : r_ctsu.h
 * Description  : CTSU HAL layer APIs.
 **********************************************************************************************************************/


/*******************************************************************************************************************//**
 * @ingroup HAL_Library
 * @defgroup CTSU CTSU
 * @brief Driver for the Capacitive Touch Sensing Unit (CTSU).
 *
 * @{
 **********************************************************************************************************************/


#ifndef R_CTSU_H
#define R_CTSU_H

/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
#include "bsp_api.h"
#include "stdlib.h"

#include "../../../src/driver/r_ctsu/hw/common/hw_ctsu_common.h"

#include "r_ctsu_api.h"
#include "r_ctsu_cfg.h"

/* Common macro for SSP header files. There is also a corresponding SSP_FOOTER macro at the end of this file. */
SSP_HEADER

/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/
#define CTSU_CODE_VERSION_MAJOR (1U)
#define CTSU_CODE_VERSION_MINOR (4U)

/* Maximum available hardware channels */
#define CTSU_MAX_CHANNELS    (36)

/***********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/
/** Instance control structure */
typedef struct st_ctsu_instance_ctrl_t
{
    transfer_api_t const * p_api_transfer;      ///< Pointer to lower level Transfer driver function pointers
    transfer_ctrl_t       * p_lowerl_lvl_transfer_read_ctrl;  ///< Pointer to the Transfer Read control
    transfer_ctrl_t       * p_lowerl_lvl_transfer_write_ctrl; ///< Pointer to the Transfer Write control
    bool                    ctsu_opened;        ///< Store initialization state.
    uint8_t                 ctsu_unit;          ///< CTSU Unit in use.
    ctsu_hw_cfg_t         * p_ctsu_hw_cfg;      ///< Pointer to a CTSU configuration.
    ctsu_process_option_t   ctsu_open_option;   ///< Software options to use when performing Open and Process.
    ctsu_process_option_t   ctsu_update_option; ///< Software options to use when performing parameter Update process.
    ctsu_close_option_t     ctsu_close_option;  ///< Software options to use when closing touch operation.
    ctsu_action_t           ctsu_process_state; ///< Variable to observe CTSU processing state machine operation.
    void                 (* p_callback)(ctsu_callback_args_t * p_args); ///< Callback to the function to use when updating dependent parameters is complete.
    void                  * p_context;          ///< Pointer to data that should be passed to update_complete notification.
    R_CTSU_Type           * p_reg;              ///< Pointer to base register address
} ctsu_instance_ctrl_t;

/**********************************************************************************************************************
 * Exported global variables
 **********************************************************************************************************************/
/** @cond INC_HEADER_DEFS_SEC */
extern const ctsu_api_t g_ctsu_on_ctsu;
/** @endcond */

/* Common macro for SSP header files. There is also a corresponding SSP_HEADER macro at the top of this file. */
SSP_FOOTER

#endif // R_CTSU_H

/*******************************************************************************************************************//**
 * @} (end defgroup CTSU)
 **********************************************************************************************************************/

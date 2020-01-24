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

#ifndef R_CTSU_PRIVATE_H
#define R_CTSU_PRIVATE_H

/* Common macro for SSP header files. There is also a corresponding SSP_FOOTER macro at the end of this file. */
SSP_HEADER

/**********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/
/**
 * Before adjusting the sensor offset when initial auto-tuning is running, the
 * software filters values N times. N is defined below. */
#define STABILIZATION_TIME                      (1U << CTSU_CFG_FILTER_DEPTH)

#define STARTUP_TIME                            (STABILIZATION_TIME * 4)


/**********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/
/** Auto-tuning state values for each channel during the initial auto-tuning algorithm. */
typedef enum e_autotuning_status
{
    CTSU_AUTO_TUNING_FAILED     = -1,       ///< Initial Auto-tuning failed because due to incorrect CTSU operation
    CTSU_AUTO_TUNING_NOT_STARTED = 0,   ///< Initial auto-tuning is not started yet.
    CTSU_AUTO_TUNING_INITIALIZED = 1,   ///< Channel is initialized for auto-tuning.
    CTSU_AUTO_TUNING_RUNNING        = 2,    ///< Initial auto-tuning is being performed on the channel. i.e. bringing CTSUSC values down to the CTSURC value.
    CTSU_AUTO_TUNING_COMPLETE   = 3,    ///< CTSUSC values are approximately equal to CTSURC values for the given channel
}at_status_t;

/** Commands which can be passed to the Control function. User may not logically 'OR' these. */
typedef enum e_ctsu_cmd
{
    CTSU_CMD_GET_MODE,                  ///< Command to request the CTSU configuration mode.
    CTSU_CMD_GET_ACTIVE_CHANNELS,       ///< Command to request number of active channels in the current configuration
    CTSU_CMD_START_SCAN,                ///< Command to start a scan
    CTSU_CMD_IS_CH_RX,                  ///< Command to check if channel is active. *(p_data) = int8_t
    CTSU_CMD_IS_CH_TX,                  ///< Command to check if channel is active and a transmit channel. *(p_data) = int8_t
    CTSU_CMD_GET_THRESHOLD,         ///< Command to acquire thresholds being used for the configuration
    CTSU_CMD_GET_HYSTERESIS,            ///< Command to get hyeteresis values being used for the configuration
    CTSU_CMD_SET_THRESHOLD,         ///< Command to set thresholds being used for the configuration
    CTSU_CMD_SET_HYSTERESIS,            ///< Command to set thresholds being used for the configuration
    CTSU_CMD_RD_CH_SET,             ///< Command to read settings being written to the CTSU for each channel during CTSUWR interrupt
    CTSU_CMD_WR_CH_SET,             ///< Command to change settings being written to the CTSU for each channel during CTSUWR interrupt
    CTSU_CMD_GET_CHANNEL_OFFSET,        ///< Returns the channel offset.
    CTSU_CMD_GET_CHANNEL_OFFSET_RAW,    ///< Returns the offset of the channel in the raw output
}ctsu_cmd_t;
/** The different modes of the CTSU. */
typedef enum e_ctsu_mode
{
    CTSU_MODE_UNCONFIGURED = -1,        ///< CTSU is not configured.
    CTSU_MODE_NOT_SUPPORTED = 0,        ///< CTSU mode is not supported by this driver. Use CTSU_MODE_SELF_CAPACITANCE instead.
    CTSU_MODE_SELF_CAPACITANCE = 1, ///< CTSU is operating in self capacitance mode.
    CTSU_MODE_PROHIBITED = 2,           ///< CTSU mode is prohibited by the MCU. Use CTSU_MODE_MUTUAL_CAPACITANCE instead.
    CTSU_MODE_MUTUAL_CAPACITANCE = 3,   ///< CTSU is operating in mutual capacitance mode.
    CTSU_MODE_CLOSING = 4,              ///< CTSU is being shut down.
}ctsu_mode_t;

/* Common macro for SSP header files. There is also a corresponding SSP_HEADER macro at the top of this file. */
SSP_FOOTER

#endif //R_CTSU_PRIVATE_H

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
 
/***********************************************************************************************************************
* File Name : r_ctsu_api.h
* Description : API Interface for the Touch peripheral on SC32 MCUs.
**********************************************************************************************************************/
#ifndef DRV_CTSU_API_H_
#define DRV_CTSU_API_H_

/*******************************************************************************************************************//**
 * @ingroup Interface_Library
 * @defgroup CTSU_API CTSU Interface
 * @brief Interface for Capacitive Touch Controllers.
 *
 * @section CTSU_API_SUMMARY Summary
 * The CTSU interface provides the functionality necessary to open, close, run and control the
 * CTSU depending upon the configuration passed as arguments.
 *
 * Implemented by:
 * @ref CTSU
 *
 * Related SSP architecture topics:
 *  - @ref ssp-interfaces
 *  - @ref ssp-predefined-layers
 *  - @ref using-ssp-modules
 *
 * CTSU Interface description: @ref HALCTSUInterface
 * @{
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * Includes
***********************************************************************************************************************/
/** Includes board and MCU related header files. */
#include "bsp_api.h"
#include "r_transfer_api.h"

/* Common macro for SSP header files. There is also a corresponding SSP_FOOTER macro at the end of this file. */
SSP_HEADER

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/
/** Version of the API defined in this file */
#define CTSU_API_VERSION_MAJOR           (1U)
#define CTSU_API_VERSION_MINOR           (3U)

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/
/** CTSU callback event definitions  */
typedef enum e_ctsu_cb_event
{
    CTSU_EVENT_SCAN_COMPLETE,      ///< Scan cycle is complete. This callback is occurring from an ISR.
    CTSU_EVENT_PARAMETERS_UPDATED, ///< All derived parameters from raw CTSU data are updated. Callback is not from ISR.
} ctsu_event_t;

/** CTSU callback arguments definitions  */
typedef struct st_ctsu_callbackb_args
{
    ctsu_event_t  event;         ///< CTSU callback event
    void const  * p_context;     ///< Placeholder for user data
} ctsu_callback_args_t;


/** Structure storing information about a touch sensor element */
typedef struct st_ctsu_channel_pair
{
    int8_t rx; ///< Denotes the primary channel
    int8_t tx; ///< Denotes the secondary channel (used only for mutual capacitance mode)
} ctsu_channel_pair_t;

/** Structure to be holding values to be written to the SFRs in the WR ISR*/
typedef struct st_ctsu_channel_setting
{
    volatile uint16_t ctsussc; ///< Holds value for the CTSUSSC register
    volatile uint16_t ctsuso0; ///< Holds value for the CTSUSO0 register
    volatile uint16_t ctsuso1; ///< Holds value for the CTSUSO1 register
} ctsu_channel_setting_t;

/** Immediate raw data readings from the CTSU registers CTSUSC(sensor count) and CTSURC(Reference count)
 * in self-capacitance mode. */
typedef struct st_ctsu_channel_measurement_self
{
    uint16_t    sensor_count;    ///< Raw sensor count
    uint16_t    reference_count; ///< Raw reference count
} ctsu_channel_data_self_t;

/** Immediate raw data readings from the CTSU registers CTSUSC(sensor count)
  and CTSURC(Reference count) in mutual-capacitance mode. */
typedef struct st_ctsu_channel_measurement_mutual
{
    uint16_t sen_cnt_1; ///< Raw Sensor count primary reading
    uint16_t ref_cnt_1; ///< Raw reference ICO count primary reading
    uint16_t sen_cnt_2; ///< Raw sensor ICO count secondary reading
    uint16_t ref_cnt_2; ///< Raw reference ICO count secondary reading
} ctsu_channel_data_mutual_t;

/** Structure defining a hardware configuration to be passed to the Open function */
typedef struct st_ctsu_hw_cfg
{
    R_CTSU_Type              ctsu_settings;     ///< User defined SFR settings for CR0, CR1, SDPRS, SST, CHACn, CHTRCn, DCLKC.
    ctsu_channel_setting_t * write_settings;    ///< User defined settings for SSC, SO0, SO1 for each active channel.
    ///< User defined initial settings for thresholds for each active channel . Threshold is difference between runtime baseline and filtered output of sensor count.
    uint16_t               * threshold;
    uint16_t               * hysteresis;        ///< User defined settings for tolerance in count values.
    uint16_t               * baseline;          ///< A baseline of the expected count for each active channel when not touched.
    void                   * raw_result;        ///< A pointer to a buffer which will hold raw results of CTSU measurement.
    void                   * filter_output;     ///< A pointer to a buffer which will hold output after filtering raw results.
    void                   * binary_result;     ///< A pointer to a location where binary data can be stored.
    ctsu_channel_pair_t    * excluded;          ///< A pointer to an array which contains a list of channel pairs which need to be ignored in ascending order of rx and then tx.
    int8_t                   num_excluded;      ///< Number of elements in the array excluded.
    const uint16_t         * series_resistance; ///< Resistance of the channel (to determine RC constant when tuning).
} ctsu_hw_cfg_t;

/** User defined functions that can be used to override the default processing functions used
 * internally by the driver.
 * For advanced usage only. */
typedef struct st_data_proc_funcs
{
    int32_t (* preFilter)(void * p_args);   ///< Used for calculating raw data SNR before data gets filtered.
    int32_t (* filter)(volatile uint16_t * output, volatile uint16_t * input); ///< Weighted averaging using CTSU_CFG_FILTER_DEPTH
    int32_t (* postFilter)(void * p_args);  ///< Processing the filter results
    int32_t (* ctsuDecode)(void * p_args);  ///< Algorithm to decide if channel is touched or not
    int32_t (* otDriftComp)(void * p_args); ///< Algorithm to perform manipulations to baseline, envelope, and thresholds on initialization
    int32_t (* rtDriftComp)(void * p_args); ///< Algorithm to perform run time manipulations to baseline, envelope, and thresholds
    int32_t (* otAutoTune)(void * p_args);  ///< Function called once on initialization of system
    int32_t (* rtAutoTune)(void * p_args);  ///< Function called to auto tune sensor when system is running
} ctsu_functions_t;

/** Different options with which a user can query the results of a CTSU scan. User cannot logic 'or' these together. */
typedef  enum e_ctsu_read
{
    CTSU_READ_BINARY_DATA_ALL             = 0,    ///< Read binary string of touch determination
    CTSU_READ_BINARY_DATA_SEL             = 0x80, ///< Read binary string of touch determination for selected channels only
    CTSU_READ_RAW_SENSOR_OUTPUT_ALL       = 0x81, ///< Read raw CTSU output for all active channels
    CTSU_READ_RAW_SENSOR_OUTPUT_SEL       = 0x01, ///< Read raw CTSU output for selected channels
    CTSU_READ_FILTERED_SENSOR_VALUES_ALL  = 0x82, ///< Read scount values output from filter for all active channels
    CTSU_READ_FILTERED_SENSOR_VALUES_SEL  = 0x02, ///< Read scount values output from filter for all active channels
    CTSU_READ_DELTA_COUNT_ALL             = 0x84, ///< Write out the difference between sensor and baseline count to p_dest array for all active sensors
    CTSU_READ_DELTA_COUNT_SEL             = 0x04, ///< Write out the difference between sensor and baseline count to p_dest array for selected channels
    CTSU_READ_BASELINE_COUNT_ALL          = 0x88, ///< Read the current sensor baseline used to determine if a channel is being touched for all active channels.
    CTSU_READ_BASELINE_COUNT_SEL          = 0x08, ///< Read the current sensor baseline used to determine if a channel is being touched for selected channels.
    CTSU_READ_FILTERED_REF_ICO_VALUES_ALL = 0x10, ///< Read the rcount values output from filter for all active channels.
    CTSU_READ_FILTERED_REF_ICO_VALUES_SEL = 0x90, ///< Read the rcount values output from filter for selected channels only
} ctsu_read_t;

/** Different options that can be provided to the Process function. */
typedef enum e_ctsu_process_option
{
    CTSU_PROCESS_OPTION_DEFAULT_SETTING   = 0x61, ///< Recommended option for most use cases.
    CTSU_PROCESS_OPTION_AUTO_SCAN         = 0x01, ///< Automatically start a scan after all internal variables have been updated.
    CTSU_PROCESS_OPTION_ENABLE_DRIFT_COMP = 0x21, ///< Perform drift compensation and start a new scan when internal variables are updated.
    CTSU_PROCESS_OPTION_ENABLE_AUTO_CALIB = 0x41, ///< Perform auto-calibration and start a new scan when internal variables are updated.
    CTSU_PROCESS_OPTION_NONE              = 0,    ///< Skip all optional actions.
} ctsu_process_option_t;

/** Options which can be passed to the Close function. User can logically 'OR' these together. */
typedef enum e_ctsu_close_option
{
    CTSU_CLOSE_OPTION_SUSPEND         = 1,    ///< Suspend the CTSU by setting the CTSUSNZ bit.
    CTSU_CLOSE_OPTION_SAVE_CONFIG     = 2,    ///< Save the configuration in the second argument provided to the close function.
    CTSU_CLOSE_OPTION_RESET_SFRS      = 4,    ///< Bring the SFRs back to their P-O-RS values.
    CTSU_CLOSE_OPTION_POWER_DOWN      = 0x08, ///< Disable clock supply to the CTSU and set it into low power mode.
}ctsu_close_option_t;

/** State of the Process/Update CTSU sensor data function */
typedef enum e_ctsu_action
{
    CTSU_ACTION_START_NEW_SCAN = 0,        ///< Start new scan.
    CTSU_ACTION_WAITING_FOR_SCAN_COMPLETE, ///< Wait for scan complete.
    CTSU_ACTION_CHECK_FOR_ERRORS,          ///< Check for errors.
    CTSU_ACTION_FILTER_DATA,               ///< Filter data.
    CTSU_ACTION_UPDATE_TOUCH_INFO,         ///< Update touch info.
    CTSU_ACTION_UPDATE_BUTTONS,            ///< Update buttons.
    CTSU_ACTION_UPDATE_SLIDERS,            ///< Update sliders.
    CTSU_ACTION_RUN_DRIFT_COMP,            ///< Run drift compensation.
    CTSU_ACTION_RUN_AUTO_TUNE,             ///< Run auto tuning.
    CTSU_ACTION_RESTART_STATE_MACHINE,     ///< Restart state machine.
}ctsu_action_t;

/** Structure defining a configuration structure for the CTSU driver */
typedef struct st_ctsu_cfg_t
{
    transfer_instance_t const * const p_lower_lvl_transfer_read;  ///< Pointer to the Transfer instance to read results
    transfer_instance_t const * const p_lower_lvl_transfer_write; ///< Pointer to the Transfer instance to write cfg
    ctsu_hw_cfg_t       * p_ctsu_hw_cfg;        ///< Pointer to a CTSU configuration.
    ctsu_functions_t    * p_ctsu_functions;     ///< Pointer to a place holder for custom data functions.
    void               (* p_callback)(ctsu_callback_args_t * p_args); ///< Callback to the function to use when scan is complete.
    void                * p_context;        ///< Pointer to data that should be passed to update_complete notification.
    ctsu_process_option_t ctsu_soft_option;     ///< Software options to use when performing Open and Process.
    ctsu_close_option_t   ctsu_close_option;    ///< Software options to use when closing touch operation.
    uint8_t               write_ipl;            ///< Write interrupt priority
    uint8_t               read_ipl;             ///< Read interrupt priority
    uint8_t               end_ipl;              ///< End interrupt priority
} ctsu_cfg_t;

/** CTSU control block.  Allocate an instance specific control block to pass into the CTSU API calls.
 * @par Implemented as
 * - ctsu_instance_ctrl_t
 */
typedef void ctsu_ctrl_t;

/** CTSU HAL driver API structure. Functions implemented at the HAL layer will follow this API. */
typedef struct st_ctsu_api
{
    /** Initialize the CTSU; enable power and clock and set the register configuration.
     * @par Implemented as
     *  - R_CTSU_Open()
     *
     * @param[in]  p_ctrl   Pointer to control handle structure
     * @param[in]  p_cfg    Pointer to configuration structure
     */
    ssp_err_t (* open)(ctsu_ctrl_t * p_ctrl, ctsu_cfg_t * p_cfg);

    /** Close the CTSU by ending any scan in progress, disabling interrupts, and removing power to the peripheral and
     * saving configurations according to options.
     * @par Implemented as
     *  - R_CTSU_Close()
     *
     * @param[in]  p_ctrl   Pointer to control handle structure
     * @param[in]  opts     Closing options
     */
    ssp_err_t (* close)(ctsu_ctrl_t * p_ctrl, ctsu_close_option_t opts);

    /** Start off a single CTSU scan.
     * @par Implemented as
     *  - R_CTSU_Start_Scan()
     *
     * @param[in]  p_ctrl   Pointer to control handle structure
     */
    ssp_err_t (* scan)(ctsu_ctrl_t * p_ctrl);

    /**Update the CTSU internal data including the touch decision and other derived data according to the results
     * of the scan.
     * @par Implemented as
     *  - R_CTSU_Update_Parameters()
     *
     * @param[in]  p_ctrl   Pointer to control handle structure
     */
    ssp_err_t (* update)(ctsu_ctrl_t * p_ctrl);

    /** Read the results from the CTSU including raw data, binary data and other derived data according to the
     * selected options.
     * @par Implemented as
     *  - R_CTSU_Read_Results()
     *
     * @param[in]  p_ctrl   Pointer to control handle structure
     * @param[out] p_dest   Pointer to the destination location for the read data
     * @param[in]  opts     Read options, use only one option per call to read, do not logically OR values
     * @param[in]  channels Specify the channel/channel pairs to read data for
     * @param[in]  count    Specify the number of channel/channel pairs to read data for
     */
    ssp_err_t (* read)(ctsu_ctrl_t * p_ctrl, void * p_dest, ctsu_read_t opts,
                       const ctsu_channel_pair_t * channels, const uint16_t count);

    /** Retrieve the API version.
     * @par Implemented as
     *  - R_CTSU_VersionGet()
     *
     * @pre This function retrieves the API version.
     * @param[in]  p_version   Pointer to version structure
     */
    ssp_err_t (* versionGet)(ssp_version_t * const p_version);
} ctsu_api_t;

/** This structure encompasses everything that is needed to use an instance of this interface. */
typedef struct st_ctsu_instance
{
    ctsu_ctrl_t         * p_ctrl; ///< Pointer to the control structure for this instance
    ctsu_cfg_t          * p_cfg;  ///< Pointer to the configuration structure for this instance
    ctsu_api_t    const * p_api;  ///< Pointer to the API structure for this instance
} ctsu_instance_t;

/* Common macro for SSP header files. There is also a corresponding SSP_HEADER macro at the top of this file. */
SSP_FOOTER

#endif /* DRV_CTSU_API_H_ */
/*******************************************************************************************************************//**
 * @} (end addtogroup CTSU_API)
 **********************************************************************************************************************/

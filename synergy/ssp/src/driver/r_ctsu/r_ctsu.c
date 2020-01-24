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
 Includes
 **********************************************************************************************************************/
#include <string.h>
/* Synergy includes */
#include "bsp_api.h"
#include "r_ctsu_cfg.h"
#include "r_ctsu_api.h"
#include "r_ctsu.h"
#include "r_ctsu_private.h"
#include "r_ctsu_private_api.h"

#if (USING_DTC_FOR_CTSU > 0)
#include "r_dtc.h"
#endif

/***********************************************************************************************************************
 Macro definitions
 **********************************************************************************************************************/
/* This macro is used to suppress compiler messages about a parameter not being used in a function. The nice thing
 * about using this implementation is that it does not take any extra RAM or ROM.
 */
#ifndef SSP_PARAMETER_NOT_USED
#define SSP_PARAMETER_NOT_USED(p) ((void) (p))
#endif

/** Maximum value of CTSUSO register bits in CTSUSO0*/
#define CTSU_CTSUSO_MAX 		(1023)

/** Number of valid channels per variant word. */
#define CTSU_NUM_CHANNELS_PER_VARIANT_WORD   (16U)

/** Number of bits to shift variant data to get channels pinned out on this MCU (pinned out channels are in the upper
 * 16 bits). */
#define CTSU_VARIANT_SHIFT                   (16U)

/** Number of bits per word. */
#define CTSU_BITS_PER_WORD                   (8U)

/***********************************************************************************************************************
 Typedef definitions
 **********************************************************************************************************************/
/** Define the type of integer used to hold the baseline */
typedef uint16_t sensor_baseline_t;
/** Define the size of dc_counter element */
typedef uint16_t dc_counter_t;
/** Define the size of at_counter element */
typedef uint16_t at_counter_t;
/** Define the size of running_sum element */
typedef uint32_t running_sum_t;
/***********************************************************************************************************************
 Private function prototypes
 **********************************************************************************************************************/
/** Function used for filtering raw output */
static int32_t siso_filter(volatile uint16_t* output, volatile uint16_t* input);

#if (USING_DTC_FOR_CTSU > 0)
static ssp_err_t initialize_dtc_ctsu_elements(uint32_t wr_set_addr, uint32_t p_ctsu_output_addr,
        int16_t wr_cnt, int16_t rd_cnt, ctsu_cfg_t const * const p_cfg, ctsu_instance_ctrl_t * const p_ctrl);
#endif //(USING_DTC_FOR_CTSU > 0)

/** Pointer to the transfer instance used for writing CTSU configurations */
transfer_instance_t * p_transfer_ctsuwr_block_xfer;
/** Pointer to the transfer instance used for reading CTSU results */
transfer_instance_t * p_transfer_ctsurd_block_xfer;


#if (CFG_AUTO_TUNE > 0)
/** Function used for one time auto tuning; during initialization with R_CTSU_Open */
static int32_t default_onetime_auto_tune_self(void * p_args);
/** Function used for run time auto tuning; through R_CTSU_Update_Parameters */
static int32_t default_runtime_auto_tune_self(void * p_args);
/** Function used for one time auto tuning; during initialization with R_CTSU_Open in Mutual-Capacitance mode */
static int32_t default_onetime_auto_tune_mutual(void * p_args);
/** Function used for run time auto tuning; through R_CTSU_Update_Parameters in mutual mode. */
static int32_t default_runtime_auto_tune_mutual(void * p_args);
#endif

#if (CFG_DRIFT_COMPENSATION > 0)
/** Function used for initializing the sensor baselines in self capacitance mode. */
static int32_t default_sensor_baseline_init_self(void * p_args);
/** Function used for adjusting sensor baselines when called from R_CTSU_Update_Parameters with CTSU in self-cap mode. */
static int32_t default_sensor_baseline_adjust_self(void * p_args);
/** Function used for initializing the sensor baselines in mutual capacitance mode. */
static int32_t default_sensor_baseline_init_mutual(void * p_args);
/** Function used for adjusting sensor baselines when called from R_CTSU_Update_Parameters with CTSU in mutual-cap mode. */
static int32_t default_sensor_baseline_adjust_mutual(void * p_args);
#endif
/***********************************************************************************************************************
 Private variables
 **********************************************************************************************************************/
/** Keeps track of the mode for the current CTSU configuration*/
static volatile ctsu_mode_t mode = CTSU_MODE_UNCONFIGURED;
/** Holds the state machine for the R_Touch layer */
static volatile ctsu_action_t r_touch_state = CTSU_ACTION_START_NEW_SCAN;
/** Flag to denote if CTSU peripheral is busy */
static volatile int8_t ctsu_busy = false;
/** Keeps count of the number of active channels */
static int16_t num_active_channels = -1;
/** Keep count of number of CTSUWR and CTSURD interrupts that will occur for a configuration */
static int16_t num_wr_irq_requests = -1;
static int16_t num_rd_irq_requests = -1;
/** Keeps track of channels excluded */
static int8_t num_excluded_channels = 0;
/** Pointer to the area holding filtered output for sensor ICO count output */
static uint16_t* scount = NULL;
/** Pointer to the area holding filtered output for reference ICO count output */
static uint16_t* rcount = NULL;
/** Keeps track of the active (receive) channels */
static int8_t rx_channels[CTSU_MAX_CHANNELS] = {-1};
static int8_t num_rx = 0;
/** Keeps track of active transmit channels (when mutual mode is used) */
static int8_t tx_channels[CTSU_MAX_CHANNELS] = {-1};
static int8_t num_tx = 0;
/** Pointer to function to apply on raw output of data */
static int32_t (*filter)(volatile uint16_t* output, volatile uint16_t* input) = NULL;
/** Keep track of the location where the user wants to store the filtered output */
void* p_ctsu_output = NULL;
/** Function to call when ISR executes */
static void (*p_isrcallback)(ctsu_callback_args_t * p_args) = NULL;
/** Pre-filter is generally used for calculating raw data SNR before data gets filtered. */
static int32_t (* pre_filter)(void * p_args) = NULL;
/** Function used for processing the filter results */
static int32_t (* post_filter)(void * p_args) = NULL;
/** Algorithm used to decide if channel is touched or not */
static int32_t (* ctsu_decode)(void * p_args) = NULL;
#if (CFG_AUTO_TUNE > 0)
/** Function used for one time initial auto tuning */
static int32_t (* p_ot_auto_tune)(void * p_args) = NULL;
/** Function used for run time auto tuning */
static int32_t (* p_rt_auto_tune)(void * p_args) = NULL;
#endif

#if (CFG_DRIFT_COMPENSATION > 0)
/** Function used for drift compensation at runtimes */
static int32_t (* p_rt_drift_comp)(void * p_args) = NULL;
#endif

/***********************************************************************************************************************
Variables which can be allocated dynamically
***********************************************************************************************************************/
/** Array of offsets of channels excluded */
static int16_t excluded_channel_offset[CTSU_CFG_MAX_ACTIVE_CHANNEL_COMBOS] = {-1};

/** Keeps track of where the wr interrupt should grab input values for SSC, SO0, SO1 */
static volatile ctsu_channel_setting_t wr_set[CTSU_CFG_MAX_ACTIVE_CHANNEL_COMBOS] = {{0,0,0}};

 /** Array to hold the raw results when the user does not provide a buffer through the configuration. */
static volatile uint16_t raw_results[CTSU_CFG_MAX_ACTIVE_CHANNEL_COMBOS*sizeof(ctsu_channel_data_mutual_t)/sizeof(uint16_t)] = {0};

/** Array that holds to the hysteresis values for each channel */
static uint16_t hysteresis[CTSU_CFG_MAX_ACTIVE_CHANNEL_COMBOS] = {0};
/** Array that holds to the threshold values for each channel */
static uint16_t threshold[CTSU_CFG_MAX_ACTIVE_CHANNEL_COMBOS] = {0};

/** Buffer to store the output of the L-point running average filter */
static uint16_t lpraf_output[CTSU_CFG_MAX_ACTIVE_CHANNEL_COMBOS*sizeof(ctsu_channel_data_mutual_t)/sizeof(uint16_t)] = {0};
static ctsu_channel_data_mutual_t* filter_result_mut = (ctsu_channel_data_mutual_t*)lpraf_output;

/** Buffers used exclusively for mutual capacitance operation */
static uint16_t diff_scount_mutual[CTSU_CFG_MAX_ACTIVE_CHANNEL_COMBOS] = {0};
/** Buffers used exclusively for mutual capacitance operation */
static uint16_t diff_rcount_mutual[CTSU_CFG_MAX_ACTIVE_CHANNEL_COMBOS] = {0};


/** Array that holds the sensor baseline. Sensor baseline is a value that represents the non-touched sensor ICO count for the channel. */
static sensor_baseline_t sensor_baseline[CTSU_CFG_MAX_ACTIVE_CHANNEL_COMBOS] = {0};

/** Array that holds the difference between the sensor baseline and the touched count for each channel */
static int16_t dcount[CTSU_CFG_MAX_ACTIVE_CHANNEL_COMBOS] = {0};
/** Binary data which indicates if a channel is being touched or not */
uint8_t g_channels_touched[(CTSU_CFG_MAX_ACTIVE_CHANNEL_COMBOS>>3)+1] = {0};
#if (CFG_DRIFT_COMPENSATION > 0)
/** Buffer used to count up to threshold of DC_TIMING_STEADY_STATE */
static dc_counter_t dc_counter[CTSU_CFG_MAX_ACTIVE_CHANNEL_COMBOS] = {0};
#if (DRIFT_COMP_METHOD>=DRIFT_COMP_ALT_1)
static dc_counter_t dc_limit[CTSU_CFG_MAX_ACTIVE_CHANNEL_COMBOS] = {0};
#endif
/** Holds a running sum of the values until dc_counter reaches threshold. */
static running_sum_t dc_running_sum[CTSU_CFG_MAX_ACTIVE_CHANNEL_COMBOS] = {0};
#endif//CFG_DRIFT_COMPENSATION
#if (CFG_AUTO_TUNE > 0)
/** Buffer used to count up to threshold of DC_TIMING_STEADY_STATE */
static at_counter_t at_counter[CTSU_CFG_MAX_ACTIVE_CHANNEL_COMBOS] = {0};
/** Holds a running sum of the values until dc_counter reaches threshold. */
static running_sum_t at_running_sum[CTSU_CFG_MAX_ACTIVE_CHANNEL_COMBOS] = {0};
#endif

static int16_t stabilization_time = STARTUP_TIME;

/** Macro for error logger. */
#ifndef CTSU_ERROR_RETURN
    /*LDRA_INSPECTED 77 S This macro does not work when surrounded by parentheses. */
    #define CTSU_ERROR_RETURN(a, err) SSP_ERROR_RETURN((a), (err), &g_module_name[0], &g_ctsu_version)
#endif
#if (0 != BSP_CFG_ERROR_LOG)
static const char g_module_name[] = "ctsu";
#endif

/** Flag to indicate that the R_Touch layer is ready for the user to use. */
static int8_t r_touch_ready = 0;

/***********************************************************************************************************************
Private global variables and functions
***********************************************************************************************************************/
const ctsu_api_t g_ctsu_on_ctsu =
{
    .open       = R_CTSU_Open,
    .close      = R_CTSU_Close,
    .read       = R_CTSU_Read_Results,
    .update     = R_CTSU_Update_Parameters,
    .scan       = R_CTSU_Start_Scan,
    .versionGet = R_CTSU_VersionGet,
};

#if defined(__GNUC__)
/* This structure is affected by warnings from the GCC compiler bug https://gcc.gnu.org/bugzilla/show_bug.cgi?id=60784
 * This pragma suppresses the warnings in this structure only, and will be removed when the SSP compiler is updated to
 * v5.3.*/
/*LDRA_INSPECTED 69 S */
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#endif
/** Version data structure used by error logger macro. */
static const ssp_version_t g_ctsu_version =
{
    .api_version_minor  = CTSU_API_VERSION_MINOR,
    .api_version_major  = CTSU_API_VERSION_MAJOR,
    .code_version_major = CTSU_CODE_VERSION_MAJOR,
    .code_version_minor = CTSU_CODE_VERSION_MINOR
};
#if defined(__GNUC__)
/* Restore warning settings for 'missing-field-initializers' to as specified on command line. */
/*LDRA_INSPECTED 69 S */
#pragma GCC diagnostic pop
#endif

void* g_p_channels_touched_binary = NULL;

/** Mask of valid channels on this MCU. */
static uint64_t g_ctsu_valid_channel_mask = 0U;

static bsp_feature_ctsu_t g_ctsu_feature = {0U};

/*******************************************************************************************************************//**
 * @addtogroup CTSU
 * @brief Driver for operating the CTSU. Provides functions to start up the CTSU and maintain necessary parameters to
 * determine if a channel is being touched or not.
 *
 * It implements the following interface:
 *  - @ref CTSU_API defined in r_ctsu_api.h
 * @{
 **********************************************************************************************************************/
/***********************************************************************************************************************
 Functions
 **********************************************************************************************************************/

/*******************************************************************************************************************//**
* @brief Returns the current version of this module. The version number is encoded where the top 2 bytes are the
*                major version number and the bottom 2 bytes are the minor version number. For example, Version 4.25
*                would be returned as 0x00040019.
* 
***********************************************************************************************************************/
ssp_err_t R_CTSU_VersionGet(ssp_version_t * const p_version)
{
#if CTSU_CFG_PARAM_CHECKING_ENABLE
    SSP_ASSERT(NULL != p_version);
#endif
    p_version->version_id =  g_ctsu_version.version_id;
    return SSP_SUCCESS;
}
/**
 * *********************************************************************************************************************
 * @brief Initializes the CTSU, initializes software driver parameters for touch determination,
 *                  with initial auto-tuning(optional) and baseline initialization(optional).
 *
 * @retval SSP_SUCCESS              No errors. CTSU succesfully initialized.
 * @retval SSP_ERR_HW_LOCKED        CTSU driver already in use
 * @retval SSP_ERR_ASSERTION        Invalid mode of operation (SELF or MUTUAL capacitance), invalid soft option, or invalid close option
 * @retval SSP_ERR_INVALID_POINTER  NULL pointer passed as an argument for a required parameter -OR- Memory
 *                                  allocation failed.
 * @retval SSP_ERR_INVALID_ARGUMENT Illegal parameter was found.
 * @retval SSP_ERR_CTSU_OFFSET_ADJUSTMENT_FAILED   Initial auto tuning failed because either the filter depth is too large,
 *                                                  or the tuning is not valid for either the board or the current conditions.
 * @note Data processing functions should be used by advanced users only. Beginners should provide a dummy argument
 *      (except NULL). Members inside should be kept NULL.
 * @note If CFG_AUTO_TUNE is enabled, then the user must ensure that global interrupt operation is enabled before Open
 *       gets invoked.
 **********************************************************************************************************************/
ssp_err_t R_CTSU_Open(ctsu_ctrl_t * p_api_ctrl, ctsu_cfg_t * p_cfg)
{
    ctsu_instance_ctrl_t * p_ctrl = (ctsu_instance_ctrl_t *) p_api_ctrl;

#if (USING_DTC_FOR_CTSU > 0)
    ssp_err_t err = SSP_SUCCESS;
#endif

#if (1 == CTSU_CFG_PARAM_CHECKING_ENABLE)
    /** Perform error checking */
    CTSU_ERROR_RETURN(NULL != p_ctrl, SSP_ERR_INVALID_POINTER);
    CTSU_ERROR_RETURN(NULL != p_cfg, SSP_ERR_INVALID_POINTER);
    CTSU_ERROR_RETURN(NULL != p_cfg->p_ctsu_hw_cfg, SSP_ERR_INVALID_POINTER);
    CTSU_ERROR_RETURN(NULL != p_cfg->p_ctsu_hw_cfg->threshold, SSP_ERR_INVALID_POINTER);
    CTSU_ERROR_RETURN(NULL != p_cfg->p_ctsu_hw_cfg->hysteresis, SSP_ERR_INVALID_POINTER);
    CTSU_ERROR_RETURN(NULL != p_cfg->p_ctsu_hw_cfg->baseline, SSP_ERR_INVALID_POINTER);
    CTSU_ERROR_RETURN(NULL != p_cfg->p_ctsu_hw_cfg->write_settings, SSP_ERR_INVALID_POINTER);
    CTSU_ERROR_RETURN(NULL != p_cfg->p_ctsu_functions, SSP_ERR_INVALID_POINTER);

#if (USING_DTC_FOR_CTSU > 0)
    SSP_ASSERT(NULL != p_cfg->p_lower_lvl_transfer_read);
    SSP_ASSERT(NULL != p_cfg->p_lower_lvl_transfer_read->p_ctrl);
    SSP_ASSERT(NULL != p_cfg->p_lower_lvl_transfer_read->p_api);
    SSP_ASSERT(NULL != p_cfg->p_lower_lvl_transfer_read->p_api->open);
    SSP_ASSERT(NULL != p_cfg->p_lower_lvl_transfer_read->p_cfg);

    SSP_ASSERT(NULL != p_cfg->p_lower_lvl_transfer_write);
    SSP_ASSERT(NULL != p_cfg->p_lower_lvl_transfer_write->p_ctrl);
    SSP_ASSERT(NULL != p_cfg->p_lower_lvl_transfer_write->p_api);
    SSP_ASSERT(NULL != p_cfg->p_lower_lvl_transfer_write->p_api->open);
    SSP_ASSERT(NULL != p_cfg->p_lower_lvl_transfer_write->p_cfg);

    /* Read and write api should be the same for DTC transfers */
    SSP_ASSERT(p_cfg->p_lower_lvl_transfer_read->p_api == p_cfg->p_lower_lvl_transfer_write->p_api);
#endif

    /* Validate requested mode of operation */
    SSP_ASSERT((CTSU_MODE_MUTUAL_CAPACITANCE ==
                    (0x3U & (p_cfg->p_ctsu_hw_cfg->ctsu_settings.CTSUCR1 >> 6)))   ||
               (CTSU_MODE_SELF_CAPACITANCE   ==
                    (0x3U & (p_cfg->p_ctsu_hw_cfg->ctsu_settings.CTSUCR1 >> 6)))   );

    /* Validate ctsu_soft_option */
    SSP_ASSERT((CTSU_PROCESS_OPTION_NONE            == p_cfg->ctsu_soft_option  )   ||
               (CTSU_PROCESS_OPTION_DEFAULT_SETTING == p_cfg->ctsu_soft_option  )   );

    /* Validate ctsu_close_option */
    SSP_ASSERT((CTSU_CLOSE_OPTION_RESET_SFRS == p_cfg->ctsu_close_option) ||
               (CTSU_CLOSE_OPTION_SUSPEND == p_cfg->ctsu_close_option));

#endif

    /******************************************************************************************************************/
    /**                     Perform quick checks                                                                      */
    /******************************************************************************************************************/

    /* Get CTSU features from BSP. */
    R_BSP_FeatureCtsuGet(&g_ctsu_feature);

    ssp_feature_t ssp_feature = {{(ssp_ip_t) 0U}};
    ssp_feature.channel = 0U;
    ssp_feature.unit = 0U;
    ssp_feature.id = SSP_IP_CTSU;
    fmi_feature_info_t info = {0U};
    err = g_fmi_on_fmi.productFeatureGet(&ssp_feature, &info);
    CTSU_ERROR_RETURN(SSP_SUCCESS == err, err);
    p_ctrl->p_reg = (R_CTSU_Type *) info.ptr;

    /** Set the valid channel mask based on variant data. */
    if (0U == g_ctsu_valid_channel_mask)
    {
        uint32_t * p_extended_data = (uint32_t *) info.ptr_extended_data;
        /* Make sure the extended data fits in the valid channel mask. */
        uint32_t max_variant_words_supported = sizeof(g_ctsu_valid_channel_mask) * CTSU_BITS_PER_WORD
                / CTSU_NUM_CHANNELS_PER_VARIANT_WORD;
        CTSU_ERROR_RETURN(info.extended_data_count < max_variant_words_supported, SSP_ERR_INTERNAL);
        for (uint32_t i = 0U; i < info.extended_data_count; i += 1U)
        {
            uint64_t valid_channels = (p_extended_data[i] >> CTSU_VARIANT_SHIFT);
            g_ctsu_valid_channel_mask |= valid_channels << (i * CTSU_NUM_CHANNELS_PER_VARIANT_WORD);
        }
    }

    ssp_vector_info_t * p_vector_info;
    fmi_event_info_t event_info = {(IRQn_Type) 0U};
    g_fmi_on_fmi.eventInfoGet(&ssp_feature, SSP_SIGNAL_CTSU_WRITE, &event_info);
    IRQn_Type write_irq = event_info.irq;
    CTSU_ERROR_RETURN(SSP_INVALID_VECTOR != write_irq, SSP_ERR_IRQ_BSP_DISABLED);
    g_fmi_on_fmi.eventInfoGet(&ssp_feature, SSP_SIGNAL_CTSU_READ, &event_info);
    IRQn_Type read_irq = event_info.irq;
    CTSU_ERROR_RETURN(SSP_INVALID_VECTOR != read_irq, SSP_ERR_IRQ_BSP_DISABLED);
    g_fmi_on_fmi.eventInfoGet(&ssp_feature, SSP_SIGNAL_CTSU_END, &event_info);
    IRQn_Type end_irq = event_info.irq;
    CTSU_ERROR_RETURN(SSP_INVALID_VECTOR != end_irq, SSP_ERR_IRQ_BSP_DISABLED);

    /** Cannot reconfigure CTSU. You should unload active configuration by calling Close before proceeding. */
    CTSU_ERROR_RETURN(SSP_ERR_IN_USE != R_BSP_HardwareLock(&ssp_feature), SSP_ERR_HW_LOCKED);

    R_SSP_VectorInfoGet(write_irq, &p_vector_info);
    NVIC_SetPriority(write_irq, p_cfg->write_ipl);
    *(p_vector_info->pp_ctrl) = p_ctrl;
    R_SSP_VectorInfoGet(read_irq, &p_vector_info);
    NVIC_SetPriority(read_irq, p_cfg->read_ipl);
    *(p_vector_info->pp_ctrl) = p_ctrl;
    R_SSP_VectorInfoGet(end_irq, &p_vector_info);
    NVIC_SetPriority(end_irq, p_cfg->end_ipl);
    *(p_vector_info->pp_ctrl) = p_ctrl;

    ctsu_hw_cfg_t * p_ctsu_hw_cfg = p_cfg->p_ctsu_hw_cfg;
    ctsu_functions_t * p_ctsu_functions  = p_cfg->p_ctsu_functions;
    void (* p_callback)(ctsu_callback_args_t * p_data) = p_cfg->p_callback;
    ctsu_process_option_t ctsu_soft_option = p_cfg->ctsu_soft_option;
    /* Create local for mode, internal state mode will be updated at the end of open function */
    const ctsu_mode_t mode_local = (ctsu_mode_t)(0x3U & (p_ctsu_hw_cfg->ctsu_settings.CTSUCR1 >> 6));
    /* Save p_cfg->ctsu_close_option in control structure */
    p_ctrl->ctsu_close_option = p_cfg->ctsu_close_option;

#if (USING_DTC_FOR_CTSU > 0)
    /** Assign Transfer API pointer to the control structure for later usage */
    p_ctrl->p_api_transfer = p_cfg->p_lower_lvl_transfer_read->p_api;
#endif

    /** Power ON the CTSU */
    R_BSP_ModuleStart(&ssp_feature);

    uint64_t ch_en_mask = (uint64_t)p_ctsu_hw_cfg->ctsu_settings.CTSUCHAC4;
    ch_en_mask <<= 8U;
    ch_en_mask |= (uint64_t)p_ctsu_hw_cfg->ctsu_settings.CTSUCHAC3;
    ch_en_mask <<= 8;
    ch_en_mask |= (uint64_t)p_ctsu_hw_cfg->ctsu_settings.CTSUCHAC2;
    ch_en_mask <<= 8;
    ch_en_mask |= (uint64_t)p_ctsu_hw_cfg->ctsu_settings.CTSUCHAC1;
    ch_en_mask <<= 8;
    ch_en_mask |= (uint64_t)p_ctsu_hw_cfg->ctsu_settings.CTSUCHAC0;

    uint64_t ch_tx_mask = (uint64_t)p_ctsu_hw_cfg->ctsu_settings.CTSUCHTRC4;
    ch_tx_mask <<= 8;
    ch_tx_mask |= (uint64_t)p_ctsu_hw_cfg->ctsu_settings.CTSUCHTRC3;
    ch_tx_mask <<= 8;
    ch_tx_mask |= (uint64_t)p_ctsu_hw_cfg->ctsu_settings.CTSUCHTRC2;
    ch_tx_mask <<= 8;
    ch_tx_mask |= (uint64_t)p_ctsu_hw_cfg->ctsu_settings.CTSUCHTRC1;
    ch_tx_mask <<= 8;
    ch_tx_mask |= (uint64_t)p_ctsu_hw_cfg->ctsu_settings.CTSUCHTRC0;

    /* Check the channel selections */
    #if (1 == CTSU_CFG_PARAM_CHECKING_ENABLE)
        CTSU_ERROR_RETURN(0 == ((uint64_t)ch_en_mask & ~(uint64_t)g_ctsu_valid_channel_mask), SSP_ERR_INVALID_CHANNEL);
        CTSU_ERROR_RETURN(0 == ((uint64_t)ch_tx_mask & ~(uint64_t)g_ctsu_valid_channel_mask), SSP_ERR_INVALID_CHANNEL);
        CTSU_ERROR_RETURN(0 == (p_ctsu_hw_cfg->ctsu_settings.CTSUCR0 & ~g_ctsu_feature.ctsucr0_mask), SSP_ERR_INVALID_MODE);
        CTSU_ERROR_RETURN(0 == (p_ctsu_hw_cfg->ctsu_settings.CTSUCR1 & ~g_ctsu_feature.ctsucr1_mask), SSP_ERR_INVALID_MODE);
    #endif

    /******************************************************************************************************************/
    /**                          Blind copy the configuration settings to initialize CTSU                             */
    /******************************************************************************************************************/
    HW_CTSU_CTSUCR1Set(g_ctsu_feature.ctsucr1_mask & p_ctsu_hw_cfg->ctsu_settings.CTSUCR1);
    HW_CTSU_CTSUSDPRSSet(p_ctsu_hw_cfg->ctsu_settings.CTSUSDPRS);
    HW_CTSU_CTSUDCLKCSet(p_ctsu_hw_cfg->ctsu_settings.CTSUDCLKC);
    HW_CTSU_CTSUSSTSet(p_ctsu_hw_cfg->ctsu_settings.CTSUSST);
    HW_CTSU_CTSUCHAC0Set(p_ctsu_hw_cfg->ctsu_settings.CTSUCHAC0);
    HW_CTSU_CTSUCHAC1Set(p_ctsu_hw_cfg->ctsu_settings.CTSUCHAC1);
    HW_CTSU_CTSUCHAC2Set(p_ctsu_hw_cfg->ctsu_settings.CTSUCHAC2);
    if(3 < g_ctsu_feature.ctsuchac_register_count)
    {
        HW_CTSU_CTSUCHAC3Set(p_ctsu_hw_cfg->ctsu_settings.CTSUCHAC3);
    }

    if(4 < g_ctsu_feature.ctsuchac_register_count)
    {
        HW_CTSU_CTSUCHAC4Set(p_ctsu_hw_cfg->ctsu_settings.CTSUCHAC4);
    }

    HW_CTSU_CTSUCHTRC0Set(p_ctsu_hw_cfg->ctsu_settings.CTSUCHTRC0);
    HW_CTSU_CTSUCHTRC1Set(p_ctsu_hw_cfg->ctsu_settings.CTSUCHTRC1);
    HW_CTSU_CTSUCHTRC2Set(p_ctsu_hw_cfg->ctsu_settings.CTSUCHTRC2);
    if(3 < g_ctsu_feature.ctsuchtrc_register_count)
    {
        HW_CTSU_CTSUCHTRC3Set(p_ctsu_hw_cfg->ctsu_settings.CTSUCHTRC3);
    }

    if(4 < g_ctsu_feature.ctsuchtrc_register_count)
    {
        HW_CTSU_CTSUCHTRC4Set(p_ctsu_hw_cfg->ctsu_settings.CTSUCHTRC4);
    }

    HW_CTSU_CTSUCR0Set(g_ctsu_feature.ctsucr0_mask & p_ctsu_hw_cfg->ctsu_settings.CTSUCR0);

    /*****************************************************************************************************************/
    /*                                          Discover active channels                                             */
    /*****************************************************************************************************************/
    num_tx = 0;
    num_rx = 0;

    for(int8_t i = 0; i < CTSU_MAX_CHANNELS; i++)
    {/* Max CTSU channels = 36 */
        if((ch_en_mask & ((uint64_t)1<<i)) && (ch_tx_mask & ((uint64_t)1<<i)))
        {/* CTSU will be operated in mutual mode. */
            tx_channels[num_tx] = i;
            num_tx++;
        }
        else if(ch_en_mask & ((uint64_t)1<<i))
        {/* CTSU will be operated in self mode. */
            rx_channels[num_rx] = i;
            num_rx++;
        }
        else
        {/* Set to unused */
            tx_channels[i] = -1;
            rx_channels[i] = -1;
        }
    }

    /* Calculate the total count of active channels */
    num_active_channels = (int16_t)(mode_local==CTSU_MODE_MUTUAL_CAPACITANCE) ? ((num_rx*num_tx) & INT16_MAX) : (num_rx);
    CTSU_ERROR_RETURN(num_active_channels > 0, SSP_ERR_INVALID_ARGUMENT);

    /* Calculate the total count of CTSUWR interrupts which will occur */
    num_wr_irq_requests = (int16_t)(mode_local==CTSU_MODE_MUTUAL_CAPACITANCE) ? ((num_rx*num_tx) & INT16_MAX) : (num_rx);
    /* Calculate the total count of CTSURD interrupts which will occur */
    num_rd_irq_requests = (int16_t)(mode_local==CTSU_MODE_MUTUAL_CAPACITANCE) ? ((num_rx*num_tx*2) & INT16_MAX) : (num_rx);
    /*****************************************************************************************************************/
    /**                                         Discover excluded channels                                           */
    /*****************************************************************************************************************/
    num_excluded_channels = p_ctsu_hw_cfg->num_excluded;

    /** The number of active channels must be greater than the number of excluded channels */
    CTSU_ERROR_RETURN(num_active_channels > num_excluded_channels, SSP_ERR_INVALID_ARGUMENT);

    {
        /** Variables to be used in the loop below */
        int i = 0;
        int j = 0;
        int k = 0;
        for(; (i < num_excluded_channels) && (p_ctsu_hw_cfg->excluded != NULL); i++)
        {/* Populate offsets which should be excluded */
            for(j = 0; j < num_rx; j++)
            {/* Get offset for Rx channel */
                if(rx_channels[j]==p_ctsu_hw_cfg->excluded[i].rx)
                {
                    break;
                }
            }
            if(j==num_rx)
            {/* This should not have happened. */
                return SSP_ERR_INVALID_ARGUMENT;
            }

            if(mode_local==CTSU_MODE_MUTUAL_CAPACITANCE)
            {/* Mutual mode */
                for(k = 0; k < num_tx; k++)
                {/* Get offset for Tx channel */
                    if(tx_channels[k]==p_ctsu_hw_cfg->excluded[i].tx)
                    {
                        break;
                    }
                }
                if(k==num_tx)
                {/* This should not have happened. */
                    return SSP_ERR_INVALID_ARGUMENT;
                }
                excluded_channel_offset[i] = (((j*num_tx) + k)& INT16_MAX);
            }
            else
            {/* Self Mode*/
                excluded_channel_offset[i] = p_ctsu_hw_cfg->excluded[i].rx;
            }
        }
    }

    /*****************************************************************************************************************/
    /**                                         Initialize buffers used                                              */
    /*****************************************************************************************************************/

    switch(mode_local)
    {
    case CTSU_MODE_SELF_CAPACITANCE:
        /********************************************/
        /*      Self capacitance operation          */
        /********************************************/
        if(p_ctsu_hw_cfg->raw_result==NULL)
        {/* User did not provide buffer to store results */
            p_ctsu_output = (ctsu_channel_data_self_t*)raw_results;
        }
        else
        {/* User provided a buffer. */
            p_ctsu_output = p_ctsu_hw_cfg->raw_result;
        }

        /* Initialize p_ctsu_output to 0 */
        memset((uint8_t*)p_ctsu_output,0, ((uint16_t)num_active_channels* sizeof(ctsu_channel_data_self_t)));

        if(p_ctsu_hw_cfg->filter_output==NULL)
        {/* User did not provide a location to store filtered outputs */
            scount = &lpraf_output[0];
            rcount = &lpraf_output[num_active_channels-num_excluded_channels];
        }
        else
        {/* User provided a location for storing filter output. */
            scount = &((uint16_t*)p_ctsu_hw_cfg->filter_output)[0];
            rcount = &((uint16_t*)p_ctsu_hw_cfg->filter_output)[num_active_channels-num_excluded_channels];
        }
        break;

    case CTSU_MODE_MUTUAL_CAPACITANCE:
        /*********************************************/
        /*      Mutual capacitance operation         */
        /*********************************************/
        if(p_ctsu_hw_cfg->raw_result==NULL)
        {/* User did not provide buffer to store results */
            p_ctsu_output = (ctsu_channel_data_mutual_t*)raw_results;
        }
        else
        {/* User provided a buffer. Initialize it. */
            p_ctsu_output = p_ctsu_hw_cfg->raw_result;
        }

        /* Initialize p_ctsu_output to 0 */
        memset((uint8_t*)p_ctsu_output,0, (uint16_t)num_active_channels * sizeof(ctsu_channel_data_mutual_t));

        if(p_ctsu_hw_cfg->filter_output==NULL)
        {/* User did not provide a location to store filtered outputs */
            filter_result_mut = (ctsu_channel_data_mutual_t*)lpraf_output;
            memset((void*)filter_result_mut, 0, (size_t)(num_active_channels - num_excluded_channels)*sizeof(ctsu_channel_data_mutual_t));
            scount = diff_scount_mutual;
            rcount = diff_rcount_mutual;
        }
        else
        {
            filter_result_mut = &((ctsu_channel_data_mutual_t*)p_ctsu_hw_cfg->filter_output)[0];
            memset((void*)filter_result_mut, 0, (size_t)(num_active_channels - num_excluded_channels)*sizeof(ctsu_channel_data_mutual_t));
            scount = diff_scount_mutual;
            rcount = diff_rcount_mutual;
        }
        break;

    default:

        break;
    }

    if(p_ctsu_hw_cfg->binary_result==NULL)
    {/* User did not provide a location for storing the binary */
        g_p_channels_touched_binary = g_channels_touched;
    }
    else
    {/* Point to user location */
        g_p_channels_touched_binary = p_ctsu_hw_cfg->binary_result;
    }

    /*****************************************************************************************************************/
    /**                                         Populate primary buffers                                             */
    /*****************************************************************************************************************/
    /* Copy the initial write settings into a buffer */
    for(int i = 0; i < num_active_channels; i++)
    {
        wr_set[i].ctsussc = p_ctsu_hw_cfg->write_settings[i].ctsussc;
        wr_set[i].ctsuso0 = p_ctsu_hw_cfg->write_settings[i].ctsuso0;
        wr_set[i].ctsuso1 = p_ctsu_hw_cfg->write_settings[i].ctsuso1;
    }

    memcpy(sensor_baseline, p_ctsu_hw_cfg->baseline, (uint16_t)num_active_channels*sizeof(sensor_baseline_t));
    memcpy(threshold, p_ctsu_hw_cfg->threshold, (uint16_t)num_active_channels*sizeof(uint16_t));
    memcpy(hysteresis, p_ctsu_hw_cfg->hysteresis, (uint16_t)num_active_channels*sizeof(uint16_t));

    memset((void*)dcount, 0, (uint16_t)(num_active_channels - num_excluded_channels)*sizeof(int16_t));
    memset((void*)scount, 0, (uint16_t)(num_active_channels - num_excluded_channels)*sizeof(uint16_t));
    memset((void*)rcount, 0, (uint16_t)(num_active_channels - num_excluded_channels)*sizeof(uint16_t));

#if (CFG_DRIFT_COMPENSATION > 0)
#if (DRIFT_COMP_METHOD>=DRIFT_COMP_ALT_1)
    for(int i = 0; i < (num_active_channels - num_excluded_channels); i++)
    {
        dc_limit[i] = DC_TIMING_INITIAL_RATE;
    }
#else
    memset(dc_limit, 0, (uint16_t)(num_active_channels - num_excluded_channels)*sizeof(dc_counter_t));
#endif

    memset(dc_running_sum,0, (uint16_t)(num_active_channels - num_excluded_channels)*sizeof(running_sum_t));
    memset(dc_counter,0, (uint16_t)(num_active_channels - num_excluded_channels)*sizeof(dc_counter_t));
#endif

#if (CFG_AUTO_TUNE > 0)
    memset(at_running_sum,0, (uint16_t)num_active_channels*sizeof(running_sum_t));
    memset(at_counter,0, (uint16_t)num_active_channels*sizeof(at_counter_t));
#endif
    /*****************************************************************************************************************/
    /**                                         Select operation functions                                           */
    /*****************************************************************************************************************/
    if(p_ctsu_functions->filter==NULL)
    {/* User does not have a filter function to use. */
        filter = siso_filter;
    }
    else
    {/* User has provided a filter. Use it. */
        filter = p_ctsu_functions->filter;
    }

    /* Update ctsu_decode to the current configuration */
    /* Otherwise the previous config's ctsu_decode might be used in error */
    /* User function to decode touch. Can be NULL pointer. */
    ctsu_decode = p_ctsu_functions->ctsuDecode;

    /* Update pre_filter to the current configuration */
    /* Otherwise the previous config's pre_filter might be used in error */
    /* User function for pre-filtering. Can be NULL pointer. */
    pre_filter = p_ctsu_functions->preFilter;

    /* Update post_filter to the current configuration */
    /* Otherwise the previous config's post_filter might be used in error */
    /* User function for post-filtering. Can be NULL pointer. */
    post_filter = p_ctsu_functions->postFilter;

#if (CFG_AUTO_TUNE > 0)
    if(p_ctsu_functions->otAutoTune==NULL)
    {/* User has not specified a function for initial auto-tuning. We can use our own. */
        if(CTSU_MODE_SELF_CAPACITANCE == mode_local)
        {/* Self mode*/
            p_ot_auto_tune = default_onetime_auto_tune_self;
        }
        else
        {/* Mutual mode */
            p_ot_auto_tune = default_onetime_auto_tune_mutual;
        }
    }
    else
    {/* User is providing the algorithm for initial auto-tuning. */
        p_ot_auto_tune = p_ctsu_functions->otAutoTune;
    }

    if(p_ctsu_functions->rtAutoTune==NULL)
    {
        if(CTSU_MODE_SELF_CAPACITANCE == mode_local)
        {/* Self mode */
            p_rt_auto_tune = default_runtime_auto_tune_self;
        }
        else
        {/* Mutual mode */
            p_rt_auto_tune = default_runtime_auto_tune_mutual;
        }
    }
    else
    {/* User is providing the runtime algorithm for auto-tuning. */
        p_rt_auto_tune = p_ctsu_functions->rtAutoTune;
    }
#endif //(CFG_AUTO_TUNE > 0)

    /* Update callbacks to the current configuration */
    /* Otherwise the previous config's callback might be used in error */
    p_isrcallback = p_callback;

    /*****************************************************************************************************************/
    /**                                         Enable interrupt operation                                           */
    /*****************************************************************************************************************/
    /* Synergy MCU NVIC setup Synergy sets IPRs through bsp_irq_cfg.h files. Look for:
     * BSP_IRQ_CFG_CTSU_WRITE,
     * BSP_IRQ_CFG_CTSU_READ,
     * BSP_IRQ_CFG_CTSU_END.
     */

    R_BSP_IrqStatusClear(write_irq);
    R_BSP_IrqStatusClear(read_irq);
    R_BSP_IrqStatusClear(end_irq);

    NVIC_ClearPendingIRQ(write_irq);
    NVIC_ClearPendingIRQ(read_irq);
    NVIC_ClearPendingIRQ(end_irq);

    NVIC_EnableIRQ(write_irq);
    NVIC_EnableIRQ(read_irq);
    NVIC_EnableIRQ(end_irq);

#if (USING_DTC_FOR_CTSU > 0)

#if (USING_DYNAMIC_MEMORY == 0)
    err = initialize_dtc_ctsu_elements((uint32_t)&wr_set[0], (uint32_t)p_ctsu_output, num_wr_irq_requests, num_rd_irq_requests, p_cfg, p_ctrl);
#else
    err = initialize_dtc_ctsu_elements((uint32_t)&wr_set, (uint32_t)p_ctsu_output, num_wr_irq_requests, num_rd_irq_requests, p_cfg, p_ctrl);

#endif
    /** If initialization failed, return an error */
    CTSU_ERROR_RETURN(SSP_SUCCESS == err, err);
#endif
    /*****************************************************************************************************************/
    /**                                         Perform optional actions                                             */
    /*****************************************************************************************************************/
#if (CFG_AUTO_TUNE > 0)
    if(((uint32_t)ctsu_soft_option & (uint32_t)CTSU_PROCESS_OPTION_ENABLE_AUTO_CALIB) == (uint32_t)CTSU_PROCESS_OPTION_ENABLE_AUTO_CALIB)
    {/** (Optional) Perform auto-tuning such that sensor count ~~ reference count */
        /* We perform this because the run-time environment may be slightly different from where the calibration
         * was performed. The objective of calibration was to make sensor count <= reference count.
         * NOTE: Interrupts should be enabled here! */
        int32_t channels_tuned = p_ot_auto_tune(NULL);
        if(channels_tuned >= 0)
        {
            if(channels_tuned == (num_active_channels - num_excluded_channels))
            {/* Great! Initial Auto-tune was complete success. */

            }
            else if(channels_tuned < (num_active_channels - num_excluded_channels))
            {/* Partial success. User may want to increment # of iterations permitted for Auto-tuning. */
                return SSP_ERR_CTSU_OFFSET_ADJUSTMENT_FAILED;
            }
        }
        else /* if channels_tuned is -1 or less */
        {/* Auto-tuning failed. (icomp error or reference counter overflowed )*/
            /* reset the initial write settings in buffer */
            memcpy((void*)wr_set, p_ctsu_hw_cfg->write_settings, (uint16_t)num_active_channels*sizeof(ctsu_channel_setting_t));
            return SSP_ERR_CTSU_OFFSET_ADJUSTMENT_FAILED;
        }
    }
#endif

#if (CFG_DRIFT_COMPENSATION > 0)
    /** (Optional) Perform drift compensation */
    if(p_ctsu_functions->otDriftComp!=NULL)
    {
        p_ctsu_functions->otDriftComp(NULL);
    }
    if((ctsu_soft_option & CTSU_PROCESS_OPTION_ENABLE_DRIFT_COMP) == CTSU_PROCESS_OPTION_ENABLE_DRIFT_COMP)
    {
        if(CTSU_MODE_MUTUAL_CAPACITANCE == mode_local)
        {/* Mutual mode */
            default_sensor_baseline_init_mutual(NULL);
        }
        else
        {/* Self mode */
            default_sensor_baseline_init_self(NULL);
        }
    }

    if(p_ctsu_functions->rtDriftComp!=NULL)
    {/* User has provided a function to perform baseline adjustment. */
        p_rt_drift_comp =p_ctsu_functions->rtDriftComp;
    }
    else if(CTSU_MODE_MUTUAL_CAPACITANCE == mode_local)
    {
        p_rt_drift_comp = default_sensor_baseline_adjust_mutual;
    }
    else
    {/* User has not provided a function to perform baseline adjustment. */
        p_rt_drift_comp = default_sensor_baseline_adjust_self;
    }
#endif
    /*****************************************************************************************************************/
    /**                                         Indicate that R_Touch is ready                                       */
    /*****************************************************************************************************************/
    /** Save which CTSU configuration has been opened */
    p_ctrl->p_ctsu_hw_cfg = p_ctsu_hw_cfg;

    /** Save the CTSU options used during open */
    p_ctrl->ctsu_open_option = ctsu_soft_option;
    /** Save the CTSU options to use when updating parameters */
    p_ctrl->ctsu_update_option = ctsu_soft_option;
    /* Identify the callback to be invoked for all information to be passed to upper layers*/
    p_ctrl->p_callback = p_callback;
    /** Save the notification function argument to be passed */
    p_ctrl->p_context = p_cfg->p_context;

    /* Save the mode being used for the current configuration */
    mode = mode_local;

    /* Indicate that R_Touch is ready */
    r_touch_ready = 1;

    return SSP_SUCCESS;
}/* End of function R_CTSU_Open */

/**
 * *********************************************************************************************************************
 * @brief Starts off a CTSU hardware scan. The rate at which this function is called is essentially the scan rate.
 *
 * @retval SSP_SUCCESS     No errors. CTSU scan successfully initialized.
 * @retval SSP_ERR_IN_USE  A scan is ongoing
 **********************************************************************************************************************/

ssp_err_t R_CTSU_Start_Scan(ctsu_ctrl_t * p_api_ctrl)
{
    ctsu_instance_ctrl_t * p_ctrl = (ctsu_instance_ctrl_t *) p_api_ctrl;

    SSP_PARAMETER_NOT_USED(p_ctrl);
    ssp_err_t ctsu_status = SSP_SUCCESS;

    CTSU_ERROR_RETURN((CTSU_MODE_MUTUAL_CAPACITANCE == mode) ||
       (CTSU_MODE_SELF_CAPACITANCE == mode), SSP_ERR_NOT_OPEN);

    /** CTSU h/w is ready to start a new scan. */
    if(HW_CTSU_CTSUSTGetBitCTSUSTC()==0U)
    {
        /** Start a new scan. */
        HW_CTSU_CTSUCR0SetBitCTSUINIT(1U);
        HW_CTSU_CTSUCR0SetBitCTSUSTRT(1U);
    }
    else
    {
        /** Hardware is busy */
        ctsu_status = SSP_ERR_IN_USE;
    }

    return ctsu_status;
}

/**
 * *********************************************************************************************************************
 * @brief Function that must be called after each scan is complete to process the results of the scanned data.
 *
 * It performs the following tasks:
 *
 *                  0. Checks for any errors in CTSU operation.
 *                  1. Runs a filter on the raw values output by the CTSU.
 *                  2. Determines if a channel is being touched or not and updates the binary and difference between
 *                      sensor_baseline and the current sensor value.
 *                  3. When the update is complete, the user specified callback is called from this function with
 *                     the CTSU_EVENT_PARAMETERS_UPDATED event.
 *                  4. (Optional) Performs auto-tuning if (a single/all) channels are not being touched.
 *                  5. (Optional) Performs drift compensation if (a single/all channels) are not being touched.
 *                          Drift compensation := move sensor baseline after N readings.
 *
 *
 * @retval SSP_SUCCESS  No errors.
 * @retval SSP_ERR_INVALID_POINTER  NULL pointer passed as an argument for a required parameter. -OR- Memory
 *                          allocation failed.
 * @retval SSP_ERR_INVALID_ARGUMENT  Illegal parameter was found.
 * @retval SSP_ERR_CTSU_OFFSET_ADJUSTMENT_FAILED  Auto tuning failed because CTSU encountered a ICOMP error or
 *                          Check the configuration parameters for each channel by locating the point of failure.
 **********************************************************************************************************************/

ssp_err_t R_CTSU_Update_Parameters(ctsu_ctrl_t * p_api_ctrl)
{
    ctsu_instance_ctrl_t * p_ctrl = (ctsu_instance_ctrl_t *) p_api_ctrl;

#if (1 == CTSU_CFG_PARAM_CHECKING_ENABLE)
    CTSU_ERROR_RETURN(NULL != p_ctrl, SSP_ERR_INVALID_POINTER);
#endif

    ssp_err_t ctsu_status = SSP_SUCCESS;
    volatile ctsu_channel_data_mutual_t * raw_result_mut;
    uint16_t channels_touched = 0;
    volatile ctsu_channel_data_self_t * raw_result_self;
    volatile uint8_t * touch_binary = g_p_channels_touched_binary;
    uint16_t i = 0U, j = 0U, k = 0U, m = 0U, n = 0U;

    CTSU_ERROR_RETURN((CTSU_MODE_MUTUAL_CAPACITANCE == mode) ||
       (CTSU_MODE_SELF_CAPACITANCE == mode), SSP_ERR_NOT_OPEN);

    CTSU_ERROR_RETURN(HW_CTSU_CTSUSTGetBitCTSUSTC() == 0, SSP_ERR_IN_USE);

    /************************************/
    /** Check for CTSU error conditions */
    /************************************/
    CTSU_ERROR_RETURN(1 != HW_CTSU_CTSUSTGetBitCTSUROVF(), SSP_ERR_CTSU_RC_OVERFLOW);
    CTSU_ERROR_RETURN(1 != HW_CTSU_CTSUSTGetBitCTSUSOVF(), SSP_ERR_CTSU_SC_OVERFLOW);
    CTSU_ERROR_RETURN(1 != HW_CTSU_CTSUERRSGetBitCTSUICOMP(), SSP_ERR_CTSU_ICOMP);
    CTSU_ERROR_RETURN(1 != HW_CTSU_CTSUSTGetBitCTSUDTSR(), SSP_ERR_IN_USE);

    if(pre_filter!=NULL)
    {
        pre_filter(p_ctsu_output);
    }

    /*********************************/
    /** Update each channel measured */
    /*********************************/
    /** Initialize variables for loop below */
    i = 0U;
    j = 0U;
    k = 0U;
    for(; i < (uint16_t)num_active_channels; i++)
    {
        if(i == (uint16_t)excluded_channel_offset[j])
        {
            /** Channel is to be excluded. Bypass calculations. */
            j++;
        }
        else
        {
            switch(mode)
            {
            case CTSU_MODE_SELF_CAPACITANCE:
                /* Typecast the result area */
                raw_result_self = p_ctsu_output;
                /* Run the filter to update filter values. */
                filter(&scount[k], &raw_result_self[i].sensor_count);
                filter(&rcount[k], &raw_result_self[i].reference_count);
                break;

            case CTSU_MODE_MUTUAL_CAPACITANCE:
                /* Typecast the result area */
                raw_result_mut = p_ctsu_output;
                /* Run the filter to update filter values. */
                filter(&filter_result_mut[k].sen_cnt_1, &raw_result_mut[i].sen_cnt_1);
                filter(&filter_result_mut[k].sen_cnt_2, &raw_result_mut[i].sen_cnt_2);
                filter(&filter_result_mut[k].ref_cnt_1, &raw_result_mut[i].ref_cnt_1);
                filter(&filter_result_mut[k].ref_cnt_2, &raw_result_mut[i].ref_cnt_2);

                /* Extra step for mutual mode. Update needed variables */
                if(filter_result_mut[k].sen_cnt_2 > filter_result_mut[k].sen_cnt_1)
                {
                    scount[k] = (uint16_t)(filter_result_mut[k].sen_cnt_2 - filter_result_mut[k].sen_cnt_1);
                }
                else
                {
                    scount[k] = 0U;
                }
                break;

            default:
                break;
            }
            /* Move to next valid channel. */
            k++;
        }
    }

    if(post_filter!=NULL)
    {
        post_filter(p_ctsu_output);
    }

    if(stabilization_time > 0)
    {/* Wait for CTSU readings to stabilize (TSCAP to sufficiently charge up) */
        --stabilization_time;
        return ctsu_status;
    }
    /********************************************/
    /** Check which channels are being touched. */
    /********************************************/
    if(ctsu_decode!=NULL)
    {
        ctsu_decode(p_ctsu_output);
    }
    /** Initialize variables for loop below */
    i = 0U;
    j = 0U;
    k = 0U;
    m = 0U;
    n = 0U;
    for(; i < (uint16_t)num_active_channels; i++)
    {
        /* Figure out which bit is about to be written to. */
        m = (k >> 3);   //byte selection
        n = (k & 7);    //bit selection
        if(i == (uint16_t)excluded_channel_offset[j])
        {
            /** Channel is to be excluded. Bypass calculations. */
            j++;
        }
        else
        {
            switch(mode)
            {
            case CTSU_MODE_SELF_CAPACITANCE:
                if(scount[k] > (sensor_baseline[i]+threshold[i]))
                {/* TS channel is being touched. */
                    dcount[k] = (int16_t)(scount[k] - sensor_baseline[i]);
                    touch_binary[m] = (uint8_t)(touch_binary[m] |(1 << n));
#if (CFG_AUTO_TUNE > 0)
                    if ((uint32_t)((uint32_t)p_ctrl->ctsu_update_option & (uint32_t)CTSU_PROCESS_OPTION_ENABLE_AUTO_CALIB) == (uint32_t)CTSU_PROCESS_OPTION_ENABLE_AUTO_CALIB)
                    {/* Reset auto-tuning parameters. */
                        at_counter[i] = 0U;
                        at_running_sum[i] = 0U;
                    }
#endif
                }
                else if(scount[k] < ((sensor_baseline[i] + threshold[i]) - hysteresis[i]))
                {/* TS channel is not being touched. */
                    dcount[k] = 0;
                    touch_binary[m] = (uint8_t)(touch_binary[m] & ~(1 << n));
                }
                break;

            case CTSU_MODE_MUTUAL_CAPACITANCE:
                if(scount[k] < (sensor_baseline[i]-threshold[i]) )
                {/* Channel is being touched */
                    dcount[k] = (int16_t)(sensor_baseline[i] - scount[k]);
                    touch_binary[m] = (uint8_t)(touch_binary[m]|(1 << n));
#if (CFG_AUTO_TUNE > 0)
                    if ((uint32_t)((uint32_t)p_ctrl->ctsu_update_option & (uint32_t)CTSU_PROCESS_OPTION_ENABLE_AUTO_CALIB) == (uint32_t)CTSU_PROCESS_OPTION_ENABLE_AUTO_CALIB)
                    {/* Reset auto-tuning parameters. */
                        at_counter[i] = 0U;
                        at_running_sum[i] = 0U;
                    }
#endif
                }
                else if(scount[k] > ((sensor_baseline[i] - threshold[i]) + hysteresis[i]) )
                {/* Channel is not being touched */
                    dcount[k] = 0;
                    touch_binary[m] &= (uint8_t)(~(1 << n));
                }
                break;

            default:
                break;
            }
            /* Count channels touched */
            if((touch_binary[m] & (1 << n))==(1 << n))
            {
                ++channels_touched;
            }

            /* Move to next valid channel. */
            k++;
        }
    }
    /***********************************************************************/
    /*             Update dependent frameworks using callback              */
    /***********************************************************************/
    if(p_ctrl->p_callback != NULL)
    {/** Do the callback to provide a notification */
        ctsu_callback_args_t args;
        args.event = CTSU_EVENT_PARAMETERS_UPDATED;
        args.p_context = p_ctrl->p_context;
        p_ctrl->p_callback(&args);
    }

    /************************************/
    /**     Perform drift compensation  */
    /************************************/
#if (CFG_DRIFT_COMPENSATION > 0)
    if((uint32_t)((uint32_t)p_ctrl->ctsu_update_option & (uint32_t)CTSU_PROCESS_OPTION_ENABLE_DRIFT_COMP) == (uint32_t)CTSU_PROCESS_OPTION_ENABLE_DRIFT_COMP)
    {
#if (DRIFT_COMP_METHOD>=DRIFT_COMP_ALT_1)
        if(channels_touched > 0U)
        {
            /** Initialize variables for loop below */
            i = 0U;
            j = 0U;
            k = 0U;
            m = 0U;
            n = 0U;
            for(; i < (uint16_t)num_active_channels; i++)
            {
                /* Figure out which bit is about to be written to. */
                m = (k >> 3);   //byte selection
                n = (k & 7);    //bit selection
                if(i == (uint16_t)excluded_channel_offset[j])
                {/* Channel is to be excluded. Bypass calculations. */
                    j++;
                }
                else
                {/* Reset drift compensation parameters. */
#if (DRIFT_COMP_METHOD==DRIFT_COMP_ALT_1)||(DRIFT_COMP_METHOD==DRIFT_COMP_ALT_2)
                    if((touch_binary[m] & (1<<n))==(1<<n))
#endif
                    {
                        dc_counter[k] = 0U;
                        dc_limit[k] = DC_TIMING_BUTTON_RELEASE_RATE;
                        dc_running_sum[k] = 0U;
                    }
                    k++;
                }
            }
        }
#endif /* (DRIFT_COMP_METHOD>=DRIFT_COMP_ALT_1) */

#if (CTSU_DRIFTCOMP_WHEN_ALL_INACTIVE > 0)
        /* When multi-touch is enabled, drift compensate only when all channels are not touched.*/
        if((p_rt_drift_comp != NULL) && (channels_touched == 0U))
        {/* Time to drift compensate */
            p_rt_drift_comp(NULL);
        }
#else
        if(p_rt_drift_comp!=NULL)
        {/* Time to drift compensate */
            p_rt_drift_comp(NULL);
        }
#endif//CTSU_DRIFTCOMP_WHEN_ALL_INACTIVE

    }
#endif//CFG_DRIFT_COMPENSATION
    /****************************************/
    /*  Perform run-time auto-calibration   */
    /****************************************/
#if (CFG_AUTO_TUNE > 0)
    if ((uint32_t)((uint32_t)p_ctrl->ctsu_update_option & (uint32_t)CTSU_PROCESS_OPTION_ENABLE_AUTO_CALIB) == (uint32_t)CTSU_PROCESS_OPTION_ENABLE_AUTO_CALIB)
    {
#if CTSU_DRIFTCOMP_WHEN_ALL_INACTIVE > 0
        if((p_rt_auto_tune != NULL) && (channels_touched == 0U))
#else
            if (p_rt_auto_tune != NULL)
#endif//CTSU_DRIFTCOMP_WHEN_ALL_INACTIVE
            {/* We've reached the period for the auto-tuning, and no channels are determined as touched. */
                p_rt_auto_tune(NULL);
            }

    }
#endif//CFG_AUTO_TUNE

    return ctsu_status;
}

/**
 * *********************************************************************************************************************
 * @brief Function that the user must call from the main loop. This function is deprecated by the
 * R_CTSU_Update_Parameters function and is not exposed to the user in the interface API.
 *
 * It performs the following tasks:
 *
 *                  0. Checks for any errors in CTSU operation.
 *                  1. Runs a filter on the raw values output by the CTSU.
 *                  2. Determines if a channel is being touched or not and updates the binary and difference between
 *                      sensor_baseline and the current sensor value.
 *                  3. (Optional) Performs auto-tuning if (a single/all) channels are not being touched.
 *                  4. (Optional) Performs drift compensation if (a single/all channels) are not being touched.
 *                          Drift compensation := move sensor baseline after N readings.
 *                  5. (Optional) Starts off a new scan.
 *
 *
 * @retval SSP_SUCCESS  No errors.
 * @retval SSP_ERR_INVALID_POINTER  NULL pointer passed as an argument for a required parameter. -OR- Memory
 *                          allocation failed.
 * @retval SSP_ERR_INVALID_ARGUMENT  Illegal parameter was found.
 * @retval SSP_ERR_CTSU_OFFSET_ADJUSTMENT_FAILED  Auto tuning failed because CTSU encountered a ICOMP error or
 *                          Check the configuration parameters for each channel by locating the point of failure.
 **********************************************************************************************************************/
ssp_err_t R_CTSU_Process( ctsu_ctrl_t* p_api_ctrl, ctsu_process_option_t opts)
{
    ctsu_instance_ctrl_t * p_ctrl = (ctsu_instance_ctrl_t *) p_api_ctrl;

    volatile ssp_err_t ret_val = SSP_SUCCESS;
    volatile ctsu_channel_data_mutual_t* raw_result_mut;
    static uint16_t channels_touched = 0;
    volatile ctsu_channel_data_self_t* raw_result_self;
    volatile uint8_t* touch_binary = g_p_channels_touched_binary;
    uint8_t i = 0U, j = 0U, k = 0U, m = 0U, n = 0U;


    CTSU_ERROR_RETURN((CTSU_MODE_MUTUAL_CAPACITANCE == mode) ||
       (CTSU_MODE_SELF_CAPACITANCE == mode) ||
       (CTSU_MODE_UNCONFIGURED == mode), SSP_ERR_NOT_OPEN);

    switch(r_touch_state)
    {
    case CTSU_ACTION_WAITING_FOR_SCAN_COMPLETE:
        if(HW_CTSU_CTSUSTGetBitCTSUSTC()==0)
        {
            r_touch_state = CTSU_ACTION_CHECK_FOR_ERRORS;
        }
        break;
    case CTSU_ACTION_CHECK_FOR_ERRORS:
        /************************************/
        /** Check for CTSU error conditions */
        /************************************/
        CTSU_ERROR_RETURN(1 != HW_CTSU_CTSUSTGetBitCTSUROVF(), SSP_ERR_CTSU_RC_OVERFLOW);
        CTSU_ERROR_RETURN( 1 != HW_CTSU_CTSUSTGetBitCTSUSOVF(), SSP_ERR_CTSU_SC_OVERFLOW);
        CTSU_ERROR_RETURN(1 != HW_CTSU_CTSUERRSGetBitCTSUICOMP(), SSP_ERR_CTSU_ICOMP);
        CTSU_ERROR_RETURN(1 != HW_CTSU_CTSUSTGetBitCTSUDTSR(), SSP_ERR_IN_USE);
        channels_touched = 0U;
        r_touch_state = CTSU_ACTION_FILTER_DATA;

        if(pre_filter!=NULL)
        {
            pre_filter(p_ctsu_output);
        }
        break;



    case CTSU_ACTION_FILTER_DATA:
        /*********************************/
        /** Update each channel measured */
        /*********************************/
        /** Initialize variables for loop below */
        i = 0U;
        j = 0U;
        k = 0U;
        for(; i < (uint16_t)num_active_channels; i++)
        {
            if(i==excluded_channel_offset[j])
            {/* Channel is to be excluded. Bypass calculations. */
                j++;
            }
            else
            {
                switch(mode)
                {
                case CTSU_MODE_SELF_CAPACITANCE:
                    /* Typecast the result area */
                    raw_result_self = p_ctsu_output;
                    /* Run the filter to update filter values. */
                    filter(&scount[k], &raw_result_self[i].sensor_count);
                    filter(&rcount[k], &raw_result_self[i].reference_count);
                    break;

                case CTSU_MODE_MUTUAL_CAPACITANCE:
                    /* Typecast the result area */
                    raw_result_mut = p_ctsu_output;
                    /* Run the filter to update filter values. */
                    filter(&filter_result_mut[k].sen_cnt_1, &raw_result_mut[i].sen_cnt_1);
                    filter(&filter_result_mut[k].sen_cnt_2, &raw_result_mut[i].sen_cnt_2);
                    filter(&filter_result_mut[k].ref_cnt_1, &raw_result_mut[i].ref_cnt_1);
                    filter(&filter_result_mut[k].ref_cnt_2, &raw_result_mut[i].ref_cnt_2);

                    /* Extra step for mutual mode. Update needed variables */
                    if(filter_result_mut[k].sen_cnt_2 > filter_result_mut[k].sen_cnt_1)
                    {
                        scount[k] = (uint16_t)(filter_result_mut[k].sen_cnt_2 - filter_result_mut[k].sen_cnt_1);
                    }
                    else
                    {
                        scount[k] = 0U;
                    }
                    break;

                default:
                    break;
                }
                /* Move to next valid channel. */
                k++;
            }
        }

        if(post_filter!=NULL)
        {
            post_filter(p_ctsu_output);
        }

        if(stabilization_time>0)
        {/* Wait for CTSU readings to stabilize (TSCAP to sufficiently charge up) */
            --stabilization_time;
            r_touch_state = CTSU_ACTION_START_NEW_SCAN;
        }
        else
        {/* Minimum number of scans have been performed. */
            r_touch_state = CTSU_ACTION_UPDATE_TOUCH_INFO;
        }
        break;

    case CTSU_ACTION_UPDATE_TOUCH_INFO:
        if(ctsu_decode!=NULL)
        {
            ctsu_decode(p_ctsu_output);
        }
        /** Initialize variables for loop below */
        i = 0U;
        j = 0U;
        k = 0U;
        m = 0U;
        n = 0U;
        for(; i < (uint16_t)num_active_channels; i++)
        {
            /* Figure out which bit is about to be written to. */
            m = (k >> 3);   //byte selection
            n = (k & 7);    //bit selection
            if(i==excluded_channel_offset[j])
            {/* Channel is to be excluded. Bypass calculations. */
                j++;
            }
            else
            {
                /********************************************/
                /** Check which channels are being touched. */
                /********************************************/
                switch(mode)
                {
                case CTSU_MODE_SELF_CAPACITANCE:
                    if(scount[k] > (sensor_baseline[i]+threshold[i]))
                    {/* TS channel is being touched. */
                        dcount[k] = (int16_t)(scount[k] - sensor_baseline[i]);
                        touch_binary[m] |= (uint8_t)(1 << n);
#if (CFG_AUTO_TUNE > 0)
                        if ((uint32_t)((uint32_t)opts & (uint32_t)CTSU_PROCESS_OPTION_ENABLE_AUTO_CALIB) == (uint32_t)CTSU_PROCESS_OPTION_ENABLE_AUTO_CALIB)
                        {/* Reset auto-tuning parameters. */
                            at_counter[i] = 0U;
                            at_running_sum[i] = 0U;
                        }
#endif
                    }
                    else if(scount[k] < ((sensor_baseline[i] + threshold[i]) - hysteresis[i]))
                    {/* TS channel is not being touched. */
                        dcount[k] = 0;
                        touch_binary[m] &= (uint8_t)~(1 << n);
                    }
                    break;

                case CTSU_MODE_MUTUAL_CAPACITANCE:
                    if(scount[k] < (sensor_baseline[i]-threshold[i]) )
                    {/* Channel is being touched */
                        dcount[k] = (int16_t)(sensor_baseline[i] - scount[k]);
                        touch_binary[m] |= (uint8_t)(1 << n);
#if (CFG_AUTO_TUNE > 0)
                        if ((uint32_t)((uint32_t)opts & (uint32_t)CTSU_PROCESS_OPTION_ENABLE_AUTO_CALIB) == (uint32_t)CTSU_PROCESS_OPTION_ENABLE_AUTO_CALIB)
                        {/* Reset auto-tuning parameters. */
                            at_counter[i] = 0U;
                            at_running_sum[i] = 0U;
                        }
#endif
                    }
                    else if(scount[k] > ((sensor_baseline[i] - threshold[i]) + hysteresis[i]) )
                    {/* Channel is not being touched */
                        dcount[k] = 0;
                        touch_binary[m] &= (uint8_t)~(1 << n);
                    }
                    break;

                default:
                    break;
                }
                /* Count channels touched */
                if((touch_binary[m] & (1 << n))==(1 << n))
                {
                    ++channels_touched;
                }

                /* Move to next valid channel. */
                k++;
            }
        }

        r_touch_state = CTSU_ACTION_UPDATE_BUTTONS;
        break;

    case CTSU_ACTION_UPDATE_BUTTONS:
        /********************************/
        /**     Update buttons          */
        /********************************/
        r_touch_state = CTSU_ACTION_UPDATE_SLIDERS;
        break;

    case CTSU_ACTION_UPDATE_SLIDERS:
        /********************************/
        /**     Update Slider           */
        /********************************/
        r_touch_state = CTSU_ACTION_RUN_DRIFT_COMP;
        break;

    case CTSU_ACTION_RUN_DRIFT_COMP:
        /************************************/
        /**     Perform drift compensation  */
        /************************************/
#if (CFG_DRIFT_COMPENSATION > 0)
        if((uint32_t)((uint32_t)opts & (uint32_t)CTSU_PROCESS_OPTION_ENABLE_DRIFT_COMP) == (uint32_t)CTSU_PROCESS_OPTION_ENABLE_DRIFT_COMP)
        {
#if (DRIFT_COMP_METHOD>=DRIFT_COMP_ALT_1)
            if(channels_touched > 0U)
            {
                /** Initialize variables for loop below */
                i = 0U;
                j = 0U;
                k = 0U;
                m = 0U;
                n = 0U;
                for(; i < (uint16_t)num_active_channels; i++)
                {
                    /* Figure out which bit is about to be written to. */
                    m = (k >> 3);   //byte selection
                    n = (k & 7);    //bit selection
                    if(i==excluded_channel_offset[j])
                    {/* Channel is to be excluded. Bypass calculations. */
                        j++;
                    }
                    else
                    {/* Reset drift compensation parameters. */
#if (DRIFT_COMP_METHOD==DRIFT_COMP_ALT_1)||(DRIFT_COMP_METHOD==DRIFT_COMP_ALT_2)
                        if((touch_binary[m] & (1<<n))==(1<<n))
#endif
                        {
                            dc_counter[k] = 0U;
                            dc_limit[k] = DC_TIMING_BUTTON_RELEASE_RATE;
                            dc_running_sum[k] = 0U;
                        }
                        k++;
                    }
                }
            }
#endif//(DRIFT_COMP_METHOD==n)

#if (CTSU_DRIFTCOMP_WHEN_ALL_INACTIVE > 0)
            /* When multi-touch is enabled, drift compensate only when all channels are not touched.*/
            if((p_rt_drift_comp != NULL) && (channels_touched == 0U))
            {/* Time to drift compensate */
                p_rt_drift_comp(NULL);
            }
#else
            if(p_rt_drift_comp!=NULL)
            {/* Time to drift compensate */
                p_rt_drift_comp(NULL);
            }
#endif//CTSU_DRIFTCOMP_WHEN_ALL_INACTIVE

        }
#endif//CFG_DRIFT_COMPENSATION
        r_touch_state = CTSU_ACTION_RUN_AUTO_TUNE;
        break;

    case CTSU_ACTION_RUN_AUTO_TUNE:
        /****************************************/
        /*  Perform run-time auto-calibration   */
        /****************************************/
#if (CFG_AUTO_TUNE > 0)
        if ((uint32_t)((uint32_t)opts & (uint32_t)CTSU_PROCESS_OPTION_ENABLE_AUTO_CALIB) == (uint32_t)CTSU_PROCESS_OPTION_ENABLE_AUTO_CALIB)
        {
#if CTSU_DRIFTCOMP_WHEN_ALL_INACTIVE > 0
            if((p_rt_auto_tune != NULL) && (channels_touched == 0U))
#else
            if (p_rt_auto_tune != NULL)
#endif//CTSU_DRIFTCOMP_WHEN_ALL_INACTIVE
            {/* We've reached the period for the auto-tuning, and no channels are determined as touched. */
                p_rt_auto_tune(NULL);
            }

        }
#endif//CFG_AUTO_TUNE
        r_touch_state = CTSU_ACTION_START_NEW_SCAN;
        break;

    case CTSU_ACTION_START_NEW_SCAN:
        /********************************/
        /**     CTSU is ready to run    */
        /********************************/
        if((uint32_t)((uint32_t)opts & (uint32_t)CTSU_PROCESS_OPTION_AUTO_SCAN) == (uint32_t)CTSU_PROCESS_OPTION_AUTO_SCAN)
        {
            /** Start a new scan. */
            HW_CTSU_CTSUCR0SetBitCTSUINIT(1U);
            HW_CTSU_CTSUCR0SetBitCTSUSTRT(1U);
            ctsu_busy = true;
            r_touch_state = CTSU_ACTION_WAITING_FOR_SCAN_COMPLETE;
        }
        break;

    default:
        return ret_val;
        break;
    }
    /** Save a copy of the state machine */
    p_ctrl->ctsu_process_state = r_touch_state;

    return ret_val;
}/* End of function R_CTSU_Process */

/**
 * *********************************************************************************************************************
 * @brief Function to allow the user to read the results after executing a scan.
 *
 * @retval SSP_SUCCESS              No errors.
 * @retval CTSU_ERR_INVALID_CMD     Command option provided is invalid.
 * @retval SSP_ERR_INVALID_POINTER  NULL pointer passed as an argument for a required parameter. -OR- Memory
 *                                  allocation failed.
 * @retval SSP_ERR_INVALID_ARGUMENT Illegal parameter was found.
 * @retval SSP_ERR_CTSU_OFFSET_ADJUSTMENT_FAILED  Auto tuning failed because CTSU encountered a ICOMP error.
 *                          Check the configuration parameters for each channel by locating the point of failure.
 **********************************************************************************************************************/
ssp_err_t R_CTSU_Read_Results(ctsu_ctrl_t * p_api_ctrl, void * p_dest, ctsu_read_t opts,
		                      const ctsu_channel_pair_t * channels, const uint16_t count)
{
    ctsu_instance_ctrl_t * p_ctrl = (ctsu_instance_ctrl_t *) p_api_ctrl;

    SSP_PARAMETER_NOT_USED(p_ctrl);
    ssp_err_t ret_val = SSP_SUCCESS;
    uint8_t arr[4];
    int16_t ch_offset = 0;
    uint16_t i = 0U;

    CTSU_ERROR_RETURN((CTSU_MODE_MUTUAL_CAPACITANCE == mode) ||
       (CTSU_MODE_SELF_CAPACITANCE == mode), SSP_ERR_NOT_OPEN);

#if (1 == CTSU_CFG_PARAM_CHECKING_ENABLE)

    /* Check pointers */
    /* p_ctrl not currently used, but add test in case code is updated later to use it */
    CTSU_ERROR_RETURN(NULL != p_ctrl, SSP_ERR_INVALID_POINTER);
    CTSU_ERROR_RETURN(NULL != p_dest, SSP_ERR_INVALID_POINTER);

    /** Get results for the requested options */
    switch(opts)
    {
    case CTSU_READ_BINARY_DATA_SEL:
        /* No break, Fall through */
    case CTSU_READ_RAW_SENSOR_OUTPUT_SEL:
        /* No break, Fall through */
    case CTSU_READ_FILTERED_SENSOR_VALUES_SEL:
        /* No break, Fall through */
    case CTSU_READ_DELTA_COUNT_SEL:
        /* No break, Fall through */
    case CTSU_READ_BASELINE_COUNT_SEL:
        /* No break, Fall through */
    case CTSU_READ_FILTERED_REF_ICO_VALUES_SEL:
        /* Check channels pointer */
        CTSU_ERROR_RETURN(NULL != channels, SSP_ERR_INVALID_POINTER);
        for(i = 0U; i < count; i++)
        {
            ret_val = R_CTSU_Control(CTSU_CMD_IS_CH_RX, (void*)&channels[i].rx);
            CTSU_ERROR_RETURN(SSP_SUCCESS == ret_val, ret_val);

            if(CTSU_MODE_MUTUAL_CAPACITANCE == mode)
            {/* Current mode is mutual mode */
                ret_val = R_CTSU_Control(CTSU_CMD_IS_CH_TX, (void*)&channels[i].tx);
                CTSU_ERROR_RETURN(SSP_SUCCESS == ret_val, ret_val);
            }
        }
        break;

    default:
        /* No channels pointer check */
        break;
    }
#endif
    /** Get results for the requested options */
    switch(opts)
    {
    case CTSU_READ_BINARY_DATA_ALL:
        /* Copy the binary data */
        for(i = 0U; i < (uint16_t)(num_active_channels - num_excluded_channels) ; i = (uint16_t)(i + 8))
        {
            ((uint8_t*)p_dest)[i>>3] = ((uint8_t*)g_p_channels_touched_binary)[i>>3];
        }
        break;

    case CTSU_READ_BINARY_DATA_SEL:
        for(i = 0U; i < count; i++)
        {
            /* Calculate the button offset from the active channels */
            arr[0] = (uint8_t)channels[i].rx;
            arr[1] = (uint8_t)channels[i].tx;
            arr[2] = 0U;
            arr[3] = 0U;

            /* Check if the button exists as a valid input */
            ret_val = R_CTSU_Control(CTSU_CMD_GET_CHANNEL_OFFSET, &arr[0]);
            /** Assemble arr[3] and arr[2] as the high and low byte */
            ch_offset = (int16_t)arr[3];
            ch_offset = (int16_t)(((uint16_t)ch_offset << 8) | (uint16_t)arr[2]);

            if((ret_val != SSP_SUCCESS) || (ch_offset == -1))
            {/* Break out of loop and return error value */
                break;
            }

            if((((uint8_t*)g_p_channels_touched_binary)[(uint16_t)ch_offset>>3] >> ((uint16_t)ch_offset & 0x07)) & 0x01)
            {
                ((uint8_t*)p_dest)[i>>3] |= (uint8_t)(1<<i);
            }

        }
        break;

    case CTSU_READ_RAW_SENSOR_OUTPUT_ALL:
        if(mode == CTSU_MODE_SELF_CAPACITANCE)
        {/* CTSU is in self-mode*/
            memcpy(p_dest, p_ctsu_output, (uint16_t)num_active_channels*sizeof(ctsu_channel_data_self_t));
        }
        else if(CTSU_MODE_MUTUAL_CAPACITANCE == mode)
        {/* CTSU is in mutual mode */
            memcpy(p_dest, p_ctsu_output, (uint16_t)num_active_channels*sizeof(ctsu_channel_data_mutual_t));
        }
        break;

    case CTSU_READ_RAW_SENSOR_OUTPUT_SEL:
        for(i = 0U; i < count; i++)
        {
            /* Calculate the button offset from the active channels */
            arr[0] = (uint8_t)channels[i].rx;
            arr[1] = (uint8_t)channels[i].tx;
            arr[2] = 0U;
            arr[3] = 0U;

            /* Check if the button exists as a valid input */
            ret_val = R_CTSU_Control(CTSU_CMD_GET_CHANNEL_OFFSET_RAW, &arr[0]);
            /** Assemble arr[3] and arr[2] as the high and low byte */
            ch_offset = (int16_t)arr[3];
            ch_offset = (int16_t)(((uint16_t)ch_offset << 8) | (uint16_t)arr[2]);

            if((ret_val != SSP_SUCCESS) || (ch_offset == -1))
            {/* Break out of loop and return error value */
                break;
            }

            if(CTSU_MODE_MUTUAL_CAPACITANCE == mode)
            {/* Mutual-capacitance mode */
                ((uint16_t*)p_dest)[i]= (uint16_t)(((ctsu_channel_data_mutual_t*)p_ctsu_output)[ch_offset].sen_cnt_2 - ((ctsu_channel_data_mutual_t*)p_ctsu_output)[ch_offset].sen_cnt_1);
            }
            else
            {/* Self-capacitance */
                ((uint16_t*)p_dest)[i] = ((ctsu_channel_data_self_t*)p_ctsu_output)[ch_offset].sensor_count;
            }

        }
        break;

    case CTSU_READ_FILTERED_SENSOR_VALUES_ALL:
        memcpy(p_dest, (void*)scount, (uint16_t)(num_active_channels - num_excluded_channels)*sizeof(uint16_t));
        break;

    case CTSU_READ_FILTERED_SENSOR_VALUES_SEL:
        for(i = 0U; i < count; i++)
        {
            /* Calculate the button offset from the active channels */
            arr[0] = (uint8_t)channels[i].rx;
            arr[1] = (uint8_t)channels[i].tx;
            arr[2] = 0U;
            arr[3] = 0U;

            /* Check if the button exists as a valid input */
            ret_val = R_CTSU_Control(CTSU_CMD_GET_CHANNEL_OFFSET, &arr[0]);
            /** Assemble arr[3] and arr[2] as the high and low byte */
            ch_offset = (int16_t)arr[3];
            ch_offset = (int16_t)(((uint16_t)ch_offset << 8) | (uint16_t)arr[2]);

            if((ret_val != SSP_SUCCESS) || (ch_offset == -1))
            {/* Break out of loop and return error value */
                break;
            }
            /* Write out the value */
            ((uint16_t*)p_dest)[i] = scount[ch_offset];
        }
        break;

    case CTSU_READ_DELTA_COUNT_ALL:
        memcpy(p_dest, dcount, (uint16_t)(num_active_channels - num_excluded_channels)*sizeof(int16_t));
        break;

    case CTSU_READ_DELTA_COUNT_SEL:
        for(i = 0U; i < count; i++)
        {
            /* Calculate the button offset from the active channels */
            arr[0] = (uint8_t)channels[i].rx;
            arr[1] = (uint8_t)channels[i].tx;
            arr[2] = 0U;
            arr[3] = 0U;

            /* Check if the button exists as a valid input */
            ret_val = R_CTSU_Control(CTSU_CMD_GET_CHANNEL_OFFSET, &arr[0]);
            /** Assemble arr[3] and arr[2] as the high and low byte */
            ch_offset = (int16_t)arr[3];
            ch_offset = (int16_t)(((uint16_t)ch_offset << 8) | (uint16_t)arr[2]);

            if((ret_val != SSP_SUCCESS) || (ch_offset == -1))
            {/* Break out of loop and return error value */
                break;
            }

            /* Write out the value */
            ((uint16_t*)p_dest)[i] = (uint16_t)dcount[ch_offset];
        }
        break;

    case CTSU_READ_BASELINE_COUNT_ALL:
        /* CTSU is in self-mode*/
            memcpy(p_dest, sensor_baseline, (uint16_t)num_active_channels*sizeof(sensor_baseline_t));
        break;

    case CTSU_READ_BASELINE_COUNT_SEL:
        for(i = 0U; i < count; i++)
        {
            /* Calculate the button offset from the active channels */
            arr[0] = (uint8_t)channels[i].rx;
            arr[1] = (uint8_t)channels[i].tx;
            arr[2] = 0U;
            arr[3] = 0U;

            /* Check if the button exists as a valid input */
            ret_val = R_CTSU_Control(CTSU_CMD_GET_CHANNEL_OFFSET_RAW, &arr[0]);
            /** Assemble arr[3] and arr[2] as the high and low byte */
            ch_offset = (int16_t)arr[3];
            ch_offset = (int16_t)(((uint16_t)ch_offset << 8) | (uint16_t)arr[2]);

            if((ret_val != SSP_SUCCESS) || (ch_offset == -1))
            {/* Break out of loop and return error value */
                break;
            }

            ((uint16_t*)p_dest)[i] = sensor_baseline[ch_offset];
        }
        break;

    case CTSU_READ_FILTERED_REF_ICO_VALUES_ALL:
            memcpy(p_dest, (void*)rcount, (uint16_t)(num_active_channels - num_excluded_channels)*sizeof(uint16_t));
            break;

    case CTSU_READ_FILTERED_REF_ICO_VALUES_SEL:
		for(i = 0U; i < count; i++)
        {
            /* Calculate the button offset from the active channels */
            arr[0] = (uint8_t)channels[i].rx;
            arr[1] = (uint8_t)channels[i].tx;
            arr[2] = 0U;
            arr[3] = 0U;

            /* Check if the button exists as a valid input */
            ret_val = R_CTSU_Control(CTSU_CMD_GET_CHANNEL_OFFSET, &arr[0]);
            /** Assemble arr[3] and arr[2] as the high and low byte */
            ch_offset = (int16_t)arr[3];
            ch_offset = (int16_t)(((uint16_t)ch_offset << 8) | (uint16_t)arr[2]);

            if((ret_val != SSP_SUCCESS) || (ch_offset == -1))
            {/* Break out of loop and return error value */
                break;
            }
            /* Write out the value */
            ((uint16_t*)p_dest)[i] = rcount[ch_offset];
        }
        break;

    default:
        ret_val = SSP_ERR_INVALID_ARGUMENT;
        break;
    }

    return ret_val;
}/* End of function R_Touch_Read_Results */

/**
 * ********************************************************************************************************************
 * @brief Allows the user to query manipulate R_Touch layer, and CTSU operation. This function is not exposed for usage
 * in the interface API.
 * @retval SSP_SUCCESS              No errors.
 * @retval CTSU_ERR_INVALID_CMD     Command option provided is invalid.
 * @retval SSP_ERR_IN_USE           CTSU Scan is ongoing.

 **********************************************************************************************************************/
ssp_err_t R_CTSU_Control(ctsu_cmd_t cmd, void* p_data)
{
    volatile int8_t rx_ch, tx_ch = 0;
    int8_t offset_rx = 0;
    int8_t offset_tx = 0;
    int16_t offset, offset_cpy = 0;
    ssp_err_t ret_val = SSP_SUCCESS;
    uint64_t rx_en = 0U, tx_en = 0U;

    CTSU_ERROR_RETURN((CTSU_MODE_MUTUAL_CAPACITANCE == mode) ||
       (CTSU_MODE_SELF_CAPACITANCE == mode), SSP_ERR_NOT_OPEN);

#if (1 == CTSU_CFG_PARAM_CHECKING_ENABLE)
    switch(cmd)
    {/* Following commands cannot be run while CTSU is operational */
    case CTSU_CMD_WR_CH_SET:
        CTSU_ERROR_RETURN(0 == HW_CTSU_CTSUSTGetBitCTSUSTC(), SSP_ERR_IN_USE);
        break;

    /* Others can be run while ctsu is operating */
    default:

        break;
    }
#endif
    /** Execute the control request */
    switch(cmd)
    {

    case CTSU_CMD_GET_MODE:
        *(ctsu_mode_t*)p_data = (ctsu_mode_t)mode;
        break;
    case CTSU_CMD_GET_ACTIVE_CHANNELS:
        *(int16_t*)p_data = num_active_channels;
        break;

    case CTSU_CMD_GET_CHANNEL_OFFSET_RAW:
        rx_ch = ((int8_t*)p_data)[0];
        tx_ch = ((int8_t*)p_data)[1];

        for(int8_t i = 0; (i < num_rx) && (rx_ch > -1); i++)
        {/* Get the offset in rx_channels */
            if(rx_channels[i] == rx_ch)
            {
                offset_rx = i;
                break;
            }
        }
        for(int8_t i = 0; (i < num_tx) && (tx_ch > -1); i++)
        {/* Get the offset in tx_channels */
            if(tx_channels[i] == tx_ch)
            {
                offset_tx = i;
                break;
            }
        }
        /* Get the raw offset */
        if(CTSU_MODE_SELF_CAPACITANCE == mode)
        {
            offset = offset_rx;
            offset_cpy = offset;
        }
        else
        {
            offset = (int16_t)((offset_rx * num_tx) + (offset_tx));
            offset_cpy = offset;
        }
        {
            uint16_t temp_swap_offset = (uint16_t) offset;
            ((uint8_t*)p_data)[2] = (uint8_t)(temp_swap_offset & 0xFF);
            ((uint8_t*)p_data)[3] = (uint8_t)((temp_swap_offset >> 8) & 0xFF);
        }
        break;

    case CTSU_CMD_GET_CHANNEL_OFFSET:
        rx_ch = ((int8_t*)p_data)[0];
        tx_ch = ((int8_t*)p_data)[1];

        for(int8_t i = 0; (i < num_rx) && (rx_ch > -1); i++)
        {/* Get the offset in rx_channels */
            if(rx_channels[i] == rx_ch)
            {
                offset_rx = i;
                break;
            }
        }
        for(int8_t i = 0; (i < num_tx) && (tx_ch > -1); i++)
        {/* Get the offset in tx_channels */
            if(tx_channels[i] == tx_ch)
            {
                offset_tx = i;
                break;
            }
        }
        /* Get the raw offset */
        if(CTSU_MODE_SELF_CAPACITANCE == mode)
        {
            offset = offset_rx;
            offset_cpy = offset;
        }
        else
        {
            offset = (int16_t)((offset_rx * num_tx) + offset_tx);
            offset_cpy = offset;
        }
        for(int8_t i = 0; i < num_excluded_channels; i++)
        {
            if(excluded_channel_offset[i] < offset_cpy)
            {
                offset = (int16_t)(offset - 1);
            }
            else if(excluded_channel_offset[i] == offset_cpy)
            {
                offset = -1;
            }
        }
        ((uint8_t*)p_data)[2] = (uint8_t)((uint16_t)offset & 0xFF);
        ((uint8_t*)p_data)[3] = (uint8_t)(((uint16_t)offset >> 8) & 0xFF);
        break;

    case CTSU_CMD_IS_CH_RX:
            rx_ch = *(int8_t*)p_data;

            if(4 < g_ctsu_feature.ctsuchtrc_register_count)
            {
                rx_en =  HW_CTSU_CTSUCHAC4Get();
                tx_en = HW_CTSU_CTSUCHTRC4Get();
                rx_en <<= 8;
                tx_en <<= 8;
            }

            if(3 < g_ctsu_feature.ctsuchtrc_register_count)
            {
                rx_en |=  HW_CTSU_CTSUCHAC3Get();
                tx_en |= HW_CTSU_CTSUCHTRC3Get();
                rx_en <<= 8;
                tx_en <<= 8;
            }

            rx_en |=  HW_CTSU_CTSUCHAC2Get();
            tx_en |= HW_CTSU_CTSUCHTRC2Get();
            rx_en <<= 8;
            tx_en <<= 8;
            rx_en |=  HW_CTSU_CTSUCHAC1Get();
            tx_en |= HW_CTSU_CTSUCHTRC1Get();
            rx_en <<= 8;
            tx_en <<= 8;
            rx_en |=  HW_CTSU_CTSUCHAC0Get();
            tx_en |= HW_CTSU_CTSUCHTRC0Get();

            if((rx_en & ((uint64_t)1<<rx_ch))==0)
            {/* Channel is not enabled */
                ret_val = SSP_ERR_INVALID_ARGUMENT;
            }
            if((tx_en & ((uint64_t)1<<rx_ch))!=0)
            {/* Channel is configured to be in transmit mode */
                ret_val = SSP_ERR_INVALID_ARGUMENT;
            }
            break;

        case CTSU_CMD_IS_CH_TX:
            tx_ch = *(int8_t*)p_data;

            if(4 < g_ctsu_feature.ctsuchtrc_register_count)
            {
                rx_en =  HW_CTSU_CTSUCHAC4Get();
                tx_en = HW_CTSU_CTSUCHTRC4Get();
                rx_en <<= 8;
                tx_en <<= 8;
            }

            if(3 < g_ctsu_feature.ctsuchtrc_register_count)
            {
                rx_en |=  HW_CTSU_CTSUCHAC3Get();
                tx_en |= HW_CTSU_CTSUCHTRC3Get();
                rx_en <<= 8;
                tx_en <<= 8;
            }

            rx_en |=  HW_CTSU_CTSUCHAC2Get();
            tx_en |= HW_CTSU_CTSUCHTRC2Get();
            rx_en <<= 8;
            tx_en <<= 8;
            rx_en |=  HW_CTSU_CTSUCHAC1Get();
            tx_en |= HW_CTSU_CTSUCHTRC1Get();
            rx_en <<= 8;
            tx_en <<= 8;
            rx_en |=  HW_CTSU_CTSUCHAC0Get();
            tx_en |= HW_CTSU_CTSUCHTRC0Get();

            if((rx_en & ((uint64_t)1<<tx_ch))==0)
            {/* Channel is not enabled */
                ret_val = SSP_ERR_INVALID_ARGUMENT;
            }
            if((tx_en & ((uint64_t)1<<tx_ch))==0)
            {/* Channel is configured to be in receive mode */
                ret_val = SSP_ERR_INVALID_ARGUMENT;
            }
            break;

    case CTSU_CMD_GET_THRESHOLD:
        memcpy(p_data, (void*)threshold, (uint16_t)num_active_channels*sizeof(uint16_t));
        break;

    case CTSU_CMD_GET_HYSTERESIS:
        memcpy(p_data, (void*)hysteresis, (uint16_t)num_active_channels*sizeof(uint16_t));
        break;

    case CTSU_CMD_RD_CH_SET:
        memcpy(p_data, (void*)wr_set, (uint16_t)num_active_channels*sizeof(ctsu_channel_setting_t));
        break;

    case CTSU_CMD_WR_CH_SET:
        /* Reload channel settings written in WR interrupt */
        memcpy((void*)wr_set, p_data, (uint16_t)num_active_channels*sizeof(ctsu_channel_setting_t));
        break;

    default:
        ret_val = SSP_ERR_INVALID_ARGUMENT;
        break;
    }

    return ret_val;
}/* End of function R_CTSU_Control */

/**
 * *********************************************************************************************************************
 * @brief Function used to close the CTSU driver, stop clocking of the CTSU peripheral, reset all internal structures
 * and close the DTC Transfer instances. The function can also optionally save the configuration for a rapid wake-up
 * on re-opening.
 * @retval SSP_SUCCESS  No errors.
 **********************************************************************************************************************/
ssp_err_t R_CTSU_Close( ctsu_ctrl_t* p_api_ctrl, ctsu_close_option_t opts)
{
    ctsu_instance_ctrl_t * p_ctrl = (ctsu_instance_ctrl_t *) p_api_ctrl;

    ctsu_hw_cfg_t * p_config = p_ctrl->p_ctsu_hw_cfg;
    ssp_err_t ret_val = SSP_SUCCESS;

    /* Wait for scan to complete */
    while((0!=HW_CTSU_CTSUSTGetBitCTSUSTC()) && (1==HW_CTSU_CTSUCR0GetBitCTSUSTRT()))
    {
        if((1==HW_CTSU_CTSUSTGetBitCTSUSOVF()) || (1==HW_CTSU_CTSUSTGetBitCTSUROVF()) ||
                (1==HW_CTSU_CTSUERRSGetBitCTSUICOMP()))
        {/* An error occurred. User should clear error before proceeding to close. */
            break;
        }
    }

    /* Change mode to indicate driver is being shut down. */
    /* mode will be changed to unconfigured when the close is complete. */
    mode = CTSU_MODE_CLOSING;

    /* Stop all conversions */
    HW_CTSU_CTSUCR0SetBitCTSUSTRT(0U);
    /* Re-initialize CTSU SFRs*/
    HW_CTSU_CTSUCR0SetBitCTSUINIT(1U);

    /* Disable interrupts */
    ssp_feature_t ssp_feature = {{(ssp_ip_t) 0U}};
    ssp_feature.channel = 0U;
    ssp_feature.unit = 0U;
    ssp_feature.id = SSP_IP_CTSU;

    ssp_vector_info_t * p_vector_info;
    fmi_event_info_t event_info = {(IRQn_Type) 0U};
    g_fmi_on_fmi.eventInfoGet(&ssp_feature, SSP_SIGNAL_CTSU_WRITE, &event_info);
    IRQn_Type write_irq = event_info.irq;
    g_fmi_on_fmi.eventInfoGet(&ssp_feature, SSP_SIGNAL_CTSU_READ, &event_info);
    IRQn_Type read_irq = event_info.irq;
    g_fmi_on_fmi.eventInfoGet(&ssp_feature, SSP_SIGNAL_CTSU_END, &event_info);
    IRQn_Type end_irq = event_info.irq;

    NVIC_DisableIRQ(write_irq);
    NVIC_DisableIRQ(read_irq);
    NVIC_DisableIRQ(end_irq);

    R_BSP_IrqStatusClear(write_irq);
    R_BSP_IrqStatusClear(read_irq);
    R_BSP_IrqStatusClear(end_irq);

    NVIC_ClearPendingIRQ(write_irq);
    NVIC_ClearPendingIRQ(read_irq);
    NVIC_ClearPendingIRQ(end_irq);

    R_SSP_VectorInfoGet(write_irq, &p_vector_info);
    *(p_vector_info->pp_ctrl) = NULL;
    R_SSP_VectorInfoGet(read_irq, &p_vector_info);
    *(p_vector_info->pp_ctrl) = NULL;
    R_SSP_VectorInfoGet(end_irq, &p_vector_info);
    *(p_vector_info->pp_ctrl) = NULL;

    /* Return SFRs to P-O-R values */
    if((uint32_t)opts & (uint32_t)CTSU_CLOSE_OPTION_RESET_SFRS)
    {
        HW_CTSU_CTSUCR0Set(0);
        HW_CTSU_CTSUCR1Set(0);
        HW_CTSU_CTSUSDPRSSet(0);
        HW_CTSU_CTSUDCLKCSet(0);
        HW_CTSU_CTSUSSTSet(0);
        HW_CTSU_CTSUCHAC0Set(0);
        HW_CTSU_CTSUCHAC1Set(0);
        HW_CTSU_CTSUCHAC2Set(0);
            if(3 < g_ctsu_feature.ctsuchac_register_count)
            {
                HW_CTSU_CTSUCHAC3Set(0);
            }
            if(4 < g_ctsu_feature.ctsuchac_register_count)
            {
                HW_CTSU_CTSUCHAC4Set(0);
            }
        HW_CTSU_CTSUCHTRC0Set(0);
        HW_CTSU_CTSUCHTRC1Set(0);
        HW_CTSU_CTSUCHTRC2Set(0);
            if(3 < g_ctsu_feature.ctsuchtrc_register_count)
            {
                HW_CTSU_CTSUCHTRC3Set(0);
            }
            if(4 < g_ctsu_feature.ctsuchtrc_register_count)
            {
                HW_CTSU_CTSUCHTRC4Set(0);
            }
    }

    if((uint32_t)opts & (uint32_t)CTSU_CLOSE_OPTION_SUSPEND)
    {/* Snooze ON*/
        HW_CTSU_CTSUCR0SetBitCTSUSNZ(1U);
    }

    if((uint32_t)opts & (uint32_t)CTSU_CLOSE_OPTION_POWER_DOWN)
    {
        R_BSP_ModuleStop(&ssp_feature);
    }

    if((uint32_t)opts & (uint32_t)CTSU_CLOSE_OPTION_SAVE_CONFIG)
    {/* Save the parameters which need to be reloaded if user decides to re-run the config. */
        if(p_config == NULL)
        {
            ret_val = SSP_ERR_INVALID_POINTER;
        }
        else
        {
            if(p_config->baseline!=NULL)
            {
                memcpy(p_config->baseline, &sensor_baseline[0], (uint16_t)num_active_channels*sizeof(sensor_baseline_t));
            }

            if(p_config->write_settings!=NULL)
            {
                memcpy(p_config->write_settings, (void*)wr_set, (uint16_t)num_active_channels * sizeof(ctsu_channel_setting_t));
            }
        }
    }

#if (USING_DTC_FOR_CTSU > 0)
    /** Close the Transfer Instances*/
    if((NULL != p_ctrl->p_api_transfer->close) && (NULL != p_ctrl->p_api_transfer))
    {
        if(NULL != p_ctrl->p_lowerl_lvl_transfer_read_ctrl)
        {
            ret_val =  p_ctrl->p_api_transfer->close(p_ctrl->p_lowerl_lvl_transfer_read_ctrl);
        }

        if(NULL != p_ctrl->p_lowerl_lvl_transfer_write_ctrl)
        {
            ret_val = p_ctrl->p_api_transfer->close(p_ctrl->p_lowerl_lvl_transfer_write_ctrl);
        }
    }
#endif

    /* Reset the stabilization time */
    stabilization_time = STARTUP_TIME;
    /* Reset the process state machine */
    r_touch_state = CTSU_ACTION_START_NEW_SCAN;

    g_p_channels_touched_binary = NULL;

    r_touch_ready = 0;

    R_BSP_HardwareUnlock(&ssp_feature);

    /* Indicate that we are no longer in any valid mode. */
    mode = CTSU_MODE_UNCONFIGURED;

    return ret_val;
}/* End of function R_Touch_Close */

/*******************************************************************************************************************//**
 * @} (end defgroup CTSU)
 **********************************************************************************************************************/


/**********************************************************************************************************************/
/*                                     Local function definitions                                                     */
/**********************************************************************************************************************/

#if (CFG_DRIFT_COMPENSATION > 0)
#if USE_DYNAMIC_MEMORY > 0
ctsu_channel_data_self_t** calib_buffer = NULL;
#else
/* Make sure there is enough memory available for calibration buffer. */
ctsu_channel_data_self_t calib_buffer[CALIB_SCAN_COUNT][CTSU_CFG_MAX_ACTIVE_CHANNEL_COMBOS];
#endif
static int32_t default_sensor_baseline_init_self(void* arg)
{
    /** Used to prevent compiler warning for unused variable */
    SSP_PARAMETER_NOT_USED(arg);
    int8_t i = 0;
    uint16_t ch;
    uint16_t loop_max;
    uint16_t    max_num;

    uint32_t   work_buff[6];
    ctsu_channel_data_self_t* p_backup = p_ctsu_output;
    /* Initialize the buffer we are storing data to. */
    memset(&calib_buffer[0][0], 0 , sizeof(calib_buffer));

    /* Wait for any previous scans to complete */
    while((HW_CTSU_CTSUSTGetBitCTSUSTC() != 0) && (HW_CTSU_CTSUCR0GetBitCTSUSTRT() == 1))
    {
        /* Do nothing but wait */
    }

    /* Gather data from  CALIB_SCAN_COUNT number of scans */
    for( i = 0; i < CALIB_SCAN_COUNT; i++ )
    {
        /* CTSU should dump data to the calib location*/
        p_ctsu_output =  &calib_buffer[i][0];
        /* Start scan */
        HW_CTSU_CTSUCR0SetBitCTSUINIT(1U);
        HW_CTSU_CTSUCR0SetBitCTSUSTRT(1U);

        do{
            /* Wait for scan to complete */
            if(((HW_CTSU_CTSUSTGetBitCTSUROVF()) || (HW_CTSU_CTSUERRSGetBitCTSUICOMP())) || (HW_CTSU_CTSUSTGetBitCTSUSOVF()))
            {/* Error occurred. Restore output location and exit */
                p_ctsu_output = p_backup;
                return -1;
            }
        }while(((HW_CTSU_CTSUSTGetBitCTSUSTC()) != 0) && ((HW_CTSU_CTSUCR0GetBitCTSUSTRT()) == 1));
    }

    for( ch = 0U; ch < (uint16_t)num_active_channels; ch++ )
    {
        work_buff[0] = (uint32_t)(calib_buffer[ 0 ][ch].sensor_count + calib_buffer[ 1 ][ch].sensor_count);    /* buff0 + buff1 */
        work_buff[1] = (uint32_t)(calib_buffer[ 0 ][ch].sensor_count + calib_buffer[ 2 ][ch].sensor_count);    /* buff0 + buff2 */
        work_buff[2] = (uint32_t)(calib_buffer[ 0 ][ch].sensor_count + calib_buffer[ 3 ][ch].sensor_count);    /* buff0 + buff3 */
        work_buff[3] = (uint32_t)(calib_buffer[ 1 ][ch].sensor_count + calib_buffer[ 2 ][ch].sensor_count);    /* buff1 + buff2 */
        work_buff[4] = (uint32_t)(calib_buffer[ 1 ][ch].sensor_count + calib_buffer[ 3 ][ch].sensor_count);    /* buff1 + buff3 */
        work_buff[5] = (uint32_t)(calib_buffer[ 2 ][ch].sensor_count + calib_buffer[ 3 ][ch].sensor_count);    /* buff2 + buff3 */

        loop_max = 0U;
        max_num = 0U;
        for(; loop_max < 5U ; loop_max++)
        {
            if( work_buff[ max_num ] < work_buff[ loop_max + 1 ] )
            {
                max_num = (uint8_t)(loop_max + 1);
            }
            else
            {
                /* Do Nothing */
            }
        }
        sensor_baseline[ ch ] = (sensor_baseline_t)( work_buff[max_num] / 2 );
    }

    /* Restore the location for raw data storage */
    p_ctsu_output = p_backup;

    return 0;
}

/***********************************************************************************************************************
 * Function Name: default_sensor_baseline_adjust_self
 * Description  : Recommended algorithm for auto-tuning the channels during initialization. Auto tuning is basically
 *                  changing the offset sensor setting such that the difference between the sensor count is slightly
 *                  less than or equal to the reference count. User should not touch electrodes while this executes.
 * Arguments    : channel -
 *                    Channel number to enable (Use 0 only).
 * Return Value : none
 * Notes: REA:ONR: Tested this function 11 times with a CTSU configuration file. CTSUICOMP was not occuring.
 **********************************************************************************************************************/
static int32_t default_sensor_baseline_adjust_self(void* arg)
{
    /** Used to prevent compiler warning for unused variable */
    SSP_PARAMETER_NOT_USED(arg);
    uint8_t* touch_binary = g_p_channels_touched_binary;
    /** Initialize variables for loop below */
    uint16_t i = 0U;
    uint16_t j = 0U;
    uint16_t k = 0U;
    uint16_t m = 0U;
    uint16_t n = 0U;
    for(; i < (uint16_t)num_active_channels ; i++)
    {
        /* Figure out which bit is about to be written to. */
        m = (k >> 3);   //byte selection
        n = (k & 7);    //bit selection
        if(i == (uint16_t)excluded_channel_offset[j])
        {/* Channel is to be excluded */
            j++;
        }
        else
        {/* Channel is not excluded. */
            if((touch_binary[m] & (1<<n))==0)
            {/* Channel is not being touched. */
#if (DRIFT_COMP_METHOD>=DRIFT_COMP_ALT_1)
                if(dc_counter[k] < dc_limit[k])
#else
                if(dc_counter[k] < DC_TIMING_STEADY_STATE)
#endif
                {/* Increment the counter. */
                    dc_counter[k]++;
                    dc_running_sum[k] += scount[k];
                }
                else
                {/* Channel is not touched and it's time to drift compensate. */
                    dc_running_sum[k] += scount[k];
                    dc_running_sum[k] /= (running_sum_t)(dc_counter[k]+1);
                    sensor_baseline[i] = (sensor_baseline_t)dc_running_sum[k];
                    dc_running_sum[k] = 0U;
#if (DRIFT_COMP_METHOD==DRIFT_COMP_ALT_1) || (DRIFT_COMP_METHOD==DRIFT_COMP_ALT_3)
                    if(dc_limit[k] > DC_TIMING_STEADY_STATE)
                    {
                        dc_limit[k]--;
                    }
                    else if(dc_limit[k] < DC_TIMING_STEADY_STATE)
                    {
                        dc_limit[k]++;
                    }
#elif (DRIFT_COMP_METHOD==DRIFT_COMP_ALT_2)
                    dc_limit[k] = DC_TIMING_STEADY_STATE;
#endif
                    dc_counter[k] = 0U;
                }
            }
            k++;
        }
    }

    return 0;
}
/***********************************************************************************************************************
 * Function Name: default_sensor_baseline_init_mutual
 * Description  : Recommended algorithm for auto-tuning the channels during initialization. Auto tuning is basically
 *                  changing the offset sensor setting such that the difference between the sensor count is slightly
 *                  less than or equal to the reference count. User should not touch electrodes while this executes.
 * Arguments    : channel -
 *                    Channel number to enable (Use 0 only).
 * Return Value : none
 * Notes: REA:ONR: Tested this function 11 times with a CTSU configuration file. CTSUICOMP was not occuring.
 **********************************************************************************************************************/
/* Make sure there is enough memory available for calibration buffer. */
uint16_t calib_buffer_mutual[CALIB_SCAN_COUNT][CTSU_CFG_MAX_ACTIVE_CHANNEL_COMBOS];
static int32_t default_sensor_baseline_init_mutual(void* arg)
{
    SSP_PARAMETER_NOT_USED(arg);
    uint8_t ch;
    uint8_t loop_max;
    uint8_t    max_num;

    uint32_t   work_buff[6];
    ctsu_channel_data_mutual_t* ctsu_raw_result = p_ctsu_output;
    /* Initialize the buffer we are storing data to. */
    memset(&calib_buffer_mutual[0][0], 0 , sizeof(calib_buffer_mutual));
    /* Wait for any previous scans to complete */
    while((HW_CTSU_CTSUSTGetBitCTSUSTC()!=0) && (HW_CTSU_CTSUCR0GetBitCTSUSTRT() == 1))
    {
        /* Wait */
    }

    /* Gather data from  CALIB_SCAN_COUNT number of scans */
    for( int i = 0; i < CALIB_SCAN_COUNT; i++ )
    {
        /* Start scan */
        HW_CTSU_CTSUCR0SetBitCTSUINIT(1U);
        HW_CTSU_CTSUCR0SetBitCTSUSTRT(1U);
        /* Wait for scan to complete */
        do{
            if( ((HW_CTSU_CTSUSTGetBitCTSUROVF()) || (HW_CTSU_CTSUERRSGetBitCTSUICOMP())) || (HW_CTSU_CTSUSTGetBitCTSUSOVF()))
            {/* Error occurred. Restore output location and exit */
                return -1;
            }
        }while(((HW_CTSU_CTSUSTGetBitCTSUSTC()) != 0) && ((HW_CTSU_CTSUCR0GetBitCTSUSTRT()) == 1));

        {
            /** Declare and initialize variables for loop below */
            int j = 0, k = 0, l = 0;
            for(; j < num_active_channels; j++)
            {
                if(j==excluded_channel_offset[k])
                {/* Channel is to be excluded */
                    k++;
                }
                else
                {
                    filter(&filter_result_mut[l].sen_cnt_1, &ctsu_raw_result[j].sen_cnt_1);
                    filter(&filter_result_mut[l].sen_cnt_2, &ctsu_raw_result[j].sen_cnt_2);
                    filter(&filter_result_mut[l].ref_cnt_1, &ctsu_raw_result[j].ref_cnt_1);
                    filter(&filter_result_mut[l].ref_cnt_2, &ctsu_raw_result[j].ref_cnt_2);

                    if(filter_result_mut[l].sen_cnt_2 > filter_result_mut[l].sen_cnt_1)
                    {
                        calib_buffer_mutual[i][j] = (uint16_t)(filter_result_mut[l].sen_cnt_2 - filter_result_mut[l].sen_cnt_1);
                        scount[l] = calib_buffer_mutual[i][j];
                    }
                    else
                    {
                        scount[l] = 0U;
                    }
                    l++;
                }
            }
        }
    }

    for( ch = 0U; ch < (uint16_t)num_active_channels; ch++ )
    {
        work_buff[0] = (uint32_t)(calib_buffer_mutual[ 0 ][ch] + calib_buffer_mutual[ 1 ][ch]);    /* buff0 + buff1 */
        work_buff[1] = (uint32_t)(calib_buffer_mutual[ 0 ][ch] + calib_buffer_mutual[ 2 ][ch]);    /* buff0 + buff2 */
        work_buff[2] = (uint32_t)(calib_buffer_mutual[ 0 ][ch] + calib_buffer_mutual[ 3 ][ch]);    /* buff0 + buff3 */
        work_buff[3] = (uint32_t)(calib_buffer_mutual[ 1 ][ch] + calib_buffer_mutual[ 2 ][ch]);    /* buff1 + buff2 */
        work_buff[4] = (uint32_t)(calib_buffer_mutual[ 1 ][ch] + calib_buffer_mutual[ 3 ][ch]);    /* buff1 + buff3 */
        work_buff[5] = (uint32_t)(calib_buffer_mutual[ 2 ][ch] + calib_buffer_mutual[ 3 ][ch]);    /* buff2 + buff3 */

        loop_max = 0U;
        max_num = 0U;
        for( ; loop_max < 5U ; loop_max++)
        {
            if( work_buff[ max_num ] < work_buff[ loop_max + 1 ] )
            {
                max_num = (uint8_t)(loop_max + 1);
            }
            else
            {
                /* Do Nothing */
            }
        }
        sensor_baseline[ ch ] = (sensor_baseline_t)( work_buff[max_num] / 2 );
    }
    return 0;
}
/***********************************************************************************************************************
 * Function Name: default_sensor_baseline_adjust_mutual
 * Description  : Recommended algorithm for auto-tuning the channels during initialization. Auto tuning is basically
 *                  changing the offset sensor setting such that the difference between the sensor count is slightly
 *                  less than or equal to the reference count. User should not touch electrodes while this executes.
 * Arguments    : channel -
 *                    Channel number to enable (Use 0 only).
 * Return Value : none
 * Notes: REA:ONR: Tested this function 11 times with a CTSU configuration file. CTSUICOMP was not occuring.
 **********************************************************************************************************************/
static int32_t default_sensor_baseline_adjust_mutual(void* arg)
{
    /** Used to prevent compiler warning for unused variable */
    SSP_PARAMETER_NOT_USED(arg);
    uint8_t* touch_binary = g_p_channels_touched_binary;
    uint16_t i = 0, j = 0, k = 0, m = 0, n = 0;
    for(; i < (uint16_t)num_active_channels ; i++)
    {
        /* Figure out which bit is about to be written to. */
        m = (k >> 3);   //byte selection
        n = (k & 7);    //bit selection
        if(i == (uint16_t)excluded_channel_offset[j])
        {/* Channel is to be excluded */
            j++;
        }
        else
        {
            if((touch_binary[m] & (1<<n))==0)
            {/* Channel is not being touched. Perform adjustment to baselines */
#if (DRIFT_COMP_METHOD>=DRIFT_COMP_ALT_1)
                if(dc_counter[k] < dc_limit[k])
#else
                if(dc_counter[k] < DC_TIMING_STEADY_STATE)
#endif
                {/* Increment the counter. */
                    dc_counter[k]++;
                    if(filter_result_mut[k].sen_cnt_2 > filter_result_mut[k].sen_cnt_1)
                    {
                        dc_running_sum[k] += (running_sum_t)(filter_result_mut[k].sen_cnt_2 - filter_result_mut[k].sen_cnt_1);
                    }
                }
                else
                {/* Channel is not touched and it's time to drift compensate. */
                    if(filter_result_mut[k].sen_cnt_2 > filter_result_mut[k].sen_cnt_1)
                    {
                        dc_running_sum[k] += (running_sum_t)(filter_result_mut[k].sen_cnt_2 - filter_result_mut[k].sen_cnt_1);
                    }
                    dc_running_sum[k] /= (running_sum_t)(dc_counter[k]+1);
                    sensor_baseline[i] = (sensor_baseline_t)dc_running_sum[k];
                    dc_running_sum[k] = 0U;
#if (DRIFT_COMP_METHOD==DRIFT_COMP_ALT_1) || (DRIFT_COMP_METHOD==DRIFT_COMP_ALT_3)
                    if(dc_limit[k] > DC_TIMING_STEADY_STATE)
                    {
                        dc_limit[k] = (dc_counter_t)(dc_limit[k] - 1);
                    }
                    else if(dc_limit[k] < DC_TIMING_STEADY_STATE)
                    {
                        dc_limit[k] = (dc_counter_t)(dc_limit[k] + 1);
                    }
#elif (DRIFT_COMP_METHOD==DRIFT_COMP_ALT_2)
                    dc_limit[k] = DC_TIMING_STEADY_STATE;
#endif
                    dc_counter[k] = 0U;
                }
            }
            k++;
        }
    }
    return 0;
}
#endif//CFG_NO_DRIFT_COMPENSATION

#if (CFG_AUTO_TUNE > 0)
/***********************************************************************************************************************
 * Function Name: test_set_so0
 * Description  : Function to test-and-set sensor offset register 0 (CTSUSO0)
 * Arguments    : uint16_t* -
 *                    output points to the location where the calculated output is stored.
 *                uint16_t* -
 *                    input points to the location where the new value is located.
 * Return Value : int8_t -
 *
 *                  (0) - Success.
 *                  (-1) - Failure
 **********************************************************************************************************************/
static int8_t test_set_so0( int32_t i, int16_t set_val )
{
    int16_t wk_offset, wk;
    int8_t rt_dt = -1;

    if( i < 0 )
    {
        return -1;
    }

    wk_offset = wr_set[i].ctsuso0 & 0x03FF;
    wk = (int16_t)(wk_offset + set_val);

    if( set_val <= 0 )
    {
        if( 0 < wk )
        {
            wr_set[i].ctsuso0 = (uint16_t)((int16_t)wr_set[i].ctsuso0 + set_val);
            rt_dt = 0;
        }
    }else{
        if( CTSU_CTSUSO_MAX >= wk )
        {
            wr_set[i].ctsuso0 = (uint16_t)((int16_t)wr_set[i].ctsuso0 + set_val);
            rt_dt = 0;
        }
    }
    return( rt_dt );
}


/***********************************************************************************************************************
 * Function Name: default_onetime_auto_tune_self
 * Description  : Recommended algorithm for auto-tuning the channels during initialization. Auto tuning is basically
 *                  changing the offset sensor setting such that the difference between the sensor count is slightly
 *                  less than or equal to the reference count. User should not touch electrodes while this executes.
 * Arguments    : void* -
 *                  Unused pointer.
 * Return Value :
 *                int8_t - Indicates state of auto tuning when exiting
 *                  -2 - Dynamic memory allocation failed
 *                  -1 - Auto tuning failed due to CTSU operation malfunction
 *                   +ve number - Number of channels auto-tuned successfully.
 *
 * Notes: REA:ONR: Tested this function 11 times with a CTSU configuration file. CTSUICOMP was not occuring.
 **********************************************************************************************************************/
static int8_t at_status_self[CTSU_CFG_MAX_ACTIVE_CHANNEL_COMBOS] = {0};
static uint16_t so0_backup_self[CTSU_CFG_MAX_ACTIVE_CHANNEL_COMBOS] = {0};
static uint16_t so0_backup2_self[CTSU_CFG_MAX_ACTIVE_CHANNEL_COMBOS] = {0};
static uint16_t scount_backup_self[CTSU_CFG_MAX_ACTIVE_CHANNEL_COMBOS] = {0};
static int32_t default_onetime_auto_tune_self(void* arg)
{
    /** Prevent compiler warnings for unused variables */
    SSP_PARAMETER_NOT_USED(arg);
    int ret_val = 0;
    uint8_t num_channels_tuned;
    int16_t diff_scount;
    int16_t mod_diff_scount;
    int16_t diff_offset;
    int16_t mod_diff_offset;
    int16_t d_scount_over_d_offset;     //delta(scount)/delta(offset)
    int16_t delta_srcount;
    int16_t mod_delta_srcount;
    int16_t new_offset;
    uint32_t num_measurements_made = 0;
    ctsu_channel_data_self_t* ctsu_raw_result = p_ctsu_output;

    /* Zero out all arrays */
    memset(at_status_self, 0, (uint16_t)num_active_channels*sizeof(int8_t));
    memset(so0_backup_self, 0, (uint16_t)num_active_channels*sizeof(uint16_t));
    memset(so0_backup2_self, 0, (uint16_t)num_active_channels*sizeof(uint16_t));
    memset(scount_backup_self, 0, (uint16_t)num_active_channels*sizeof(uint16_t));

    /* First scan */
    HW_CTSU_CTSUCR0SetBitCTSUINIT(1U);
    HW_CTSU_CTSUCR0SetBitCTSUSTRT(1U);
    /* Wait for scan to complete. */
    do{
        if((HW_CTSU_CTSUSTGetBitCTSUROVF()) || (HW_CTSU_CTSUERRSGetBitCTSUICOMP()))
        {
            /* TSCAP error. Check your settings for the configuration. */
            return CTSU_AUTO_TUNING_FAILED;
        }
    }while((HW_CTSU_CTSUSTGetBitCTSUSTC()!=0) && (HW_CTSU_CTSUCR0GetBitCTSUSTRT() == 1));

    /* Auto-tuning needs to finish within a fixed number of iterations. */
    while(num_measurements_made < AT_MAX_DURATION)
    {
        /* Check if any errors occurred */
        if((HW_CTSU_CTSUSTGetBitCTSUROVF()) || (HW_CTSU_CTSUERRSGetBitCTSUICOMP()))
        {
            ret_val = CTSU_AUTO_TUNING_FAILED;
            break;
        }

        num_channels_tuned = 0U;

        switch(HW_CTSU_CTSUSTGetBitCTSUSTC())
        {
        case 0U:
        {
            /** Filter the data */
            /** Declare and initialize local loop variables */
            int i = 0, j = 0, k = 0;
            for(; i < num_active_channels ; i++)
            {
                if(i==excluded_channel_offset[j])
                {/* Channel is to be excluded */
                    j++;
                }
                else
                {/* Channel is not excluded */
                    switch(at_status_self[k])
                    {
                    case CTSU_AUTO_TUNING_NOT_STARTED:
                        /* Filter is uninitialized */
                        scount[k] = ctsu_raw_result[i].sensor_count;
                        rcount[k] = ctsu_raw_result[i].reference_count;
                        at_status_self[k] = CTSU_AUTO_TUNING_INITIALIZED;
                        break;

                    case CTSU_AUTO_TUNING_INITIALIZED:
                        /* Filter ready to run */
                        filter(&scount[k], &ctsu_raw_result[i].sensor_count);
                        filter(&rcount[k], &ctsu_raw_result[i].reference_count);
                        /***************/
                        /* Auto-tuning */
                        /***************/
                        if( scount[k] >= 0xFFF0)
                        {
                            if( HW_CTSU_CTSUSTGetBitCTSUSOVF() )
                            {
                                HW_CTSU_CTSUSTSetBitCTSUSOVF(0);
                            }
                            if(test_set_so0(i,+0x20)==-1)
                            {/*Auto-tuning error. Can't auto-tune channel */
                                at_status_self[k] = CTSU_AUTO_TUNING_FAILED;
                            }
                        }
                        else if(scount[k] < rcount[k])
                        {
                            if((wr_set[i].ctsuso0 & 0x3FF) == 0)
                            {
                                at_status_self[k] = CTSU_AUTO_TUNING_COMPLETE;
                            }
                            else
                            {
                                scount_backup_self[k] = scount[k];
                                so0_backup_self[k] = (wr_set[i].ctsuso0 & 0x3FF);
                                test_set_so0(i, -0x02);
                                so0_backup2_self[k] = 0x03FF;
                                at_status_self[k] = CTSU_AUTO_TUNING_RUNNING;
                            }
                        }
                        else if(scount[k] > rcount[k])
                        {
                            scount_backup_self[k] = scount[k];
                            so0_backup_self[k] = (wr_set[i].ctsuso0 & 0x3FF);
                            test_set_so0(i, +0x02);
                            so0_backup2_self[k] = 0x03FF;
                            at_status_self[k] = CTSU_AUTO_TUNING_RUNNING;
                        }
                        break;

                    case CTSU_AUTO_TUNING_RUNNING:
                        /**************/
                        /* Run filter */
                        /**************/
                        filter(&scount[k], &ctsu_raw_result[i].sensor_count);
                        filter(&rcount[k], &ctsu_raw_result[i].reference_count);
                        if(stabilization_time<=0)
                        {/* We've bypassed enough scans to let the changed setting affect the output. */
                            if(k >= (num_active_channels-num_excluded_channels-1))
                            {
                                stabilization_time = STABILIZATION_TIME;
                            }
                            /***************/
                            /* Auto-tuning */
                            /***************/
                            diff_scount = (int16_t)(scount[k] - scount_backup_self[k]);
                            mod_diff_scount = ((diff_scount > 0) ? diff_scount:-diff_scount)& INT16_MAX;

                            diff_offset = (int16_t)((wr_set[i].ctsuso0 & 0x3FF) - so0_backup_self[k]);
                            mod_diff_offset = ((diff_offset > 0) ? diff_offset:-diff_offset) & INT16_MAX;
                            /* Prevent divide by zero */
                            if(mod_diff_offset == 0)
                            {
                                mod_diff_offset = 1;
                            }
                            /* delta(scount)/delta(offset) */
                            d_scount_over_d_offset = (int16_t)(mod_diff_scount/mod_diff_offset);
                            /* Prevent divide by zero */
                            if(d_scount_over_d_offset == 0)
                            {
                                d_scount_over_d_offset = 1;
                            }
                            /* Save back up */
                            scount_backup_self[k] = scount[k];
                            so0_backup_self[k] = (wr_set[i].ctsuso0 & 0x3FF);
                            /* Find difference between current sensor count and current reference count */
                            delta_srcount = (int16_t)(scount[k] - rcount[k]);
                            mod_delta_srcount = ((delta_srcount > 0) ? delta_srcount : -delta_srcount) & INT16_MAX;
                            /**********************/
                            /* Meat of the matter */
                            /**********************/
                            new_offset = (int16_t)(mod_delta_srcount/d_scount_over_d_offset);
                            if(delta_srcount > 0)
                            {/* sensor > reference */
                                if(new_offset > so0_backup2_self[k])
                                {
                                    new_offset = (int16_t)(so0_backup2_self[k]);
                                }
                                so0_backup2_self[k] = (uint16_t)new_offset;

                                if(new_offset == 0)
                                {/* Tuned */
                                    at_status_self[k] = CTSU_AUTO_TUNING_COMPLETE;
                                }
                                else if( new_offset <= 3 )
                                {/* previous change caused a small change in delta_srcount */
                                    test_set_so0(i, +new_offset);
                                }
                                else
                                {/* previous change caused a large change in delta_srcount */
                                    test_set_so0(i, (+new_offset) >> 2);
                                }

                            }
                            else
                            {/* reference > sensor */
                                if(new_offset > so0_backup2_self[k])
                                {
                                    new_offset = (int16_t)so0_backup2_self[k];
                                }
                                so0_backup2_self[k] = (uint16_t)new_offset;

                                if(new_offset == 0)
                                {/* Tuned */
                                    at_status_self[k] = CTSU_AUTO_TUNING_COMPLETE;
                                }
                                else if( new_offset <= 3 )
                                {/* previous change caused a small change in delta_srcount */
                                    test_set_so0(i, (int16_t)-new_offset);
                                }
                                else
                                {/* previous change caused a large change in delta_srcount */
                                    test_set_so0(i, (int16_t)(-new_offset) >> 2);
                                }
                            }
                            /**********************/
                            /*  EOM(End of Meat)  */
                            /**********************/
                        }
                        break;

                    case CTSU_AUTO_TUNING_COMPLETE:
                        /**************/
                        /* Run filter */
                        /**************/
                        filter(&scount[k], &ctsu_raw_result[i].sensor_count);
                        filter(&rcount[k], &ctsu_raw_result[i].reference_count);
                        /* Check if the value in a remaining stable range. */
                        if(scount[k] > rcount[k])
                        {
                            if( (scount[k] - rcount[k]) > (threshold[i] >> AT_THRESHOLD_PERCENT) )
                            {/* Channels needs adjustment */
                                test_set_so0( i, +1 );
                            }
#if defined(STRICT_AUTO_TUNE)
                            else
                            {/* Channel stays tuned */
                                num_channels_tuned++;
                            }
#endif
                        }
                        else
                        {
                            if( (rcount[k] - scount[k]) > (threshold[i] >> AT_THRESHOLD_PERCENT) )
                            {/* Channels needs adjustment */
                                test_set_so0( i, -1 );
                            }
#if defined(STRICT_AUTO_TUNE)
                            else
                            {/* Channel stays tuned */
                                num_channels_tuned++;
                            }
#endif
                        }
#if !defined(STRICT_AUTO_TUNE)
                        /* Channel is assumed to be tuned. Without strict auto-tuning, we can exit the loop faster. */
                        num_channels_tuned++;
#endif
                        if(num_channels_tuned >= (num_active_channels-num_excluded_channels))
                        {/* All channels are tuned. */
                            ret_val = num_channels_tuned;
                            /* Let's end this. Break early from while loop */
                            break;
                        }
                        break;

                    default:
return CTSU_AUTO_TUNING_FAILED;
break;
                    }
                    ++k;
                }
            }
        }
        if(num_channels_tuned < (num_active_channels-num_excluded_channels))
        {/* Start a new measurement */
            HW_CTSU_CTSUCR0SetBitCTSUINIT(1U);
            HW_CTSU_CTSUCR0SetBitCTSUSTRT(1U);
            /* Decrement stabilization time. */
            if(stabilization_time > 0)
            {
                stabilization_time--;
            }
        }
        /* Increment the number of measurements made. Keep track. */
        num_measurements_made++;
        break;

        default:

            break;
        }
        if(num_channels_tuned >= (num_active_channels-num_excluded_channels))
        {/* All channels are tuned. */
            ret_val = num_channels_tuned;
            /* Let's end this. Break early from while loop */
            break;
        }
    }
    if(ret_val >= 0)
    {/* Take a final count to return */
        num_channels_tuned = 0U;
        for(int i = 0; i < (num_active_channels-num_excluded_channels); i++)
        {
            if(at_status_self[i] == 3)
            {
                num_channels_tuned++;
            }
        }
        ret_val = num_channels_tuned;
    }
    return ret_val;
}

/***********************************************************************************************************************
 * Function Name: default_runtime_auto_tune_self
 * Description  : Recommended algorithm for auto-tuning the channels. Auto tuning is basically changing the offset
 *                  sensor setting such that the difference between the sensor count and reference count is as low as
 *                  possible. Used in self mode.
 * Arguments    : void* -
 *                  Unused pointer.
 * Return Value :
 *                int8_t
 *                  0 - Success
 *                  -1 - Failed to perform auto_tuning
 **********************************************************************************************************************/
static int32_t default_runtime_auto_tune_self(void* arg)
{
    /** Used to prevent compiler warning for unused variable */
    SSP_PARAMETER_NOT_USED(arg);
    int8_t ret_val = 0;
    uint8_t* touch_binary = g_p_channels_touched_binary;

    if(touch_binary==NULL)
    {/* Touched binary location is not assigned. Failure. */
        return ret_val;
    }
#if 0
    /* Check if any channel is touched */
    for(i = 0, m = 0, n = 0 ; i < (num_active_channels-num_excluded_channels) ; i++)
    {
            m = (i >> 3);   //byte selection
            n = (i & 7);    //bit selection
            if((touch_binary[m] & (1<<n))==(1<<n))
            {/* A channel is touched. Exit. */
                return ret_val;
            }
    }
#endif
    /** Auto-calibration is performed only when channels are not touched. */
    /** Declare and initialize loop variables */
    int i = 0, j = 0, k = 0, m = 0, n = 0;
    for(; i < num_active_channels ; i++)
    {
        /* Channel is determined as not touched. Perform auto-tuning on it. */
        if(i==excluded_channel_offset[j])
        {/* Channel is to be excluded */
            j++;
        }
        else
        {
            if((touch_binary[m] & (1<<n))==0)
            {/* Channel is not touched */
                if(at_counter[i]<AT_TIMING)
                {/* Increment the counter. */
                    at_counter[i]++;
                    at_running_sum[i] += scount[k];
                }
                else
                {/* Channel is not touched and it's time to drift compensate. */
                    at_running_sum[i] += scount[k];
                    at_running_sum[i] /= (running_sum_t)(at_counter[i]+1);

                    /* Perform sensor offset tuning if difference between sensor and reference count is outside dead-band.
                     *  i.e. +-0.125* threshold value */
                    if(at_running_sum[i] > rcount[k])
                    {/* Crude check to see if sensor is being touched */
                        if( (at_running_sum[i] - rcount[k]) > (threshold[i] >> AT_THRESHOLD_PERCENT))
                        {/* Channels needs adjustment */
                            ret_val = test_set_so0( i, +1 );
                        }
                    }
                    else
                    {
                        if( (rcount[k] - at_running_sum[i]) > (threshold[i] >> AT_THRESHOLD_PERCENT) )
                        {/* Channels needs adjustment */
                            ret_val = test_set_so0( i, -1 );
                        }
                    }

                    at_counter[i] = 0U;
                }
            }

            k++;
        }

    }
    return ret_val;
}

/***********************************************************************************************************************
 * Function Name: default_onetime_auto_tune_mutual
 * Description  : Recommended algorithm for auto-tuning the channels during initialization. Auto tuning is basically
 *                  changing the offset sensor setting such that the difference between the sensor count is slightly
 *                  less than or equal to the reference count. User should not touch electrodes while this executes.
 * Arguments    : channel -
 *                    Channel number to enable (Use 0 only).
 * Return Value : none
 * Notes: REA:ONR: Tested this function 11 times with a CTSU configuration file. CTSUICOMP was not occuring.
 **********************************************************************************************************************/
static int8_t at_status_mut[CTSU_CFG_MAX_ACTIVE_CHANNEL_COMBOS] = {0};
static uint16_t so0_backup_mut[CTSU_CFG_MAX_ACTIVE_CHANNEL_COMBOS] = {0};
static uint16_t so0_backup2_mut[CTSU_CFG_MAX_ACTIVE_CHANNEL_COMBOS] = {0};
static uint16_t scount_backup_mut[CTSU_CFG_MAX_ACTIVE_CHANNEL_COMBOS] = {0};
static int32_t default_onetime_auto_tune_mutual(void* arg)
{
    /** Used to prevent compiler warning for unused variable */
    SSP_PARAMETER_NOT_USED(arg);
    int ret_val = 0;
    uint32_t num_measurements_made = 0;
    uint8_t num_channels_tuned;
    uint16_t wk_scount, wk_offset, wk_count, wk_diff_count;
    ctsu_channel_data_mutual_t* ctsu_raw_result = p_ctsu_output;

    /* Zero out all arrays */
    memset(at_status_mut, 0, (uint16_t)num_active_channels*sizeof(int8_t));
    memset(so0_backup_mut, 0, (uint16_t)num_active_channels*sizeof(uint16_t));
    memset(so0_backup2_mut, 0, (uint16_t)num_active_channels*sizeof(uint16_t));
    memset(scount_backup_mut, 0, (uint16_t)num_active_channels*sizeof(uint16_t));

    /* First scan */
    HW_CTSU_CTSUCR0SetBitCTSUINIT(1U);
    HW_CTSU_CTSUCR0SetBitCTSUSTRT(1U);
    /* Wait for scan to complete. */
    do{
        if((HW_CTSU_CTSUSTGetBitCTSUROVF()) || (HW_CTSU_CTSUERRSGetBitCTSUICOMP()))
        {
            /* TSCAP error. Check your settings for the configuration. */
            return CTSU_AUTO_TUNING_FAILED;
        }
    }while(((HW_CTSU_CTSUSTGetBitCTSUSTC()) != 0) && ((HW_CTSU_CTSUCR0GetBitCTSUSTRT()) == 1));
    /* Auto-tuning needs to finish within a fixed number of iterations */
    while(num_measurements_made < AT_MAX_DURATION)
    {
        /* Check if any errors occurred */
        if((HW_CTSU_CTSUSTGetBitCTSUROVF()) || (HW_CTSU_CTSUERRSGetBitCTSUICOMP()))
        {
            ret_val = CTSU_AUTO_TUNING_FAILED;
            break;
        }

        num_channels_tuned = 0U;

        switch(HW_CTSU_CTSUSTGetBitCTSUSTC())
        {
        case 0U:
            /* Filter the data */
        {
            /** Initialize variables for loop below */
            int i = 0, j = 0, k = 0;
            for(; i < num_active_channels ; i++)
            {
                if(i==excluded_channel_offset[j])
                {/* Channel is to be excluded */
                    j++;
                }
                else
                {/* Channel is not excluded */
                    switch(at_status_mut[k])
                    {
                    case CTSU_AUTO_TUNING_NOT_STARTED:
                        /* Filter is uninitialized */
                        filter_result_mut[k].sen_cnt_1 = ctsu_raw_result[i].sen_cnt_1;
                        filter_result_mut[k].ref_cnt_1 = ctsu_raw_result[i].ref_cnt_1;
                        filter_result_mut[k].sen_cnt_2 = ctsu_raw_result[i].sen_cnt_2;
                        filter_result_mut[k].ref_cnt_2 = ctsu_raw_result[i].ref_cnt_2;
                        at_status_mut[k] = 1;
                        break;

                    case CTSU_AUTO_TUNING_INITIALIZED:
                        /* Filter is initialized. */
                        filter(&filter_result_mut[k].sen_cnt_1, &ctsu_raw_result[i].sen_cnt_1);
                        filter(&filter_result_mut[k].sen_cnt_2, &ctsu_raw_result[i].sen_cnt_2);
                        filter(&filter_result_mut[k].ref_cnt_1, &ctsu_raw_result[i].ref_cnt_1);
                        filter(&filter_result_mut[k].ref_cnt_2, &ctsu_raw_result[i].ref_cnt_2);
                        /***************/
                        /* Auto-tuning */
                        /***************/
                        if( filter_result_mut[k].sen_cnt_1 >= 0xFFF0 )                        //        << overflow error mode
                        {
                            if( test_set_so0( i, +0x020 ) == -1 )
                            {
                                at_status_mut[ k ]  = CTSU_AUTO_TUNING_FAILED;
                            }
                        }
                        else if( filter_result_mut[k].sen_cnt_1 < filter_result_mut[k].ref_cnt_1 )            //        << under Reference counter mode
                        {
                            if( (wr_set[i].ctsuso0 & 0x03FF) == 0 )
                            {
                                at_status_mut[ k ]  = CTSU_AUTO_TUNING_COMPLETE;
                            }
                            else
                            {
                                scount_backup_mut[ k ] = filter_result_mut[k].sen_cnt_1;
                                so0_backup_mut[ k ]  = wr_set[i].ctsuso0 & 0x03FF;
                                test_set_so0( i, -0x002 );
                                so0_backup2_mut[ k ] = 0x03FF;            // set dummy data
                                at_status_mut[ k ] = CTSU_AUTO_TUNING_RUNNING;
                            }
                        }
                        else                                                    //        << 0xFFFE - Reference counter mode
                        {
                            scount_backup_mut[ k ] = filter_result_mut[k].sen_cnt_1;
                            so0_backup_mut[ k ]  = wr_set[i].ctsuso0 & 0x03FF;
                            test_set_so0( i, +0x002 );
                            so0_backup2_mut[ k ] = 0x03FF;                // set dummy data
                            at_status_mut[ k ] = CTSU_AUTO_TUNING_RUNNING;
                        }
                        break;

                    case CTSU_AUTO_TUNING_RUNNING:
                        /* Filter the data */
                        filter(&filter_result_mut[k].sen_cnt_1, &ctsu_raw_result[i].sen_cnt_1);
                        filter(&filter_result_mut[k].sen_cnt_2, &ctsu_raw_result[i].sen_cnt_2);
                        filter(&filter_result_mut[k].ref_cnt_1, &ctsu_raw_result[i].ref_cnt_1);
                        filter(&filter_result_mut[k].ref_cnt_2, &ctsu_raw_result[i].ref_cnt_2);
                        if(stabilization_time <= 0)
                        {
                            if(k>=(num_active_channels-num_excluded_channels-1))
                            {
                                stabilization_time = STABILIZATION_TIME;
                            }
                            /***************/
                            /* Auto-tuning */
                            /***************/
                            if( scount_backup_mut[ k ] > filter_result_mut[k].sen_cnt_1 )
                            {
                                wk_scount = (uint16_t)(scount_backup_mut[ k ] - filter_result_mut[k].sen_cnt_1);
                            }else{
                                wk_scount = (uint16_t)(filter_result_mut[k].sen_cnt_1 - scount_backup_mut[ k ]);
                            }
                            wk_offset = (wr_set[i].ctsuso0 & 0x03FF);
                            if( wk_offset > so0_backup_mut[ k ] )
                            {
                                wk_offset = (uint16_t)(wk_offset - so0_backup_mut[ k ]);
                            }else{
                                wk_offset = (uint16_t)(so0_backup_mut[ k ] - wk_offset);
                            }
                            wk_count = wk_scount / wk_offset;

                            if( wk_count == 0U )
                            {
                                wk_count = 1;
                            }

                            scount_backup_mut[ k ] = filter_result_mut[k].sen_cnt_1;
                            so0_backup_mut[ k ] = (wr_set[i].ctsuso0 & 0x03FF);

                            if( filter_result_mut[k].sen_cnt_1 > filter_result_mut[k].ref_cnt_1 )
                            {
                                wk_diff_count = (uint16_t)(filter_result_mut[k].sen_cnt_1 - filter_result_mut[k].ref_cnt_1);
                                wk_offset = wk_diff_count / wk_count;

                                if( wk_offset > so0_backup2_mut[ k ] )                      // protect unstable data
                                {
                                    wk_offset = so0_backup2_mut[ k ];
                                }
                                so0_backup2_mut[ k ] = wk_offset;

                                if( wk_offset == 0U )
                                {
                                    at_status_mut[ k ] = CTSU_AUTO_TUNING_COMPLETE;
                                }else
                                    if( wk_offset < 3 )
                                    {
                                        test_set_so0( i, (int16_t)+wk_offset );
                                    }else{
                                        test_set_so0( i, (int16_t)+(wk_offset/3) );
                                    }
                            }else{
                                wk_diff_count = (uint16_t)(filter_result_mut[k].ref_cnt_1 - filter_result_mut[k].sen_cnt_1);
                                wk_offset = wk_diff_count / wk_count;

                                if( wk_offset > so0_backup2_mut[ k ] )                      // protect unstable data
                                {
                                    wk_offset = so0_backup2_mut[ k ];
                                }
                                so0_backup2_mut[ k ] = wk_offset;

                                if( wk_offset == 0U )
                                {
                                    at_status_mut[ k ] = CTSU_AUTO_TUNING_COMPLETE;
                                }else
                                    if( wk_offset < 3 )
                                    {
                                        test_set_so0( i, (int16_t)-wk_offset );
                                    }else{
                                        test_set_so0( i, (int16_t)-(wk_offset/3) );
                                    }
                            }
                        }
                        break;

                    case CTSU_AUTO_TUNING_COMPLETE:
                        /* Filter the data */
                        filter(&filter_result_mut[k].sen_cnt_1, &ctsu_raw_result[i].sen_cnt_1);
                        filter(&filter_result_mut[k].sen_cnt_2, &ctsu_raw_result[i].sen_cnt_2);
                        filter(&filter_result_mut[k].ref_cnt_1, &ctsu_raw_result[i].ref_cnt_1);
                        filter(&filter_result_mut[k].ref_cnt_2, &ctsu_raw_result[i].ref_cnt_2);


                        if( filter_result_mut[k].sen_cnt_1 > filter_result_mut[k].ref_cnt_1 )
                        {
                            if( (filter_result_mut[k].sen_cnt_1 - filter_result_mut[k].ref_cnt_1) > (threshold[i] >> AT_THRESHOLD_PERCENT) )
                            {
                                test_set_so0( i, +1 );
                            }
#if defined(STRICT_AUTO_TUNE)
                            else
                            {
                                num_channels_tuned++;
                            }
#endif
                        }else{
                            if( (filter_result_mut[k].ref_cnt_1 - filter_result_mut[k].sen_cnt_1) > (threshold[i] >> AT_THRESHOLD_PERCENT) )
                            {
                                test_set_so0( i, -1 );
                            }
#if defined(STRICT_AUTO_TUNE)
                            else
                            {
                                num_channels_tuned++;
                            }
#endif
                        }

                        if(filter_result_mut[k].sen_cnt_2 > filter_result_mut[k].sen_cnt_1)
                        {
                            scount[k] = (uint16_t)(filter_result_mut[k].sen_cnt_2 - filter_result_mut[k].sen_cnt_1);
                        }
                        else
                        {
                            scount[k] = 0U;
                        }
#if !defined(STRICT_AUTO_TUNE)
                        num_channels_tuned++;
#endif
                        break;

                    default:
                        return CTSU_AUTO_TUNING_FAILED;
                        break;
                    }

                    if(num_channels_tuned >= (num_active_channels-num_excluded_channels))
                    {/* All channels are tuned. */
                        ret_val = num_channels_tuned;
                        /* Let's end this. Break early from while loop */
                        break;
                    }
                    k++;
                }

            }
        }
        if(num_channels_tuned < (num_active_channels-num_excluded_channels))
        {/* Start a new measurement */
            HW_CTSU_CTSUCR0SetBitCTSUINIT(1U);
            HW_CTSU_CTSUCR0SetBitCTSUSTRT(1U);
            if(stabilization_time > 0)
            {
                stabilization_time--;
            }
        }
        /* Increment the number of measurements made. Keep track. */
        num_measurements_made++;
        break;

        default:

            break;
        }
        if(num_channels_tuned >= (num_active_channels-num_excluded_channels) )
        {/* End the tuning early. */
            break;
        }
    }
    if(ret_val >= 0)
    {/* Take a final count to return */
        num_channels_tuned = 0U;
        for(int i = 0; i < num_active_channels; i++)
        {
            if(at_status_mut[i] == 3)
            {
                num_channels_tuned++;
            }
        }
        ret_val = num_channels_tuned;
    }

    return ret_val;
}

/***********************************************************************************************************************
 * Function Name: default_runtime_auto_tune_mutual
 * Description  : Recommended algorithm for auto-tuning the channels during initialization. Auto tuning is basically
 *                  changing the offset sensor setting such that the difference between the sensor count is slightly
 *                  less than or equal to the reference count. User should not touch electrodes while this executes.
 * Arguments    : channel -
 *                    Channel number to enable (Use 0 only).
 * Return Value : none
 * Notes: REA:ONR: Tested this function 11 times with a CTSU configuration file. CTSUICOMP was not occuring.
 **********************************************************************************************************************/
static int32_t default_runtime_auto_tune_mutual(void* arg)
{
    /** Used to prevent compiler warning for unused variable */
    SSP_PARAMETER_NOT_USED(arg);
    int8_t ret_val = 0;
    uint8_t* touch_binary = g_p_channels_touched_binary;
#if 0
    /* Check if any channel is touched */
    for(int i = 0, m = 0, n = 0 ; i < (num_active_channels-num_excluded_channels) ; i++)
    {/* Figure out which bit is about to be written to. */
        m = (i >> 3);   //byte selection
        n = (i & 7);    //bit selection
        if((touch_binary[m] & (1<<n))==(1<<n))
        {/* A channel is touched. */
            return ret_val;
        }
    }
#endif
    /** Auto-calibration is performed only when channels are not touched. */
    /** Declare and initialize loop variables */
    int i = 0, j = 0, k = 0, m = 0, n = 0;
    for(; i < num_active_channels ; i++)
    {
        m = (i >> 3);   //byte selection
        n = (i & 7);    //bit selection
        if(i==excluded_channel_offset[j])
        {/* Channel is to be excluded */
            j++;
        }
        else
        {
            if((touch_binary[m] & (1<<n))==0)
            {/* Channel is not touched */
                if(at_counter[i]<AT_TIMING)
                {/* Increment the counter. */
                    at_counter[i]++;
                    at_running_sum[i] += scount[k];
                }
                else
                {/* Channel is not touched and it's time to drift compensate. */
                    at_running_sum[i] += scount[k];
                    at_running_sum[i] /= (running_sum_t)(at_counter[i]+1);
#if 0
                    /* Perform sensor offset tuning if difference between sensor and reference count is outside dead-band.
                     *  i.e. +-0.125* threshold value */
                    if(at_running_sum[i] > (filter_result_mut[k].ref_cnt_1 + filter_result_mut[k].ref_cnt_2))
                    {/* Crude check to see if sensor is being touched */
                        if( (at_running_sum[i] - rcount[k]) > (threshold[i] >> AT_THRESHOLD_PERCENT))
                        {/* Channels needs adjustment */
                            ret_val = test_set_so0( i, +1 );
                        }
                    }
                    else
                    {
                        if( ((filter_result_mut[k].ref_cnt_1 + filter_result_mut[k].ref_cnt_2) - at_running_sum[i]) > (threshold[i] >> AT_THRESHOLD_PERCENT) )
                        {/* Channels needs adjustment */
                            ret_val = test_set_so0( i, -1 );
                        }
                    }
#else
            if( filter_result_mut[k].sen_cnt_1 > filter_result_mut[k].ref_cnt_1 )
            {
                if( (filter_result_mut[k].sen_cnt_1 - filter_result_mut[k].ref_cnt_1) > (threshold[i] >> AT_THRESHOLD_PERCENT) )
                {
                    test_set_so0( i, +1 );
                }
            }else{
                if( (filter_result_mut[k].ref_cnt_1 - filter_result_mut[k].sen_cnt_1) > (threshold[i] >> AT_THRESHOLD_PERCENT) )
                {
                    test_set_so0( i, -1 );
                }
            }
#endif
                    at_counter[i] = 0U;
                }
            }

            k++;
        }
    }

    return ret_val;
}
#endif//(CFG_AUTO_TUNE > 0)

/**
 * ********************************************************************************************************************
 * @brief Single input, single output filter.
 * @param output points to the location where the calculated output is stored.
 * @param input points to the location where the new value is located.
 * @return Status of the filter. (Future use only)
 **********************************************************************************************************************/
static int32_t siso_filter(volatile uint16_t* output, volatile uint16_t* input)
{
    int32_t ret_val = 0;

    uint16_t temp = *output;

    temp = (uint16_t)(temp -(temp >> CTSU_CFG_FILTER_DEPTH));
    temp = (uint16_t)(temp + ((*input) >> CTSU_CFG_FILTER_DEPTH));

    *output = temp;

    /* return status of filter. */
    return ret_val;
}

/**********************************************************************************************************************/
/*                                              Interrupt service routines                                            */
/**********************************************************************************************************************/
/** Keeps count of the number of calls made to WR and RD. In self mode, iterator gets
 * incremented by 2. In mutual mode, iterator gets incremented by 4. */
volatile uint8_t itr = 0;


#if (USING_DTC_FOR_CTSU > 0)
const ctsu_channel_setting_t * wr_set_address_temp = (ctsu_channel_setting_t *)&wr_set[0];
/*******************************************************************************************************************//**
 * @brief   initialize_dtc_ctsu_elements : Initialize the DTC for CTSU usage
 *
 * This function configures the Transfer API to write the configuration data to the CTSU before the scan and
 * read the result data after the scan.
 *
 * @param[in]  wr_set_addr         :  Channel configuration data
 * @param[in]  p_ctsu_output_addr  :  Address to move the read data to
 * @param[in]  wr_cnt              :  Write Count
 * @param[in]  rd_cnt              :  Read Count
 * @param[in]  p_cfg               :  Configuration pointer
 * @param[in]  p_ctrl              :  Control structure pointer
 *
 * @retval  SSP_SUCCESS -       Successful
 * @retval  SSP_ERR_INVALID_ARGUMENT -  Mode or element of p_cfg structure has invalid value or is invalid based on mode
 *
 * @note Refer to the Transfer API for other return codes
***********************************************************************************************************************/
static ssp_err_t initialize_dtc_ctsu_elements(uint32_t wr_set_addr, uint32_t p_ctsu_output_addr,
        int16_t wr_cnt, int16_t rd_cnt, ctsu_cfg_t const * const p_cfg, ctsu_instance_ctrl_t * const p_ctrl)
{
    ssp_err_t err = SSP_SUCCESS;


    /** Configure the control pointers */
    p_ctrl->p_lowerl_lvl_transfer_read_ctrl = p_cfg->p_lower_lvl_transfer_read->p_ctrl;
    p_ctrl->p_lowerl_lvl_transfer_write_ctrl = p_cfg->p_lower_lvl_transfer_write->p_ctrl;

    /** Make a local copy of the transfer configuration structure to modify it */
    transfer_cfg_t temp_wr_cfg =
    {
        .p_info = NULL,
        .activation_source = ELC_EVENT_ELC_SOFTWARE_EVENT_0,
        .auto_enable = false,
        .p_callback = NULL,
        .p_context = NULL,
        .p_extend = NULL,
    };

    memcpy(&temp_wr_cfg, p_cfg->p_lower_lvl_transfer_write->p_cfg, sizeof(temp_wr_cfg));
    /** Modify the Write transfer descriptors; The rest of the elements of the descriptor
     * do not need to be re-initialized since that is already done in the structure that is
     * passed in.*/
    temp_wr_cfg.p_info[0].p_src      = (void const * )wr_set_addr;
    temp_wr_cfg.p_info[0].num_blocks = (uint16_t)wr_cnt;
    temp_wr_cfg.p_info[0].p_dest = (void * )&R_CTSU->CTSUSSC;
    temp_wr_cfg.p_info[0].length = 3;
    temp_wr_cfg.p_info[1].length = 1;
    temp_wr_cfg.p_info[1].p_src = (void const * )&wr_set_address_temp;
    temp_wr_cfg.p_info[2].p_src = (void const * )&num_wr_irq_requests;
    temp_wr_cfg.p_info[2].length = 1;

    /** Modify the Read transfer descriptors */
    transfer_cfg_t temp_rd_cfg =
    {
        .p_info = NULL,
        .activation_source = ELC_EVENT_ELC_SOFTWARE_EVENT_0,
        .auto_enable = false,
        .p_callback = NULL,
        .p_context = NULL,
        .p_extend = NULL,
    };

    memcpy(&temp_rd_cfg, p_cfg->p_lower_lvl_transfer_read->p_cfg, sizeof(temp_rd_cfg));
    temp_rd_cfg.p_info[0].p_dest     = (void * )p_ctsu_output_addr;
    temp_rd_cfg.p_info[0].num_blocks = (uint16_t)rd_cnt;
    temp_rd_cfg.p_info[0].p_src = (void const * )&(R_CTSU->CTSUSC);
    temp_rd_cfg.p_info[0].length = 2;
    temp_rd_cfg.p_info[1].length = 1;
    temp_rd_cfg.p_info[1].p_src = (void const * )&p_ctsu_output;
    temp_rd_cfg.p_info[2].p_src = (void const * )&num_rd_irq_requests;
    temp_rd_cfg.p_info[2].length = 1;

    /** Initialize the Transfer driver */
    err  = p_cfg->p_lower_lvl_transfer_write->p_api->open(p_ctrl->p_lowerl_lvl_transfer_write_ctrl, &temp_wr_cfg);

    if (SSP_SUCCESS == err)
    {
        err = p_cfg->p_lower_lvl_transfer_read->p_api->open(p_ctrl->p_lowerl_lvl_transfer_read_ctrl, &temp_rd_cfg);

    }
    return err;
}

/*******************************************************************************************************************//**
 * @brief CTSU Write ISR routine
 *
 * This function implements the CTSU Write interrupt handler
 *
***********************************************************************************************************************/
void ctsu_write_isr(void);
void ctsu_write_isr(void)
{
    /* Save context if RTOS is used */
    SF_CONTEXT_SAVE
    /* Synergy MCU's need this for now. */
    R_BSP_IrqStatusClear(R_SSP_CurrentIrqGet());

    /* Restore context if RTOS is used */
    SF_CONTEXT_RESTORE

}
/*******************************************************************************************************************//**
 * @brief CTSU Read ISR routine
 *
 * This function implements the CTSU Read interrupt handler
 *
***********************************************************************************************************************/
void ctsu_read_isr(void);
void ctsu_read_isr(void)
{
    /* Save context if RTOS is used */
    SF_CONTEXT_SAVE
    /* Synergy MCU's need this for now. */
    R_BSP_IrqStatusClear(R_SSP_CurrentIrqGet());

    /* Restore context if RTOS is used */
    SF_CONTEXT_RESTORE
}


#else
/* ISRs need to be defined when DTC is not used */
static volatile uint8_t g_offset;

/*******************************************************************************************************************//**
 * @brief CTSU Write ISR routine
 *
 * This function implements the CTSU Write interrupt handler
 *
***********************************************************************************************************************/
void ctsu_write_isr(void);
void ctsu_write_isr(void)
{
    /* Save context if RTOS is used */
    SF_CONTEXT_SAVE
    /* Synergy MCU's need this for now. */
    R_BSP_IrqStatusClear(R_SSP_CurrentIrqGet());

    /** Get the shift count for the iterator which depends upon the mode. */
    uint8_t shift_count = (mode==CTSU_MODE_MUTUAL_CAPACITANCE) ? 2:1;

    /** itr increments by two in RD interrupt. So divide it by two/four here.*/
    /** Fill the registers with the corresponding values from the wr_set table. */
    g_offset = (uint8_t)(itr>>shift_count);

    HW_CTSU_CTSUSSCSet(wr_set[g_offset].ctsussc);
    HW_CTSU_CTSUSO0Set(wr_set[g_offset].ctsuso0);
    HW_CTSU_CTSUSO1Set(wr_set[g_offset].ctsuso1);

    /* Restore context if RTOS is used */
    SF_CONTEXT_RESTORE
}

/*******************************************************************************************************************//**
 * @brief CTSU Read ISR routine
 *
 * This function implements the CTSU Read interrupt handler
 *
***********************************************************************************************************************/
void ctsu_read_isr(void);
void ctsu_read_isr(void)
{
    /* Save context if RTOS is used */
    SF_CONTEXT_SAVE
    /* Synergy MCU's need this for now. */
    R_BSP_IrqStatusClear(R_SSP_CurrentIrqGet());

    /** Set up a pointer to the location where data is to be dumped. */
    volatile uint16_t* p_result_loc = (uint16_t*)p_ctsu_output;
    /** Copy the results. */
    p_result_loc[itr] = HW_CTSU_CTSUSCGet();
    p_result_loc[itr+1] = HW_CTSU_CTSURCGet();
    itr = (uint8_t)(itr + 2);

    /* Restore context if RTOS is used */
    SF_CONTEXT_RESTORE
}
#endif

/*******************************************************************************************************************//**
 * @brief CTSU End ISR routine
 *
 * This function implements the CTSU End interrupt handler
 *
***********************************************************************************************************************/
void ctsu_end_isr(void);
void ctsu_end_isr(void)
{
    /* Save context if RTOS is used */
    SF_CONTEXT_SAVE
    /* Synergy MCU's need this for now. */
    R_BSP_IrqStatusClear(R_SSP_CurrentIrqGet());

    ctsu_busy = false;
    if((p_isrcallback!=NULL) && (r_touch_ready > 0))
    {
        ctsu_callback_args_t args;

        args.event = CTSU_EVENT_SCAN_COMPLETE;
        args.p_context = NULL;

        p_isrcallback(&args);
    }

    itr = 0U;
    /* Restore context if RTOS is used */
    SF_CONTEXT_RESTORE
}

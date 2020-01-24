/* generated configuration header file - do not edit */
#ifndef R_CTSU_CFG_H_
#define R_CTSU_CFG_H_
/**
 * Specify whether to include code for API parameter checking. Valid settings include:
 *    - 1 : Includes parameter checking
 *    - 0 : Compiles out parameter checking
 */
#define CTSU_CFG_PARAM_CHECKING_ENABLE			((BSP_CFG_PARAM_CHECKING_ENABLE))

/**
 * Define the maximum active channels that the user expects to be active in any
 *  configuration. */
#define CTSU_CFG_MAX_ACTIVE_CHANNEL_COMBOS		(5)

/**
 * Define the window size of the l-point running average filter.
 * Filter output = ((pow(2, CTSU_CFG_FILTER_DEPTH)-1)*previous output + input)/ pow(2, CTSU_CFG_FILTER_DEPTH);
 * This is used in the default filter function used in ctsu_functions_t
 * If the filter depth larger the initial auto tuning (if enabled) may take longer
 * */
#define CTSU_CFG_FILTER_DEPTH					(1)

/**
 * Enable/Disable Auto-tuning.
 * Auto-tuning ensures that the sensor count is approx. equal to the reference
 * count output by the CTSU when the channel is not touched. This is helpful
 * for maximizing sensitivity, greatly reducing the conventional constraints on
 * PCB layout, limits to overlay thicknesses, materials and negating the need
 * for calibration during manufacture process. */
#define CFG_AUTO_TUNE							((1))

/**
 * Perform auto-tune and drift compensation only when all channels are untouched
 * This is a feature where auto-tuning and/or drift-compensation are executed
 * only when no channels are being touched.
 * */
#define CTSU_DRIFTCOMP_WHEN_ALL_INACTIVE						((1))

/**
 * Define if drift compensation is to be used or not. Drift compensation will
 * adjust sensor baseline. This is useful when the sensor value is drifting
 * due to temperature, humidity, dust/dirt accumulation, etc. */
#define CFG_DRIFT_COMPENSATION					((1))

#if (CFG_DRIFT_COMPENSATION > 0)
/**
 * Define the number of scans to take into account when calculating the initial
 * sensor baseline value. */
#define CALIB_SCAN_COUNT						(4)
/** Drift compensation occurs every N times. N is defined below. */
#define DC_TIMING_STEADY_STATE                              (500)
#define DC_TIMING_INITIAL_RATE                              (5)
#define DC_TIMING_BUTTON_RELEASE_RATE                       (500)

/* Methods available for drift compensation*/
#define DRIFT_COMP_BASIC	(0)
#define DRIFT_COMP_ALT_1	(1)
#define DRIFT_COMP_ALT_2	(2)
#define DRIFT_COMP_ALT_3	(3)
#define DRIFT_COMP_METHOD	(DRIFT_COMP_ALT_1)

#endif//(CFG_DRIFT_COMPENSATION > 0)

#if (CFG_AUTO_TUNE > 0)
/** Auto-tuning is run every N scans. N is defined using AT_TIMING
 *  Auto-tuning should occur more frequently than drift compensation
 *  If drift compensation is used the auto-tuning rate is set to double the steady state drift compensation rate
 */
#if defined(DC_TIMING_STEADY_STATE)
#define AT_TIMING                               (DC_TIMING_STEADY_STATE >> 1)
#else
#define AT_TIMING                               (800)
#endif
/** Define the amount of deviation with respect to the threshold at which run-time offset tuning should kick in. */
#define AT_THRESHOLD_PERCENT                    (2)
/**
 * Initial auto-tuning can take a good amount of time. User may abort if it
 * extends beyond the below defined number of scans. */
#if defined(CTSU_CFG_MAX_ACTIVE_CHANNEL_COMBOS)
#define AT_MAX_DURATION                         (CTSU_CFG_MAX_ACTIVE_CHANNEL_COMBOS * 50 * CTSU_CFG_FILTER_DEPTH)
#else
#error "User should device how long initial auto-tuning is allowed to operate. "
#endif
#endif//(CFG_AUTO_TUNE > 0)

#if !defined(CTSU_CFG_MAX_ACTIVE_CHANNEL_COMBOS)
#error "User should allow dynamic memory usage or define the maximum active channels intended for use."
#endif

/**
 * Define if DTC block transfers should be used for moving data instead of CPU ISR
 *
 * 	0 - Not used
 * 	1 - Use DTC block transfers in long address mode (Recommended)
 * */
#define USING_DTC_FOR_CTSU	 (1)

#if (CFG_AUTO_TUNE <= 0)
#warning "Initial and Run-time auto-calibration/tuning has been disabled. This is not recommended."
#endif

#if (CFG_DRIFT_COMPENSATION <=0)
#warning "Initial baseline setup and Run-time drift compensation has been disabled. This is not recommended."
#endif
#endif /* R_CTSU_CFG_H_ */

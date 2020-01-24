/* generated HAL header file - do not edit */
#ifndef HAL_DATA_H_
#define HAL_DATA_H_
#include <stdint.h>
#include "bsp_api.h"
#include "common_data.h"
#include "r_dtc.h"
#include "r_transfer_api.h"
/* Include the headers for the r_touch API use */
#include "r_ctsu_api.h"
#include "r_ctsu.h"
#include "r_dtc.h"
#include "r_icu.h"
#include "r_external_irq_api.h"
#include "r_gpt.h"
#include "r_timer_api.h"
#ifdef __cplusplus
extern "C"
{
#endif
/* Transfer on DTC Instance. */
extern const transfer_instance_t g_transfer0;
#ifndef NULL
void NULL(transfer_callback_args_t *p_args);
#endif
/** Pointer to a tuned CTSU configuration */
extern ctsu_hw_cfg_t g_ctsu_config;

/** Pointer to CTSU API */
extern const ctsu_api_t *g_ctsu0_api;

/** Pointer to CTSU configuration */
extern ctsu_cfg_t g_ctsu0_cfg_on_ctsu;

/** Pointer to control handle */
extern ctsu_instance_ctrl_t g_ctsu0_ctrl;

/**  CTSU instance */
extern ctsu_instance_t g_ctsu0;

/** Pointer to Advanced Functions*/
extern ctsu_functions_t g_ctsu0_functions;

#ifndef NULL
void NULL(ctsu_callback_args_t *p_args);
#endif
/* External IRQ on ICU Instance. */
extern const external_irq_instance_t g_external_irq1;
#ifndef sclb
void sclb(external_irq_callback_args_t *p_args);
#endif
/* External IRQ on ICU Instance. */
extern const external_irq_instance_t g_external_irq0;
#ifndef switch_callback
void switch_callback(external_irq_callback_args_t *p_args);
#endif
/** Timer on GPT Instance. */
extern const timer_instance_t g_gpt0;
#ifndef NULL
void NULL(timer_callback_args_t *p_args);
#endif
void hal_entry(void);
void g_hal_init(void);
#ifdef __cplusplus
} /* extern "C" */
#endif
#endif /* HAL_DATA_H_ */

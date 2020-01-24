/* generated HAL source file - do not edit */
#include "hal_data.h"
#if (BSP_IRQ_DISABLED) != BSP_IRQ_DISABLED
#if !defined(SSP_SUPPRESS_ISR_g_transfer0) && !defined(SSP_SUPPRESS_ISR_DTCELC_EVENT_CTSU_END)
#define DTC_ACTIVATION_SRC_ELC_EVENT_CTSU_END
#if defined(DTC_ACTIVATION_SRC_ELC_EVENT_ELC_SOFTWARE_EVENT_0) && !defined(DTC_VECTOR_DEFINED_SOFTWARE_EVENT_0)
SSP_VECTOR_DEFINE(elc_software_event_isr, ELC, SOFTWARE_EVENT_0);
#define DTC_VECTOR_DEFINED_SOFTWARE_EVENT_0
#endif
#if defined(DTC_ACTIVATION_SRC_ELC_EVENT_ELC_SOFTWARE_EVENT_1) && !defined(DTC_VECTOR_DEFINED_SOFTWARE_EVENT_1)
SSP_VECTOR_DEFINE(elc_software_event_isr, ELC, SOFTWARE_EVENT_1);
#define DTC_VECTOR_DEFINED_SOFTWARE_EVENT_1
#endif
#endif
#endif

dtc_instance_ctrl_t g_transfer0_ctrl;
transfer_info_t g_transfer0_info =
{ .dest_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED,
  .repeat_area = TRANSFER_REPEAT_AREA_SOURCE,
  .irq = TRANSFER_IRQ_END,
  .chain_mode = TRANSFER_CHAIN_MODE_DISABLED,
  .src_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED,
  .size = TRANSFER_SIZE_2_BYTE,
  .mode = TRANSFER_MODE_BLOCK,
  .p_dest = (void *) NULL,
  .p_src = (void const *) NULL,
  .num_blocks = 1,
  .length = 1, };
const transfer_cfg_t g_transfer0_cfg =
{ .p_info = &g_transfer0_info,
  .activation_source = ELC_EVENT_CTSU_END,
  .auto_enable = false,
  .p_callback = NULL,
  .p_context = &g_transfer0,
  .irq_ipl = (BSP_IRQ_DISABLED) };
/* Instance structure to use this module. */
const transfer_instance_t g_transfer0 =
{ .p_ctrl = &g_transfer0_ctrl, .p_cfg = &g_transfer0_cfg, .p_api = &g_transfer_on_dtc };
#if (4) != BSP_IRQ_DISABLED
#if !defined(SSP_SUPPRESS_ISR_g_ctsu0) && !defined(SSP_SUPPRESS_ISR_CTSU)
SSP_VECTOR_DEFINE( ctsu_write_isr, CTSU, WRITE);
#endif
#endif
#if (4) != BSP_IRQ_DISABLED
#if !defined(SSP_SUPPRESS_ISR_g_ctsu0) && !defined(SSP_SUPPRESS_ISR_CTSU)
SSP_VECTOR_DEFINE( ctsu_read_isr, CTSU, READ);
#endif
#endif
#if (4) != BSP_IRQ_DISABLED
#if !defined(SSP_SUPPRESS_ISR_g_ctsu0) && !defined(SSP_SUPPRESS_ISR_CTSU)
SSP_VECTOR_DEFINE( ctsu_end_isr, CTSU, END);
#endif
#endif

/** DTC Structures that will be used by the CTSU */

extern const transfer_instance_t g_transfer0_ctsuwr_block_xfer;
extern const transfer_instance_t g_transfer0_ctsurd_block_xfer;

dtc_instance_ctrl_t g_transfer0_ctsuwr_block_xfer_ctrl;
transfer_info_t g_transfer0_ctsuwr_block_xfer_info[3] =
{ [0] =
{ .dest_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED,
  .repeat_area = TRANSFER_REPEAT_AREA_DESTINATION,
  .irq = TRANSFER_IRQ_END,
  .chain_mode = TRANSFER_CHAIN_MODE_END,
  .src_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED,
  .size = TRANSFER_SIZE_2_BYTE,
  .mode = TRANSFER_MODE_BLOCK,
  .p_src = (void *) NULL,
  .num_blocks = 65535,
  .length = 3, },
  [1] =
  {/* Reloads p_src in the 0th block */
  .dest_addr_mode = TRANSFER_ADDR_MODE_FIXED,
  .repeat_area = TRANSFER_REPEAT_AREA_SOURCE,
  .irq = TRANSFER_IRQ_END,
  .chain_mode = TRANSFER_CHAIN_MODE_END,
  .src_addr_mode = TRANSFER_ADDR_MODE_FIXED,
  .size = TRANSFER_SIZE_4_BYTE,
  .mode = TRANSFER_MODE_REPEAT,
  .p_dest = (void *) &g_transfer0_ctsuwr_block_xfer_info[0].p_src,
  .length = 1, },
  [2] =
  {/* Reloads num_blocks in the 0th block */
  .dest_addr_mode = TRANSFER_ADDR_MODE_FIXED,
  .repeat_area = TRANSFER_REPEAT_AREA_SOURCE,
  .irq = TRANSFER_IRQ_END,
  .chain_mode = TRANSFER_CHAIN_MODE_DISABLED,
  .src_addr_mode = TRANSFER_ADDR_MODE_FIXED,
  .size = TRANSFER_SIZE_2_BYTE,
  .mode = TRANSFER_MODE_REPEAT,
  .p_dest = (void *) &g_transfer0_ctsuwr_block_xfer_info[0].num_blocks,
  .length = 1, }, };
const transfer_cfg_t g_transfer0_ctsuwr_block_xfer_cfg =
{ .p_info = &g_transfer0_ctsuwr_block_xfer_info[0],
  .activation_source = ELC_EVENT_CTSU_WRITE,
  .auto_enable = true,
  .p_callback = NULL,
  .p_context = &g_transfer0_ctsuwr_block_xfer, };
/* Instance structure to use this module. */
const transfer_instance_t g_transfer0_ctsuwr_block_xfer =
{ .p_ctrl = &g_transfer0_ctsuwr_block_xfer_ctrl, .p_cfg = &g_transfer0_ctsuwr_block_xfer_cfg, .p_api =
          &g_transfer_on_dtc };
/******************************************************/
/* CTSUWR Transfer element to use for Synergy driver  */
/******************************************************/
dtc_instance_ctrl_t g_transfer0_ctsurd_block_xfer_ctrl;
transfer_info_t g_transfer0_ctsurd_block_xfer_info[3] =
{ [0] =
{ .dest_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED,
  .repeat_area = TRANSFER_REPEAT_AREA_SOURCE,
  .irq = TRANSFER_IRQ_END,
  .chain_mode = TRANSFER_CHAIN_MODE_END,
  .src_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED,
  .size = TRANSFER_SIZE_2_BYTE,
  .mode = TRANSFER_MODE_BLOCK,
  .p_dest = (void *) NULL,
  .num_blocks = 65535,
  .length = 2, },
  [1] =
  {/* Reloads p_src in the 0th block */
  .dest_addr_mode = TRANSFER_ADDR_MODE_FIXED,
  .repeat_area = TRANSFER_REPEAT_AREA_SOURCE,
  .irq = TRANSFER_IRQ_END,
  .chain_mode = TRANSFER_CHAIN_MODE_END,
  .src_addr_mode = TRANSFER_ADDR_MODE_FIXED,
  .size = TRANSFER_SIZE_4_BYTE,
  .mode = TRANSFER_MODE_REPEAT,
  .p_dest = (void *) &g_transfer0_ctsurd_block_xfer_info[0].p_dest,
  .length = 1, },
  [2] =
  {/* Reloads num_blocks in the 0th block */
  .dest_addr_mode = TRANSFER_ADDR_MODE_FIXED,
  .repeat_area = TRANSFER_REPEAT_AREA_SOURCE,
  .irq = TRANSFER_IRQ_END,
  .chain_mode = TRANSFER_CHAIN_MODE_DISABLED,
  .src_addr_mode = TRANSFER_ADDR_MODE_FIXED,
  .size = TRANSFER_SIZE_2_BYTE,
  .mode = TRANSFER_MODE_REPEAT,
  .p_dest = (void *) &g_transfer0_ctsurd_block_xfer_info[0].num_blocks,
  .length = 1, }, };
const transfer_cfg_t g_transfer0_ctsurd_block_xfer_cfg =
{ .p_info = &g_transfer0_ctsurd_block_xfer_info[0],
  .activation_source = ELC_EVENT_CTSU_READ,
  .auto_enable = true,
  .p_callback = NULL,
  .p_context = &g_transfer0_ctsurd_block_xfer, };
/* Instance structure to use this module. */
const transfer_instance_t g_transfer0_ctsurd_block_xfer =
{ .p_ctrl = &g_transfer0_ctsurd_block_xfer_ctrl, .p_cfg = &g_transfer0_ctsurd_block_xfer_cfg, .p_api =
          &g_transfer_on_dtc };

/* Data functions to use for advanced usage.*/
ctsu_functions_t g_ctsu0_functions =
{ 0 };

ctsu_instance_ctrl_t g_ctsu0_ctrl;
ctsu_cfg_t g_ctsu0_cfg =
{ .p_lower_lvl_transfer_read = &g_transfer0_ctsurd_block_xfer,
  .p_lower_lvl_transfer_write = &g_transfer0_ctsuwr_block_xfer,
  .p_ctsu_hw_cfg = &g_ctsu_config,
  .p_ctsu_functions = &g_ctsu0_functions,
  .p_callback = NULL,
  .p_context = &g_ctsu0_ctrl,
  .ctsu_soft_option = CTSU_PROCESS_OPTION_DEFAULT_SETTING,
  .ctsu_close_option = CTSU_CLOSE_OPTION_RESET_SFRS,
  .write_ipl = (4),
  .read_ipl = (4),
  .end_ipl = (4), };

ctsu_instance_t g_ctsu0 =
{ .p_ctrl = &g_ctsu0_ctrl, .p_cfg = &g_ctsu0_cfg, .p_api = &g_ctsu_on_ctsu, };
const ctsu_api_t *g_ctsu0_api = &g_ctsu_on_ctsu;
#if (5) != BSP_IRQ_DISABLED
#if !defined(SSP_SUPPRESS_ISR_g_external_irq1) && !defined(SSP_SUPPRESS_ISR_ICU10)
SSP_VECTOR_DEFINE( icu_irq_isr, ICU, IRQ10);
#endif
#endif
static icu_instance_ctrl_t g_external_irq1_ctrl;
static const external_irq_cfg_t g_external_irq1_cfg =
{ .channel = 10,
  .trigger = EXTERNAL_IRQ_TRIG_RISING,
  .filter_enable = false,
  .pclk_div = EXTERNAL_IRQ_PCLK_DIV_BY_64,
  .autostart = true,
  .p_callback = sclb,
  .p_context = &g_external_irq1,
  .p_extend = NULL,
  .irq_ipl = (5), };
/* Instance structure to use this module. */
const external_irq_instance_t g_external_irq1 =
{ .p_ctrl = &g_external_irq1_ctrl, .p_cfg = &g_external_irq1_cfg, .p_api = &g_external_irq_on_icu };
#if (5) != BSP_IRQ_DISABLED
#if !defined(SSP_SUPPRESS_ISR_g_external_irq0) && !defined(SSP_SUPPRESS_ISR_ICU11)
SSP_VECTOR_DEFINE( icu_irq_isr, ICU, IRQ11);
#endif
#endif
static icu_instance_ctrl_t g_external_irq0_ctrl;
static const external_irq_cfg_t g_external_irq0_cfg =
{ .channel = 11,
  .trigger = EXTERNAL_IRQ_TRIG_RISING,
  .filter_enable = false,
  .pclk_div = EXTERNAL_IRQ_PCLK_DIV_BY_64,
  .autostart = true,
  .p_callback = switch_callback,
  .p_context = &g_external_irq0,
  .p_extend = NULL,
  .irq_ipl = (5), };
/* Instance structure to use this module. */
const external_irq_instance_t g_external_irq0 =
{ .p_ctrl = &g_external_irq0_ctrl, .p_cfg = &g_external_irq0_cfg, .p_api = &g_external_irq_on_icu };
#if (BSP_IRQ_DISABLED) != BSP_IRQ_DISABLED
#if !defined(SSP_SUPPRESS_ISR_g_gpt0) && !defined(SSP_SUPPRESS_ISR_GPT0)
SSP_VECTOR_DEFINE_CHAN(gpt_counter_overflow_isr, GPT, COUNTER_OVERFLOW, 0);
#endif
#endif
static gpt_instance_ctrl_t g_gpt0_ctrl;
static const timer_on_gpt_cfg_t g_gpt0_extend =
{ .gtioca =
{ .output_enabled = true, .stop_level = GPT_PIN_LEVEL_HIGH },
  .gtiocb =
  { .output_enabled = false, .stop_level = GPT_PIN_LEVEL_LOW },
  .shortest_pwm_signal = GPT_SHORTEST_LEVEL_OFF, };
static const timer_cfg_t g_gpt0_cfg =
{ .mode = TIMER_MODE_PWM, .period = 50, .unit = TIMER_UNIT_FREQUENCY_HZ, .duty_cycle = 50, .duty_cycle_unit =
          TIMER_PWM_UNIT_PERCENT,
  .channel = 0, .autostart = false, .p_callback = NULL, .p_context = &g_gpt0, .p_extend = &g_gpt0_extend, .irq_ipl =
          (BSP_IRQ_DISABLED), };
/* Instance structure to use this module. */
const timer_instance_t g_gpt0 =
{ .p_ctrl = &g_gpt0_ctrl, .p_cfg = &g_gpt0_cfg, .p_api = &g_timer_on_gpt };
void g_hal_init(void)
{
    g_common_init ();
}

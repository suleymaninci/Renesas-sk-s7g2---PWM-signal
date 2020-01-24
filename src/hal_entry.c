#include "hal_data.h"
#include "bsp_api.h"

#define led_mode (*((unsigned int *)0x400400C0U))

volatile uint32_t flag1=0;
volatile uint32_t flag2=0;
ssp_err_t status;
timer_size_t duty_cycle = 5;
uint32_t period=50;

void hal_entry(void)
{
    /* Open GPT Timer */
        status = g_gpt0.p_api->open(g_gpt0.p_ctrl, g_gpt0.p_cfg);
        if(status != SSP_SUCCESS)
        {
            while(1);
        }

        /* Start GPT Timer */
        status = g_gpt0.p_api->start(g_gpt0.p_ctrl);
        if(status != SSP_SUCCESS)
        {
            while(1);
        }

        //start
            while(1)
        {
        if(status != SSP_SUCCESS)
                        {
                            while(1);
                        }


        g_external_irq0.p_api->open(g_external_irq0.p_ctrl,g_external_irq0.p_cfg);
        status=g_external_irq0.p_api->enable(g_external_irq0.p_ctrl);

                if (status != SSP_SUCCESS)
                {
                    while(1);
                }
        g_external_irq1.p_api->open(g_external_irq1.p_ctrl,g_external_irq1.p_cfg);
                status=g_external_irq1.p_api->enable(g_external_irq1.p_ctrl);
                if (status != SSP_SUCCESS)
                       {
                           while(1);
                       }

                    led_mode=0x7;


                if(flag1==1) //s4 button presssed
                    {
                        g_gpt0.p_api->periodSet(g_gpt0.p_ctrl, period ,TIMER_UNIT_FREQUENCY_HZ);
                    status= g_gpt0.p_api->dutyCycleSet(g_gpt0.p_ctrl, duty_cycle, TIMER_PWM_UNIT_PERCENT,0);
                    R_BSP_SoftwareDelay(150, BSP_DELAY_UNITS_MILLISECONDS);

                    if (duty_cycle == 8){
                    duty_cycle=4;
                    led_mode=0x1;
                    }
                    else
                    {
                    duty_cycle= duty_cycle+1;
                    led_mode=0x6;
                    }
                    R_BSP_SoftwareDelay(150, BSP_DELAY_UNITS_MILLISECONDS);
                    led_mode=0x0;
                    }
                else if (flag2==1) //s5 button pressed
                    {
                        g_gpt0.p_api->periodSet(g_gpt0.p_ctrl, period ,TIMER_UNIT_FREQUENCY_HZ);
                        status= g_gpt0.p_api->dutyCycleSet(g_gpt0.p_ctrl, duty_cycle, TIMER_PWM_UNIT_PERCENT,0);
                        R_BSP_SoftwareDelay(500, BSP_DELAY_UNITS_MILLISECONDS);

                        if (duty_cycle == 10){
                        duty_cycle=2;
                        led_mode=0x2;
                        }
                        else
                        {
                        duty_cycle= duty_cycle+1;
                        led_mode=0x5;
                        }
                        R_BSP_SoftwareDelay(500, BSP_DELAY_UNITS_MILLISECONDS);
                        led_mode=0x0;
                    }

}
        }
void switch_callback(external_irq_callback_args_t *p_args){

            flag1=1;
            flag2=0;

}


void sclb(external_irq_callback_args_t *p_args)
{

    flag2=1;
    flag1=0;
}

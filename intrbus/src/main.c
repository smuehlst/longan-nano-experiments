

#include <stdio.h>
#include <stdlib.h>
#include "gd32v_pjt_include.h"
#include "systick.h"


/**
    \brief      configure the GPIO ports
    \param[in]  none
    \param[out] none
    \retval     none
  */
static void gpio_config(void)
{
    /* Configure led GPIO port */ 
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
    // gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1 | GPIO_PIN_2);
}

static void rcu_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);

    rcu_periph_clock_enable(RCU_TIMER1);
}

#define BOP_SET_BIT(X) (X)
#define BOP_CLR_BIT(X) (X << 16)

/**
    \brief      configure the TIMER peripheral
    \param[in]  none
    \param[out] none
    \retval     none
  */
static
void timer_config(void)
{
    /* ----------------------------------------------------------------------------
    TIMER1 Configuration: 
    TIMER1CLK = SystemCoreClock/5400 = 20KHz.
    TIMER1 configuration is timing mode, and the timing is 0.2s(4000/20000 = 0.2s).
    CH0 update rate = TIMER1 counter clock/CH0CV = 20000/4000 = 5Hz.
    ---------------------------------------------------------------------------- */
    timer_oc_parameter_struct timer_ocinitpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER1);

    timer_deinit(TIMER1);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER1 configuration */
    timer_initpara.prescaler         = 5399;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 4000;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init(TIMER1, &timer_initpara);

    /* initialize TIMER channel output parameter struct */
    timer_channel_output_struct_para_init(&timer_ocinitpara);
    /* CH0,CH1 and CH2 configuration in OC timing mode */
    timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_channel_output_config(TIMER1, TIMER_CH_0, &timer_ocinitpara);

    /* CH0 configuration in OC timing mode */
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_0, 2000);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_0, TIMER_OC_MODE_TIMING);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);

    timer_interrupt_enable(TIMER1, TIMER_INT_CH0);
    timer_enable(TIMER1);
}

/* Must not be static! This is a global predefined symbol. */
void TIMER1_IRQHandler(void)
{
    static uint32_t counter;

    if (SET == timer_interrupt_flag_get(TIMER1, TIMER_INT_CH0))
    {
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(TIMER1, TIMER_INT_CH0);

        LEDR_TOG;
        LEDG_TOG;

#if 0
        if (counter % 10 == 0)
        {
            LEDB_TOG;
        }
        counter += 1;
#endif
    }
}

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    rcu_config();
    gpio_config();
    LEDR(0);
    // LEDB(0);
    LEDG(1);
    eclic_global_interrupt_enable();
    eclic_set_nlbits(ECLIC_GROUP_LEVEL3_PRIO1);
    eclic_irq_enable(TIMER1_IRQn, 1, 0);
    timer_config();
    while (1);
}

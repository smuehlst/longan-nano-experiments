/*!
    \file  main.c
    \brief TIMER1 oc timebase demo
    
    \version 2019-6-5, V1.0.0, firmware for GD32VF103
*/

/*
    Copyright (c) 2019, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include <stdio.h>
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

    // Use PA0 ... PA3 as motor control
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,
        GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
}

static void rcu_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_TIMER1);
}

#define BOP_SET_BIT(X) (X)
#define BOP_CLR_BIT(X) (X << 16)

static void standby(void)
{
    GPIO_BOP(GPIOA) = BOP_CLR_BIT(GPIO_PIN_0) | BOP_CLR_BIT(GPIO_PIN_1) | BOP_CLR_BIT(GPIO_PIN_2) | BOP_CLR_BIT(GPIO_PIN_3);
}

typedef struct
{
    uint32_t timer;
    uint32_t timer_channel;
    uint32_t gpio_periph;
#if 0
    uint32_t volatile steps_left;
    uint32_t direction;
#endif
    uint32_t steps[8]; // 1010 -> 0110 -> 0101 -> 1001
#if 0
    uint32_t standby; // 0000
#endif
} motor_runtime;


/**
    \brief      configure the TIMER peripheral
    \param[in]  none
    \param[out] none
    \retval     none
  */
void timer_config(uint32_t step_delay_us)
{
    /* ----------------------------------------------------------------------------
    TIMER1 Configuration: 
    TIMER1CLK = SystemCoreClock/108 = 1MHz.
    TIMER1 configuration is timing mode, and the timing is how many microseconds
    it takes for two motor steps.
    ---------------------------------------------------------------------------- */
    timer_oc_parameter_struct timer_ocinitpara;
    timer_parameter_struct timer_initpara;

    timer_deinit(TIMER1);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER1 configuration */
    timer_initpara.prescaler         = (uint16_t) (SystemCoreClock / 1000000U);
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = step_delay_us;
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
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_0, 0);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_0, TIMER_OC_MODE_TIMING);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);

    timer_interrupt_enable(TIMER1, TIMER_INT_CH0);

    timer_enable(TIMER1);
}

static uint32_t volatile steps_left;
// static unsigned int direction_up;

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    uint32_t number_of_steps = 4096;
    uint32_t rpm = 10;
    uint32_t step_delay = 60 * 1000 * 1000 / number_of_steps / rpm;

    /* peripheral clock enable */
    rcu_config();

    /* GPIO configure */
    gpio_config();

    // longan_led_init();
    LEDR(1);
    LEDG(1);
    LEDB(1);

    eclic_global_interrupt_enable();
    eclic_set_nlbits(ECLIC_GROUP_LEVEL3_PRIO1);
    eclic_irq_enable(TIMER1_IRQn, 1, 0);

    while (1)
    {
        LEDR(1);
        steps_left = 4096;

        timer_config(step_delay);

        while (steps_left > 0);

        LEDR(0);
        delay_1ms(1000);
    }
}

static uint32_t const steps[] =
{
    BOP_CLR_BIT(GPIO_PIN_0) | BOP_CLR_BIT(GPIO_PIN_1) | BOP_CLR_BIT(GPIO_PIN_2) | BOP_SET_BIT(GPIO_PIN_3), // 0001
    BOP_CLR_BIT(GPIO_PIN_0) | BOP_CLR_BIT(GPIO_PIN_1) | BOP_SET_BIT(GPIO_PIN_2) | BOP_SET_BIT(GPIO_PIN_3), // 0011
    BOP_CLR_BIT(GPIO_PIN_0) | BOP_CLR_BIT(GPIO_PIN_1) | BOP_SET_BIT(GPIO_PIN_2) | BOP_CLR_BIT(GPIO_PIN_3), // 0010
    BOP_CLR_BIT(GPIO_PIN_0) | BOP_SET_BIT(GPIO_PIN_1) | BOP_SET_BIT(GPIO_PIN_2) | BOP_CLR_BIT(GPIO_PIN_3), // 0110

    BOP_CLR_BIT(GPIO_PIN_0) | BOP_SET_BIT(GPIO_PIN_1) | BOP_CLR_BIT(GPIO_PIN_2) | BOP_CLR_BIT(GPIO_PIN_3), // 0100
    BOP_SET_BIT(GPIO_PIN_0) | BOP_SET_BIT(GPIO_PIN_1) | BOP_CLR_BIT(GPIO_PIN_2) | BOP_CLR_BIT(GPIO_PIN_3), // 1100
    BOP_SET_BIT(GPIO_PIN_0) | BOP_CLR_BIT(GPIO_PIN_1) | BOP_CLR_BIT(GPIO_PIN_2) | BOP_CLR_BIT(GPIO_PIN_3), // 1000
    BOP_SET_BIT(GPIO_PIN_0) | BOP_CLR_BIT(GPIO_PIN_1) | BOP_CLR_BIT(GPIO_PIN_2) | BOP_SET_BIT(GPIO_PIN_3) // 1001
};

static motor_runtime const motor1 = {
    .timer = TIMER1,
    .timer_channel = TIMER_INT_CH0,
    .gpio_periph = GPIOA,
    .steps = { 0 }
};

static
void process_motor_intr(motor_runtime const * const mrt)
{
    bool const ch0_intr = SET == timer_interrupt_flag_get(mrt->timer, mrt->timer_channel);

    if (ch0_intr) {
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(mrt->timer, mrt->timer_channel);
        
        if (steps_left == 0)
        {
            standby();
            timer_interrupt_disable(mrt->timer, mrt->timer_channel);
        }
        else
        {
            uint32_t const idx = steps_left % (sizeof(steps) / sizeof(steps[0]));
            GPIO_BOP(mrt->gpio_periph) = steps[idx];
            steps_left -= 1;
        }
    }   
}

void TIMER1_IRQHandler(void)
{
    process_motor_intr(&motor1);
}

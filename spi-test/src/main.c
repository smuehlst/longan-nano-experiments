/*!
    \file  main.c
    \brief running SPI test
    
    This program is a small test program to test SPI receive-only and full duplect modes.
*/

#include "gd32vf103.h"
#include "systick.h"
#include <stdio.h>

/* BUILTIN LED OF LONGAN BOARDS IS PIN PC13 */
#define LED_PIN GPIO_PIN_13
#define LED_GPIO_PORT GPIOC
#define LED_GPIO_CLK RCU_GPIOC

void longan_spi_init()
{
    rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_SPI1);
	rcu_periph_clock_enable(RCU_AF);

	/* SPI SCK/PB13, SPI MISO/PB14, SPI MOSI/PB15 GPIO pin configuration */
	gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_15 | GPIO_PIN_14 | GPIO_PIN_13);

	/* SPI CS/PB12 GPIO pin configuration,
	 * GPIO_MODE_OUT_PP as it is controlled in software.
	 */
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);

	/* CS disabled */
	gpio_bit_set(GPIOB, GPIO_PIN_12);

    spi_parameter_struct spi_init_struct;
 
    /* deinitialize SPI and the parameters */
    spi_i2s_deinit(SPI1);
    spi_struct_para_init(&spi_init_struct);

    /* SPI1 parameter config */
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_2EDGE;
    spi_init_struct.nss                  = SPI_NSS_SOFT;
    spi_init_struct.prescale             = SPI_PSC_256;
    spi_init_struct.endian               = SPI_ENDIAN_MSB;
    spi_init(SPI1, &spi_init_struct);
}

void longan_led_init()
{
    /* enable the led clock */
    rcu_periph_clock_enable(LED_GPIO_CLK);
    /* configure led GPIO port */ 
    gpio_init(LED_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LED_PIN);

    GPIO_BC(LED_GPIO_PORT) = LED_PIN;
}

void longan_led_on()
{
    GPIO_BOP(LED_GPIO_PORT) = LED_PIN;
}

void longan_led_off()
{
    GPIO_BC(LED_GPIO_PORT) = LED_PIN;
}

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    uint32_t volatile counter = 0;

    longan_led_init();
    longan_spi_init();

    spi_enable(SPI1);
    while (1) {
        longan_led_on();
        gpio_bit_reset(GPIOB, GPIO_PIN_12);

        while(RESET == spi_i2s_flag_get(SPI1, SPI_FLAG_TBE));
        spi_i2s_data_transmit(SPI1, (uint16_t) counter);

	    while(RESET == spi_i2s_flag_get(SPI1, SPI_FLAG_RBNE));
        uint16_t volatile indata = spi_i2s_data_receive(SPI1);

        longan_led_off();
        gpio_bit_set(GPIOB, GPIO_PIN_12);
        delay_1ms(1);

        counter += 1;
    }
}

/*!
    \file    main.c
    \brief   Attach MAX6675 thermocouple to Longan Nano
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

#include "gd32v_pjt_include.h"
#include "lcd/lcd.h"
#include <string.h>

void rcu_config(void);
void gpio_config(void);
void spi1_config(void);
void longan_oled_init(void);
void longan_led_init(void);

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    /* peripheral clock enable */
    rcu_config();

    /* GPIO configure */
    gpio_config();

    /* NSS high -> diable SPI1 */
    gpio_bit_set(GPIOB, GPIO_PIN_12);

    longan_oled_init();

    longan_led_init();

    /* SPI configure */
    spi1_config();

    /* SPI enable */
    spi_enable(SPI1);

    LCD_ShowString(24, 0, (u8 const *) "Starting!", GBLUE);
    delay_1ms(1000);

    uint32_t cntr = 0;
    while (TRUE)
    {
        gpio_bit_reset(GPIOB, GPIO_PIN_12);

        uint16_t const data = spi_i2s_data_receive(SPI1);

        gpio_bit_set(GPIOB, GPIO_PIN_12);

        char buf[32];
        sprintf(buf, "SPI data %u", data);
        LCD_ShowString(24, 0, (u8 const *) buf, GBLUE);

        static const char blanks[] = "        ";
        strcpy(buf, "        ");
        buf[cntr % (sizeof(blanks) - 1)] = 'o';
        LCD_ShowString(24, 16, (u8 const *) buf, MAGENTA);

        delay_1ms(1000);
        cntr += 1;

        LEDR_TOG;
    }

    while (1);
}

/*!
    \brief      configure different peripheral clocks
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);

    rcu_periph_clock_enable(RCU_SPI1);
    rcu_periph_clock_enable(RCU_AF);
}

void longan_oled_init(void)
{
    Lcd_Init();			// init OLED
    LCD_Clear(BLACK);
    BACK_COLOR = BLACK;
}

void longan_led_init(void)
{
    /* 1 means off! */
    LEDR(1);
    LEDG(1);
    LEDB(1);
}

/*!
    \brief      configure the GPIO peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void gpio_config(void)
{
#if 0
    /* SPI1 GPIO config: SCK1/PB13 + NSS1/PB12 + MISO1/PB14 */
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14);
    /* MISO1/PB14 */
    // gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_14);
#endif

    /* SPI1_SCK(PB13), SPI1_MISO(PB14) GPIO pin configuration */
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);
    gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_14);
    /* SPI1_CS(PB12) GPIO pin configuration */
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);

    /* configure led GPIO port */ 
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1|GPIO_PIN_2);
}

/*!
    \brief      configure the SPI peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void spi1_config(void)
{
    spi_parameter_struct spi_init_struct;
    /* deinitilize SPI and the parameters */

    spi_i2s_deinit(SPI1);
    spi_struct_para_init(&spi_init_struct);

    /* SPI1 parameter config */
    spi_init_struct.trans_mode           = SPI_TRANSMODE_RECEIVEONLY;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_16BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;
    spi_init_struct.nss                  = SPI_NSS_HARD;
    spi_init_struct.prescale             = SPI_PSC_256;
    spi_init_struct.endian               = SPI_ENDIAN_MSB;
    spi_init(SPI1, &spi_init_struct);

    spi_nss_output_enable(SPI1);
}

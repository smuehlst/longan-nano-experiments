#ifndef SPI_H
#define SPI_H

#include <Arduino.h>

namespace replacement
{

#define SPI_LSBFIRST 0
#define SPI_MSBFIRST 1

#define SPI_MODE0 0
#define SPI_MODE1 1
#define SPI_MODE2 2
#define SPI_MODE3 3

// byte constexpr SS = PB12;

class SPISettings {
public:
    SPISettings()
        : _freq(1000000)
        , _bitOrder(SPI_MSBFIRST)
        , _dataMode(SPI_MODE0) {}

    SPISettings(uint32_t freq, uint8_t bitOrder, uint8_t dataMode)
        : _freq(freq)
        , _bitOrder(bitOrder)
        , _dataMode(dataMode) {}

    uint32_t _freq;
    uint8_t _bitOrder;
    uint8_t _dataMode;
};

class SPIClass {
private:
#if 0
    uint32_t _dev;
    uint32_t _dev_clk;
    uint32_t _mosi_bank;
    uint32_t _miso_bank;
    uint32_t _sclk_bank;
    uint32_t _ssel_bank;
    uint32_t _mosi_bit;
    uint32_t _miso_bit;
    uint32_t _sclk_bit;
    uint32_t _ssel_bit;
    uint32_t _mosi_bank_clk;
    uint32_t _miso_bank_clk;
    uint32_t _sclk_bank_clk;
    uint32_t _ssel_bank_clk;
    uint8_t _dataMode;
    uint8_t _bitOrder;
    uint32_t _freq;
    uint8_t _ssel_hard;
#endif

    uint32_t _spi_periph;
    uint32_t _spi_gpio;
    rcu_periph_enum _rcu_periph_gpio;
    rcu_periph_enum _rcu_periph_spi;
    uint32_t _mosi;
    uint32_t _miso;
    uint32_t _sclk;
    uint32_t _ssel;

    // void beginTransaction();

public:
    SPIClass(uint32_t spi_periph, uint32_t spi_gpio,
        rcu_periph_enum rcu_periph_gpio,
        rcu_periph_enum rcu_periph_spi,
        uint32_t mosi, uint32_t miso, uint32_t sclk, uint32_t ssel)
        : _spi_periph(spi_periph),
            _spi_gpio(spi_gpio),
            _rcu_periph_gpio(rcu_periph_gpio),
            _rcu_periph_spi(rcu_periph_spi),
            _mosi(mosi),
            _miso(miso),
            _sclk(sclk),
            _ssel(ssel)
    {
    }

    SPIClass() : SPIClass(0, 0, RCU_GPIOB, RCU_SPI1, 0, 0, 0, 0) {}

    void begin()
    {
    }

    void end()
    {
    }

    void beginTransaction(SPISettings const& spiSettings = SPISettings())
    {
        rcu_periph_clock_enable(_rcu_periph_gpio);
        rcu_periph_clock_enable(_rcu_periph_spi);
        rcu_periph_clock_enable(RCU_AF);

        /* SPI SCK, SPI MISO, SPI MOSI GPIO pin configuration */
        gpio_init(_spi_gpio, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, _mosi | _miso | _sclk);

        /* SPI CS GPIO pin configuration,
        * GPIO_MODE_OUT_PP as it is controlled in software.
        */
        gpio_init(_spi_gpio, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, _ssel);

        /* CS disabled */
        gpio_bit_set(_spi_gpio, _ssel);

        spi_parameter_struct spi_init_struct;
 
        /* deinitilize SPI and the parameters */
        spi_i2s_deinit(_spi_periph);
        spi_struct_para_init(&spi_init_struct);

        /* SPI1 parameter config */
        spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
        spi_init_struct.device_mode          = SPI_MASTER;
        spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
        spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
        spi_init_struct.nss                  = SPI_NSS_SOFT;
        spi_init_struct.prescale             = SPI_PSC_256;
        spi_init_struct.endian               = SPI_ENDIAN_MSB;

        spi_init(_spi_periph, &spi_init_struct);

        spi_enable(_spi_periph);
    }

    void endTransaction()
    {
        spi_disable(_spi_periph);
    }

    // timeout in ms
    void transfer(uint8_t const *data, uint32_t size, uint32_t timeout = 1)
    {
        uint64_t const startT = millis();

        gpio_bit_reset(_spi_gpio, _ssel);

        for (size_t i = 0; i < size; i++)
        {
            while (!spi_i2s_flag_get(_spi_periph, SPI_FLAG_TBE))
            {
                if (millis() > startT + timeout)
                {
                    gpio_bit_set(_spi_gpio, _ssel);
                    return;
                }
            }
            spi_i2s_data_transmit(_spi_periph, *data);
            data++;
        }

        gpio_bit_set(_spi_gpio, _ssel);
    }

    void transfer(
        uint8_t const *txdata, uint8_t *rxdata, uint32_t size, uint32_t timeout = 1)
    {
        uint64_t const startT = millis();

        gpio_bit_reset(_spi_gpio, _ssel);

        for (size_t i = 0; i < size; i++)
        {
            while (!spi_i2s_flag_get(_spi_periph, SPI_FLAG_TBE))
            {
                if (millis() > startT + timeout)
                {
                    gpio_bit_set(_spi_gpio, _ssel);
                    return;
                }
            }
            spi_i2s_data_transmit(_spi_periph, *txdata);
            txdata++;
            while (!spi_i2s_flag_get(_spi_periph, SPI_FLAG_RBNE))
            {
                if (millis() > startT + timeout)
                {
                    gpio_bit_set(_spi_gpio, _ssel);
                    return;
                }
            }
            *rxdata = spi_i2s_data_receive(_spi_periph);
            rxdata++;
        }

        gpio_bit_set(_spi_gpio, _ssel);
    }

#if 0
    void setBitOrder(uint8_t bitOrder);
    void setDataMode(uint8_t dataMode);
    void setFrequency(uint32_t freq);
#endif
};

}

static constexpr byte SS = PA0;

#endif // SPI_H
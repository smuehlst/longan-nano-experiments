#include <Arduino.h>

#include <MFRC522.h>
#include <NoSerial.h>
#include "gd32v_pjt_include.h"
extern "C" {
#include "lcd/lcd.h"
}

MFRC522 mfrc522(SPI1, GPIOB, RCU_GPIOB, RCU_SPI1,
          GPIO_PIN_15, GPIO_PIN_14, GPIO_PIN_13, GPIO_PIN_12);  // Create MFRC522 instance

static void gpio_config(void)
{
    /* configure led GPIO port */ 
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1 | GPIO_PIN_2);
}

static void rcu_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOC);
}

static void longan_led_init(void)
{
    /* 1 means off! */
    LEDR(1);
    LEDG(1);
    LEDB(1);
}

static void longan_oled_init(void)
{
    Lcd_Init();			// init OLED
    LCD_Clear(BLACK);
    BACK_COLOR = BLACK;
}

void setup()
{
	rcu_config();

	gpio_config();

	longan_oled_init();

	longan_led_init();

#if 0
	rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);

    rcu_periph_clock_enable(RCU_SPI1);
    rcu_periph_clock_enable(RCU_AF);

	gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);

    /* CS disabled */
    gpio_bit_set(GPIOB, GPIO_PIN_12);

#else
	uint32_t _spi_periph = SPI1;
	uint32_t _gpio_periph = GPIOB;
	rcu_periph_enum _rcu_periph_gpio = RCU_GPIOB;
	rcu_periph_enum _rcu_periph_spi = RCU_SPI1;
	uint32_t _mosi = GPIO_PIN_15;
	uint32_t _miso = GPIO_PIN_14;
	uint32_t _sclk = GPIO_PIN_13;
	uint32_t _ssel = GPIO_PIN_12;

	rcu_periph_clock_enable(_rcu_periph_gpio);
	rcu_periph_clock_enable(_rcu_periph_spi);
	rcu_periph_clock_enable(RCU_AF);

	/* SPI SCK, SPI MISO, SPI MOSI GPIO pin configuration */
	gpio_init(_gpio_periph, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, _mosi | _miso | _sclk);

	/* SPI CS GPIO pin configuration,
	* GPIO_MODE_OUT_PP as it is controlled in software.
	*/
	// gpio_init(_spi_periph, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, _ssel);
	gpio_init(_gpio_periph, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, _ssel);

	/* CS disabled */
	gpio_bit_set(_gpio_periph, _ssel);

#if 0
	spi_parameter_struct spi_init_struct;

	/* deinitilize SPI and the parameters */
	spi_i2s_deinit(_spi_periph);
	spi_struct_para_init(&spi_init_struct);

	/* SPI1 parameter config */
	spi_init_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
	spi_init_struct.device_mode = SPI_MASTER;
	spi_init_struct.frame_size = SPI_FRAMESIZE_8BIT;
	spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
	spi_init_struct.nss = SPI_NSS_SOFT;
	spi_init_struct.prescale = SPI_PSC_256;
	spi_init_struct.endian = SPI_ENDIAN_MSB;

	spi_init(_spi_periph, &spi_init_struct);
	spi_enable(_spi_periph);
#else
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
    spi_init_struct.prescale             = SPI_PSC_256; // ~5274 Hz serial frequency
    spi_init_struct.endian               = SPI_ENDIAN_MSB;
    spi_init(_spi_periph, &spi_init_struct);
	spi_enable(_spi_periph);
#endif

#endif

	mfrc522.PCD_Init();				   // Init MFRC522
	delay(4);						   // Optional delay. Some board do need more time after init to be ready, see Readme
	mfrc522.PCD_DumpVersionToSerial(); // Show details of PCD - MFRC522 Card Reader details

	LEDG_TOG;
}

void loop() {
	// Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
	if ( ! mfrc522.PICC_IsNewCardPresent()) {
		return;
	}

  	LEDR_TOG;

	// Select one of the cards
	if ( ! mfrc522.PICC_ReadCardSerial()) {
		return;
	}

  LEDB_TOG;

	// Dump debug info about the card; PICC_HaltA() is automatically called
	mfrc522.PICC_DumpToSerial(&(mfrc522.uid));
}
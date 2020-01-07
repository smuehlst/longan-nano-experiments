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
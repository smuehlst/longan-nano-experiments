#include <Arduino.h>

#include <MFRC522.h>
#include <NoSerial.h>
#include "gd32v_pjt_include.h"
extern "C" {
#include "lcd/lcd.h"
}
#include "lcdlog.h"

MFRC522 mfrc522(SPI1, GPIOB, RCU_GPIOB, RCU_SPI1,
          GPIO_PIN_15, GPIO_PIN_14, GPIO_PIN_13, GPIO_PIN_12);  // Create MFRC522 instance

LcdLog lcdLog;

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

static void longan_serial_init(void)
{
	/* connect port to USARTx_Tx */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

    /* connect port to USARTx_Rx */
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

    /* USART configure */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);
}

/* retarget the C library printf function to the USART */
int _put_char(int ch)
{
    usart_data_transmit(USART0, (uint8_t) ch );
    while (usart_flag_get(USART0, USART_FLAG_TBE) == RESET){
    }

    return ch;
}

void setup()
{
	rcu_config();

	gpio_config();

	longan_serial_init();
	longan_oled_init();
	longan_oled_init();

	longan_led_init();

    Serial.println("Started");

	mfrc522.PCD_Init();				   // Init MFRC522
	delay(4);						   // Optional delay. Some board do need more time after init to be ready, see Readme
	mfrc522.PCD_DumpVersionToSerial(); // Show details of PCD - MFRC522 Card Reader details

	LEDG_TOG;
}

void loop() {
    delay_1ms(500);
    
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
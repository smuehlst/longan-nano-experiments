/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <Arduino.h>
extern "C" {
#include "lcd/lcd.h"
}
#include <stdio.h>

// Set LED_BUILTIN if it is not defined by Arduino framework
// #define LED_BUILTIN 2

static void longan_oled_init(void)
{
    Lcd_Init();
    LCD_Clear(BLACK);
    BACK_COLOR = BLACK;
}

void setup()
{
  longan_oled_init();

  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  unsigned long const time = micros();

  char buf[64];
  sprintf(buf, "time %ld             ", time);
  LCD_ShowString(0, 0, (u8 const *) buf, GBLUE);

  // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_BUILTIN, HIGH);
  // wait for a second
  delay(1000);
  // turn the LED off by making the voltage LOW
  digitalWrite(LED_BUILTIN, LOW);
  // wait for a second
  delay(1000);
}

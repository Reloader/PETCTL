/*
Для работы экрана необходимо подключить библиотеки :
Adafruit_GFX
Adafruit_PCD8544

Работа с экраном напрямую
https://count-zero.ru/2017/pcd8544/https://count-zero.ru/2017/pcd8544/

*/

#define ShowScreen_period 300  // период обновления экрана в мс

#define _contrast_ 0x25
#define _bias_ 0x4

#define _lcd_RST A1
#define _lcd_CE A2
#define _lcd_DC A3
#define _lcd_DIN A4
#define _lcd_CLK A5

#include <SPI.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

Adafruit_PCD8544 display = Adafruit_PCD8544(_lcd_CLK, _lcd_DIN, _lcd_DC, _lcd_CE, _lcd_RST);




void screen_Init() {
  display.begin();
  display.setContrast(_contrast_);
  display.setBias(_bias_);
  display.clearDisplay();  // clears the screen and buffer
}

void screen_logo() {

  
}

void screen_clear() {
  display.clearDisplay();  // clears the screen and buffer
}

void showscreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.println("test");
  display.display();
  delay(2000);
  
  

  
}

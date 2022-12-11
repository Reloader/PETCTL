// Подключение экрана от присвятой Nokia 5110 (3310)
/*
  Примеры
  https://arduinomaster.ru/uroki-arduino/podklyuchenie-displeya-nokia-5110-k-arduino/

  тут кажется ссылка на легковесную библиотеку
  https://arduinoplus.ru/arduino-nokia-5110/

  каая-то библиотека
  https://iarduino.ru/file/52.html

  Большой шрифт - 12 символов на строку
*/

#define ShowScreen_period 300  // период обновления экрана в мс

#include <SPI.h>
#include <Wire.h>
#include "src/NOKIA5110_TEXT.h"


// LCD Nokia 5110 pinout left to right
// RST / CE / DC / DIN / CLK / VCC /LIGHT / GND
#define _lcd_RST A1
#define _lcd_CE A2
#define _lcd_DC A3
#define _lcd_DIN A4
#define _lcd_CLK A5

/*

#define _lcd_RST 6
#define _lcd_CE 5
#define _lcd_DC 4
#define _lcd_DIN 3
#define _lcd_CLK 2

*/


// Create an LCD object
NOKIA5110_TEXT nokia_lcd(_lcd_RST, _lcd_CE, _lcd_DC, _lcd_DIN, _lcd_CLK);

#define inverse  false // set to true to invert display pixel color
//#define inverse  true // set to true to invert display pixel color
#define contrast 0xB7// default is 0xBF set in LCDinit, Try 0xB1 <-> 0xBF if your display is too dark/dim
#define bias 0x12 // LCD bias mode 1:48: Try 0x12 or 0x13 or 0x14


void screen_Init() {
  delay(50);
  nokia_lcd.LCDInit(inverse, contrast, bias); // init  the lCD
  nokia_lcd.LCDClear(0x00); // Clear whole screen
}

void screen_logo() {

  
}

void screen_clear() {
  nokia_lcd.LCDClear(0x00); // Clear whole screen
}

void showscreen() {

  nokia_lcd.LCDFont(LCDFont_Default); // Set the font
  nokia_lcd.LCDgotoXY(0, 0); // (go to (X , Y) (0-84 columns, 0-5 blocks) top left corner
  nokia_lcd.LCDString("HELLO WORLD"); // print
  nokia_lcd.LCDgotoXY(0, 1);
  nokia_lcd.LCDString("-=<TEST>=-"); // print
  delay(5000);
  screen_clear();
}

/*

Работа с экраном напрямую
https://count-zero.ru/2017/pcd8544/

Перенести код библиотеки из статьи в файл 
"src/pcd8544.h"



*/

#define ShowScreen_period 300  // период обновления экрана в мс

#define _contrast_ 0x25
#define _bias_ 0x4

#define PIN_SCE   A2
#define PIN_RESET A1
#define PIN_DC    A3
#define PIN_SDIN  A4
#define PIN_SCLK  A5

#define LCD_C     LOW
#define LCD_D     HIGH

#define LCD_X     84
#define LCD_Y     48


//#define _lcd_RST A1
//#define _lcd_CE A2
//#define _lcd_DC A3
//#define _lcd_DIN A4
//#define _lcd_CLK A5

#include <SPI.h>
#include <Wire.h>
#include "src/pcd8544.h"

//#include <Adafruit_GFX.h>
//#include <Adafruit_PCD8544.h>

//Adafruit_PCD8544 display = Adafruit_PCD8544(_lcd_CLK, _lcd_DIN, _lcd_DC, _lcd_CE, _lcd_RST);





void LcdWrite(byte dc, byte data)
{
  digitalWrite(PIN_DC, dc);
  digitalWrite(PIN_SCE, LOW);
  shiftOut(PIN_SDIN, PIN_SCLK, MSBFIRST, data);
  digitalWrite(PIN_SCE, HIGH);
}

void LcdCharacter(char character)
{
  LcdWrite(LCD_D, 0x00);
  for (int index = 0; index < 5; index++)
  {
    LcdWrite(LCD_D, ASCII[character - 0x20][index]);
  }
  LcdWrite(LCD_D, 0x00);
}

void LcdString(char *characters)
{
  while (*characters)
  {
    LcdCharacter(*characters++);
  }
}



void screen_Init() {
  //display.begin();
  //display.setContrast(_contrast_);
  //display.setBias(_bias_);
  //display.display();
  //display.clearDisplay();  // clears the screen and buffer

  pinMode(PIN_SCE, OUTPUT);
  pinMode(PIN_RESET, OUTPUT);
  pinMode(PIN_DC, OUTPUT);
  pinMode(PIN_SDIN, OUTPUT);
  pinMode(PIN_SCLK, OUTPUT);
  digitalWrite(PIN_RESET, LOW);
  digitalWrite(PIN_RESET, HIGH);
  LcdWrite(LCD_C, 0x21 );  // LCD Extended Commands.
  //LcdWrite(LCD_C, 0xBA );  // Set LCD Vop (Contrast). Здесь константа в оригинале была B1 (c)flanker 
  LcdWrite(LCD_C, _contrast_ );
  LcdWrite(LCD_C, 0x04 );  // Set Temp coefficent. //0x04
  //LcdWrite(LCD_C, 0x14 );  // LCD bias mode 1:48. //0x13
  LcdWrite(LCD_C, _bias_ );
  LcdWrite(LCD_C, 0x20 );  // LCD Basic Commands
  LcdWrite(LCD_C, 0x0C );  // LCD in normal mode.
}

void pcd8544_set_cursor(uint8_t x, uint8_t y) {
    x=x%12; y=y%6;
    pcd8544_send(LCD_C, 0x40+y);
    pcd8544_send(LCD_C, 0x80+x*7);
} 




//==========================================================================


void screen_clear() {
  //display.clearDisplay();  // clears the screen and buffer

  for (int index = 0; index < LCD_X * LCD_Y / 8; index++)
  {
    LcdWrite(LCD_D, 0x00);
  }
}

void screen_logo() {

  
}

void showscreen() {
  //display.clearDisplay();
  //display.setTextSize(1);
  //display.setTextColor(BLACK);
  //display.println("test");
  //display.display();
  
  
  //LcdClear();
  screen_clear();
  LcdString("Hello World!");
  
  delay(2000);

  
  

  
}

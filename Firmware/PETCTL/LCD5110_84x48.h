/*
Конвертор изображений 
http://javl.github.io/image2cpp/

Графический редактор
https://www.photopea.com/


*/

#define ShowScreen_period 300  // период обновления экрана в мс

#define _contrast_ 0x25
#define _bias_ 0x4

#define _lcd_RST A1
#define _lcd_CE A2
#define _lcd_DC A3
#define _lcd_DIN A4
#define _lcd_CLK A5

#define line_spacing 12
#define start_line 2
#define str_1 start_line
#define str_2 start_line + line_spacing
#define str_3 start_line + (line_spacing*2)
#define str_4 start_line + (line_spacing*3)

// #define str_1 0
// #define str_2 12
// #define str_3 24
// #define str_4 36


#include <SPI.h>
#include <Wire.h>
//#include "src/pcd8544.h"

#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

Adafruit_PCD8544 display = Adafruit_PCD8544(_lcd_CLK, _lcd_DIN, _lcd_DC, _lcd_CE, _lcd_RST);

// 'ok_01', 8x8px
const unsigned char epd_bitmap_ok_01 [] PROGMEM = {
	0x00, 0x00, 0x02, 0x06, 0x8c, 0xd8, 0x70, 0x20
};

// 'degree_celsius', 4x4px
const unsigned char epd_bitmap_degree_celsius [] PROGMEM = {
	0x60, 0x90, 0x90, 0x60
};







void screen_Init() {
  display.begin();
  display.setContrast(_contrast_);
  display.setBias(_bias_);
  display.display();
  display.clearDisplay();  // clears the screen and buffer

  //pcd8544_Init();
}






//==========================================================================


void screen_clear() {
  display.clearDisplay();  // clears the screen and buffer

 //pcd8544_clear();
}

void screen_logo() {

  
}

void showscreen() {

  curTemp = 232.345;
  targetTemp = 250;
  float tmpspeed = 11.4;
  float tmpMilage = 10.324;



  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(BLACK);

  display.drawBitmap(0, str_1,  epd_bitmap_ok_01, 8, 8, BLACK); // OK температура
  display.setCursor(9, str_1);
  display.print(curTemp,1);
  display.setCursor(60, str_1);
  display.print(targetTemp,0);

  display.drawBitmap(42, str_1,  epd_bitmap_degree_celsius, 4, 4, BLACK); // градус сельсия
  display.setCursor(48, str_1);
  display.print("C");

  display.drawBitmap(0, str_2,  epd_bitmap_ok_01, 8, 8, BLACK); // OK температура
  display.setCursor(9, str_2);
  display.print(tmpspeed,1);

  //display.setCursor(42, 9);
  display.print(" mm/s");

  // tmpMilage
  display.setCursor(3, str_3);
  display.print(tmpMilage,1);
  display.print(" meters");

  display.drawBitmap(0, str_4,  epd_bitmap_ok_01, 8, 8, BLACK); 
  display.setCursor(9, str_4);
  display.print("Ext. Load");






  // oled.print(tmpspeed, 1);


  //display.setTextColor(WHITE, BLACK);

  //display.setCursor(8, 8);


  


   
  


  // градус сельсия
  // epd_bitmap_degree_celsius



  display.display();

  // 'lamp', 10x14px
//const unsigned char epd_bitmap_lamp
  
  delay(2000);

  
  

  
}

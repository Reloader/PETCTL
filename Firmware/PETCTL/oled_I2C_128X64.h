

#define USE_SCREEN_BUFFER
#define ShowScreen_period 500  // период обновления экрана в мс

#include "src/GyverOLED.h"
#ifdef USE_SCREEN_BUFFER
GyverOLED<SSD1306_128x64, OLED_BUFFER> oled;
#else
GyverOLED<SSD1306_128x64, OLED_NO_BUFFER> oled;
#endif


void screen_Init() {
  oled.init();              // инициализация
  // ускорим вывод, ВЫЗЫВАТЬ ПОСЛЕ oled.init()!!!
  Wire.setClock(400000L);   // макс. 800'000
  oled.clear();
  #ifdef USE_SCREEN_BUFFER
  oled.update();
  #endif
}


void screen_logo() {

  oled.setScale(3);
  oled.setCursor(13, 2);
  oled.println("PETCTL");
  oled.setScale(1);
  oled.setCursor(13, 6);
  oled.print(_developer_);
  oled.setCursor(20, 7);
  oled.print("mvb   V ");
  oled.print(_version_);
  #ifdef USE_SCREEN_BUFFER
  oled.update();
  #endif

}


void screen_clear() {
  oled.clear();
  #ifdef USE_SCREEN_BUFFER
  oled.update();
  #endif
}

// процедура выводит на экран всю информацию
// displays all the information
void showscreen() {

  
  // для отладки!!
  curTemp = 120;
  targetTemp = 140;





  oled.setScale(2); 
  oled.setCursorXY(1,5);

  oled.invertText(true);
  oled.print("*");
  oled.invertText(false);
  // Символ стрелки вправо
  

  oled.setCursorXY(12,5);
  oled.print(">");

  oled.setCursorXY(24,5);
  oled.setScale(2); 
  oled.print(curTemp,0);

  oled.setScale(1);
  oled.setCursorXY(74,5);
  oled.println("*C");

  oled.setCursorXY(86,5);
  oled.setScale(2); 
  oled.print(targetTemp,0);


  oled.setCursorXY(75,5+5+16);
  oled.setScale(1);
  oled.println("mm/s");
  oled.setCursorXY(78,5+5+5+5+32);
  oled.println("m");

  #ifdef USE_SCREEN_BUFFER
  oled.update();
  #endif
}

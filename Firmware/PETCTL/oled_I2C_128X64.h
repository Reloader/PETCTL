

//#define USE_SCREEN_BUFFER
#define ShowScreen_period 500  // период обновления экрана в мс

#include "src/GyverOLED.h"
//#ifdef USE_SCREEN_BUFFER
GyverOLED<SSD1306_128x64, OLED_BUFFER> oled;
//#else
//GyverOLED<SSD1306_128x64, OLED_NO_BUFFER> oled;
//#endif


void screen_Init() {
  oled.init();              // инициализация
  // ускорим вывод, ВЫЗЫВАТЬ ПОСЛЕ oled.init()!!!
  Wire.setClock(400000L);   // макс. 800'000
  oled.clear();
  //#ifdef USE_SCREEN_BUFFER
  oled.update();
  //#endif
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
  //#ifdef USE_SCREEN_BUFFER
  oled.update();
  //#endif
}

// процедура выводит на экран всю информацию
// displays all the information
void showscreen() {
  //#ifdef USE_SCREEN_BUFFER
    oled.clear();
  //#endif

  //float tmpspeed = (float)SpeedX10 / 10; // motor speed
  //float tmpMilage = stepper.getCurrentDeg() * REDCONST; // Milage

  
  // для отладки!!
  curTemp = 220;
  targetTemp = 240;
  float tmpspeed = 11.3;
  float tmpMilage = 13.3;






  oled.setScale(2); 
  oled.setCursorXY(1,5);

  //oled.invertText(true);
  oled.print("*");
  //oled.invertText(false);
  
  //oled.setCursorXY(12,5);
  //oled.print(">");

  oled.setCursorXY(24,5);
  oled.setScale(2); 
  oled.print(curTemp,0);

  oled.setScale(1);
  oled.setCursorXY(66,7);
  oled.println("*C");

  oled.setCursorXY(86,5);
  oled.setScale(2); 
  oled.print(targetTemp,0);

  //Вывод скорости
  oled.setCursorXY(24,26);
  oled.setScale(2); 
  oled.print(tmpspeed,1);
  int x_meterers = 64;
  if (tmpspeed > 10) x_meterers = x_meterers +10;
  oled.setCursorXY(x_meterers,30);
  //oled.setCursorXY(78,5+5+20);
  oled.setScale(1);
  oled.println("mm/s");

  // вывод пробега
  oled.setCursorXY(1,47);
  oled.setScale(2); 
  oled.print(tmpMilage,1);
  x_meterers = 44;
  if (tmpMilage > 10) x_meterers = x_meterers +10;
  if (tmpMilage > 100) x_meterers = x_meterers +10;

  oled.setScale(1);
  oled.setCursorXY(x_meterers,52);
  //oled.setCursorXY(64,5+5+5+5+32);
  oled.println("m");

  // Вывод индикатора внешней нагрузки
  oled.setScale(2); 
  oled.setCursorXY(104,47);
  oled.print("*");
  oled.setCursorXY(118,47);
  oled.invertText(true);
  oled.print("l");
  oled.invertText(false);

  //#ifdef USE_SCREEN_BUFFER
  oled.update();
  //#endif
}

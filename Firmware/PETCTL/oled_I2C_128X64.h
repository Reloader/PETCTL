

#define ShowScreen_period 300  // период обновления экрана в мс
#include "GyverOLED.h"
GyverOLED<SSD1306_128x64, OLED_NO_BUFFER> oled;




void screen_Init() {
  oled.init();              // инициализация
  // ускорим вывод, ВЫЗЫВАТЬ ПОСЛЕ oled.init()!!!
  Wire.setClock(400000L);   // макс. 800'000
  oled.clear();
}




void screen_logo() {

  oled.setScale(3);
  oled.setCursor(13, 2);
  oled.println("PETCTL");
  oled.setScale(1);
  oled.setCursor(20, 7);
  oled.print("mvb   V 0.11");

}


void screen_clear() {

}

// процедура выводит на экран всю информацию
// displays all the information
void showscreen() {

}

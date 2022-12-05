#define ShowScreen_period 300  // период обновления экрана в мс
// первичное тестирование https://wokwi.com/

// Картинки созданы в онлайн графическом редакторе  https://www.photopea.com/
// Конвертированы в онлайн конверторе https://javl.github.io/image2cpp/
/* Настройки : 
Code output format : plain bytes
Draw mode: Vertical - 1 bit per pixel
*/

const uint8_t bitmap_checked[] PROGMEM = {
  0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xc0, 0xe0, 0x70, 0x03, 0x07, 0x0e, 0x1c, 0x0e, 0x07,
  0x03, 0x01, 0x00, 0x00
};

const uint8_t bitmap_half[] PROGMEM = {
  // 'half', 10x14px
  0x00, 0x80, 0x40, 0x20, 0x90, 0xc8, 0xc4, 0xc2, 0x81, 0x00, 0x01, 0x00, 0x00, 0x00, 0x20, 0x30,
  0x38, 0x2c, 0x27, 0x00
};

const uint8_t bitmap_lamp[] PROGMEM = {
  // 'lamp', 10x14px
  0x00, 0x38, 0x44, 0x92, 0x12, 0x12, 0x92, 0x44, 0x38, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x3c, 0x3c,
  0x1f, 0x00, 0x00, 0x00
};

const uint8_t bitmap_lamp_inv[] PROGMEM = {
  // 'lamp_inv', 10x14px
  0xff, 0xc7, 0xbb, 0x6d, 0xed, 0xed, 0x6d, 0xbb, 0xc7, 0xff, 0x3f, 0x3f, 0x3f, 0x20, 0x03, 0x03,
  0x20, 0x3f, 0x3f, 0x3f
};



#include "src/GyverOLED.h"
GyverOLED<SSD1306_128x64, OLED_BUFFER> oled;


boolean symbol_enable = true;

void screen_Init() {
  oled.init();  // инициализация
  // ускорим вывод, ВЫЗЫВАТЬ ПОСЛЕ oled.init()!!!
  Wire.setClock(400000L);  // макс. 800'000
  oled.clear();
  oled.update();
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
  oled.update();
}

// Вызывается по таймеру на миллс и используется для генерации анимации
void screen_logic() {
  symbol_enable = !symbol_enable;
}

// процедура выводит на экран всю информацию
// displays all the information
void showscreen() {
  screen_logic();  // анимация
  oled.clear();

  // Блок обработки ошибок
  if (ErrorStatus == OVERHEAT || ErrorStatus == THERMISTOR_ERROR) {

    oled.setScale(2);

    if (ErrorStatus == OVERHEAT) {
      oled.setCursorXY(20, 26);
      oled.print("OVERHEAT");
    } else {
      oled.setCursorXY(2, 12);
      oled.print("THERMISTOR");
      oled.setCursorXY(35, 40);
      oled.print("ERROR");
    }

  } else {

    float tmpspeed = (float)SpeedX10 / 10;                 // motor speed
    float tmpMilage = stepper.getCurrentDeg() * REDCONST;  // Milage

    // вывод символа наличие/отсутствие нагрева
    // display of the symbol presence / absence of heating
    if (Heat || ErrorStatus == ENDSTOP_TAPE || ErrorStatus == ENDSTOP_FILAMENT) {
      if (Heat || ErrorStatus == ENDSTOP_TAPE) {
        oled.drawBitmap(1, 5, bitmap_checked, 10, 14);
      } else {
        // Вывод символы остановки
        oled.setScale(2);
        oled.setCursorXY(1, 5);
        oled.print("X");
      }
    }


    oled.setCursorXY(24, 5);
    oled.setScale(2);
    oled.print(curTemp, 0);

    oled.setScale(1);
    oled.setCursorXY(66, 7);
    oled.println("*C");

    oled.setCursorXY(86, 5);
    oled.setScale(2);

    // вывод символа отметки управления температурой
    // temperature control label symbol output
    if (MenuMode == MENU_MODE_TEMP) {
      oled.invertText(true);
      oled.print(targetTemp, 0);
      oled.invertText(false);
    } else {
      oled.print(targetTemp, 0);
    }


    // индикация вращения мотора
    // motor rotation indication
    if (runMotor || ErrorStatus == ENDSTOP_TAPE || ErrorStatus == ENDSTOP_FILAMENT) {
      if (ErrorStatus == ENDSTOP_TAPE) {
        if (symbol_enable == true) {
          oled.drawBitmap(1, 26, bitmap_half, 10, 14);
        }
      } else if (ErrorStatus == ENDSTOP_FILAMENT) {
        oled.setScale(2);
        oled.setCursorXY(1, 26);
        oled.print("X");
      } else {
        oled.drawBitmap(1, 26, bitmap_checked, 10, 14);
      }
    }

    // вывод символа отметки управления мотором
    // motor control mark symbol output
    oled.setCursorXY(24, 26);
    oled.setScale(2);
    if (MenuMode == MENU_MODE_MOTOR) {
      oled.invertText(true);
      oled.print(tmpspeed, 1);
      oled.invertText(false);
    } else {
      oled.print(tmpspeed, 1);
    }


    int x_meterers = 64;
    if (tmpspeed > 9.9) x_meterers = x_meterers + 10;
    oled.setCursorXY(x_meterers, 30);
    oled.setScale(1);
    oled.println("mm/s");

    // вывод пробега
    oled.setCursorXY(1, 47);
    oled.setScale(2);
    oled.print(tmpMilage, 1);
    x_meterers = 44;
    if (tmpMilage > 10) x_meterers = x_meterers + 10;
    if (tmpMilage > 100) x_meterers = x_meterers + 10;

    oled.setScale(1);
    oled.setCursorXY(x_meterers, 52);
    oled.println("m");

// Вывод индикатора внешней нагрузки
// ******************************************************
#if defined(_externalLoad_)
    // управление нагрузкой
    // load management
    if (MenuMode == MENU_MODE_LOAD) {
      oled.drawBitmap(118, 47, bitmap_lamp_inv, 10, 14);
    } else {
      oled.drawBitmap(118, 47, bitmap_lamp, 10, 14);
    }

    if (loadEnable == true || ErrorStatus == ENDSTOP_TAPE || ErrorStatus == ENDSTOP_FILAMENT) {
      if (loadEnable == true || ErrorStatus == ENDSTOP_TAPE) {
        oled.drawBitmap(104, 47, bitmap_checked, 10, 14);
      } else {
        oled.setScale(2);
        oled.setCursorXY(104, 47);
        oled.print("X");
      }
    }
#endif
  }
  oled.update();
}


// connecting screen 1602 with i2c module
// подключение экрана 1602 с i2c модулем
#define _LCD_1602_adress_ 0x3F // адрес экрана на шине

#define ShowScreen_period 300  // период обновления экрана в мс

int motor_icon = 0; // Номер символа иконки вращения мотора

#include "Wire.h"
#include "src/LiquidCrystal_I2C.h"
LiquidCrystal_I2C lcd(_LCD_1602_adress_, 16, 2);

// additional symbols for the screen 1602
// Дополнительные символы для экрана
void add_Symbols() {
  // Символы генерировались с помощью онлайн генератора
  // https://www.quinapalus.com/hd44780udg.html


  // Символ стрелки вправо
  byte customChar[8] = {0, 8, 12, 14, 12, 8, 0, 0};
  lcd.createChar(0, customChar);

  // символ для выключенного состояния
  byte customChar1[8] = {0, 14, 17, 17, 17, 14, 0, 0};
  lcd.createChar(1, customChar1);

  // Символ включенного состояния 1
  byte customChar2[8] = {0, 14, 31, 31, 31, 14, 0, 0};
  lcd.createChar(2, customChar2);

  // Символ включенного состояния 2
  byte customChar3[8] = {0, 14, 17, 21, 17, 14, 0, 0};
  lcd.createChar(3, customChar3);

  // Символ обрыва ленты
  byte customChar4[8] = { 0, 1, 3, 6, 12, 24, 16, 0};
  lcd.createChar(4, customChar4);

  // Символ исполнительного устройства
  //byte customChar5[8] = { 28, 2, 1, 5, 9, 30, 8, 4};
  byte customChar5[8] = { 0, 14, 21, 31, 21, 14, 0};
  lcd.createChar(5, customChar5);


}

// Вызывается по таймеру на миллс и используется для генерации анимации
void screen_logic(){

  // анимация вращения мотора
    motor_icon++;
    if (motor_icon > 3) motor_icon = 0;
}

void screen_Init() {
  lcd.init();
  lcd.backlight();
  add_Symbols();
}

void screen_logo() {
  lcd.clear();
  lcd.setCursor (0, 0);
  lcd.print(" PETCTL V 0.11");
  lcd.setCursor (0, 1);
  lcd.print("RELOADER ELECTRO");
}

void screen_clear() {
  lcd.clear();
}


// процедура вывода анимации активности пункта
void activeItem() {
  if (ErrorStatus == ENDSTOP_FILAMENT) {
    lcd.print("X");
  } else {
    switch (motor_icon) {
      case 0:
        lcd.print(char(2));
        break;
      case 1:
        lcd.print(char(2));
        break;
      case 2:
        lcd.print(char(3));
        break;
      case 3:
        lcd.print(char(3));
        break;
      default:
        lcd.print(char(42));
    }
  }
}


// Процедура выводит символ для анимации движения мотора
void motorSymbol() {
  if (ErrorStatus == ENDSTOP_FILAMENT) {
    lcd.print("X");
  } else {
    switch (motor_icon) {
      case 0:
        lcd.print(char(4));
        break;
      case 1:
        lcd.print(char(4));
        break;
      case 2:
        lcd.print(" ");
        break;
      case 3:
        lcd.print(" ");
        break;
      default:
        lcd.print(char(42));
    }
  }
}

// процедура выводит на экран всю информацию
// displays all the information
void showscreen() {

  screen_logic(); // анимация

  // Блок обработки ошибок
  if (ErrorStatus == OVERHEAT ||  ErrorStatus == THERMISTOR_ERROR ) {
    lcd.clear();
    lcd.setCursor (0, 0);
    lcd.print("   -= ERROR =-");
    lcd.setCursor (0, 1);

    if (ErrorStatus == OVERHEAT) {
      lcd.print(" -= OVERHEAT =-");
    } else {
      lcd.print("THERMISTOR ERROR");
    }

  } else {

    float tmpspeed = (float)SpeedX10 / 10; // motor speed
    float tmpMilage = stepper.getCurrentDeg() * REDCONST; // Milage

    lcd.setCursor (0, 0);

    // вывод символа наличие/отсутствие нагрева
    // display of the symbol presence / absence of heating
    if (Heat || ErrorStatus == ENDSTOP_TAPE || ErrorStatus == ENDSTOP_FILAMENT) {
      //lcd.print(char(42));
      activeItem();
    } else {
      //lcd.print(char(46));
      lcd.print(char(1));
    }

    // вывод символа отметки управления температурой
    // temperature control label symbol output
    if (MenuMode == MENU_MODE_TEMP) {
      lcd.print(char(0));
    } else {
      lcd.print(" ");
    }

    // вывод текущей температуры
    // output of the current temperature
    lcd.print(curTemp, 0);
    if (curTemp < 10) lcd.print(" ");
    if (curTemp < 100) lcd.print(" ");

    lcd.setCursor (6, 0);
    //lcd.print("<");
    lcd.print("t");

    // Вывод целевой температуры
    // Target temperature output
    lcd.setCursor (8, 0);
    lcd.print(targetTemp, 0);
    if (targetTemp < 10) lcd.print(" ");
    if (targetTemp < 100) lcd.print(" ");

#if defined(_externalLoad_)
    // управление нагрузкой
    // load management
    lcd.setCursor (12, 0);
    if (MenuMode == MENU_MODE_LOAD) {
      //lcd.print(char(126));
      lcd.print(char(0));
    } else {
      lcd.print(" ");
    }

    lcd.print(char(5));
    lcd.print(" ");

    if (loadEnable == true || ErrorStatus == ENDSTOP_TAPE || ErrorStatus == ENDSTOP_FILAMENT) {
      activeItem();
    } else {
      lcd.print(char(1));
    }
#endif

    // индикация вращения мотора
    // motor rotation indication
    lcd.setCursor (0, 1);
    if (runMotor || ErrorStatus == ENDSTOP_TAPE || ErrorStatus == ENDSTOP_FILAMENT) {
      if (ErrorStatus == ENDSTOP_TAPE) {
        // вывод символа окончания ленты
        motorSymbol();
      } else {
        activeItem();
      }
    } else {

      lcd.print(char(1));
    }

    // вывод символа отметки управления мотором
    // motor control mark symbol output
    if (MenuMode == MENU_MODE_MOTOR) {
      //lcd.print(char(126));
      lcd.print(char(0));
    } else {
      lcd.print(" ");
    }

    // Установленная скорость мотора
    // current motor speed

    lcd.print(tmpspeed, 1);
    lcd.print(" ");

    // отображение пробега
    // show mileage
    if (tmpMilage > 100) {
      lcd.setCursor (9, 1);
    } else if (tmpMilage > 10) {
      lcd.setCursor (10, 1);
    } else {
      lcd.setCursor (11, 1);
    }
    lcd.print(tmpMilage, 2);
    lcd.print("m");
  }
}

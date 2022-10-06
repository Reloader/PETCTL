// connecting screen 1602 with i2c module
// подключение экрана 1602 с i2c модулем
#define _LCD_1602_adress_ 0x3F // адрес экрана на шине


#include "Wire.h"
#include "src/LiquidCrystal_I2C.h"
LiquidCrystal_I2C lcd(_LCD_1602_adress_, 16, 2);

// additional symbols for the screen 1602
// Дополнительные символы для экрана
void add_Symbols(){
  // for the future
  // на будущее
}

void screen_Init(){
  lcd.init();
  lcd.backlight();
  add_Symbols();
}

void screen_logo(){
  lcd.clear();
  lcd.setCursor (0, 0);
  lcd.print(" PETCTL V 0.11");
  lcd.setCursor (0, 1);
  lcd.print("RELOADER ELECTRO");
}

void screen_clear(){
  lcd.clear(); 
}

// процедура выводит на экран всю информацию
// displays all the information
void showscreen() { 

float tmpspeed = (float)SpeedX10 / 10; // motor speed
float tmpMilage = stepper.getCurrentDeg()*REDCONST; // Milage

    lcd.setCursor (0, 0); 

    // вывод символа наличие/отсутствие нагрева
    // display of the symbol presence / absence of heating
    if (Heat){
        lcd.print(char(42));
    }else{
        lcd.print(char(46));  
    }

    // вывод символа отметки управления температурой
    // temperature control label symbol output
    if (MenuMode == MENU_MODE_TEMP){
      lcd.print(char(126));   
    }else{
      lcd.print(" "); 
    }

    // вывод текущей температуры
    // output of the current temperature
    lcd.print(curTemp, 0); 
    if(curTemp<10) lcd.print(" ");
    if(curTemp<100) lcd.print(" ");

    lcd.setCursor (5, 0);
    lcd.print("<");

    // Вывод целевой температуры
    // Target temperature output
    lcd.setCursor (6, 0); 
    lcd.print(targetTemp, 0); 
    if(targetTemp<10) lcd.print(" ");
    if(targetTemp<100) lcd.print(" ");


    // управление нагрузкой 
    // load management
    lcd.setCursor (10, 0);
    if (MenuMode == MENU_MODE_LOAD){
      lcd.print(char(126));   
    }else{
      lcd.print(" "); 
    }
    lcd.print("Load");

    if (loadEnable==true){
        lcd.print(char(42));
    }else{
        lcd.print(char(46));  
    }

    
    // индикация вращения мотора
    // motor rotation indication
    lcd.setCursor (0, 1); 
    if (runMotor==true){
        lcd.print(char(42));
    }else{
        lcd.print(char(46));  
    }

    // вывод символа отметки управления мотором
    // motor control mark symbol output
    if (MenuMode == MENU_MODE_MOTOR){
      lcd.print(char(126));   
    }else{
      lcd.print(" "); 
    }

    // Установленная скорость мотора 
    // current motor speed
    if (tmpspeed<10){
      lcd.print(tmpspeed, 1);
    }else{
      lcd.print(tmpspeed, 0); 
      lcd.print(" ");  
    }

    // отображение пробега
    // show mileage
    lcd.setCursor (11, 1);
    lcd.print(tmpMilage,2);
}

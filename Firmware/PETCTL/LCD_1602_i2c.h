// подключение экрана 1602 с i2c модулем
#define _LCD_1602_adress_ 0x3F // адрес экрана на шине


#include "Wire.h"
#include "LiquidCrystal_I2C.h"
#include "LiquidCrystal_sybbols.h"
LiquidCrystal_I2C lcd(_LCD_1602_adress_, 16, 2);

// дополнительные символы для экрана 1602
void add_Symbols(){
  // Закладка на будущее
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


void showscreen() { // процедура выводит на экран всю информацию

float tmpspeed = (float)SpeedX10 / 10; // скоросто мотора
float tmpMilage = stepper.getCurrentDeg()*REDCONST; // пробег

    lcd.setCursor (0, 0); 

    // вывод символа наличие/отсутствие нагрева
    if (Heat){
        lcd.print(char(42));
    }else{
        lcd.print(char(46));  
    }

    // вывод символа отметки управления температурой
    if (MenuMode == MENU_MODE_TEMP){
      lcd.print(char(126));   
    }else{
      lcd.print(" "); 
    }

    // вывод текущей температуры
    lcd.print(curTemp, 0); 
    if(curTemp<10) lcd.print(" ");
    if(curTemp<100) lcd.print(" ");

    lcd.setCursor (5, 0);
    lcd.print("<");
    
    lcd.setCursor (6, 0); 
    lcd.print(targetTemp, 0); 
    if(targetTemp<10) lcd.print(" ");
    if(targetTemp<100) lcd.print(" ");


    // управление нагрузкой 
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
    lcd.setCursor (0, 1); 
    if (runMotor==true){
        lcd.print(char(42));
    }else{
        lcd.print(char(46));  
    }

    // вывод символа отметки управления мотором
    if (MenuMode == MENU_MODE_MOTOR){
      lcd.print(char(126));   
    }else{
      lcd.print(" "); 
    }

    // Установленная скорость мотора 
    
    if (tmpspeed<10){
      lcd.print(tmpspeed, 1);
    }else{
      lcd.print(tmpspeed, 0); 
      lcd.print(" ");  
    }

    //lcd.print(" mm/s");


    lcd.setCursor (11, 1);
    lcd.print(tmpMilage,2);
}

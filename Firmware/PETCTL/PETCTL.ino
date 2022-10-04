//  =========== ДОПИЛИТЬ =============
// 1) процедура вывода на экран
// 2) кнопки и энкодер подключить через EncButton
// (https://github.com/GyverLibs/EncButtonhttps://github.com/GyverLibs/EncButton)



#include "PETCTL_cfg.h"
#define SPEED_MAX 10

#define DRIVER_STEP_TIME 6  // меняем задержку на 6 мкс
#include "GyverStepper.h"
GStepper<STEPPER2WIRE> stepper(200 * CFG_STEP_DIV, CFG_STEP_STEP_PIN, CFG_STEP_DIR_PIN, CFG_STEP_EN_PIN);

#include "GyverTimers.h"

#include "EncButton.h"
EncButton<EB_TICK, CFG_ENC_CLK, CFG_ENC_DT, CFG_ENC_SW> enc;  // энкодер управления

EncButton<EB_TICK, CFG_ENDSTOP_PIN> endstop;        // концевик остановкипо окончанию прутка
int value = 0;

// Termistor definition
float prevTemp, curTemp = 0;
float targetTemp = CFG_TEMP_INIT;

float finalLength = 0;

#include "GyverPID.h"
GyverPID regulator(CFG_PID_P, CFG_PID_I, CFG_PID_D, 200);



#define GEAR_RATIO ((float)CFG_RED_G1 * (float)CFG_RED_G2 * (float)CFG_RED_G3)
/*
  Bobin round length
  74 * Pi = 232.478
  232.478 mm - bobin round length
*/
#define BOBIN_ROUND_LENGTH ((float)3.1415926 * (float)CFG_BOBIN_DIAM)
const float REDCONST = BOBIN_ROUND_LENGTH / (360 * GEAR_RATIO * 1000);
long SpeedX10 = (float)CFG_SPEED_INIT * 10;


boolean runMotor = false;
boolean Heat = false;
boolean loadEnable = false;

/* Interactive statuses */


// режимы меню
#define MENU_MODE_NO 0
#define MENU_MODE_TEMP 1
#define MENU_MODE_MOTOR 2
#define MENU_MODE_LOAD 3

int MenuMode = MENU_MODE_TEMP;


int whatToChange = MENU_MODE_TEMP;
unsigned long interactive = millis();

/* Emergency stop reasons */
#define OVERHEAT 1
#define THERMISTOR_ERROR 2



#define ShowScreen_period 300  // период в мс
uint32_t ssp;         // переменная таймера


// Выбор экрана для вывода изображения (нужное раскоментировать)
#include "LCD_1602_i2c.h" // Использование LCD 1602 I2C

// в отдельный файл
//#include "GyverOLED.h"
//GyverOLED<SSD1306_128x64, OLED_NO_BUFFER> oled;

// ==== Прототипы для компилятора =====
void encRotationToValue (long* value, int inc = 1, long minValue = 0, long maxValue = 0);
long mmStoDeg(float mmS);
void beepE();
float getTemp();
void interactiveSet();
boolean isInteractive();
void motorCTL(long setSpeedX10);
void debugTemp(float temp, int out);
//=====================================

void setup() {

#if defined(SERIAL_DEBUG_TEMP) || defined(SERIAL_DEBUG_STEPPER) || defined(SERIAL_DEBUG_TEMP_PID)
  Serial.begin(9600);
#endif //SERIAL_DEBUG_TEMP || SERIAL_DEBUG_STEPPER
#if defined(SERIAL_DEBUG_STEPPER)
  Serial.print("Gear ratio: ");
  Serial.println(GEAR_RATIO);
  Serial.println("[deg/s],\t[step/s],\t[deg],\t[mm/s],\t[deg/s],\t[deg]");
#endif //SERIAL_DEBUG_STEPPER

#if defined(__LGT8F__)
  analogReadResolution(10);
#endif

  pinMode(CFG_SOUND_PIN, OUTPUT);
  pinMode(CFG_ENC_SW, INPUT_PULLUP);


  stepper.setRunMode(KEEP_SPEED);   // режим поддержания скорости
#if defined(CFG_STEP_INVERT)
  stepper.reverse(true);            // reverse direction
#endif //CFG_STEP_INVERT
  stepper.autoPower(true);
  stepper.setAcceleration(300);
  stepper.setSpeedDeg(mmStoDeg((float)SPEED_MAX));
  Timer2.setPeriod(stepper.getMinPeriod() / 2);
  stepper.brake();
  Timer2.enableISR();
  stepper.reset();                  // остановка и сброс позиции в 0

  screen_Init();


#if defined(_setEncReverse_)
  enc.setEncReverse(true);      // true - инвертировать направление энкодера (умолч. false)
#endif
  enc.setHoldTimeout(_encSetHoldTimeout_);     // установить время удержания кнопки, мс (до 8 000)

#if defined(CFG_SOUND_START)
  beepE();
#endif

  screen_logo();

  delay(3000);
  screen_clear();

  regulator.setLimits(0, 255);
  regulator.setpoint = targetTemp;

}

// ======================
// обработчик
ISR(TIMER2_A) {
  //enc1.tick();
  stepper.tick(); // тикаем тут
  //enc.tick();
  //endstop.tick();
}

// ======================
void yield() {
  // костыль, на всякий случай.
  stepper.tick();
}

// ======= !!!! =========
void loop() {
  enc.tick();
  stepper.tick();

  curTemp = getTemp();
  regulator.input = curTemp; // сообщаем регулятору текущую температуру
//  if (Heat == true) {
//    regulator.setpoint = targetTemp;
//  }
  if (millis() - ssp >= ShowScreen_period) {  // ищем разницу
    ssp = millis();                   // сброс таймера
    showscreen(); // Вывод информации на экран
  }


  long newTargetTemp = targetTemp;
  long newSpeedX10 = SpeedX10;
  float rest;


  if (!isInteractive()) MenuMode = MENU_MODE_NO;


  // MenuMode
  if (enc.click()) {
    interactiveSet();
    MenuMode++;
    if (MenuMode > 3) MenuMode = 1;
  }

  if (enc.held()) {
    switch (MenuMode) {
      case 1:
        //Включение/выключение нагрева
        Heat = !Heat;
        if (Heat == true) {
          regulator.setpoint = targetTemp;
        } //else {
          //regulator.setpoint = 0;
        //}
        break;
      case 2:
        //Включание/выключение мотора
        runMotor = !runMotor;
        if (runMotor) {
          motorCTL(SpeedX10);
        } else {
          motorCTL(-1);
        }
        break;
      case 3:
        // включение/выключение дополнительного чего-нибудь
        loadEnable = !loadEnable;
        break;
    }

  }

  if (enc.turn()) { // Обработка поворотов энкодера
    switch (MenuMode) {
      case 1:
        //Регулировка нагрева
        encRotationToValue(&newTargetTemp, 1, CFG_TEMP_MIN, CFG_TEMP_MAX - 10);
        break;
      case 2:
        // регулировка оборотов мотора
        encRotationToValue(&newSpeedX10, 1, 0, SPEED_MAX * 10);
        break;
    }
  }

  if (newSpeedX10 != SpeedX10) {
    SpeedX10 = newSpeedX10;
    if (runMotor) motorCTL(newSpeedX10);        // в градусах/сек
  }


  // Контроль нагрева
  if (newTargetTemp != targetTemp) {
    targetTemp = newTargetTemp;
    regulator.setpoint = targetTemp;
    //printTargetTemp(newTargetTemp);
  }


  if (Heat == true) {
    int pidOut = regulator.getResultTimer();
    analogWrite(CFG_HEATER_PIN, pidOut);
    debugTemp(curTemp, pidOut);
  } else {
    analogWrite(CFG_HEATER_PIN, 0);
    debugTemp(curTemp, 0);
  }



  // ==== старая логика ======
  //    if (enc1.isDouble()) { // режим изменение скорости
  //      whatToChange = CHANGE_SPEED;
  //      interactiveSet();
  //      printTargetTemp(targetTemp); // to clear selection
  //      printSpeed(SpeedX10);
  //    }
  //    if (enc1.isSingle()) { // режим изменение температуры
  //      whatToChange = CHANGE_TEMPERATURE;
  //      interactiveSet();
  //      printSpeed(SpeedX10); // to clear selection
  //      printTargetTemp(targetTemp);
  //    }
  //    if (!isInteractive()) { // просто вывод на экран
  //      whatToChange = CHANGE_NO;
  //      printSpeed(SpeedX10); // to clear selection
  //      printTargetTemp(targetTemp);
  //    }
  //
  //    if( whatToChange == CHANGE_TEMPERATURE) {
  //      encRotationToValue(&newTargetTemp, 1, CFG_TEMP_MIN, CFG_TEMP_MAX - 10);
  //      if (enc1.isHolded()){
  //        Heat = ! Heat;
  //        printHeaterStatus(Heat);
  //      }
  //
  //      if (newTargetTemp != targetTemp) {
  //        targetTemp = newTargetTemp;
  //        regulator.setpoint = newTargetTemp;
  //        printTargetTemp(newTargetTemp);
  //      }
  //    } else if (whatToChange == CHANGE_SPEED) {
  //      encRotationToValue(&newSpeedX10, 1, 0, SPEED_MAX * 10);
  //      if (enc1.isHolded()) {
  //        runMotor = ! runMotor;
  //        if (runMotor) {
  //          motorCTL(newSpeedX10);
  //        } else {
  //          motorCTL(-1);
  //          runMotor = false;
  //        }
  //        interactiveSet();
  //      }
  //      if (newSpeedX10 != SpeedX10) {
  //        SpeedX10 = newSpeedX10;
  //        if (runMotor) motorCTL(newSpeedX10);        // в градусах/сек
  //        printSpeed(newSpeedX10);
  //      }
  //    }
  //    if (runMotor) {
  //      printMilage(stepper.getCurrentDeg());
  //    }
  //
  //    curTemp = getTemp();
  //    stepper.tick();
  //    if (curTemp > CFG_TEMP_MAX - 10) emStop(OVERHEAT);
  //    if (curTemp < -10) emStop(THERMISTOR_ERROR);
  //    regulator.input = curTemp;
  //    if (curTemp != prevTemp) {
  //      prevTemp = curTemp;
  //      printCurrentTemp(curTemp);
  //    }
  //    if (Heat) {
  //      int pidOut = (int) constrain(regulator.getResultTimer(), 0, 255);
  //      analogWrite(CFG_HEATER_PIN, pidOut);
  //      debugTemp(curTemp, pidOut);
  //    } else {
  //      analogWrite(CFG_HEATER_PIN, 0);
  //      debugTemp(curTemp, 0);
  //    }
  //
  //    oled.setCursorXY(90, 47);
  //    if(!digitalRead(CFG_ENDSTOP_PIN)) {
  //      if(!runMotor) {
  //        oled.setScale(2);
  //        oled.println("  *");
  //      } else {
  //        if (finalLength > 0){
  //          rest = finalLength - getMilage();
  //          if (rest >= 0) {
  //            oled.setScale(1);
  //            oled.setCursorXY(90, 55);
  //            oled.println(rest*100,1); // rest in cm
  //            oled.setScale(2);
  //          } else {
  //            runMotor = false;
  //            motorCTL(0);
  //            Heat = false;
  //            printHeaterStatus(Heat);
  //            finalLength = 0;
  //            beepI();
  //          }
  //        } else {
  //          finalLength = getMilage() + CFG_PULL_EXTRA_LENGTH;
  //        }
  //      }
  //    } else {
  //      oled.setScale(2);
  //      oled.println("   ");
  //      finalLength = 0;
  //    }
  //
  //    oled.setCursorXY(112, 24);
  //    if(!digitalRead(CFG_EMENDSTOP_PIN)) {
  //      if(!runMotor) {
  //        oled.setScale(2);
  //        oled.println("X");
  //      } else {
  //        runMotor = false;
  //        motorCTL(-1);
  //        Heat = false;
  //        printHeaterStatus(Heat);
  //        beepI();
  //        beepI();
  //       }
  //    } else {
  //      oled.setScale(2);
  //      oled.println(" ");
  //    }
}
//======================================
// void encoder_handler(){
//      //#define MENU_MODE_TEMP 1
//    //#define MENU_MODE_MOTOR 2
//  if (enc.left()) {
//    if (MenuMode == MENU_MODE_TEMP){
//
//    } else if (MenuMode==MENU_MODE_MOTOR) {
//
//    }
//    //Serial.println("left");     // поворот налево
//  }
//  if (enc.right()) {
//    if (MenuMode == MENU_MODE_TEMP){
//
//    } else if (MenuMode==MENU_MODE_MOTOR) {
//
//    }
//    //Serial.println("right");   // поворот направо
//  }
// }
//======================================
void debugTemp(float temp, int out) {
#if defined(SERIAL_DEBUG_TEMP)
  static long debug_time;
  if (debug_time < millis() ) {
    debug_time = millis() + 200;
    Serial.print(temp);
#if defined(SERIAL_DEBUG_TEMP_PID)
    Serial.print(' ');
    Serial.print(out);
#endif // end SERIAL_DEBUG_TEMP_PID
    Serial.println(' ');
  }
#endif //end SERIAL_DEBUG_TEMP
}

long mmStoDeg(float mmS) {
  return mmS / (REDCONST * 1000);
}

void beepE() {
  digitalWrite(CFG_SOUND_PIN, 1);
  delay(50);
  digitalWrite(CFG_SOUND_PIN, 0);
  delay(50);
}

void beepI() {
  beepE();
  beepE();
}
void beepT() {
  digitalWrite(CFG_SOUND_PIN, 1);
  delay(600);
  digitalWrite(CFG_SOUND_PIN, 0);
  delay(200);
}

void beepO() {
  beepT();
  beepT();
  beepT();
}

void emStop(int reason) {
  runMotor = false;
  motorCTL(-1);
  stepper.disable();
  Heat = false;
  analogWrite(CFG_HEATER_PIN, 0);
  //  oled.clear();
  //  oled.setScale(3);
  //  oled.setCursorXY(0, 2);
  //  oled.println("*HALT!*");
  //  oled.setScale(2);
  //  oled.setCursorXY(3, 40);
  switch (reason) {
    case OVERHEAT:
      //oled.println("Overheat");
      break;
    case THERMISTOR_ERROR:
      //oled.println("Thermistor");
      break;
  }
  for (;;) {
    beepO();
    delay(60000);
  }
}

float getMilage() {
  return stepper.getCurrentDeg() * REDCONST;
}

void motorCTL(long setSpeedX10) {
#if defined(SERIAL_DEBUG_STEPPER)
  Serial.print(stepper.getSpeedDeg());
  Serial.print(",\t");
  Serial.print(stepper.getSpeed());
  Serial.print(",\t");
  Serial.print(stepper.getCurrent());
  Serial.print(",\t");
#endif // SERIAL_DEBUG_STEPPER
  //  oled.setScale(2);
  //  oled.setCursorXY(0, 23);

  if (setSpeedX10 > 0) {
    stepper.setSpeedDeg(mmStoDeg((float)setSpeedX10 / 10), SMOOTH);      // [degree/sec]
    //oled.println("*");
  } else if (setSpeedX10 == 0) {
    stepper.stop();
    //oled.println(".");
  } else {
    stepper.brake();
    //oled.println(".");
  }

#if defined(SERIAL_DEBUG_STEPPER)
  Serial.print((float)setSpeedX10 / 10);
  Serial.print(",\t");
  Serial.print(stepper.getSpeedDeg());
  Serial.print(",\t");
  Serial.print(stepper.getCurrent());
  Serial.println(" ");
#endif // SERIAL_DEBUG_STEPPER

}

void printHeaterStatus(boolean status) {
  //  oled.setCursorXY(0, 0);
  //  if (status)
  //    oled.println("*");
  //  else
  //    oled.println(".");
}

void encRotationToValue (long* value, int inc = 1, long minValue = 0, long maxValue = 0) {

  int tmp_inc = inc;
  if (enc.fast())  tmp_inc = inc * 5;

  if (enc.right()) {
    *value += tmp_inc;
  }

  if (enc.left()) {
    *value -= tmp_inc;
  }

  interactiveSet();

  if (*value < minValue) *value = minValue;
  if (*value > maxValue) *value = maxValue;
}

void printTargetTemp(float t) {
  //  oled.setScale(2);
  //  if (whatToChange == CHANGE_TEMPERATURE)  oled.invertText(true);
  //  oled.setCursorXY(88, 0);
  //  oled.println((int)t, 1);
  //  oled.invertText(false);
}

void printCurrentTemp(float t) {
  //  oled.setScale(2);
  //  oled.setCursorXY(12, 0);
  //  oled.print(t, 1);
  //  if (t < 99.9) oled.print(" "); //clean screen garbage
  //  if (t < 9.9) oled.print(" ");
}

void printSpeed(long s) { // вывод скорости на экран
  //  // s -speed in mm/s * 10
  //  // // pint in mm/s
  //  oled.setScale(2);
  //  oled.setCursorXY(12, 23);
  //  if (whatToChange == CHANGE_SPEED)  oled.invertText(true);
  //  oled.print((float)s / 10, 1);
  //  if (s < 100) oled.print(" "); //fix display garbage
  //  oled.invertText(false);
}

void printMilage(float m) { // вывод пробега на экран
  //  // m - current stepper position in degree
  //  // output to display in meters
  //  oled.setScale(2);
  //  oled.setCursorXY(12, 47);
  //  oled.println(m * REDCONST);
}

void interactiveSet() {
  interactive = millis() + 15000;
}

boolean isInteractive() {
  return millis() < interactive;
}

float getTemp() {
  uint8_t i;
  float average;

  average = analogRead(CFG_TERM_PIN);
  // convert the value to resistance
  average = 1023 / average - 1;
  average = CFG_TERM_SERIAL_R / average;

  float steinhart;
  steinhart = average / CFG_TERM_VALUE;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= CFG_TERM_B_COEFF;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (CFG_TERM_VALUE_TEMP + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert absolute temp to C

  return simpleKalman(steinhart);
}

// Простой “Калман”
// https://alexgyver.ru/lessons/filters/
float _err_measure = 0.8;  // примерный шум измерений
float _q = 0.02;   // скорость изменения значений 0.001-1, варьировать самому

float simpleKalman(float newVal) {
  float _kalman_gain, _current_estimate;
  static float _err_estimate = _err_measure;
  static float _last_estimate;
  _kalman_gain = (float)_err_estimate / (_err_estimate + _err_measure);
  _current_estimate = _last_estimate + (float)_kalman_gain * (newVal - _last_estimate);
  _err_estimate =  (1.0 - _kalman_gain) * _err_estimate + fabs(_last_estimate - _current_estimate) * _q;
  _last_estimate = _current_estimate;
  return _current_estimate;
}

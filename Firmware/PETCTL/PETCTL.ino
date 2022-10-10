//  =========== ДОПИЛИТЬ =============
// 1) контроль автостопа


#include "PETCTL_cfg.h"
#define SPEED_MAX 15

#define DRIVER_STEP_TIME 6  // меняем задержку на 6 мкс
#include "src/GyverStepper.h"
GStepper<STEPPER2WIRE> stepper(200 * CFG_STEP_DIV, CFG_STEP_STEP_PIN, CFG_STEP_DIR_PIN, CFG_STEP_EN_PIN);

#include "src/GyverTimers.h"

#include "src/EncButton.h"
EncButton<EB_TICK, CFG_ENC_CLK, CFG_ENC_DT, CFG_ENC_SW> enc;  // энкодер управления

EncButton<EB_TICK, CFG_ENDSTOP_PIN> endstop;        // концевик остановкипо окончанию прутка
int value = 0;

// Termistor definition
float prevTemp, curTemp = 0;
float targetTemp = CFG_TEMP_INIT;

float finalLength = 0;

#include "src/GyverPID.h"
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

int MenuMode = MENU_MODE_NO;

unsigned long interactive = millis();

/* Emergency stop reasons */
// возможные причины ошибки
#define OVERHEAT 1
#define THERMISTOR_ERROR 2
#define ENDSTOP_FILAMENT 3
#define ENDSTOP_TAPE 4

int ErrorStatus = 0; // Статус ошибки. Если 0 то это нормальная работа

int motor_icon = 0; // Номер символа иконки вращения мотора

#define ShowScreen_period 300  // период в мс
uint32_t ssp;         // переменная таймера


// Выбор экрана для вывода изображения (нужное раскоментировать)
#include "LCD_1602_i2c.h" // Использование LCD 1602 I2C


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

  pinMode(CFG_ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(CFG_EMENDSTOP_PIN, INPUT_PULLUP);

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
  stepper.tick(); // тикаем тут
}



// ======= !!!! =========
void loop() {
  enc.tick();
  stepper.tick();

  curTemp = getTemp();
  regulator.input = curTemp; // сообщаем регулятору текущую температуру

  if (millis() - ssp >= ShowScreen_period) {  // ищем разницу
    ssp = millis();                   // сброс таймера
    showscreen(); // Вывод информации на экран

    // анимация вращения мотора
    motor_icon++;
    if (motor_icon > 3) motor_icon = 0;
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
          // Если была стаботка датчика ленты то при остановке двигателя отключаем его
          if (ErrorStatus == ENDSTOP_TAPE) ErrorStatus = 0;
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
  }


  if (Heat == true) {
    int pidOut = regulator.getResultTimer();
    analogWrite(CFG_HEATER_PIN, pidOut);
    debugTemp(curTemp, pidOut);
  } else {
    analogWrite(CFG_HEATER_PIN, 0);
    debugTemp(curTemp, 0);
  }


  // Обработка ошибок
  // Датчик ленты
    if ((ErrorStatus != ENDSTOP_TAPE) && (digitalRead(CFG_ENDSTOP_PIN) == LOW) && (runMotor)) {
      ErrorStatus = ENDSTOP_TAPE;
      // понизить скорость вращения в 2 раза (!)
      SpeedX10 = SpeedX10 / 2;
      // звeковое сопровождение
      beepI();
    }

  // датчик прутка
  if ((ErrorStatus != ENDSTOP_FILAMENT) && ((digitalRead(CFG_EMENDSTOP_PIN) == LOW))) {
    // остановить нагрев
    // остановить двигатель
    // звуковое сопровождение

  }

  //// возможные причины ошибки
  //#define OVERHEAT 1
  //#define THERMISTOR_ERROR 2
  //#define ENDSTOP_FILAMENT 3
  //#define ENDSTOP_TAPE 4
  //
  //int ErrorStatus = 0; // Статус ошибки. Если 0 то это нормальная работа





  // ==== старая логика ======

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
  switch (reason) {
    case OVERHEAT:
      break;
    case THERMISTOR_ERROR:
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
  if (setSpeedX10 > 0) {
    stepper.setSpeedDeg(mmStoDeg((float)setSpeedX10 / 10), SMOOTH);      // [degree/sec]
  } else if (setSpeedX10 == 0) {
    stepper.stop();
  } else {
    stepper.brake();
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

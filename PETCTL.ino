#include "PETCTL_cfg.h"
#define SPEED_MAX 10

#define DRIVER_STEP_TIME 6  // меняем задержку на 6 мкс
#include "GyverStepper.h"
GStepper<STEPPER2WIRE> stepper(200 * CFG_STEP_DIV, CFG_STEP_STEP_PIN, CFG_STEP_DIR_PIN, CFG_STEP_EN_PIN);

#include "GyverTimers.h"

#include "GyverOLED.h"
GyverOLED<SSD1306_128x64, OLED_NO_BUFFER> oled;

#define CLK CFG_ENC_CLK
#define DT CFG_ENC_DT
#define SW CFG_ENC_SW
#include "GyverEncoder.h"
Encoder enc1(CLK, DT, SW);
int value = 0;

// Termistor definition
float prevTemp, curTemp = 0;
float targetTemp = CFG_TEMP_INIT;

// which analog pin to connect
#define THERMISTORPIN CFG_TERM_PIN         
// resistance at 25 degrees C
#define THERMISTORNOMINAL 100000      
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL CFG_TERM_VALUE_TEMP   
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT CFG_TERM_B_COEFF
// the value of the 'other' resistor
#define SERIESRESISTOR CFG_TERM_SERIAL_R    

// End stop pin
#define ENDSTOP CFG_ENDSTOP_PIN
// Extra length to pull after end stop triggered (in m)
#define EXTRA_LENGTH CFG_PULL_EXTRA_LENGTH
float finalLength = 0;

#include "GyverPID.h"
GyverPID regulator(CFG_PID_P, CFG_PID_I, CFG_PID_D, 200);

#define HEATER_PIN CFG_HEATER_PIN
boolean Heat = false;

/*
Reductor constant ~ 4.69624E-6 m/deg (length to stepper rotation degree)
 ((36/8) * (36/8) * (55/8)) = 139.21875
 139.21875 - gear ratio for RobertSa reductor
 ((36/8) * (55/8)) = 30.9375
 30.9375 - gear ratio for Zneipas reductor
*/
#define GEAR_RATIO ((float)CFG_RED_G1 * (float)CFG_RED_G2 * (float)CFG_RED_G3)
/*
Bobin round length
 74 * Pi = 232.478
 232.478 mm - bobin round length
*/
#define BOBIN_ROUND_LENGTH ((float)3.1415926 * (float)CFG_BOBIN_DIAM)
const float REDCONST = BOBIN_ROUND_LENGTH /(360 * GEAR_RATIO * 1000);
//const float REDCONST = 232.478 /(360 * 139.21875 * 1000);
boolean runMotor=false;
//long Speed = (float)CFG_SPEED_INIT/(REDCONST * 1000); // 539 degree/sec for 2.5 mm/s speed
long SpeedX10 = (float)CFG_SPEED_INIT * 10;

/* Interactive statuses */
#define CHANGE_NO 0
#define CHANGE_TEMPERATURE 1
#define CHANGE_SPEED 2
int whatToChange = CHANGE_NO;
unsigned long interactive = millis();

/* Emergency stop reasons */
#define OVERHEAT 1
#define THERMISTOR_ERROR 2

void encRotationToValue (long* value, int inc = 1, long minValue = 0, long maxValue = 0);

void setup() {
#if defined(SERIAL_DEBUG)
  Serial.begin(9600);
#endif //SERIAL_DEBUG
  pinMode(ENDSTOP, INPUT_PULLUP);
  pinMode(CFG_SOUND_PIN, OUTPUT);
  
  stepper.disable();
  // установка макс. скорости в градусах/сек
  stepper.setMaxSpeedDeg(mmStoDeg((float)SPEED_MAX));
  // установка ускорения в шагах/сек/сек
  stepper.setAcceleration(200);
  // настраиваем прерывания с периодом, при котором 
  // система сможет обеспечить максимальную скорость мотора.
  // Для большей плавности лучше лучше взять период чуть меньше, например в два раза
  Timer2.setPeriod(stepper.getMinPeriod() / 1.2);
  // взводим прерывание
  Timer2.enableISR();
  stepper.setRunMode(KEEP_SPEED);   // режим поддержания скорости
  stepper.reverse(true);            // reverse direction
  stepper.reset();                  // остановка и сброс позиции в 0

  oled.init();              // инициализация
  // ускорим вывод, ВЫЗЫВАТЬ ПОСЛЕ oled.init()!!!
  Wire.setClock(400000L);   // макс. 800'000
  
 //заставка
#if defined(CFG_SOUND_START)
  beepE();
#endif
  oled.clear();
  oled.setScale(3);
  oled.setCursor(13, 2);
  oled.println("PETCTL");
  oled.setScale(1);
  oled.setCursor(20, 7);
  oled.print("mvb    V 0.8b");
  delay(3000);
 
  oled.clear();
  oled.setScale(1);
  oled.setCursorXY(74,5);
  oled.println("*C");
  oled.setCursorXY(75,5+5+16);
  oled.println("mm/s");
  oled.setCursorXY(78,5+5+5+5+32);
  oled.println("m");

  enc1.setType(CFG_ENC_TYPE);
  enc1.setPinMode(LOW_PULL);

  regulator.setpoint = targetTemp;
  printSpeed(SpeedX10);
  printTargetTemp(targetTemp);
  printMilage(0.0);
}

// обработчик
ISR(TIMER2_A) {
  stepper.tick(); // тикаем тут
  enc1.tick();
}

void loop() {
    enc1.tick();
    //stepper.tick();

    long newTargetTemp = targetTemp;
    long newSpeedX10 = SpeedX10;
    float rest;

    if (enc1.isDouble()) {
      whatToChange = CHANGE_SPEED;
      interactiveSet();
      printTargetTemp(targetTemp); // to clear selection
      printSpeed(SpeedX10);
    }
    if (enc1.isSingle()) {
      whatToChange = CHANGE_TEMPERATURE;
      interactiveSet();
      printSpeed(SpeedX10); // to clear selection
      printTargetTemp(targetTemp);
    }
    if (!isInteractive()) {
      whatToChange = CHANGE_NO;
      printSpeed(SpeedX10); // to clear selection
      printTargetTemp(targetTemp);
    }

    if( whatToChange == CHANGE_TEMPERATURE) {
      encRotationToValue(&newTargetTemp, 1, CFG_TEMP_MIN, CFG_TEMP_MAX - 10);
      if (enc1.isHolded()){
        Heat = ! Heat;
        printHeaterStatus(Heat);
      }

      if (newTargetTemp != targetTemp) {
        targetTemp = newTargetTemp;
        regulator.setpoint = newTargetTemp;
        printTargetTemp(newTargetTemp);
      }
    } else if (whatToChange == CHANGE_SPEED) {
      encRotationToValue(&newSpeedX10, 1, -1 * SPEED_MAX * 10, SPEED_MAX * 10);
      if (enc1.isHolded()) {
        runMotor = ! runMotor;
        if (runMotor) {
          motorCTL(newSpeedX10);
        } else {
          motorCTL(0);
          runMotor = false;
        }
        interactiveSet();
      }
      if (newSpeedX10 != SpeedX10) {
        SpeedX10 = newSpeedX10;
        if (runMotor) motorCTL(newSpeedX10);        // в градусах/сек
        printSpeed(newSpeedX10);
      }
    }
    if (runMotor) {
      printMilage(stepper.getCurrentDeg());
    }

    curTemp = getTemp();
    if (curTemp > CFG_TEMP_MAX - 10) emStop(OVERHEAT);
    if (curTemp < -10) emStop(THERMISTOR_ERROR);
    regulator.input = curTemp;
    if (curTemp != prevTemp) {
      prevTemp = curTemp;
      printCurrentTemp(curTemp);
    }
#if defined(SERIAL_DEBUG)
    Serial.print(curTemp);
#endif //end SERIAL_DEBUG
    if (Heat) {
      int pidOut = (int) constrain(regulator.getResultTimer(), 0, 255);
      analogWrite(HEATER_PIN, pidOut);
#if defined(SERIAL_DEBUG)
      Serial.print(' ');
      Serial.print(pidOut);
#endif //end SERIAL_DEBUG
    } else {
      analogWrite(HEATER_PIN, 0);
#if defined(SERIAL_DEBUG)
      Serial.print(' ');
      Serial.print(0);
#endif //end SERIAL_DEBUG
    }
#ifdef SERIAL_DEBUG
    Serial.println(' ');
#endif //end SERIAL_DEBUG

    oled.setCursorXY(90, 47);
    if(!digitalRead(ENDSTOP)) {
      if(!runMotor) {
        oled.setScale(2);
        oled.println("  *");
      } else {
        if (finalLength > 0){
          rest = finalLength - getMilage();
          if (rest >= 0) {
            oled.setScale(1);
            oled.setCursorXY(90, 55);
            oled.println(rest*100,1); // rest in cm
            oled.setScale(2);
          } else {
            runMotor = false;
            motorCTL(0);
            Heat = false;
            printHeaterStatus(Heat);
            finalLength = 0;
            beepI();
          }
        } else {
          finalLength = getMilage() + EXTRA_LENGTH;
        }
      }
    } else {
      oled.setScale(2);
      oled.println("   ");
      finalLength = 0;
    }
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
  motorCTL(0);
  stepper.disable();
  Heat = false;
  analogWrite(HEATER_PIN, 0);
  oled.clear();
  oled.setScale(3);
  oled.setCursorXY(0,2);
  oled.println("*HALT!*");
  oled.setScale(2);
  oled.setCursorXY(3,40);
  switch (reason) {
    case OVERHEAT:
      oled.println("Overheat");
      break;
    case THERMISTOR_ERROR:
      oled.println("Thermistor");
      break;
  }
  for(;;){
    beepO();
    delay(60000);
  }
}
 
float getMilage() {
  return stepper.getCurrentDeg() * REDCONST;
}

void motorCTL(long setSpeedX10) {
  oled.setScale(2);
  oled.setCursorXY(0, 23);
  if (setSpeedX10 != 0) {
    stepper.setSpeedDeg(mmStoDeg((float)setSpeedX10/10), SMOOTH);        // [degree/sec]
    oled.println("*");
  } else {
    stepper.stop();
    stepper.disable();
    oled.println(".");
  }
}

void printHeaterStatus(boolean status) {
  oled.setCursorXY(0, 0);
  if(status) 
    oled.println("*");
  else
    oled.println(".");
}

void encRotationToValue (long* value, int inc = 1, long minValue = 0, long maxValue = 0) {
      if (enc1.isRight()) { *value += inc; interactiveSet(); }     // если был поворот направо, увеличиваем на 1
      if (enc1.isFastR()) { *value += inc * 5; interactiveSet(); }    // если был быстрый поворот направо, увеличиваем на 10
      if (enc1.isLeft())  { *value -= inc; interactiveSet(); }     // если был поворот налево, уменьшаем на 1
      if (enc1.isFastL()) { *value -= inc * 5; interactiveSet(); }    // если был быстрый поворот налево, уменьшаем на на 10
      //if (minValue > 0 && *value < minValue) *value = minValue;
      if (*value < minValue) *value = minValue;
      //if (maxValue > 0 && *value > maxValue) *value = maxValue;
      if (*value > maxValue) *value = maxValue;
}

void printTargetTemp(float t){
      oled.setScale(2);      
      if(whatToChange == CHANGE_TEMPERATURE)  oled.invertText(true);
      oled.setCursorXY(88, 0);
      oled.println((int)t, 1);  
      oled.invertText(false);
}

void printCurrentTemp(float t) {
      oled.setScale(2);      
      oled.setCursorXY(12, 0);
      oled.println(t, 1);   
}

void printSpeed(long s){
      // s -speed in mm/s * 10
      // // pint in mm/s
      oled.setScale(2);      
      oled.setCursorXY(12, 23);
      if(whatToChange == CHANGE_SPEED)  oled.invertText(true);
      //oled.println(s * REDCONST * 1000,2);
      oled.println((float)s/10, 1);  
      oled.invertText(false);
}

void printMilage(float m){
      // m - current stepper position in degree
      // output to display in meters
      oled.setScale(2);
      oled.setCursorXY(12, 47);
      oled.println(m * REDCONST);  
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

  average = analogRead(THERMISTORPIN);
  // convert the value to resistance
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;
  //Serial.print("Thermistor resistance "); 
  //Serial.println(average);
  
  float steinhart;
  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
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

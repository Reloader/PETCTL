// Hardware version 02

#define _setEncReverse_ // Инвертирование энкодера (изменение направления)
#define _encSetHoldTimeout_ 300     // установить время удержания кнопки энкодера, мс (до 8 000)
#define _externalLoad_  //использование внешнего управляемого устройства
#define _beeper_  //использование пищалки
//#define _activ_beeper_ // использование зуммера со встроенным генератором
//====================================================
/*
                                          +-----+
                             +------------| USB |------------+
                             |            +-----+            |
          CFG_SOUND_PIN B5   | [ ]D13/SCK        MISO/D12[ ] |   B4
                             | [ ]3.3V           MOSI/D11[ ]~|   B3
                             | [ ]V.ref     ___    SS/D10[ ]~|   B2 CFG_STEP_EN_PIN
           CFG_TERM_PIN C0   | [ ]A0       / N \       D9[ ]~|   B1 CFG_ENC_SW
                        C1   | [ ]A1      /  A  \      D8[ ] |   B0
           CFG_STEP_DIV C2   | [ ]A2      \  N  /      D7[ ] |   D7 CFG_EMENDSTOP_PIN
             CFG_ENC_DT C3   | [ ]A3       \_0_/       D6[ ]~|   D6 externalLoadPin
                        C4   | [ ]A4/SDA               D5[ ]~|   D5 CFG_HEATER_PIN
                        C5   | [ ]A5/SCL               D4[ ] |   D4 CFG_ENC_CLK
                             | [ ]A6              INT1/D3[ ]~|   D3 CFG_STEP_DIR_PIN
                             | [ ]A7              INT0/D2[ ] |   D2 CFG_STEP_STEP_PIN
                             | [ ]5V                  GND[ ] |     
                        C6   | [ ]RST                 RST[ ] |   C6
                             | [ ]GND   5V MOSI GND   TX1[ ] |   D0
                             | [ ]Vin   [ ] [ ] [ ]   RX1[ ] |   D1
                             |          [ ] [ ] [ ]          |
                             |          MISO SCK RST         |
                             | NANO-V3                       |
                             +-------------------------------+
 */
//====================================================

/*   Stepper driver microstep devision */
#define CFG_STEP_DIV 16
/* Which pin stepper driver STEP pin connected */
#define CFG_STEP_STEP_PIN 2
/* Which pin stepper driver DIR pin connected */
#define CFG_STEP_DIR_PIN 3
/* Which pin stepper driver EN pin connected */
#define CFG_STEP_EN_PIN 10
/* Invert stepper rotation direction (comment out to disable invertion)*/
#define CFG_STEP_INVERT 
/* Which pin encoder CLK pin connected */
#define CFG_ENC_CLK 4
/* Which pin encoder DT pin connected */
#define CFG_ENC_DT 17
/* Which pin encoder SW pin connected */
#define CFG_ENC_SW 9
/* Type of encoder: TYPE1 or TYPE2 */
#define CFG_ENC_TYPE TYPE2
/* Initial target temperature [degree C]*/
#define CFG_TEMP_INIT 180
/* Maximum allowed temperature [degree C], allowed to set to 10 degree less */
#define CFG_TEMP_MAX 290
/* Minimum allowed temperature to set [degree C] */
#define CFG_TEMP_MIN 120
/* Which pin termistor connected to*/
#define CFG_TERM_PIN A0
/* Thermistor resistance at 25 degrees C [Om] */
#define CFG_TERM_VALUE 100000
/* Thermistor temperature for nominal resistance (almost always 25 C) [degree C] */
#define CFG_TERM_VALUE_TEMP 25
/* The beta coefficient of the thermistor (usually 3000-4000) */
#define CFG_TERM_B_COEFF 4388
/* the value of the 'other' resistor [Om] */
#define CFG_TERM_SERIAL_R 10150
/* Which pin endstop connected to  Датчик ленты*/ 
//#define CFG_ENDSTOP_PIN 8
#define CFG_ENDSTOP_PIN 21
/* Extra length to pull after end stop triggered [m] */
#define CFG_PULL_EXTRA_LENGTH 0.07
/* Which pin emergency endstop connected to  Датчик прутка*/
#define CFG_EMENDSTOP_PIN 7
/* PID regulator coefficients */
//PID p: 12.69  PID i: 0.71 PID d: 57.11
#define CFG_PID_P 12.69
#define CFG_PID_I 0.71
#define CFG_PID_D 57.11
/* Which pin heater MOSFET connected to */
#define CFG_HEATER_PIN 5
/* Target filament bobin diameter [mm] */
#define CFG_BOBIN_DIAM 74
/* Initial pull speed [mm/s] */
#define CFG_SPEED_INIT 2.5
/* Buzzer pin connection */
#define CFG_SOUND_PIN 13
/* Enable startup sound (comment to disable).
   Special for GEORGIY (@nehilo011) :) */
#define CFG_SOUND_START

// пин дополнительной внешней нагрузки
#define externalLoadPin 6
/* 
 *  Chouse reductor type. 
 * Only one CFG_RED_RA, CFG_RED_PP1 or CFG_RED_PP2 can be uncomment
 */
/* RobertSa/Anatoly reductor variant (1:139.21875 ratio)*/
//#define CFG_RED_RA
/* PETPull Zneipas classic old reductor variant (1:30.9375 ratio)*/
//#define CFG_RED_PP1
/* PETPull-2 Zneipas reductor variant (1:65.68(18) ratio)*/
#define CFG_RED_PP2

/* === DON'T CHANGE ANYTHING AFTER THIS LINE IF YOU NOT SHURE TO 146% === */

/* Gear ratio for PETPull-2 Zneipas reductor variant */
/* 
  8 teeth gear on stepper shaft interact with
  34 teeth gear of 1-st gear.
  11 teeth of 1-st gear interact with 
  34teeth gear of 2-nd gear.
  11 teeth of 2-nd gear interact with 
  55 teeth of target bobin

  reduction ratio 65.68(18)
*/
#if defined(CFG_RED_PP2)
#define CFG_RED_G1 34/8
#define CFG_RED_G2 34/11
#define CFG_RED_G3 55/11
#endif //CFG_RED_PP2

/* Gear ratio for RobertSa/Anatoly reductor variant */
/* 
  8 teeth gear on stepper shaft interact with
  36 teeth gear of 1-st gear.
  8 teeth of 1-st gear interact with 
  36 teeth gear of 2-nd gear.
  8 teeth of 2-nd gear interact with 
  55 teeth of target bobin

  reduction ratio 139.21875
*/
#if defined(CFG_RED_RA)
#define CFG_RED_G1 36/8
#define CFG_RED_G2 36/8
#define CFG_RED_G3 55/8
#endif //CFG_RED_RA

/* Gear ratio for PetPull Zneipas reductor variant */
/* 
  8 teeth gear on stepper shaft interact with
  36 teeth gear of 1-st gear.
  8 teeth of 1-st gear interact with 
  55 teeth of target bobin
  CFG_RED_G2 1 - to exclude) 2-nd gear

  reduction ratio 30.9375
*/
#if defined(CFG_RED_PP1)
#define CFG_RED_G1 36/8
#define CFG_RED_G2 1
#define CFG_RED_G3 55/8
#endif //CFG_RED_PP1

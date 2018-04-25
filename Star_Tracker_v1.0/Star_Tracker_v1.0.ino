#define DEBUG_SERIAL
#ifdef DEBUG_SERIAL
#define SERIAL_BPS 9600
#endif

// libs y macros ***********************************************************
#define ADMUX_VCCWRT1V1 (_BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1))

#include <Servo.h>
#include <Stepper.h>
#include <Time.h>
#include <Wire.h>
#include <DS3232RTC.h> 

#define rtc_power_pin 11
#define STEPS 2038 // the number of steps in one revolution of your motor (28BYJ-48)
#define ALRM0 0x08          // Alarm register 0
#define ALRM1 0x0b          // Alarm register 1

// constantes siderales **************************************************************
/*
  A sidereal day is defined as the time for the earth to make a complete
  revolution with respect to the sky. This is not the same as a solar day
  as that includes the motion of the earth around the sun, which adds one
  complete revolution per year. Note that we make no attempt to calculate
  the absolute siderial time. Since we need to calibrate the pointer, it
  is sufficient to calculate the relative siderial time (seconds since an
  arbitrary moment, and only within one sidereal day, being one full pointer
  rotation)

  https://en.wikipedia.org/wiki/Sidereal_time states a mean sidereal day is
  23:56:4.0916 solar seconds. A solar day is 86400 solar seconds. Note that
  a sidereal day is 86400 sidereal seconds!x

*/
#define solar_ms_seconds_per_sidereal_day 86164092ull


// constantes de stepper motor ***********************************************
/*

  https://grahamwideman.wikispaces.com/Motors-+28BYJ-48+Stepper+motor+notes

  However, the motors I obtained, were 2048 steps per rotation indeed. Make
  sure you test the motor for at least 10 revolutions. The value for
  solar_ms_seconds_per_step needs to be
  42072ull  for a true 2048 steps per revolution motor
  42281ull  for a 2037.886 steps per revolution motor
  167949ull for a 513.0343 steps per revolution motor
  The calculation is solar_ms_seconds_per_sidereal_day / steps_per_revolution

  We are using a 4 phase sequence. A full revolution corresponds to a full
  sidereal day.
*/

#define solar_ms_seconds_per_step     42072ull

#define motor_pin   5     // motor driver pin (plus next 3)
#define servoPin 9
unsigned long step_delay_ms = solar_ms_seconds_per_step;
int DEC_gr;
unsigned long HA_hs;
unsigned long HA_mm;
unsigned long HA_ss;
Servo servo;  
Stepper stepper(STEPS, 8, 6, 7, 5);

uint8_t mhd[3] = {0, 0, 0};        // minute-hour-day

// setup ***********************************************************************
void setup ()
{
  delay (500);
  Serial.begin (9600);
  Serial.setTimeout(32000);
  //rtcWrite (0, 0);
  set_start_position(180 - 34,0,0,0); //points Celestial South Pole
  for (int n = 0; n <= 3; n++){ // Define los pines 2, 3, 4, 5 como outputs
    pinMode (motor_pin + n, OUTPUT);
  }
  
  pinMode (servoPin, OUTPUT);
  pinMode (rtc_power_pin, OUTPUT);
  
  servo.attach(servoPin);
  power_on ();              // power up peripherals inc Serial

  // show south position to hint proper calibration
  Serial.println ("Indique RA y DEC de la estrella a seguir");
  DEC_gr = getInt ("DEC GRADOS:");
  DEC_gr = (180 - 34) - (90 + DEC_gr); //34 es latitud de BAires
  Serial.println(DEC_gr);
  HA_hs = getInt ("HA hs:");
  HA_mm = getInt ("HA mm:");
  HA_ss = getInt ("HA ss:");
  
  set_start_position(DEC_gr, HA_hs, HA_mm, HA_ss);

}

// loop ************************************************************************
void loop ()
{
    step_to();       // adjust pointer when enough power
}

int getInt (String msg) //reads input
{
  int retval;
  Serial.print (msg);
  retval = Serial.parseInt();
  Serial.print (retval);
  Serial.println ();
  return retval;
}


time_t tmConvert_t(int YYYY, byte MM, byte DD, byte hh, byte mm, byte ss)
{
  tmElements_t tmSet;
  tmSet.Year = YYYY;
  tmSet.Month = MM;
  tmSet.Day = DD;
  tmSet.Hour = hh;
  tmSet.Minute = mm;
  tmSet.Second = ss;
  return makeTime(tmSet); 
}


// rutina de inicio ********************************************************
void set_start_position (int DEC_gr, int HA_hs, int HA_mm, int HA_ss)
{
  int step_position;
  for(int servoAngle = 180; servoAngle >= DEC_gr; servoAngle--)  //move the micro servo from 0 degrees to 180 degrees
  {                      
    servo.write(servoAngle);              
    delay(50);
  }
 
  delay (500);
  step_position = rtcRead (0);

  time_t initTime = tmConvert_t(0,1,1,HA_hs,HA_mm,HA_ss);
 
  //Serial.println("Position: " + step_position);
  
  stepper.setSpeed(6); // 6 rpm
  //stepper.step(-step_position);

  stepper.step(time_to_position(initTime));
  rtcWrite (0, 0);  // save
}

// calculo de pasos a la posicion determinada ***************************************************************

int time_to_position (time_t t)
{
  /*
    t is a 32 bit value in solar seconds since midnight 01-01-1970. Calculate
    milliseconds within the current sidereal day, then calculate steps.
    Note that the ABSOLUTE position is irrelevant, as this same routine is used
    to calibrate the pointer to a known position on a known longitude on a
    known date-time.
  */
 
  uint64_t ms;

  ms = t * 1000ull;               // in ms units and 64 bits

  // Calculation introduces 1 ms cummulative drift per day, so 0.36 seconds
  // per year. As one step corresponds to roughly 40 seconds, drift by this
  // calculation is totally neglectable over the course of several decades
  ms = ms % solar_ms_seconds_per_sidereal_day;  // ms in the sideral day

  // Calculation introduces a non cummulative error of less than 1/200th of a
  // step (0.2 seconds) and can be completely ignored.
  return (int) (ms / solar_ms_seconds_per_step);
}


// stepper motor stuff *********************************************************
void step_to ()
{  
  int step_position;
  
  unsigned long time_ms;

  step_position = rtcRead (0);

  time_ms = millis ();
  time_ms += step_delay_ms;
  
  while ( (long ) (millis () < time_ms)){
    delay(1);
   };
  stepper.setSpeed(1); // 1 rpm
  stepper.step(1);
  step_position++;
  //if (step_position = STEPS) {
  //  stepper.setSpeed(6); // 6 rpm
  //  stepper.step(-STEPS);
  //}
  rtcWrite (0, step_position);  // save
}


void power_on ()
{
  delay (1);                    // let hardware stabilize
#ifdef DEBUG_SERIAL
  Serial.begin (SERIAL_BPS);            // reinitialize serial
#endif
  digitalWrite (A4, HIGH);            // enable pull-up resistors
  digitalWrite (A5, HIGH);            // from the I2C pins
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);   // re-initialize twowire.
  digitalWrite (rtc_power_pin, HIGH);       // power up RTC NVRAM chip
  delay (1);                    // let hardware stabilize
}

// RTC NVRAM routines***********************************************************
/*  The very precise DS3231 has no additional NVRAM, so we are misusing the bcd
  values in the alarm registers for that. Since the less precise DS1307 has
  NVRAM on the locations of those alarm registers, we can simply use the same
  code, meant to stay within bounds of real (bcd) alarm values in the
  respective register positions.

  https://github.com/JChristensen/DS3232RTC

  There are only two register sets (alarm register sets) are available so the
  only valid values for adr are 0 and 1 (non zero). The allowed value range is
  -1000 to 43639. Actual range used is -sop_steps to steps_per-revolution.

  The actual time routines are implemented in the DS3232RTC.h library and no
  additional routenes are needed here.

*/
int rtcRead (int adr)
{
  uint16_t uvalue;
  // read from alarm registers 1 or 2
  RTC.readRTC (adr == 0 ? ALRM0 : ALRM1, mhd, 3);
  //    day in month               hour           min
  uvalue = (mhd[2] - 1) * (60 * 24) + (mhd[1] * 60) + mhd[0];
  // slight negative allowed for slop processing
  return uvalue - 1000;
}

void rtcWrite (int adr, int value)
{
  uint16_t uvalue;
  // slight negative allowed for slop processing
  uvalue = value + 1000;
  mhd[0] = uvalue % 60;             // min
  mhd[1] = (uvalue / 60) % 24;          // hour
  mhd[2] = 1 + (uvalue / (60 * 24));        // day in month
  // write to alarm registers 1 or 2
  RTC.writeRTC (adr == 0 ? ALRM0 : ALRM1, mhd, 3);
}


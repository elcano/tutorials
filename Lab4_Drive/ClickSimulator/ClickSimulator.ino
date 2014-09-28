/*
  Click Simulator
  Given an input of the throttle setting, Compute the speed that the vehicle would be moving
  and produce a click from the speedometer.
  TCF 9/8/14
*/
#define CLICK_PIN 13
#define THROTTLE_BUFFER 20
#define TREND_LENGTH 10
#define THROTTLE_PWM 5
#define ANALOG_THROTTLE_PIN A4
#define SerialMonitor Serial
#define SerialOdoOut  Serial1
#define SerialDrive   Serial2
#define LOOP_TIME_MS 100
#define WHEEL_DIAMETER_MM 397

#include "MsTimer2.h"

int ThrottleHistory[THROTTLE_BUFFER];
int ThrottleIndex = 0;
long ClickTime_ms = 0;
long speed_mmPs = 0;
const int MinimumThrottle = 65;  // Throttle has no effect until 1.2 V
const int FullThrottle =  227;   // A commanded throttle in the 0-255 count range
const int MinAnalogThrottle = 365;  // Result of analogRead() for 1.2V
const int MaxAnalogThrottle = 743;  // Result of analogRead() for 3.63V
const long MaxSpeed_mmPs = 13600;
const long PulseWait_us = 4200;   // PWM base frequency is 490 Hz.
const int  MinPWM_us = 8;         // Mostly low;  PWM out = 0
const int  MaxPWM_us = 2041;      // Mostly high; PWM out = 255
const int WheelCircumference_mm = WHEEL_DIAMETER_MM * PI;
const float DecayRate = 0.9296;
/*----------------------------------------------------------------------------------------------*/
void setup()
{
    int i;
    Serial.begin(115200); // monitor
    ThrottleIndex = 0;
  // On power-up, there is a slight charge on analog in, that goes away after about 2 sec.
    for (i = 0; i < THROTTLE_BUFFER; i++)
    {
         getThrottle();
 //        Serial.println(ThrottleHistory[ThrottleIndex]);
    }
    speed_mmPs = 0;
 
  // enable MakeClick interrupt.
    pinMode(CLICK_PIN, OUTPUT);
    pinMode(ANALOG_THROTTLE_PIN, INPUT);
    digitalWrite(CLICK_PIN, LOW);
    ClickTime_ms = 0;
    MsTimer2::stop();

}
/*----------------------------------------------------------------------------------------------*/
int getPWMThrottle()
{/* read the PWM signal that was output, and compute the signal (0 to 255) that produced it. 
 Currently the throttle produces an analog voltage. Steering and brakes use a PWM signal. */
  unsigned long throttle_us = pulseIn(THROTTLE_PWM, HIGH, PulseWait_us); // may block
  if (throttle_us == 0)
  {  // pulseIn timed out
     return ThrottleHistory[ThrottleIndex];
  }
  throttle_us = min(throttle_us, MaxPWM_us);
  throttle_us = max(throttle_us, MinPWM_us);
  long throttle_PWM = (throttle_us - MinPWM_us) * 255 / (MaxPWM_us - MinPWM_us);
  if (++ThrottleIndex >= THROTTLE_BUFFER)
         ThrottleIndex -= THROTTLE_BUFFER;
  ThrottleHistory[ThrottleIndex] = (int) throttle_PWM;
  return throttle_PWM;
}
/*----------------------------------------------------------------------------------------------*/
int getThrottle()
{  /* read the analog voltage that was output to the motor, and compute the throttle signal (0 to 255) that
      produced it. */
     long throttle = MinimumThrottle;
     unsigned int analog_throttle = analogRead(ANALOG_THROTTLE_PIN);
     delay(2);
     Serial.print(analog_throttle); Serial.print(',');
     /* 0 = 0V; 1023 = 5V.
      FullThrottle (227 / 255) produces 3.63V, for a reading ofMaxAnalogThrottle (743)
      MinimumThrottle (70 / 255) produces 1.2V, for a reading ofMinAnalogThrottle (245)
      These numbers will vary for different E-bike controllers.
     */
     if (analog_throttle > MaxAnalogThrottle)
         throttle = FullThrottle;
     else if (analog_throttle < MinAnalogThrottle)
         throttle = MinimumThrottle;
     else
     {   float x = (float)(analog_throttle - MinAnalogThrottle) * FullThrottle / 
          (MaxAnalogThrottle - MinAnalogThrottle);
         throttle = MinimumThrottle + x;
     }
    if (++ThrottleIndex >= THROTTLE_BUFFER)
         ThrottleIndex -= THROTTLE_BUFFER;
    ThrottleHistory[ThrottleIndex] = (int) throttle;
    return throttle;
}
/*----------------------------------------------------------------------------------------------*/
void ComputeSpeed()
{
  int i, k;
  
  // if accelerating or steady, set speed by average throttle over previous 0.3 to 1.1 sec.
   long throttlePos = 0, speedPos_mmPs = 0;
   i = (ThrottleIndex - 1100/LOOP_TIME_MS);
   while (i < 0)   // One might expect 0 <= i < THROTTLE_BUFFER, but i can be < 0.
       i += THROTTLE_BUFFER;
   /* Use average throttle over last 0.3 sec to 1.1 sec */
   for (k = 0; k <  (1100-300)/LOOP_TIME_MS; k++)
   {
       throttlePos += ThrottleHistory[i];
 /*     Serial.print(k); Serial.print(',');
      Serial.print(i); Serial.print(',');
      Serial.println(ThrottleHistory[i]);  */
      if (++i >= THROTTLE_BUFFER)
           i -= THROTTLE_BUFFER;
 
  }
   throttlePos /= k;
   // avoid overflow
   float x = (float)(throttlePos - MinimumThrottle) * (float) MaxSpeed_mmPs / (FullThrottle - MinimumThrottle);
 //  Serial.print(throttlePos); Serial.print(',');
 //  Serial.print(x); Serial.print(',');
   speedPos_mmPs = max(0, x);
     
   x = (ThrottleHistory[ThrottleIndex] - MinimumThrottle) * MaxSpeed_mmPs / (FullThrottle - MinimumThrottle); 
 //   Serial.print(x); Serial.print(',');
 // if slowing, do exponential decay of maximum speed.
  long coasting_mmPs = x * DecayRate;
  
  speed_mmPs = max(speedPos_mmPs, coasting_mmPs);
  if (speed_mmPs < 1)
  {   // not moving
      MsTimer2::stop();
      ClickTime_ms == 0;
  }
  else
  {  // moving
        if (ClickTime_ms == 0)
        {   // had been stopped
            ClickTime_ms = WheelCircumference_mm * 1000 / speed_mmPs;
            digitalWrite(CLICK_PIN, HIGH);
            digitalWrite(CLICK_PIN, LOW);    // make a click now
            MsTimer2::set(ClickTime_ms, pulse); // next click after ClickTime_ms
            MsTimer2::start();
        }
        else /* update speed after next click */
           ClickTime_ms = WheelCircumference_mm * 1000 / speed_mmPs; 
  }
}
/*----------------------------------------------------------------------------------------------*/
// Pulse the wheel click; called on interrupt from timer

void pulse() 
{
    static long OldRate_ms = 0;
    digitalWrite(CLICK_PIN, HIGH);
    digitalWrite(CLICK_PIN, LOW);
    if (OldRate_ms != ClickTime_ms)
    {  // change the rate
        MsTimer2::stop();
        MsTimer2::set(ClickTime_ms, pulse); // next click after ClickTime_ms
        MsTimer2::start();
        OldRate_ms = ClickTime_ms;
    }
}
/*---------------------------------------------------------------------------------------*/
void LogData(unsigned long time)
{  // Comma separated values to transfer to spread-sheet

    Serial.print(time); Serial.print(',');
    Serial.print(speed_mmPs); Serial.print(',');
    Serial.println(ThrottleHistory[ThrottleIndex]);
  
}
/*----------------------------------------------------------------------------------------------*/
void loop()
{
    unsigned long time, endTime;
    time = millis();
    endTime = time + LOOP_TIME_MS ;
    getThrottle();
    ComputeSpeed();
    while (time < endTime)
    {
        time = millis();
    }
    LogData(time);

}
/*----------------------------------------------------------------------------------------------*/


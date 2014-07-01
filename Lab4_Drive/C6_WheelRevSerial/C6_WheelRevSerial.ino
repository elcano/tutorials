
/* Wheel Revolution Interrupt routine
   Ben Spencer 10/21/13
   Modified by Tyler Folsom 3/16/14
   
   A cyclometer gives a click once per revolution. 
   This routine computes the speed.
*/
// CLICK_IN defined: use interrupt; not defined: simulate with timer
#define CLICK_IN
#define LOOP_TIME_MS 1000
#define CLICK_TIME_MS 1000

#define WHEEL_DIAMETER_MM 397
#define MEG 1000000
#define MAX_SPEED_KPH 200
#define MAX_SPEED_mmPs   ((MAX_SPEED_KPH * MEG) / 3600)
// MAX_SPEED_mmPs = 13,888 mm/s = 13.888 m/s
unsigned long MinTickTime_ms;
// ((WHEEL_DIAMETER_MM * 3142) / MAX_SPEED_mmPs)
// MinTickTime_ms = 89 ms
#define MIN_SPEED_mPh 500
// A speed of less than 0.5 KPH is zero.
unsigned long MaxTickTime_ms;
// ((WHEEL_DIAMETER_MM * 3142) / MIN_SPEED_mmPs)
// MinTickTime_ms = 9239 ms = 9 sec

float Odometer_m = 0;
long SpeedCyclometer_mmPs = 0;
// Speed in revolutions per second is independent of wheel size.
float SpeedCyclometer_revPs = 0.0;//revolutions per sec
volatile unsigned long TickTime = 0;
long WheelRevmillis = 0;
unsigned long OldTick = 0;
volatile unsigned long InterruptCount =0;
unsigned long ShowTime_ms;

/*---------------------------------------------------------------------------------------*/ 
// WheelRev is called by an interrupt.
void WheelRev()
{
    unsigned long tick;   
    noInterrupts();
    tick = millis();
    if (tick - TickTime > MinTickTime_ms)
    {
        TickTime = tick;
        InterruptCount++;
    }
    interrupts();
}

void setup() 
{ 
    Serial.begin(115200);//serial monitor
    Serial2.begin(115200); // C6 to C4
    
    pinMode(13, OUTPUT); //led
    digitalWrite(13, HIGH);//turn LED on
    
    pinMode(2, INPUT_PULLUP);//pulls input HIGH
    float MinTick = WHEEL_DIAMETER_MM * PI;
//    Serial.print (" MinTick = ");
//    Serial.println (MinTick);
    MinTick *= 1000.0;
    MinTick /= MAX_SPEED_mmPs;
//    Serial.print (MinTick);
    MinTickTime_ms = MinTick;
//    Serial.print (" MinTickTime_ms = ");
//    Serial.println (MinTickTime_ms);

//    Serial.print (" MIN_SPEED_mPh = ");
//    Serial.print (MIN_SPEED_mPh);
    float MIN_SPEED_mmPs =  ((MIN_SPEED_mPh * 1000.0) / 3600.0);
    // MIN_SPEED_mmPs = 135 mm/s
//    Serial.print (" MIN_SPEED_mmPs = ");
//    Serial.print (MIN_SPEED_mmPs);
    float MaxTick = (WHEEL_DIAMETER_MM * PI * 1000.0) / MIN_SPEED_mmPs;
//    Serial.print (" MaxTick = ");
//    Serial.print (MaxTick);
    MaxTickTime_ms = MaxTick;
//    Serial.print (" MaxTickTime = ");
//    Serial.println (MaxTickTime_ms);
    TickTime = millis();
    // OldTick will normally be less than TickTime.
    // When it is greater, TickTime - OldTick is a large positive number,
    // indicating that we have not moved.
    // TickTime would overflow after days of continuous operation, causing a glitch of
    // a display of zero speed.  It is unlikely that we have enough battery power to ever see this.
    OldTick = 3 + TickTime;
    ShowTime_ms = TickTime;
    InterruptCount = 0;
    tone(2, 32); // speed signal
#ifdef CLICK_IN
    attachInterrupt (0, WheelRev, RISING);//pin 2 on Mega
#endif
    Serial.println("setup complete");
}

void loop() 
{
    int i, cycles;
    unsigned long time, endTime;
      time = millis();
  if (LOOP_TIME_MS > CLICK_TIME_MS)
  {  // high speed
    cycles = LOOP_TIME_MS / CLICK_TIME_MS;
 //delay until endTime 
  //keeps a constant rate of loop calls, 
  //but don't count time in loop
    for (i=0; i<cycles; i++)
    {
      endTime = time + CLICK_TIME_MS ;//loop at 0.25 sec
      while (time < endTime)
      {
         time = millis();
      }
#ifndef CLICK_IN      
      WheelRev();
#endif
    }
    show_speed();
  }
  else  // low speed
  {
    cycles = CLICK_TIME_MS / LOOP_TIME_MS;
   for (i=0; i<cycles; i++)
    {
      endTime = time + LOOP_TIME_MS ;
      while (time < endTime)
      {
         time = millis();
      }
      show_speed();
    }
#ifndef CLICK_IN      
      WheelRev();
      show_speed();
#endif
    
  }
}
void show_speed()
{
   ShowTime_ms = millis();	
  //check if velocity has gone to zero

    if(ShowTime_ms - TickTime > MaxTickTime_ms)
    {  // stopped
       SpeedCyclometer_mmPs = 0;
       SpeedCyclometer_revPs = 0;
    }
    else
    {  // moving
      register int revolutions;
      if (TickTime > OldTick)
      {  // have new data
         noInterrupts();
      	 WheelRevmillis = TickTime - OldTick;
         revolutions = InterruptCount;
         OldTick = TickTime;
         InterruptCount = 0;
         interrupts();
      
         float Circum_mm = (revolutions*WHEEL_DIAMETER_MM * PI);
         if (WheelRevmillis > 0)
         {
             SpeedCyclometer_mmPs  = (Circum_mm * 1000) / WheelRevmillis;
             SpeedCyclometer_revPs = (revolutions*2*PI*1000.0) / WheelRevmillis;
         }
         else
         {
             SpeedCyclometer_mmPs = 0;
             SpeedCyclometer_revPs = 0;
         }
      }
    }
  // Serial 2 connects C6 to C4
    Serial2.print("SENSOR {Speed ");
    Serial2.print(SpeedCyclometer_revPs);
    Serial2.println("}");
 // C4 to C3
 // C3 to C2 via pin 18 on DB25
 // Short Pin 18 to 19 (Or have C2 repeat what it received)
 // Pin 19 on DB25 goes to Serial2 input.
    // Show on monitor
    Serial.print("SENSOR ");
    Serial.print("{Speed ");
    Serial.print(SpeedCyclometer_revPs);
    Serial.println("}");
    Odometer_m += (float)(LOOP_TIME_MS * SpeedCyclometer_mmPs) / MEG;

}
 

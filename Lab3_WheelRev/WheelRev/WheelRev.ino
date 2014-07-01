
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
#ifndef PI
#define PI ((float) 3.1415925)
#endif
#define MAX_SPEED_KPH 50  
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
    Serial.begin(9600);//serial monitor

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
//  Serial.print (" Times: Show: ");
//  Serial.print (ShowTime_ms);
//  Serial.print (", Tick: ");
//  Serial.println (TickTime);

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
//      Serial.print (" revolutions = ");
//      Serial.print (revolutions);
    }
  
    unsigned long WheelRev_s, WheelRevFraction_s;  
//    Serial.print("; WheelRevmillis = ");
//    Serial.println (WheelRevmillis);
    Serial.print(" ms ");
    Serial.print("speed: ");
    Serial.print(SpeedCyclometer_mmPs/1000);
    Serial.print(".");    
    Serial.print(SpeedCyclometer_mmPs%1000);
    Serial.print(" m/s; ");
    Serial.print(SpeedCyclometer_revPs);
    Serial.print(" Rad/s");
//    Serial.print(" last interval at: ");
//    unsigned long Interval_ms = ShowTime_ms - TickTime;
//    WheelRev_s = Interval_ms / 1000;
//    WheelRevFraction_s = Interval_ms % 1000;
//    Serial.print (WheelRev_s);
//    Serial.print(".");
//    Serial.print (WheelRevFraction_s);
//    Serial.print(" s ");
    Serial.print("; ");
    Serial.print (SpeedCyclometer_mmPs*3600.0/MEG);
    Serial.print(" km/h, ");
    Odometer_m += (float)(LOOP_TIME_MS * SpeedCyclometer_mmPs) / MEG;
    Serial.print(" distance traveled: ");
    Serial.print (Odometer_m);
    Serial.println(" m "); 

}
 

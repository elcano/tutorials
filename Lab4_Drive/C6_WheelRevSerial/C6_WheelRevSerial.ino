
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
long WheelRev_ms = 0;
volatile unsigned long OldTick = 0;
// volatile unsigned long InterruptCount =0;
#define IRQ_NONE 0
#define IRQ_FIRST 1
#define IRQ_RUNNING 2
volatile int InterruptState = IRQ_NONE;
unsigned long ShowTime_ms;

/*---------------------------------------------------------------------------------------*/ 
// WheelRev is called by an interrupt.
void WheelRev()
{
    static int flip = 0;
    unsigned long tick;   
    noInterrupts();
    tick = millis();
    if (InterruptState != IRQ_RUNNING)
    // Need to process 1st two interrupts before results are meaningful.
        InterruptState++;

    if (tick - TickTime > MinTickTime_ms)
    {
        OldTick = TickTime;
        TickTime = tick;
    }
    if (flip)
        digitalWrite(13, LOW);
    else
        digitalWrite(13, HIGH);
    flip =!flip;  
    
    interrupts();
}
/*---------------------------------------------------------------------------------------*/ 

void setup() 
{ 
    Serial.begin(115200);//serial monitor
    Serial2.begin(115200); // C6 to C4
    
    pinMode(13, OUTPUT); //led
    digitalWrite(13, HIGH);//turn LED on
    
    pinMode(2, INPUT);//pulls input HIGH
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
    OldTick = TickTime;
    ShowTime_ms = TickTime;
    InterruptState = IRQ_NONE;
    attachInterrupt (0, WheelRev, RISING);//pin 2 on Mega
    Serial.println("setup complete");
}
/*---------------------------------------------------------------------------------------*/ 

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
      WheelRev();
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
  }
}
/*---------------------------------------------------------------------------------------*/ 
void show_speed()
{
   ShowTime_ms = millis();	
   if (InterruptState == IRQ_NONE || InterruptState == IRQ_FIRST)  // no OR 1 interrupts
   {
       SpeedCyclometer_mmPs = 0;
       SpeedCyclometer_revPs = 0;
   } 
  //check if velocity has gone to zero

    if(ShowTime_ms - TickTime > MaxTickTime_ms)
    {  // stopped
       SpeedCyclometer_mmPs = 0;
       SpeedCyclometer_revPs = 0;
    }
    else
    {  // moving
        WheelRev_ms = max(TickTime - OldTick, ShowTime_ms - TickTime);
        if (InterruptState == IRQ_RUNNING)
        {  // have new data
      
            float Circum_mm = (WHEEL_DIAMETER_MM * PI);
            if (WheelRev_ms > 0)
            {
                SpeedCyclometer_revPs = 1000.0 / WheelRev_ms;
                SpeedCyclometer_mmPs  = Circum_mm * SpeedCyclometer_revPs;
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
 

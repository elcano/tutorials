/*
 Elcano Contol Module C2 Data.
 
 Gather data for use in simulation and better understand vehicle behavior. TCF Sept 1, 2014.
 
 Outputs are
 1) Analog 0-4 V signal for traction motor speed
 2) Pulse wave Modulated (PWM) signal for brakes.
 3) PWM signal for steering.
 
 
 */

// Input/Output (IO) pin names for the MegaShieldDB printed circuit board (PCB)
#include "IOPCB.h"

// When setting up the project, select
//   Sketch  |  Import Library ... |  SPI
// include the Serial Periferal Interface (SPI) library:
#include <SPI.h>
// The MegaShieldDB has a four channel Digital to Analog Converter (DAC).
// Basic Arduino cannot write a true analog signal, but only PWM.
// Many servos take PWM.
// An electric bicycle (E-bike) throttle expects an analog signal.
// We have found that feeding a pwm signal to an e-bike controller makes the motor chug at low speed.

#ifndef TRUE
#define TRUE 1
#define FALSE 0
#endif
#ifndef NaN
#define NaN 0xFFFFFFFF
#endif

#define LOOP_TIME_MS 100
#define WHEEL_DIAMETER_MM 397
#define BUFFER_SIZE 80
#define ERROR_HISTORY 20

// Values to send over DAC
const int FullThrottle =  227;   // 3.63 V
const int MinimumThrottle = 50; // was 70;  // Throttle has no effect until 1.2 V
const int TestThrottle = MinimumThrottle + (FullThrottle - MinimumThrottle)/2;
// Values to send on PWM to get response of actuators
const int FullBrake = 167;  // start with a conservative value; could go as high as 255;  
const int NoBrake = 207; // start with a conservative value; could go as low as 127;
// Steering
const int HardLeft = 250; //  could go as high as 255;
const int Straight = 187;
const int HardRight = 126;

// globals
long sensor_speed_mmPs = 0;
long drive_speed_mmPs = 0;
char IncomingMessage[BUFFER_SIZE];
int  InIndex=0;
int  throttle_control = MinimumThrottle;
int  brake_control = FullBrake;
int  steer_control = Straight;
long speed_errors[ERROR_HISTORY];
long start_time;
long SpeedCyclometer_mmPs = 0;


/*  Elcano #1 Servo range is 50 mm for brake, 100 mm for steering.

    Elcano servo has a hardware controller that moves to a
    particular position based on an input PWM signal from Arduino.
    The Arduino PWM signal is a square wave at a base frequency of 490 Hz or 2.04 ms.
    PWM changes the duty cycle to encode   
    0 is always off; 255 always on. One step is 7.92 us.
    
    Elcano servo is fully retracted on a pulse width of 2 ms;
    fully extended at 1 ms and centered at 1.5 ms.
    There is a deadband of 8 us.
    At 12v, servo operating speed is 56mm/s with no load or
    35 mm/s at maximum load.
    
    Output from hardware servo controller to either servo has five wires, with observed bahavior of:
    White: 0V
    Yellow: 5V
    Blue: 0-5V depending on position of servo.
    Black: 12V while servo extends; 0V at rest or retracting.
    Red:   12V while retracting; 0V at rest or extending.
    The reading on the Blue line has hysteresis when Elcano sends a PWM signal; 
    there appear to be different (PWM, position) pairs when retracting or extending.
    Motor speed is probably controlled by the current on the red or black line.   
*/
/*---------------------------------------------------------------------------------------*/
void initialize()
{
  for (int i = 0; i < ERROR_HISTORY; i++)
  {
      speed_errors[i] = 0;
  }

 
}
/*---------------------------------------------------------------------------------------*/
void setup()
{
    //Set up pin modes and interrupts, call serial.begin and call initialize.
    Serial.begin(115200); // monitor
    Serial3.begin(115200); // C3 to C2;  C2 to C6
	/* A typical message is 20 ASCII characters of 20 bits each
	   (2 8-bit bytes / character + start bit + stop bit)
	   At 115,200 bits/s, a typical message takes 3.5 ms.
	   Thus there is about a 10 ms delay (best case) in passing 
	   information from C6 to C2.
	*/
    
    // SPI: set the slaveSelectPin as an output:
    pinMode (SelectAB, OUTPUT);
    pinMode (SelectCD, OUTPUT);
    pinMode (10, OUTPUT);
    SPI.setDataMode( SPI_MODE0);
    SPI.setBitOrder( MSBFIRST);
    // initialize SPI:
    SPI.begin(); 
    for (int channel = 0; channel < 4; channel++)
        DAC_Write (channel, 0);   // reset did not clear previous states
 
    pinMode(Throttle, OUTPUT);
    pinMode(DiskBrake, OUTPUT);
    pinMode(Steer, OUTPUT);
    pinMode(AccelerateJoystick, INPUT);
    pinMode(SteerJoystick, INPUT);
    pinMode(JoystickCenter, INPUT);

    Serial.println("Start initialization");        
    initialize();
    setupWheelRev();
     
    moveBrake(NoBrake);   // release brake
    start_time = millis() + 1000;  // wait 1 sec to start.
    Serial.println("Initialized");
     
}
/*---------------------------------------------------------------------------------------*/
char * GetWord(char * major, char * str)
{
	char * CSp1;

	CSp1 = strstr(str, major);
	if (CSp1!=NULL)
	CSp1 += strlen(major);
	return CSp1;
}
float GetNumber(char *minor, char*Args)
{
  float data = NaN;
  if (Args == NULL) return data;
  // SENSOR, so grab the new sensor_speed.
  char * Number = GetWord(minor, Args);
  if (Number==NULL) return data;
   // change } to 0
   char* end = strchr(Number, '}');
   if (end == NULL) return NaN;
   *end = '\0';
   data = atof(Number);
   // change back to }
   *end = '}';
   // convert speed from km/h to mm/s
//   sensor_speed_mmPs = (long)(data * 1000000.0 / 3600.0);
   return data;
}
/*---------------------------------------------------------------------------------------*/
#define MSG_NONE 0
#define MSG_SENSOR 1
#define MSG_DRIVE 2
int ProcessMessage ()
{
    int kind = MSG_NONE;
    float data;
	// Determine if message is "SENSOR {Speed xxx.xx}"	
	char * Args = GetWord ("SENSOR", IncomingMessage);
	if (Args != NULL)
	{	
            data = GetNumber("Speed", Args);
    	    // convert speed from km/h to mm/s
    	    if (data != NaN) 
            {
                sensor_speed_mmPs = (long)(data * 1000000.0 / 3600.0);
                kind = MSG_SENSOR;
            }
	}

    return kind;
}
/*---------------------------------------------------------------------------------------*/
void SetDriveSpeed(unsigned long time)
{
#define RAMP_UP_TIME 4000
#define STEADY_TIME  5000
#define COAST_TIME  15000
   if (time < start_time)
     throttle_control = MinimumThrottle;
   else if (time < start_time + RAMP_UP_TIME)
   {
     long int control = MinimumThrottle + (TestThrottle - MinimumThrottle) *
       (time - start_time) / RAMP_UP_TIME;
     throttle_control = control;
   }
   else if (time < start_time  + RAMP_UP_TIME + STEADY_TIME)
   {
     throttle_control = TestThrottle; 
   }   
   else
   {
    throttle_control = MinimumThrottle;
    brake_control = FullBrake;
   }

}
/*---------------------------------------------------------------------------------------*/
void LogData(unsigned long time)
{  // Comma separated values to transfer to spread-sheet
  if (time < start_time + RAMP_UP_TIME + STEADY_TIME + COAST_TIME)
  {
    Serial.print(time); Serial.print(',');
    Serial.print(SpeedCyclometer_mmPs); Serial.print(',');
    Serial.print(throttle_control); Serial.print(',');
    Serial.println(brake_control);
  }
  
}
/*---------------------------------------------------------------------------------------*/
void loop()
{
    int incomingByte = 0;   // for incoming serial data
    unsigned long time, endTime;
    time = millis();
    endTime = time + LOOP_TIME_MS ;
    while (time < endTime)
    {
	if ( Serial3.available() > 0) 
	{
	    // read the incoming byte from C4:
	    incomingByte =  Serial3.read();
	    {
		IncomingMessage[InIndex] = (char)(incomingByte);
		if (IncomingMessage[InIndex] == '\0'
		 || InIndex >= BUFFER_SIZE-1)
		{
		    int kind = ProcessMessage();  // see what we got
                    // ProcessMessage may set sensor_speed_mmPs
		    InIndex = 0;
                    if (kind != MSG_SENSOR)  // Sensor messages originate from C6
		        Serial3.print(IncomingMessage); // pass msg on to C6
                    Serial.println(IncomingMessage);  // for monitor
		}
		else
		{
		    ++InIndex;    	
		}
	    }
	}
	time = millis();
    }
  SetDriveSpeed(time);
  show_speed();  // set cyclometer speed
  moveVehicle(throttle_control);
  moveBrake(brake_control);
  LogData(time);

  
 // apply steering
  steer_control =  Straight;
  moveSteer(steer_control); 
 
}
/*---------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------*/ 
void moveBrake(int i)
{
     analogWrite(DiskBrake, i);
}
/*---------------------------------------------------------------------------------------*/
void moveSteer(int i)
{
     analogWrite(Steer, i);
}
/*---------------------------------------------------------------------------------------*/
void moveVehicle(int counts)
{
    /* Observed behavior on ElCano #1 E-bike no load (May 10, 2013, TCF)
      0.831 V at rest       52 counts
      1.20 V: nothing       75
      1.27 V: just starting 79
      1.40 V: slow, steady  87
      1.50 V: brisker       94
      3.63 V: max          227 counts     
      255 counts = 4.08 V      
      */
   DAC_Write(0, counts);
}
/*---------------------------------------------------------------------------------------*/
/* DAC_Write applies value to address, producing an analog voltage.
// address: 0 for chan A; 1 for chan B; 2 for chan C; 3 for chan D
// value: digital value converted to analog voltage
// Output goes to mcp 4802 Digital-Analog Converter Chip via SPI
// There is no input back from the chip.
*/
void DAC_Write(int address, int value)

/*
REGISTER 5-3: WRITE COMMAND REGISTER FOR MCP4802 (8-BIT DAC)
A/B  —  GA  SHDN  D7 D6 D5 D4 D3 D2 D1 D0 x x x x
bit 15                                       bit 0

bit 15   A/B: DACA or DACB Selection bit
         1 = Write to DACB
         0 = Write to DACA
bit 14   — Don’t Care
bit 13   GA: Output Gain Selection bit
         1 = 1x (VOUT = VREF * D/4096)
         0 = 2x (VOUT = 2 * VREF * D/4096), where internal VREF = 2.048V.
bit 12   SHDN: Output Shutdown Control bit
         1 = Active mode operation. VOUT is available. 
         0 = Shutdown the selected DAC channel. Analog output is not available at the channel that was shut down.
         VOUT pin is connected to 500 k (typical)
bit 11-0 D11:D0: DAC Input Data bits. Bit x is ignored.


With 4.95 V on Vcc, observed output for 255 is 4.08V.
This is as documented; with gain of 2, maximum output is 2 * Vref

*/

{
  int byte1 = ((value & 0xF0)>>4) | 0x10; // acitve mode, bits D7-D4
  int byte2 = (value & 0x0F)<<4;           // D3-D0
  if (address < 2)
  {
      // take the SS pin low to select the chip:
      digitalWrite(SelectAB,LOW);
      if (address >= 0)
      { 
        if (address == 1)
          byte1 |= 0x80;  // second channnel
        SPI.transfer(byte1);
        SPI.transfer(byte2);
       }
      // take the SS pin high to de-select the chip:
      digitalWrite(SelectAB,HIGH);
  }
  else
  {
      // take the SS pin low to select the chip:
      digitalWrite(SelectCD,LOW);
      if (address <= 3)
      {
        if (address == 3)
          byte1 |= 0x80;  // second channnel
        SPI.transfer(byte1);
        SPI.transfer(byte2);
      }
       // take the SS pin high to de-select the chip:
      digitalWrite(SelectCD,HIGH);
  }
}
/*---------------------------------------------------------------------------------------*/ 
/*---------------------------------------------------------------------------------------*/ 
/*=======================================================================================*/

/* Wheel Revolution Interrupt routine
   Ben Spencer 10/21/13
   Modified by Tyler Folsom 3/16/14
   
   A cyclometer gives a click once per revolution. 
   This routine computes the speed.
*/
// CLICK_IN defined: use interrupt; not defined: simulate with timer
#define SerialMonitor Serial
#define CLICK_IN 1
#define LOOP_TIME_MS 1000
#define CLICK_TIME_MS 1000

#define WHEEL_DIAMETER_MM 397
#define MEG 1000000
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
/*-------------------------------------------------------------------------------*/
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

void setupWheelRev() 
{ 
    
    pinMode(13, OUTPUT); //led
    digitalWrite(13, LOW);//turn LED off
    
    pinMode(2, INPUT);//pulls input HIGH
    float MinTick = WHEEL_DIAMETER_MM * PI;
//    SerialMonitor.print (" MinTick = ");
//    SerialMonitor.println (MinTick);
    MinTick *= 1000.0;
    MinTick /= MAX_SPEED_mmPs;
//    SerialMonitor.print (MinTick);
    MinTickTime_ms = MinTick;
    SerialMonitor.print (" MinTickTime_ms = ");
    SerialMonitor.println (MinTickTime_ms);

//    SerialMonitor.print (" MIN_SPEED_mPh = ");
//    SerialMonitor.print (MIN_SPEED_mPh);
    float MIN_SPEED_mmPs =  ((MIN_SPEED_mPh * 1000.0) / 3600.0);
    // MIN_SPEED_mmPs = 135 mm/s
//    SerialMonitor.print (" MIN_SPEED_mmPs = ");
//    SerialMonitor.print (MIN_SPEED_mmPs);
    float MaxTick = (WHEEL_DIAMETER_MM * PI * 1000.0) / MIN_SPEED_mmPs;
//    SerialMonitor.print (" MaxTick = ");
//    SerialMonitor.print (MaxTick);
    MaxTickTime_ms = MaxTick;
//    SerialMonitor.print (" MaxTickTime = ");
//    SerialMonitor.println (MaxTickTime_ms);
    TickTime = millis();
    // OldTick will normally be less than TickTime.
    // When it is greater, TickTime - OldTick is a large positive number,
    // indicating that we have not moved.
    // TickTime would overflow after days of continuous operation, causing a glitch of
    // a display of zero speed.  It is unlikely that we have enough battery power to ever see this.
    OldTick = TickTime;
    ShowTime_ms = TickTime;
 //   InterruptCount = 0;
    InterruptState = IRQ_NONE;
    attachInterrupt (1, WheelRev, RISING);//pin 3 on C2 Mega
    SerialMonitor.print("TickTime: ");
    SerialMonitor.print(TickTime);
    SerialMonitor.print(" OldTick: ");
    SerialMonitor.println(OldTick);
     
    SerialMonitor.println("setup complete");
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
   else // have at least twointerrupts
   {
    if(ShowTime_ms - TickTime > MaxTickTime_ms)
    {  // stopped
/*        SerialMonitor.print("Stop. Showtime: ");
        SerialMonitor.print(ShowTime_ms);
        SerialMonitor.print(" Tick: ");
        SerialMonitor.println(TickTime); */
        SpeedCyclometer_mmPs = 0;
        SpeedCyclometer_revPs = 0;
    }
    else
    {  // moving
//        int revolutions;
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
    }
    // Show on monitor
/*    SerialMonitor.print("\nWheelRev (ms): ");
    SerialMonitor.print(WheelRev_ms);
    SerialMonitor.print(" SENSOR ");
    SerialMonitor.print("{Speed ");
    SerialMonitor.print(SpeedCyclometer_revPs);
    SerialMonitor.println("}\0"); */
    Odometer_m += (float)(LOOP_TIME_MS * SpeedCyclometer_mmPs) / MEG;
}



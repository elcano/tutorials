/*
 Elcano Contol Module C2 Basic: Bare bones low level control of vehicle.
 
 Throttle, brake and steering inputs can come from three sources: 
 1) On board joystick
     e.g. 
 2) Remotely controlled (RC) joystick and buttons
     e.g: HiTec Optic 5 RCS: http://hitecrcd.com/images/products/pdf/22_OPTIC_5_MANUAL_P.pdf
 3) An external high level computer, such as Arduino C3 Pilot.
 
 Normally the vehicle will be configured to use either (1) or (2) for manual control. 
 Automatic control (3) is always an option.

 
 The inputs consist of analog or Pulse wave Modulated (PWM) values.  Inputs are:
 1) Throttle
 2) Brake
 3) Steering
 4) a digital emergency stop (E-stop) that kills 36V power to the traction motor.
    The E-stop is electronic only, and will function if the Elcano_C2_Basic firmware is unresponsive.
    If E-stop is detected by software, it will apply brakes. Software will attempt to turn off the motor.
 5) a digital stop command.
    If the vehicle is under computer control, it will enter manual mode and begin to stop.
    Any manual activation of throttle will abort the stop.
 6) A digital start command.
    This turns control over to a high level computer and begins motion to the next waypoint.
    Waypoints are loaded into the higher level computer and are not directly settable for the C2 module.
    Once under computer control, manual control will be unresponsive.
   
   Future enhancements can make transfer between manual and automatic control similar to automotive 
   cruise control. 
 
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
// Values to send over DAC
const int FullThrottle =  227;   // 3.63 V
const int MinimumThrottle = 70;  // Throttle has no effect until 1.2 V
// Values to send on PWM to get response of actuators
const int FullBrake = 167;  // start with a conservative value; could go as high as 255;  
const int NoBrake = 207; // start with a conservative value; could go as low as 127;
// Steering
const int HardLeft = 250; //  could go as high as 255;
const int Straight = 187;
const int HardRight = 126;

// Joystick limits
const int limitStickUp = 893;  // observed
const int limitStickDown = 100;  // observed 0; but make everything from 0 to 100 full brake.
const int limitStickLeft = 900;  // guess
const int limitStickRight = 100; // guess


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

// Indices into Instruments
const int Motor = 0;
const int Brakes = 1;
const int Steering = 2;

class _Instruments
{
public:
  int StickMoved;  // bool
  int Joystick;   // position of stick
} Instrument[3];

/*---------------------------------------------------------------------------------------*/
void initialize()
{
  Instrument[Motor].Joystick = MinimumThrottle;
  Instrument[Brakes].Joystick = FullBrake;
  Instrument[Steering].Joystick = Straight;
 
}
/*---------------------------------------------------------------------------------------*/
void setup()
{
    //Set up pin modes and interrupts, call serial.begin and call initialize.
    Serial.begin(9600);
    
    // SPI: set the slaveSelectPin as an output:
    pinMode (SelectAB, OUTPUT);
    pinMode (SelectCD, OUTPUT);
    pinMode (10, OUTPUT);
    SPI.setDataMode( SPI_MODE0);
    SPI.setBitOrder( MSBFIRST);
    // initialize SPI:
    // The following line should not be neccessary. It uses a system library.
    PRR0 &= ~4;  // turn off PRR0.PRSPI bit so power isn't off
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
     
    moveBrake(NoBrake);   // release brake
    Serial.println("Initialized");
     
}


/*---------------------------------------------------------------------------------------*/
void loop()
{
  // Check instuments no faster than MinTimeStep_usec
  const unsigned long MinTimeStep_usec = 10000;

  static unsigned long time_us;
  unsigned long timeNow_us = micros();
  if (timeNow_us < time_us)
  {      // roll-over
    time_us = 0;
  }
  if (timeNow_us - time_us < MinTimeStep_usec)
    return;
  time_us = timeNow_us;  
 
  JoystickMotion();
  // apply throttle
  int desired_position =  MinimumThrottle;
  if (Instrument[Motor].StickMoved)
  {
     desired_position = Instrument[Motor].Joystick;
     Serial.print(" Motor: "); Serial.print(desired_position);
  }
  moveVehicle(desired_position);
 
  // apply brakes
  desired_position =  NoBrake;
  if (Instrument[Brakes].StickMoved)
  {
     desired_position = Instrument[Brakes].Joystick;
     Serial.print(" Brakes: "); Serial.print(desired_position);
  }
  moveBrake(desired_position);

  
 // apply steering
  desired_position =  Straight;
  if (Instrument[Steering].StickMoved)
  {
     desired_position = Instrument[Steering].Joystick;
     Serial.print(" Steering: "); Serial.print(desired_position);
  }
  moveSteer(desired_position); 
  if (Instrument[Motor].StickMoved || Instrument[Brakes].StickMoved ||
     Instrument[Steering].StickMoved)
     Serial.println(" ");
}
/*---------------------------------------------------------------------------------------*/
// Check if operator has moved joystick 
void JoystickMotion()
// All inputs are 0-1023
{
    const int deadBandLow = 80;  // out of 1023
    const int deadBandHigh = 80;
    long int x;
    int upStick = analogRead(AccelerateJoystick);
    int sideStick = analogRead(SteerJoystick);
    int center  = analogRead(JoystickCenter);
    center = 440;  // wire is broken
    if (center < 420 || center > 460)
    {
      Serial.print(" Center: "); Serial.print(center);
      Serial.print(" Up: "); Serial.print(upStick);
      Serial.print(" Side: "); Serial.println(sideStick);
      // Joystick may have lost power; put vehicle in safe state
      Instrument[Motor].Joystick = MinimumThrottle;
      Instrument[Motor].StickMoved = FALSE;
      Instrument[Steering].Joystick = Straight;
      Instrument[Steering].StickMoved =  FALSE;
      Instrument[Brakes].Joystick = FullBrake;
      Instrument[Brakes].StickMoved =  FALSE;
      return; 
   }
      //Calculate deadband info
     int tinyDown = center - deadBandLow;
     int tinyUp = center + deadBandHigh;
     
       //If stick did not move more than the "up"    deadband.
  if (upStick <= tinyUp)  
  {
    //stick not outside deadBand
    Instrument[Motor].Joystick = 0;
    Instrument[Motor].StickMoved = FALSE;
  }
  else
  {
    //stick outside of deadband
    if (upStick > limitStickUp)
        upStick = limitStickUp;
    x = (upStick - tinyUp);
    x *= (FullThrottle - MinimumThrottle);
    x /= (limitStickUp - tinyUp);
    x  += MinimumThrottle;
    Instrument[Motor].Joystick = x;
    Instrument[Motor].StickMoved =  TRUE;

  }

  //Check if Stick moved beyond "down" deadBand
  if (upStick >= tinyDown)
  {
    //Stick still in deadband
    Instrument[Brakes].Joystick = NoBrake;
    Instrument[Brakes].StickMoved =  FALSE;
  }
  else
  {
    if (upStick < limitStickDown)
        upStick = limitStickDown;
    x = (tinyDown - upStick);
    x *= (NoBrake - FullBrake);  // NoBrake = 207; FullBrake = 167
    x /= (tinyDown - limitStickDown);
    Instrument[Brakes].Joystick = NoBrake - x;
    Instrument[Brakes].StickMoved =  TRUE;
  }

  //read the steering pot in the joystick
  if (sideStick > tinyDown && sideStick <= tinyUp)
  {  //  The stick is still in the left-right deadband
    Instrument[Steering].Joystick = Straight;
    Instrument[Steering].StickMoved =  FALSE;
  }
  else if (sideStick > tinyUp)
  {
    //Stick moved past "up" deadband limit
    if (sideStick > limitStickLeft)
        sideStick = limitStickLeft;
    x = (sideStick - tinyUp);  // positive
    x *= (HardLeft - Straight);  // positive
    x /= (limitStickLeft - tinyUp);  // x is positive
    Instrument[Steering].Joystick = Straight + x;   
    Instrument[Steering].StickMoved =  TRUE;
  }
  else
  {
    if (sideStick < limitStickRight)
        sideStick = limitStickRight;
    x = (tinyDown - sideStick);  // positive
    x *= (Straight - HardRight);  // positive
    x /= (tinyDown - limitStickRight);  // x is positive
    Instrument[Steering].Joystick = Straight - x;
    Instrument[Steering].StickMoved = TRUE;
  }
}
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




/*  Base Elcano GPS code.
    Read GPS, and log it to an SD card.
*/

#define MEGA
#include "Common.h"
#include "IO.h"
#include "Matrix.h"

/*
Elcano Module C6: Navigator.
  This module contains only GPS.
  It can be expanded to include other sensors, 
  and merge them with a Kalman filter.
  
Documentation:
  NavigationSystem 
  Wiring_C6Mega.xls
  Navigation sheet of Elcano_BOM  (TO DO: Reorganize Elcano BOM).

GPS should not the primary navigation sensor. The robot should be 
able to operate indoors, in tunnels or other areas where GPS is not available
or accurate.


Serial lines:
0: Monitor
1: (reserved for INU)
2: Tx: Estimated state; 
      $C6EST,<east_mm>,<north_mm>,<speed_mmPs>,<bearing>,<time_ms><positionStndDev_mm>*CKSUM
      $C6OBS,<number>,<obstacle1_range_mm>,<obstacle1_bearing>, ...*CKSUM
   Rx: Desired course
      $C4XPC,<east_mm>,<north_mm>,<speed_mmPs>,<bearing>,<time_ms>*CKSUM
      // at leat 18 characters
3: GPS
  Rx: $GPRMC,...  typically 68 characters
      $GPGSA,...
      $GPGGA,... typically 68 characters
  Tx: $PSRF103,...
*/

/*---------------------------------------------------------------------------------------*/ 

#include <SD.h>

// On the Ethernet Shield, CS is pin 4. Note that even if it's not
// used as the CS pin, the hardware CS pin (10 on most Arduino boards,
// 53 on the Mega) must be left as an output or the SD library
// functions will not work.

// global variables
void Filter(REAL* x, REAL* P, REAL* measure, REAL deltaT, REAL* variance);

// sGPS_file contains UTC of first valid GPS
// first valid GPS acquisition typically takes 42 seconds.


// Set the GPSRATE to the baud rate of the GPS module. Most are 4800
// but some are 38400 or other. Check the datasheet!
#define GPSRATE 4800

#define MAX_WAYPOINTS 10
/*   There are two coordinate systems.
     MDF and RNDF use latitude and longitude.
     C3 and C6 assume that the earth is flat at the scale that they deal with.
     All calculations with C6 are performed in the (east_mm, north_mm)
     coordinate system. A raw GPS reading is in latitude and longitude, but it
     is transformed to the flat earth system using the distance from the origin,
     as set by (LATITUDE_ORIGIN, LONGITUDE_ORIGIN).
     A third coordinate system has its origin at the vehicle center and is
     aligned with the vehicle. This is usually transformed into the flat
     world coordinate system.
*/


// limited to 8+3 characters
#define FILE_NAME "GPSLog.csv"
File dataFile;
char GPSfile[BUFFSIZ] = "mmddhhmm.csv"; 
char ObstacleString[BUFFSIZ];
char StartTime[BUFFSIZ] = "yy,mm,dd,hh,mm,ss,xxx";
const char TimeHeader[] = "year,month,day,hour,minute,second,msec";
const char* RawKF = "Raw GPS data,,,,,,Kalman Filtered data";
const char* Header = "Latitude,Longitude,East_m,North_m,SigmaE_m,SigmaN_m,Time_s,";
const char* ObstHeader ="Left,Front,Right,Busy";

#define WHEEL_DIAMETER_MM 397
/* time (micro seconds) for a wheel revolution */
volatile long Odometer_mm = 0;
volatile long SpeedCyclometer_mmPs;
// Speed in degrees per second is independent of wheel size.
volatile long SpeedCyclometer_degPs;

// waypoint mission[MAX_WAYPOINTS];
waypoint GPS_reading;
waypoint estimated_position;
//instrument IMU;
const unsigned long LoopPeriod = 100;  // msec
//---------------------------------------------------------------------------
char* obstacleDetect()
{
// Calibration shows that readings are 5 cm low.
#define OFFSET 5
    int LeftRange =  analogRead(LEFT) + OFFSET;
    int Range =      analogRead(FRONT) + OFFSET;
    int RightRange = analogRead(RIGHT) + OFFSET;

  sprintf(ObstacleString, 
  "%d.%0.2d,%d.%0.2d,%d.%0.2d,",
  LeftRange/100, LeftRange%100, Range/100, Range%100, RightRange/100, RightRange%100);
 
  return ObstacleString;
}
/*---------------------------------------------------------------------------------------*/ 
// WheelRev is called by an interrupt.
void WheelRev()
{
    static unsigned long OldTick = 0;
    unsigned long TickTime;
    unsigned long WheelRevMicros;
    TickTime = micros();
    if (OldTick == TickTime)
        return;
    if (OldTick <= TickTime)
      	WheelRevMicros = TickTime - OldTick;
    else // overflow
      	WheelRevMicros = TickTime + ~OldTick;
    SpeedCyclometer_degPs = (360 * MEG) / WheelRevMicros;
    SpeedCyclometer_mmPs  = (WHEEL_DIAMETER_MM * MEG * PI) / WheelRevMicros;
    Odometer_mm += WHEEL_DIAMETER_MM * PI;
    OldTick = TickTime;
}
/*---------------------------------------------------------------------------------------*/ 

void initialize()
{
  pinMode(GPS_POWER, OUTPUT);
  char* GPSString;
  char* protocol =  "$PSRF100,1,4800,8,1,0*0E"; // NMEA
  char* disable =   "$PSRF103,02,00,00,01*26\r\n";
  char* querryGGA = "$PSRF103,00,01,00,01*25";
  bool GPS_available = false;
  Serial3.begin(GPSRATE);   
  digitalWrite(GPS_POWER, LOW);         // pull low to turn on!
  Serial.flush();
  Serial3.flush();
  delay(5000);
  // prints title with ending line break 
  Serial.println(" GPS parser");  
  Serial.print("Acquiring GPS RMC...");
  checksum(protocol);
  Serial3.println(protocol);
  disable[10] = '2';
  checksum(disable);
  Serial3.println(disable);   // no GSA

  GPS_available = estimated_position.AcquireGPRMC(70000);
  Serial.println(TimeHeader);
  Serial.println(StartTime);
  Serial.println(RawKF);
  Serial.print(Header);
  Serial.print(Header);
  Serial.println(ObstHeader);
  if (GPS_available)
  {
    estimated_position.sigma_mm = 1.0E4; // 10 m standard deviation
    Serial.println("OK");
  }
  else
  {
    estimated_position.latitude = 47.62130;  // set by hand here
    estimated_position.longitude = -122.35090;
    estimated_position.Compute_mm();
    estimated_position.sigma_mm = 1.0E5; // 100 m standard deviation
    estimated_position.time_ms = millis();
    Serial.println("Failed");
  }
  // Set velocity and acceleration to zero.
  estimated_position.speed_mmPs = 0;
  // Set attitude.
  estimated_position.Evector_x1000 = 1000;  // to be taken from path or set by hand
  estimated_position.Nvector_x1000 = 0;
  GPS_reading = estimated_position;
  GPSString = estimated_position.formPointString();  
  Serial.println(GPSString);
  // Set Odometer to 0.
  // Set lateral deviation to 0.
  // Read compass.
  // ReadINU.
  // SendState(C4);
    // Wait to get path from C4
//    while (mission[1].latitude > 90)
    {
    /* If (message from C4)
    {
      ReadState(C4);  // get initial route and speed
    }
     Read GPS, compass and IMU and update their estimates.
   */
    }
    /* disable GPS messages;
    for (char i='0'; i < '6'; i++)
    {
      disable[9] = i;
      checksum(disable);
      Serial3.println(disable);
    }
    Serial3.println(querryGGA);
    */
//    GPS_available = GPS_reading.AcquireGPGGA(300);
    // ready to roll
    // Fuse all position estimates.
    // Send vehicle state to C3 and C4.
    
}
/*---------------------------------------------------------------------------------------*/ 

void setup() 
{ 
    pinMode(Rx0, INPUT);
    pinMode(Tx0, OUTPUT);
    pinMode(GPS_RX, INPUT);
    pinMode(GPS_TX, OUTPUT);
    pinMode(C4_RX, INPUT);
    pinMode(C4_TX, OUTPUT);
    pinMode(INU_RX, INPUT);
    pinMode(INU_TX, OUTPUT);
    pinMode(GPS_POWER, OUTPUT);
    Serial3.begin(GPSRATE); // GPS   
    Serial.begin(9600);
    digitalWrite(GPS_POWER, LOW);         // pull low to turn on!
    // make sure that the default chip select pin is set to
    // output, even if you don't use it:
    pinMode(chipSelect, OUTPUT);
    pinMode(53, OUTPUT);  // Unused CS on Mega
    pinMode(GPS_RED_LED, OUTPUT);
    pinMode(GPS_GREEN_LED, OUTPUT);
    digitalWrite(GPS_GREEN_LED, HIGH); 
    digitalWrite(GPS_RED_LED, HIGH);
//    digitalWrite(GPS_GREEN_LED, LOW); 
//    digitalWrite(GPS_RED_LED, LOW);

    initialize();
//    Serial.print("Initializing GPS SD card...");
    
    // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect)) 
    {
      Serial.println("Card failed, or not present");
      digitalWrite(GPS_RED_LED, HIGH);
    }
    else
    {
      Serial.println("card initialized.\n");
      dataFile = SD.open(GPSfile, FILE_WRITE);
      // if the file is available, write date and time to it:
      if (dataFile) 
      {
          dataFile.println(TimeHeader);
          dataFile.println(StartTime);
          dataFile.println(RawKF);
          dataFile.print(Header);
          dataFile.print(Header);
          dataFile.println(ObstHeader);
          dataFile.close();
      }  
    }
    pinMode(CYCLOMETER, INPUT);
    attachInterrupt (5, WheelRev, RISING);

}
/*---------------------------------------------------------------------------------------*/ 
void waypoint::SetTime(char *pTime, char * pDate)
{
//    GPSfile = "mmddhhmm.CSV";
     strncpy(GPSfile,   pDate+2, 2);  // month   
     strncpy(GPSfile+2, pDate, 2);    // day    
     strncpy(GPSfile+4, pTime,2);     // GMT hour
     strncpy(GPSfile+6, pTime+2,2);   // minute
     Serial.println(GPSfile); 
    
     strncpy(StartTime,     pDate+4, 2);  // year   
     strncpy(StartTime+3,   pDate+2, 2);  // month   
     strncpy(StartTime+6,   pDate, 2);    // day    
     strncpy(StartTime+9,   pTime,2);     // GMT hour
     strncpy(StartTime+12,  pTime+2,2);   // minute
     strncpy(StartTime+15,  pTime+4,2);   // second
     strncpy(StartTime+18,  pTime+7,3);   // millisecond
     
}

/*---------------------------------------------------------------------------------------*/ 
void loop() 
{
    unsigned long deltaT_ms;
    unsigned long time = millis();
    unsigned long endTime = time + LoopPeriod;
    unsigned long work_time = time;
    int PerCentBusy;
    char* pData;
    char* pGPS;
    char* pObstacles;

    bool GPS_available = GPS_reading.AcquireGPGGA(300);

    /* Perform dead reckoning from clock and previous state
    Read compass.
    ReadINU.
    Set attitude.
    Read Hall Odometer;  */
//   IMU.Read(GPS_reading);   
 
/*  Read Optical Odometer;
    Read lane deviation;  
    If (message from C4)
    { */
//      readline(2);  // C4 Path planner on serial 2 get new route and speed
 /*   }
    If (message from C3)
    { */
//      readline(2);  // get C3 Pilot commanded wheel spin and steering
/*    }
    if (landmarks availabe)
    {  // get the position based on bearing and angle to a known location.
      ReadLandmarks(C4); 
    }
    // Fuse all position estimates with a Kalman Filter */
    deltaT_ms = GPS_reading.time_ms - estimated_position.time_ms;
    estimated_position.fuse(GPS_reading, deltaT_ms);
    estimated_position.time_ms = GPS_reading.time_ms;
/*  Serial.print("time, gps, dt_ms = "); 
    Serial.print(time, DEC); Serial.print(", ");
    Serial.print(GPS_reading.time_ms, DEC); Serial.print(", ");
    Serial.println(deltaT_ms, DEC);
*/ 
    // Send vehicle state to C3 and C4.   

    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    dataFile = SD.open(GPSfile, FILE_WRITE);
  
    // if the file is available, write to it:
    if (GPS_available) 
    {
        digitalWrite(GPS_GREEN_LED, HIGH);
        pGPS = GPS_reading.formPointString();
        if (dataFile) dataFile.print(pGPS);
        // print to the serial port too:
        Serial.print(pGPS);
        pData = estimated_position.formPointString();
        if (dataFile) dataFile.print(pData);
        Serial.print(pData);
        Serial3.print(pData);  // send data to C4 path planner
        pObstacles = obstacleDetect();
        if (dataFile) 
        {
          dataFile.print(pObstacles);
        }
        Serial.print(pObstacles);
    }  
    else 
    {
        digitalWrite(GPS_GREEN_LED, LOW);
        Serial.print("GPS not available");
    }
    
    work_time = millis() - time;
    PerCentBusy = (100 * work_time) / LoopPeriod; 
    if (dataFile) 
    {
      dataFile.println(PerCentBusy);
    }
    Serial.println(PerCentBusy);
    // if the file didn't open, pop up an error:
    if (dataFile)
    {
//        digitalWrite(GPS_RED_LED, LOW);
        dataFile.close();
    }
    else
    {
//        digitalWrite(GPS_RED_LED, HIGH);
 //     Serial.println("error opening file");
    }
  
  // delay, but don't count time in loop
  while (time < endTime)
  {
    time = millis();
  }

}

void Show(char* x)
{
//  Serial.print(x);
}
void Show(REAL x)
{
//  Serial.print(x);
//  Serial.print(", ");
}


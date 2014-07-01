 /* PlotSegments routine for C3
   Tyler Folsom 4/10/14
   
   Part of basic automated motion test
   
   C6 finds speed and passes it to C4. C4 passes speed to C3.
   
   The speed/odometer reading comes in as a serial SENSOR {speed nnnn} command.
   In C3,  Plot Segments coordinates an odometer with segments that describe the vehicle's
   desired speed.  Each piece is governed by a structure that specifies the distance to
   maintain a specific speed. Rates of acceleration and deceleration can also be specified.
   Based on how far the vehicle has traveled, this routine has the job of finding the desired speed.
   The desired speed goes out as a DRIVE {speed nnnn} serial command to C2. 
   
   The C2 computer has the task of controlling the vehicle to the desired speed.
   */
#include <stdlib.h>
//#include "Arduino.h"

#define STEPS 1
#define LOOP_TIME_MS 100
#define WHEEL_DIAMETER_MM 397
#define BUFFER_SIZE 80
//typedef char *Strings;
//#define Strings char *


// globals
long sensor_speed_mmPs = 0;
char IncomingMessage[BUFFER_SIZE];
int InIndex=0;

const long speed_tolerance_mmPs = 75;  // about 0.2 mph
const long max_decel_mmPs2 = 2500;
const long typical_decel_mmPs2 = 1000;
const long min_decel_mmPs2 = 300;
const long min_accel_mmPs2 = 200;
const long typical_accel_mmPs2 = 700;
const long max_accel_mmPs2 = 1400;
const long max_mult = 0x7FFF;  // multiplying bigger numbers might overflow.

struct rung
{
    /* distance of this step is measured starting at the point where previous step has covered its distance.
        If there was no previous step, start from zero.
        distance terminates when we have reached the speed of the next step.
        If there is no next step, come to a stop when distance is covered.
    */
    long distance_mm;
    long distance_travelled_mm;
	long distance2steady_mm;  // at this point we expect to be at steady speed 
	long slow_distance_mm; // when we get here, enter transition_out state.
    long speed_mmPs;  // when in steady state, move at this speed.
    long accel_mmPs2;  // desired rate of acceleration
    long decel_mmPs2;  // desired rate of deceleration
    int state;
//    enum state {initial, transition_in, steady, transition_out, completed};
};

struct ladder
{
    struct ladder *previous;
    rung *data;
    struct ladder *next;
};

ladder profile[STEPS];
rung    speeds[STEPS];

void InitializeSpeed (ladder* seg, long measured_speed_mmPs);
long int Find_Speed (ladder* seg, long int measured_speed_mmPs);

void SetSpeeds()
{
	for (int i = 0; i < STEPS; i++)
	{
		if (i > 0)	
		{
			profile[i].previous = &profile[i-1];
		}
		else
		{
			profile[i].previous = NULL;		
		}
		if (i+1 < STEPS)
		{
			profile[i].next = &profile[i];
		}
		else
		{
			profile[i].next = NULL;
		}
		speeds[i].state = 0; // initializing
		// Here put in the desired values for the speed profile
		speeds[i].distance_mm = 3000;
		speeds[i].speed_mmPs = 2500;
		speeds[i].accel_mmPs2 = typical_accel_mmPs2;
		speeds[i].decel_mmPs2 = typical_decel_mmPs2;
	}
	
}
void InitializeSpeed (ladder* seg, long measured_speed_mmPs)
{
	long delta_speed_mmPs;
	// check acceleration
	if (seg->data->decel_mmPs2 < 0)
	{
		seg->data->decel_mmPs2 = -seg->data->decel_mmPs2;
	}
	if (seg->data->decel_mmPs2 > max_decel_mmPs2)
	{
		seg->data->decel_mmPs2 = max_decel_mmPs2;
	}	
	if (seg->data->decel_mmPs2 < min_decel_mmPs2)
	{
		seg->data->decel_mmPs2 = min_decel_mmPs2;
	}
	if (seg->data->accel_mmPs2 < 0)
	{
		seg->data->accel_mmPs2 = -seg->data->accel_mmPs2;
	}
	if (seg->data->accel_mmPs2 > max_accel_mmPs2)
	{
		seg->data->accel_mmPs2 = max_accel_mmPs2;
	}
	if (seg->data->accel_mmPs2 < min_accel_mmPs2)
	{
		seg->data->accel_mmPs2 = min_accel_mmPs2;
	}
		
	if (!seg->previous)
	{   // no prior ladder
		seg->data->distance_travelled_mm = 0;
	}
	else
	{
		seg->data->distance_travelled_mm =
		seg->previous->data->distance_mm - seg->previous->data->distance_travelled_mm;
	}
	// compute expected distance to come up to speed
	delta_speed_mmPs = seg->data->speed_mmPs - measured_speed_mmPs;
	if (delta_speed_mmPs >= 0)
	{   // speeding up
		if (delta_speed_mmPs < max_mult)
			seg->data->distance2steady_mm = (delta_speed_mmPs * delta_speed_mmPs)
			/ seg->data->accel_mmPs2;
		else
			seg->data->distance2steady_mm = (delta_speed_mmPs/ seg->data->accel_mmPs2)
			* delta_speed_mmPs;
	}
	else
	{   // slowing down
		if (-delta_speed_mmPs < max_mult)
			seg->data->distance2steady_mm = (delta_speed_mmPs * delta_speed_mmPs)
			/ seg->data->decel_mmPs2;
		else
			seg->data->distance2steady_mm = (delta_speed_mmPs/ seg->data->decel_mmPs2)
			* delta_speed_mmPs;		
	}
	seg->data->distance2steady_mm += seg->data->distance_travelled_mm;
		
	// compute expected stopping distance
	if (!seg->next)
	{  // no next ladder; come to a halt
		delta_speed_mmPs = seg->data->speed_mmPs;
	} 
	else
	{
		delta_speed_mmPs = abs(seg->data->speed_mmPs - seg->next->data->speed_mmPs);
	}
			
	if (delta_speed_mmPs < max_mult)
		seg->data->slow_distance_mm = (delta_speed_mmPs * delta_speed_mmPs)
		/ seg->data->decel_mmPs2;
	else
		seg->data->slow_distance_mm = (delta_speed_mmPs/ seg->data->decel_mmPs2)
		* delta_speed_mmPs;
		
	seg->data->state = 1;
}

long int Find_Speed (ladder* seg, long int measured_speed_mmPs)
{
        int i;
	long set_speed_mmPs = 0;
	long final_speed_mmPs = 0;
	seg->data->distance_travelled_mm += (measured_speed_mmPs * LOOP_TIME_MS / 1000); 
	switch (seg->data->state)
	{
	case 0:  // initializing
                i = 0;
		InitializeSpeed (seg, measured_speed_mmPs);
                i++;
		// fall through
	case 1:	// coming up to speed	  
		if (seg->data->distance_travelled_mm >= seg->data->distance2steady_mm 
		    /* have completed the ramp-up on the trapezoid */ 
			|| abs(measured_speed_mmPs - seg->data->speed_mmPs) < speed_tolerance_mmPs)
			/* or have reached target speed */
		{
			seg->data->state = 2;   // steady;
			set_speed_mmPs = seg->data->speed_mmPs;	
		} else if (measured_speed_mmPs < seg->data->speed_mmPs)
		{  // accelerating
			set_speed_mmPs = seg->data->speed_mmPs - seg->data->accel_mmPs2 *
			(seg->data->distance_travelled_mm - seg->data->distance2steady_mm);
		}
		else
		{  // decelerating
			set_speed_mmPs = seg->data->speed_mmPs + seg->data->decel_mmPs2 *
			(seg->data->distance_travelled_mm - seg->data->distance2steady_mm);
		}
		
		if (seg->data->distance_travelled_mm >= seg->data->slow_distance_mm)
			seg->data->state = 3;   // slowing;
		break;
		
	case 2:  // steady speed
		set_speed_mmPs = seg->data->speed_mmPs;  // for case 2
		if (seg->data->distance_travelled_mm >= seg->data->slow_distance_mm)
			seg->data->state = 3;   // slowing; fall through
		else break;
		
	case 3: // slowing
		if (seg->next)
			final_speed_mmPs = seg->next->data->speed_mmPs;
		if (seg->data->distance_travelled_mm >= seg->data->distance_mm)
		/* have come too far */
		{
			seg->data->state = 4; // completed
			set_speed_mmPs = final_speed_mmPs;
		} else if (measured_speed_mmPs < final_speed_mmPs)
		{  // accelerating
			set_speed_mmPs = final_speed_mmPs - seg->data->accel_mmPs2 *
			(seg->data->distance_travelled_mm - seg->data->distance_mm);
		}
		else
		{  // decelerating
			set_speed_mmPs = final_speed_mmPs + seg->data->decel_mmPs2 *
			(seg->data->distance_travelled_mm - seg->data->distance_mm);
		}
		break;
		
	case 4: // completed. should be using the next ladder.
		set_speed_mmPs = 0;  // replace by next ladder
		break;
	}  // end switch(state)
	return set_speed_mmPs;
}
   
void setup() 
{ 
	sensor_speed_mmPs = 0;
	SetSpeeds();
    Serial.begin(115200); // C4 to C3 to C6
	/* A typical message is 20 ASCII characters of 20 bits each
	   (2 8-bit bytes / character + start bit + stop bit)
	   At 115,200 bits/s, a typical message takes 3.5 ms.
	   Thus there is about a 10 ms delay (best case) in passing 
	   information from C6 to C2.
	*/
}
char * GetWord(char * major, char * str)
{
	char * CSp1;

	CSp1 = strstr(str, major);
	if (CSp1!=NULL)
	CSp1 += strlen(major);
	return CSp1;
}

void ProcessMessage ()
{
	// Determine if message is "SENSOR {Speed xxx.xx}"	
	char * Args = GetWord ("SENSOR", IncomingMessage);
	if (Args == NULL)
		return;
	// SENSOR, so grab the new sensor_speed.
	char * Number = GetWord("Speed", Args);
	if (Number==NULL) return;
	// change } to 0
	char* end = strchr(Number, '}');
	if (end == NULL) return;
	*end = '\0';
	float data = atof(Number);
	// change back to }
	*end = '}';
	// convert speed from km/h to mm/s
	sensor_speed_mmPs = (long)(data * 1000000.0 / 3600.0);
	
}
void loop()
{
    static int section = 0;
    int incomingByte = 0;   // for incoming serial data
    unsigned long time, endTime;
    time = millis();
    endTime = time + LOOP_TIME_MS ;
    while (time < endTime)
    {
	if (Serial.available() > 0) 
	{
	    // read the incoming byte from C4:
	    incomingByte = Serial.read();
	    
	    IncomingMessage[InIndex] = (char)(incomingByte);
	    if (IncomingMessage[InIndex] == '\0'
		 || InIndex >= BUFFER_SIZE-1)
	    {
		    ProcessMessage();  // see what we got
		    InIndex = 0;
		    Serial.print(IncomingMessage); // pass msg on to C2
	    }
	   else
	   {
		++InIndex;    	
	   }
	    
	}
	time = millis();
    }
	
    long desired_speed_mmPs = Find_Speed (&profile[section], sensor_speed_mmPs);
    if (profile[section].data->state >= 4 && section < STEPS)
    {
	desired_speed_mmPs = Find_Speed (&profile[++section], sensor_speed_mmPs);
    }
    float desired_speed_revPs = ((float) desired_speed_mmPs) / (WHEEL_DIAMETER_MM * PI);
    Serial.print("DRIVE {Speed ");
    Serial.print(desired_speed_revPs);
    Serial.println("}");
}


tutorials
=========

Lab steps showing how vehicle components are built.

Elcano labs
Computer Software and Systems (CSS) 427, Embedded Systems
University of Washington, Bothell WA
Prof. Tyler C. Folsom

The following is a systematic way of taking small steps to get to a self-driving vehicle. Many of the steps are 
independent, and can be done in parallel. The vision and simulation tasks do not require vehicle hardware. These steps produce an experimental vehicle that can carry people at low speeds. Such a vehicle is not meant for operation on public roads. Getting to that stage will require much more testing, and careful design of critical systems. Formal methods to address fault tolerance are recommended.
The low-speed platform can be extended by adding a higher level that handles vision, lidar or radar, running on more capable platforms than Arduino. When these systems extract key information from a scene, they use USB to pass it to the low-level system. Stereo vision is much cheaper than lidar or radar, and should be the main high level sensor.
Text is on www.elcanoproject.org .
Production of a good simulator is important, but is a separate task.
1. Introduction
Vehicle architecture; Introduction to Arduino.
2. Drive-by-wire
Use PWM for servos and analog for motor on C2.
3. Cyclometer
Interrupts on C6.
4. Drive in straight line or circle 
Start and stop based on odometer. Serial communications from C6 to C3 to C2.
5. Obstacle detection
Find obstacles with sonars and steer to avoid them on C3.
6. INU
Use the compass and accelerometer on C6 to drive in a square.
7. GPS
Read GPS signal and log route to an SD card on C6.
8. Kalman filter
Fuse GPS, INU, Cyclometer and Dead Reckoning on C6.
9. Mapping
Represent local paths and use A* algorithm to select a route on C4.
10. Pilot
Send next route segment from C4 to C3 and negotiate obstacles.
11. Cone vision
Convert RGB image to mono orange / blue image. Take FFT and locate orange cones.
12. Lane vision
Detect lane edges.

Once we have two working vehicles, the next step is to make them communicate. Vehicle to vehicle (V2V) and vehicle to infrastructure (V2I) is where automated land transportation really gets interesting.


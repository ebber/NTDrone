# NTDrone
NT Drone DE
This is code to stabilize and control a drone.
3D printing design files coming soon!


Connections:
Pin	Connect to
2	MPU Interupt Pin
6	Motor C Signal Cable
9	Motor D Signal Cable
10	Motor A Signal Cable
11	Motor B Signal Cable
13	LED (blinks error codes)

A4	MPU SDA
A5 	MPU SCA



Using:
Motors/ESC: BEC ESC 10A
input voltage: {6, 7.5, 9, 12 (short periods of time)}

low signal: 900 uS
High signal: 2000uS

Arming sequence:
unneeded 

/*
Previous
Motors/ESC: HobbyKing RedBrick ESC 10A
input voltage: {6, 7.5, 9, 12 (short periods of time)}

low signal: 1000 uS
High signal: 2000uS

Arming sequence:
1000uS
delay(500);
2000uS
//armed
*/

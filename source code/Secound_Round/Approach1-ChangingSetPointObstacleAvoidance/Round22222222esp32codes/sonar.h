#include <NewPing.h>
///////////////////////////////////////////////////////////////////////// ---------------------------------------------------------------------------
// Example NewPing library sketch that pings 3 sensors 20 times a second.
// ---------------------------------------------------------------------------

#define MAX_DISTANCE 100 // Maximum distance (in cm) to ping.
#define frontDistanceThreshold 50 //cm

//left = 32
NewPing frontSonar = NewPing(33, 33, MAX_DISTANCE);
NewPing rightSonar = NewPing(32, 32, MAX_DISTANCE);
NewPing leftSonar = NewPing(23, 23, MAX_DISTANCE); 
//NewPing backSonar = NewPing(25, 25, MAX_DISTANCE); 

#define IRpin 36
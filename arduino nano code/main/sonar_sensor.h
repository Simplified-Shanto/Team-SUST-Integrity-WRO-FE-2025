#include <NewPing.h>
///////////////////////////////////////////////////////////////////////// ---------------------------------------------------------------------------
// Example NewPing library sketch that pings 3 sensors 20 times a second.
// ---------------------------------------------------------------------------

#define MAX_DISTANCE 100 // Maximum distance (in cm) to ping.
#define frontDistanceThreshold 60 //cm

NewPing middleSonar = NewPing(10, 10, MAX_DISTANCE);
NewPing leftSonar = NewPing(2, 2, MAX_DISTANCE);
NewPing rightSonar = NewPing(3, 3, MAX_DISTANCE); 
/*
#include "main.h"
#include "config.h"
#include "odom.h"
*/
/* Arrays */
/* Id's for Locations
0 = X 1 = Y
0: Blue Ramp
1: Red Ramp
2: North Roller
3: West Roller
4: East Roller
5: South Roller
6: Blue Goal
7: Red Goal
*/ /*
float coordinateLocations[2][8] {
 {0,140.2,110.98,139,29.43,1,122.63,17.78},
 {72.20,72.20,139,110.98,1,29.43,17.78,122.63}
}; */
/* Id's for goal Height
0: Lowest Goal point
1: Centered Goal point
2: Highest Goal point
*/
//float zCoordinates[3] {25,30.5,35.5};
/* Id's for Odometry offsets
0 = distance
1 = angle
0:Left Odom Wheel
1:Right Odom Wheel
2:Back Odom Wheel
*/
//float offsets[3] {1.53,1.73,8.72};// THIS NEEDS CALIBRATION ONCE ODOMETRY
//float dragWheelDiamater = 2.75; // drag wheel radius
/* Calculations */
/*double findAngle(int selector, double x = 0) { // selector uses coordniate list
//m=(y2-y1)/(x2-x1) slope formula
x = (180/M_PI)/(atan((positionY - coordinateLocations[1][selector])/(positionX - coordinateLocations[0][selector])));
return(x);
}
double findAngleMove(double X,double Y,double dest = 0) { // in degrees NOT radains
dest = (180/M_PI)*(atan((Y - positionY)/( X- positionX)));
return(dest);
}
double findDistance(double X1, double Y1, double X2, double Y2, double dist = 0) {
dist = sqrt(pow(Y2-Y1,2)+pow(X2-X1,2));
return(dist );
}

void odometryThread(void) {
    double diffrence = 0;
    double encoderLV = 0;
    double encoderRV = 0;
    double encoderBV = 0;
    while(true) {
    diffrence = encoderLV - encoderRV;        
    }
    
*/ 






//}
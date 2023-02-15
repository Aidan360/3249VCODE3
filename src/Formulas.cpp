#include "main.h"


double findDistance(double X1, double Y1, double X2, double Y2, double dist) {
    dist = sqrt(pow(Y2-Y1,2)+
    pow(X2-X1,2));
    return(dist);
}
// degrees to radians formula 
double radians(double deg, double x) {
    x = (deg*(M_PI/180));
    return(x);
}
// radians to degrees formula
double degrees(double rad, double x) {
    x = (rad*(180/M_PI));
    return(x);
}
int sgn(int n) {
    if (n > 0) return 1;
    if (n < 0) return -1;
    return 0;
}
double flyWheelVelocityCalc(float target,float totalDistance, float occislation, float angle) {
    double a = sqrt(
            (gravity*pow(totalDistance,2)
            /((2*cos(radians(angle)))
            *(target-turretOffsetZ-totalDistance*tan(radians(angle))))));
    double n = (a*60)/(2*M_PI*flyWheelRadius);
    return(n);
}
double flyWheelVelocityE(float target,float totalDistance, float occislation, float angle) {
    double a = sqrt(
            (gravity*pow(totalDistance,2)
            /((2*cos(radians(angle)))*(target-turretOffsetZ-totalDistance*tan(radians(angle))))));
    double v_final = sqrt((0.5 * discMass * pow(a,2)) / (0.5 * (flyWheelMass+discMass)));
    double n = (v_final*60)/(2*M_PI*(flyWheelRadius+occislation));
    return(n);
}
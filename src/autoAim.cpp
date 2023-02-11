#include "main.h"
#include "okapi/api/units/QAngularSpeed.hpp"
#include "pros/apix.h"
#include <cmath>
//#include "odom.h"
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
  */
  float coordinateLocations[2][8] = {
  {0,140.2,110.98,139,29.43,1,122.63,17.78},
  {72.20,72.20,139,110.98,1,29.43,17.78,122.63}
  }; 
  /* Id's for goal Height
  0: Lowest Goal point
  1: Centered Goal point
  2: Highest Goal point
  */
  float zCoordinates[3] = {25.8,30.5,35.5-0.8};
/*Formulas*/

// distance formula
double findDistance(double X1, double Y1, double X2, double Y2, double dist = 0) {
    dist = sqrt(pow(Y2-Y1,2)+
    pow(X2-X1,2));
    return(dist);
}
// degrees to radians formula 
double radians(double deg, double x = 0) {
    x = (deg*(M_PI/180));
    return(x);
}
// radians to degrees formula
double degrees(double rad, double x = 0) {
    x = (rad*(180/M_PI));
    return(x);
}
int sgn(int n) {
    if (n > 0) return 1;
    if (n < 0) return -1;
    return 0;
}
double flyWheelVelocity(float target,float totalDistance, float angle = flyWheelAngle) {
    double a = sqrt(
            (gravity*pow(totalDistance,2)
            /((2*cos(radians(angle)))*(target-turretOffsetZ-totalDistance*tan(radians(angle))))));
    double n = (a*60)/(2*M_PI*flyWheelRadius);
    return(n);
}
double flyWheelVelocityE(float target,float totalDistance, float angle = flyWheelAngle) {
    double a = sqrt(
            (gravity*pow(totalDistance,2)
            /((2*cos(radians(angle)))*(target-turretOffsetZ-totalDistance*tan(radians(angle))))));
    double v_final = sqrt((0.5 * discMass * pow(a,2)) / (0.5 * (flyWheelMass+discMass)));
    double n = (v_final*60)/(2*M_PI*flyWheelRadius);
    return(n);
}
//v_final = sqrt((0.5 * m_object * v^2) / (0.5 * m_total))


int turretPID() {
    pros::Motor turretMotor (turretMotor_PORT);
    pros::ADIGyro EXT_GyroTurret ({{expander_PORT,EXT_GyroTurretPort}});
    double factorP = 0.05;
    double factorI = 0.001;
    double factorD = 0.25;
    double error = 0;
    double intergral = 0;
    double derivitave = 0;
    double lastError = 1;
    double fixing = 0;
    double errorAverage = error;
    double lastErrorAverage = 0;
    double loopCount = 0;
    while (abs(error) >= 0) {
        error = (EXT_GyroTurret.get_value() - turretError);
        intergral = intergral + error;
        derivitave = error - lastError;
        fixing = ((error*factorP)+(intergral*factorI)+(derivitave*factorD));
    //if (turretError > 180) {
        turretMotor.move_velocity(fixing);
    //}
    //else {
        //turretMotor.move_velocity(-fixing);
    //}
        lastError = error;
        pros::c::delay(10);
        error = EXT_GyroTurret.get_value() - turretError;
        errorAverage = (errorAverage+error+lastError)/3;
        loopCount += 1;
    if (loopCount > 75) { // This is so previous larger errors don't break the infinite loop fix
        lastErrorAverage = errorAverage;
        errorAverage = 0;
    }
    if (turretBreak == true) {
        break;
    }
    }
    turretMotor.brake();
    return(1);
}
// Pissing Incels degrade at Fur fest (PID controller + Feed forward)
int turretPIDFF() {
    pros::Motor turretMotor (turretMotor_PORT);
    pros::ADIGyro EXT_GyroTurret ({{expander_PORT,EXT_GyroTurretPort}});
    
    // Factors 
        // PID factors
        double kP = 0.15; // Proportional factor, Changes proportionally to error
        double kI = 0.001; // Integral factor, Changes based on time of error
        double kD = 0.2; // Derivate factor, Changes based on the rate of change in error
        // FF Factors 
        double kS = 0.1; // Minimum voltage to get the motor moving
        double kV = 0.1; // voltage required to sustain velocity
        double kA = 0.01; // voltage required accelerate
    // Values 
        double integral = 0;
        double derivative = 0;
        double error = EXT_GyroTurret.get_value()-atan2(positionY-coordinateLocations[1][target],positionX-coordinateLocations[0][target]);
        double lastError = EXT_GyroTurret.get_value()-atan2(positionY-coordinateLocations[1][target],positionX-coordinateLocations[0][target]);
        double output;
        double goalRadius = 15.73/2; // inches for limits
        // The upper and limit changes based off of how far the robot is away from the goal

        double upperLimit = atan2(
            (positionY - (coordinateLocations[1][target] + goalRadius)),
            (positionX - (coordinateLocations[0][target] + goalRadius)));     
        double lowerLimit = atan2(            
            (positionY - (coordinateLocations[1][target] + goalRadius)),
            (positionX - (coordinateLocations[0][target] + goalRadius)));
        double pidVoltOutput; // Output = kP*error + kI*Integral[error] + kD*Derivative[error]
        double ffVoltOutput; // output = (kS * sgn(V)) + (kV * V) + (kA * A) + kG
    while (true) {
        error = EXT_GyroTurret.get_value()-atan2(positionY-coordinateLocations[1][target],positionX-coordinateLocations[0][target]);
        integral = integral+error; // integral scales overtime
        derivative = error-lastError; // derivative changes based on feedback 
      
        pidVoltOutput = kP*error + kI*integral + kD*derivative; // PID final calculation
        ffVoltOutput = (kS* sgn(turretVelocity))+(kV*turretVelocity) + (kA * turretAcceleration); // feed forward final calculation
        output = pidVoltOutput + ffVoltOutput; // combines PID + FF
      
        if (sgn(error) == 1) { // because I'm running voltages I gotta set the motor direction 
            turretMotor.set_reversed(true);
        }
        else {
            turretMotor.set_reversed(false); 
        }
        turretMotor.move_voltage(output); // Final output to voltages. Voltages are powerful enough to move the turret whenever it wants to 
        lowerLimit = atan2(            
            (positionY - (coordinateLocations[1][target] + goalRadius)),
            (positionX - (coordinateLocations[0][target] + goalRadius)));
        upperLimit = atan2(
            (positionY - (coordinateLocations[1][target] + goalRadius)),
            (positionX - (coordinateLocations[0][target] + goalRadius)));

        if (upperLimit > EXT_GyroTurret.get_value() > lowerLimit) {
            lockOn = true;
        }  
        pros::c::task_delay(10);
        lastError = error;
    }
}

int flywheelPIDFF() {
    pros::Motor flyWheel (flyWheel_PORT);
    // Factors 
        // PID factors
        double kP = 0.15; // Proportional factor, Changes proportionally to error
        double kI = 0.001; // Integral factor, Changes based on time of error
        double kD = 0.2; // Derivate factor, Changes based on the rate of change in error
        // FF Factors 
        double kS = 0.9; // Minimum voltage to get the motor moving
        double kV = 0.85; // voltage required to sustain velocity
        double kA = 1; // voltage required accelerate
    // Values 
    
        double integral = 0;
        double derivative = 0;
        double totalDistance = findDistance(coordinateLocations[0][target],positionX,coordinateLocations[1][target],positionY);
        double error = flyWheel.get_actual_velocity()*18 - flyWheelVelocity(zCoordinates[1],totalDistance);
        double lastError;
        double output;
        double goalRadius = 15.73/2; // inches for limits
        // The upper and limit changes based off of how far the robot is away from the goal
        double upperLimit = flyWheelVelocity(zCoordinates[2],totalDistance);     
        double lowerLimit = flyWheelVelocity(zCoordinates[0],totalDistance);
        double pidVoltOutput; // Output = kP*error + kI*Integral[error] + kD*Derivative[error]
        double ffVoltOutput; // output = (kS * sgn(V)) + (kV * V) + (kA * A) + kG
    while (true) {
        error = flyWheel.get_actual_velocity()*18 - flyWheelVelocityE(zCoordinates[1],2);
        integral = integral+error; // integral scales overtime
        derivative = error-lastError; // derivative changes based on feedback 
      
        pidVoltOutput = kP*error + kI*integral + kD*derivative; // PID final calculation
        ffVoltOutput = (kS* sgn(turretVelocity))+(kV*turretVelocity) + (kA * turretAcceleration); // feed forward final calculation
        output = pidVoltOutput + ffVoltOutput; // combines PID + FF
        flyWheel.move_voltage(output); // Final output to voltages. Voltages are powerful enough to move the turret whenever it wants to 
        lowerLimit = flyWheelVelocityE(zCoordinates[0],totalDistance);
        upperLimit = flyWheelVelocityE(zCoordinates[2],totalDistance);

        if (upperLimit > flyWheel.get_actual_velocity()*18 > lowerLimit) {
            lockOn = true;
        }  
        pros::c::task_delay(10);
        lastError = error;
    }
}



int aimBotThread() { // auto aim thread
    pros::ADIGyro EXT_GyroTurret ({{expander_PORT,EXT_GyroTurretPort}});
    
    positionX = chassis_controller -> getState().x.convert(okapi::inch);
    positionY = chassis_controller -> getState().y.convert(okapi::inch);
    
    degHead = chassis_controller -> getState().theta.convert(okapi::degree);
    //float flyWheelContactAngle = 70; // needs calibration
    double totalDistance = 0;
    double totalTurretRotation = EXT_GyroTurret.get_value(); // to make sure that we don't go overboard and twist/rip wires
    //float maxTotalTurretRotation = 720;
    //float pneumaticSpeed =  8;// in inches per second idk how this is that fast
    double relativeVelocity;
    double ejectVelocity;
    double flywheelVelocity;
    double velocityXsec = velocityX*1000;
    double velocityYsec = velocityY*1000;
    double v_final;
    double springForce = (11501.492602*(M_PI*2*flyWheelRadius))/(flyWheelRadius*2);
    
    if ((redTeam = true)) {
        target = 7;
    }
    else {
        target = 8;
    }
    pros::Motor flyWheel (flyWheel_PORT);
    //task myTask = task(turretPID);
    while(true) {
        //totalTurretRotation += degHead - lastdegHead;
        //velocityXsec = velocityX*100; // inches per ms to inches per second
        //velocityYsec = velocityY*100;
        //relativeVelocity = sqrt(pow(velocityXsec,2)+pow(velocityYsec,2)+2*velocityX*velocityYsec*cos(90-findAngle(target)));
        //totalDistance = sqrt(pow(findDistance(positionX,positionY,coordinateLocations[0][target],coordinateLocations[1][target]),2)
        totalDistance = atan2(positionY,positionX);
        //+pow(tan(radians(flyWheelAngle))*findDistance(positionX,positionY,coordinateLocations[0][target],coordinateLocations[1][target]),2));
        if((aimBot=true)) {
            // Turret movement very simple :D
            // it isnt D:
        
            // decides if the turret is within acceptable limits
            // sqrt(g*d^2/(2*cos(a)*)(hg-hr-d*tan(a))
            ejectVelocity = sqrt((gravity*pow(totalDistance,2)/((2*cos(radians(flyWheelAngle)))*(zCoordinates[1]-turretOffsetZ-totalDistance*tan(radians(flyWheelAngle)))))); // inches per second
            // RPM = (v*60)/(2*pi*r_wheel*sqrt(i_object/i_wheel+i_deltaWheel))
            v_final = sqrt(abs((0.5*discMass*pow(ejectVelocity,2))/(0.5*massTotal)-(0.5*springForce*flyWheelCompression)/(0.5*massTotal)));
            flywheelVelocity = (ejectVelocity*60)/(2*M_PI*flyWheelRadius);
            //flywheelVelocity = ((ejectVelocity*60)/(2*M_PI*flyWheelRadius*sqrt(discInertia/(flyWheelInertia+flyWheelInertialIncrease))))/flyWheelGearRatio;
        
            flyWheel.move_velocity(flywheelVelocity);
        // flyWheel2.move_velocity(flywheelVelocity,rpm);
        }
        else {
        //  EXT_GyroTurret.get_value() = totalTurretRotation;
            //turretError = degHead;
        }
            pros::c::task_delay(10);
    }
}
#include "main.h"
#include <iostream>
//#include "pros/apix.h"
//#include <cmath>
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
float varChange = 0;

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
void turretPIDFF() {
    pros::Motor turretMotor (turretMotor_PORT);
    pros::ADIGyro EXT_GyroTurret ({{expander_PORT,EXT_GyroTurretPort}});


    /* log  
    1: kP = 0.1 F
    2: kP = 0.2 F
    3: kP = 0.3 F
    4: kP = 1 F
    5: kP = 2 F
    6: kP = 10 F
    7: kP = 100 F
    8: kP = 1000 Worked lmao
    */



    // Factors 
        // PID factors
        double kP = 1000; // Proportional factor, Changes proportionally to error
        double kI = 0.00; // Integral factor, Changes based on time of error
        double kD = 0; // Derivate factor, Changes based on the rate of change in error
        // FF Factors 
     //   double kS = 1.6; // Minimum voltage to get the motor moving
     //   double kV = 1.7; // voltage required to sustain velocity
     //   double kA = 2.3; // voltage required accelerate
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
        double ffVoltOutput = 0; // output = (kS * sgn(V)) + (kV * V) + (kA * A) + kG
    while (true) {
        error = EXT_GyroTurret.get_value()-atan2(positionY-coordinateLocations[1][target],positionX-coordinateLocations[0][target]);
        integral = integral+error; // integral scales overtime
        derivative = error-lastError; // derivative changes based on feedback 
      
        pidVoltOutput = kP*error + kI*integral + kD*derivative; // PID final calculation
        //ffVoltOutput = (kS* sgn(turretVelocity))+(kV*turretVelocity) + (kA * turretAcceleration); // feed forward final calculation
        output = pidVoltOutput; //+ ffVoltOutput; // combines PID + FF
      
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



void flywheelPIDFF() {
    pros::Motor flyWheel (flyWheel_PORT);

    // Factors 
        // PID factors
        double kP = 0.5; // Proportional factor, Changes proportionally to error
        double kI = 0; // Integral factor, Changes based on time of error
        double kD = 0; // Derivate factor, Changes based on the rate of change in error
        // FF Factors 
        double kS = 0; // Minimum voltage to get the motor moving
        double kV = 0; // voltage required to sustain velocity
        double kA = 0; // voltage required accelerate
    // Values 
    
        double integral = 0;
        double derivative = 0;
        double totalDistance = findDistance(coordinateLocations[0][target],positionX,coordinateLocations[1][target],positionY);
        double error = flyWheel.get_actual_velocity()*18 - flyWheelVelocityCalc(zCoordinates[1],totalDistance);
        double lastError;
        double output;

        // The upper and limit changes based off of how far the robot is away from the goal
        double upperLimit = flyWheelVelocityCalc(zCoordinates[2],totalDistance);     
        double lowerLimit = flyWheelVelocityCalc(zCoordinates[0],totalDistance);
        double pidVoltOutput; // Output = kP*error + kI*Integral[error] + kD*Derivative[error]
        double ffVoltOutput; // output = (kS * sgn(V)) + (kV * V) + (kA * A) + kG
    //debug 
    std::shared_ptr<graphy::AsyncGrapher> grapher(new graphy::AsyncGrapher("Flywheel Velocity vs. Time"));

    // Add data types
     grapher->addDataType("Desired Vel", COLOR_ORANGE);
     grapher->addDataType("Actual Vel", COLOR_AQUAMARINE);

    // Start grapher task
     grapher->startTask();

    while (true) {
        error = flyWheel.get_actual_velocity() - flyWheelVelocityCalc(zCoordinates[1],totalDistance);
        integral = integral+error; // integral scales overtime
        derivative = error-lastError; // derivative changes based on feedback 
      
        pidVoltOutput = kP*error + kI*integral + kD*derivative; // PID final calculation
        
        ffVoltOutput = (kS* sgn(flyWheelVelocityCalc(zCoordinates[1],totalDistance)))+(kV*flyWheelVelocityCalc(zCoordinates[1],totalDistance)) + (kA * (flyWheelVelocityCalc(zCoordinates[1],totalDistance)/10)); // feed forward final calculation
        
        output = pidVoltOutput + ffVoltOutput; // combines PID + FF
        flyWheel.set_reversed(true);
        flyWheel.move_voltage(output); // Final output to voltages. Voltages are powerful enough to move the turret whenever it wants to 
        
        
        lowerLimit = flyWheelVelocityCalc(zCoordinates[0],totalDistance);
        upperLimit = flyWheelVelocityCalc(zCoordinates[2],totalDistance);
      
        if (upperLimit > flyWheel.get_actual_velocity()*18 > lowerLimit) {
            lockOn = true;
        }  
        grapher->update("Desired Vel", flyWheelVelocityCalc(zCoordinates[1],totalDistance));
        grapher->update("Actual Vel", flyWheel.get_actual_velocity());
        pros::c::task_delay(10);
        lastError = error;
        
    }
}

    pros::ADIGyro EXT_GyroTurret ({{expander_PORT,EXT_GyroTurretPort}});
    std::shared_ptr<okapi::OdomChassisController> chassis_controller;
    std::shared_ptr<okapi::AsyncVelocityController<double, double> > flywheel_controller;

void aimBotThread() { // auto aim thread
   // positionX = chassis_controller -> getState().x.convert(okapi::inch);
   // positionY = chassis_controller -> getState().y.convert(okapi::inch);    
   // degHead = chassis_controller -> getState().theta.convert(okapi::degree);
    //float flyWheelContactAngle = 70; // needs calibration
 double totalDistance = findDistance(coordinateLocations[0][target],positionX,coordinateLocations[1][target],positionY);
  //  double totalTurretRotation = EXT_GyroTurret.get_value(); // to make sure that we don't go overboard and twist/rip wires
    //float maxTotalTurretRotation = 720;
    //float pneumaticSpeed =  8;// in inches per second idk how this is that fast
    //double relativeVelocity;
    //double ejectVelocity;
    //double flywheelVelocityCalc;
    //double velocityXsec = velocityX*1000;
    //double velocityYsec = velocityY*1000;
    //double v_final;
    //double springForce = (11501.492602*(M_PI*2*flyWheelRadius))/(flyWheelRadius*2);
    
    if ((redTeam = true)) {
        target = 7;
    }
    else {
        target = 8;
    }
    pros::Motor flyWheel (flyWheel_PORT);
    //task myTask = task(turretPID);
    while(true) {
        
      positionX = chassis_controller -> getState().x.convert(okapi::inch);
       // positionY = chassis_controller -> getState().y.convert(okapi::inch);    
      // degHead = chassis_controller -> getState().theta.convert(okapi::degree);
       totalDistance = findDistance(coordinateLocations[0][target],positionX,coordinateLocations[1][target],positionY);
  //      flywheel_controller -> setTarget(flyWheelVelocityCalc(zCoordinates[1],totalDistance));
        pros::c::task_delay(10);
    }
}

void competition_initialize() {
   // pros::Task my_task1(aimBotThread);
    pros::Task my_task5(displayThread);
      // pros::Task my_task3(flywheelPIDFF);
       //pros::Task my_task4(thread4);
    pros::Task my_task2(turretPIDFF);
    

}
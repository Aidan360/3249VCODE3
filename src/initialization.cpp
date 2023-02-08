#include "main.h"
#include "config.h"
#include "okapi/api.hpp"

void initialize() {






    pros::Motor leftFrontMotor_initializer (leftFrontMotor_PORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    pros::Motor leftBackMotor_initializer (leftBackMotor_PORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    pros::Motor rightFrontMotor_initializer (rightFrontMotor_PORT, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
    pros::Motor rightBackMotor_initializer (rightBackMotor_PORT, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
    pros::Motor turretMotor_initializer (turretMotor_PORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    pros::Motor intake_initializer (intakeMotor_PORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    pros::Motor flywheel_initializer (flyWheel_PORT,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_DEGREES);
    pros::ADIEncoder encoderL_initializer (encoderLeftTop_PORT,encoderLeftBottom_PORT, false);
    pros::ADIEncoder encoderR_initializer (encoderRightTop_PORT,encoderRightBottom_PORT, false);
    pros::ADIEncoder encoderB_initializer (encoderBackTop_PORT,encoderBackBottom_PORT, false);
    pros::ADIDigitalOut indexer_initializer (indexer_PORT);
    pros::ADIDigitalOut expansion_initializer (expansion_PORT);
    pros::ADIDigitalIn intakeSensor_initializer ({{expander_PORT,EXT_IntakeSensorPort}});  
    pros::ADIGyro EXT_GyroTurret_initializer ({{expander_PORT,EXT_GyroTurretPort}});


  //  chassis_controller = okapi::ChassisControllerBuilder()
       // .withOdometry
    // EXT_GyroTurret.calibrate;
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello PROS User!");


    positionX = 0;
    positionY = 0;
    dragWheelDiamater = 2.75; // drag wheel radius
     dragWheelCirc = dragWheelDiamater*M_PI;
     gravity = -386.08858267717; // inches per second
/*Launch Math*/


// Polycarb = 0.1941916032lbs
// flywheel Weight = 0.24 lbs
// flywheel = 0.26 lbs
  flyWheelMass = 0.49; // pounds (m)
  flyWheelRadius = 2; // inches (r)
  flyWheelCrossArea = M_PI*2*2; // inches (A)
  flyWheelCompression = 0.15; // inches (l)
  flyWheelAngle = 35; // angle in degrees (a)
  flyWheelGearRatio = 18; // multipler for gearing
// Compression Force (F=E*A*l)/r
  flyWheelCompressionForce = (flyWheelCrossArea*flyWheelCompression)/(flyWheelRadius*2); // (f)
// Inertia Calc, m*r^2
  discInertia = 0.121254*pow(5.5/2,2); // i_disc
  discMass = 0.121254;
  massTotal = flyWheelMass+discMass;
  flyWheelInertia = (flyWheelMass*pow(flyWheelRadius,2))/2; // i_wheel
// Inertial increase from Compression i_delta = (F*r^2)/(3*E)
  flyWheelInertialIncrease = (flyWheelCompressionForce*pow((flyWheelRadius*2),2))/(3); // i_wheelDelta
turretThreashold = 3;
turretOffsetZ = 11.5; // turret offset height from ground so it can properly aim
flywheelOffset = 5; // needs to be measured
middleLineX1 = 0;
middleLineX2 = 140.02;
middleLineY1 = 0;
middleLineXY = 140.02;
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
/*coordinateLocations[2][8] = {
{0,140.2,110.98,139,29.43,1,122.63,17.78},
{72.20,72.20,139,110.98,1,29.43,17.78,122.63}
}; */
/* Id's for goal Height
0: Lowest Goal point
1: Centered Goal point
2: Highest Goal point
*/
//zCoordinates[3] = {25,30.5,35.5};
/* Id's for Odometry offsets
0 = distance
1 = angle
0:Left Odom Wheel
1:Right Odom Wheel
2:Back Odom Wheel
*/
//offsets[3] = {1.53,1.73,8.72};// THIS NEEDS CALIBRATION ONCE ODOMETRY WHEELS ARE IN THE ROBOT < ------------------------------------------------------------
leftRightLength = 5.5; // length between left odom wheel and right odom wheel
// Low Z and High Z is for aiming. EX if Turret aim is between Zlow < Yangle < ZHigh then Fire



}


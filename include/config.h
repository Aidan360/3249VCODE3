#include "main.h"
#include "api.h"

// Smart ports
#define leftFrontMotor_PORT 1
#define leftBackMotor_PORT 2
#define rightFrontMotor_PORT 3
#define rightBackMotor_PORT 4
#define flyWheel_PORT 5
#define turretMotor_PORT 6
#define intakeMotor_PORT 7
//#define eigthMotorPort 8;
#define expander_PORT 9
// Brain 3-Wire Ports
#define potentiometer_PORT 1
#define expansionPneumatic_Port 2
#define encoderLeftPortTop_Port 3
#define encoderLeftPortBottom_Port 4
#define encoderRightPortTop_Port 5
#define encoderRightPortBottom 6
#define encoderBackPortTop 7
#define encoderBackPortBottom 8
// Expander Ports
#define exIntakeSensorPort 1
#define exFlywheelSensorPort 2
#define exIndexerPneumaticPort 3
#define exGyroTurretPort 4

/*
 leftFrontMotor.spin(forward,(Controller1.Axis3.position() + Controller1.Axis1.position())^2/100,velocityUnits::pct); // Arcade control
 leftBackMotor.spin(forward,(Controller1.Axis3.position() + Controller1.Axis1.position())^2/100,velocityUnits::pct);
 rightFrontMotor.spin(forward,(Controller1.Axis3.position() - Controller1.Axis1.position())^2/100,velocityUnits::pct);
 rightBackMotor.spin(forward,(Controller1.Axis3.position() - Controller1.Axis1.position())^2/100,velocityUnits::pct);
*/
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
#define indexer_PORT 2
#define encoderLeftTop_PORT 5
#define encoderLeftBottom_PORT 6
#define encoderRightTop_PORT 3
#define encoderRightBottom_PORT 4
#define encoderBackTop_PORT 7
#define encoderBackBottom_PORT 8
// Expander Ports
#define EXT_IntakeSensorPort 1
#define EXT_FlywheelSensorPort 2
#define EXT_ExpansionPneumaticPort 3
#define EXT_GyroTurretPort 4

/*
 leftFrontMotor.spin(forward,(Controller1.Axis3.position() + Controller1.Axis1.position())^2/100,velocityUnits::pct); // Arcade control
 leftBackMotor.spin(forward,(Controller1.Axis3.position() + Controller1.Axis1.position())^2/100,velocityUnits::pct);
 rightFrontMotor.spin(forward,(Controller1.Axis3.position() - Controller1.Axis1.position())^2/100,velocityUnits::pct);
 rightBackMotor.spin(forward,(Controller1.Axis3.position() - Controller1.Axis1.position())^2/100,velocityUnits::pct);
*/
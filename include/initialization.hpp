#include "api.h"
#include "okapi/api.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"
#include "okapi/api/control/async/asyncVelocityController.hpp"
// Smart ports
#define leftFrontMotor_PORT 1
#define leftBackMotor_PORT 2
#define rightFrontMotor_PORT 3
#define rightBackMotor_PORT 4
#define flyWheel_PORT 9
#define turretMotor_PORT 6
#define intakeMotor_PORT 8
//#define eigthMotorPort 8;
#define expander_PORT 10
// Brain 3-Wire Ports
#define expansion_PORT 1
#define indexer_PORT 2
#define encoderLeftTop_PORT 3
#define encoderLeftBottom_PORT 4
#define encoderRightTop_PORT 5
#define encoderRightBottom_PORT 6
#define encoderBackTop_PORT 7
#define encoderBackBottom_PORT 8
// Expander Ports
#define EXT_IntakeSensorPort 4
#define EXT_FlywheelSensorPort 2
#define EXT_ExpansionPneumaticPort 3
#define EXT_GyroTurretPort 'a'
extern std::shared_ptr<okapi::SkidSteerModel> chassis_model;
/*Launch Math*/
extern okapi::ADIEncoder encoderBack;
extern okapi::ADIEncoder encoderLeft;
extern okapi::ADIEncoder encoderRight;
extern okapi::MotorGroup leftMotors;
extern okapi::MotorGroup rightMotors;
extern okapi::Motor flyWheel_Motor;
extern std::shared_ptr<okapi::OdomChassisController> chassis_controller;
extern std::shared_ptr<okapi::AsyncVelocityController<double, double>> flywheel_controller;
// Threads
extern void flywheelPIDFF();
extern void displayThread();
extern void turretPIDFF(); 


extern float varChange; 
//extern double M_PI = 3.14;
#include "main.h"
#include "api.h"
#include "okapi/api.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"

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
#define EXT_IntakeSensorPort 1
#define EXT_FlywheelSensorPort 2
#define EXT_ExpansionPneumaticPort 3
#define EXT_GyroTurretPort 4
extern std::shared_ptr<okapi::SkidSteerModel> chassis_model;
extern float dragWheelDiamater; // drag wheel radius
extern double dragWheelCirc;
extern double gravity ; // inches per second
/*Launch Math*/
extern okapi::ADIEncoder encoderBack;
extern okapi::ADIEncoder encoderLeft;
extern okapi::ADIEncoder encoderRight;
extern okapi::MotorGroup leftMotors;
extern okapi::MotorGroup rightMotors;
// Polycarb = 0.1941916032lbs
// flywheel Weight = 0.24 lbs
// flywheel = 0.26 lbs
extern double flyWheelMass; // pounds (m)
extern double flyWheelRadius; // inches (r)
extern double flyWheelCrossArea; // inches (A)
extern double flyWheelCompression; // inches (l)
extern double flyWheelAngle; // angle in degrees (a)
extern double flyWheelGearRatio; // multipler for gearing
// Compression Force (F=E*A*l)/r
extern double flyWheelCompressionForce; // (f)
// Inertia Calc, m*r^2
extern double discInertia; // i_disc
extern double discMass;
extern double massTotal;
extern double flyWheelInertia; // i_wheel
// Inertial increase from Compression i_delta = (F*r^2)/(3*E)
extern double flyWheelInertialIncrease; // i_wheelDelta

extern double turretVelocity;
extern double turretAcceleration;

/* Odometry */
// PROGRAM USES RADIANS
extern double degHead; // in degrees
extern double lastdegHead;
extern double turretError;
extern bool turretBreak;
extern float turretThreashold;
extern float turretOffsetZ ; // turret offset height from ground so it can properly aim
extern float flywheelOffset; // needs to be measured
extern float distanceFromCorners; // Needs calibration
extern int discLoad;
extern bool intakeDisc;
extern bool turretDisc;

extern bool flyWheelDisc;
// Starting Offsets is how far the sensor is away from the walls or 0,0 on a grid
extern double startingOffsetX;
extern double startingOffsetY;
// Grid is for marking objects and where they are
extern double positionX;
extern double positionY;
extern double velocityX;
extern double velocityY;
extern double rotVelocity;
extern double dist;
// blue near the red high goal and red is near the blue high goal  0,0 is the bottom RIGHT of the feild
// Object coordinates for object locations from 0,0 in inchesf
// for saftey it can be if X = Y and autonmous = true then break?
extern float middleLineX1;
extern float middleLineX2;
extern float middleLineY1;
extern float middleLineXY;
extern std::shared_ptr<okapi::OdomChassisController> chassis_controller;

extern float offsets[3];// THIS NEEDS CALIBRATION ONCE ODOMETRY WHEELS ARE IN THE ROBOT < ------------------------------------------------------------
extern float leftRightLength; // length between left odom wheel and right odom wheel
// Low Z and High Z is for aiming. EX if Turret aim is between Zlow < Yangle < ZHigh then Fire

extern bool redTeam;
extern int target;
extern bool aimBot;
extern bool flyWheelCalc;
extern bool lockOn;
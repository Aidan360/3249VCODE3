#include "main.h"
#include "okapi/api.hpp"
#include "initialization.hpp"
okapi::MotorGroup leftMotors = {1,2};
okapi::MotorGroup rightMotors = {3,4};
okapi::ADIEncoder encoderLeft = {'C','D'};
okapi::ADIEncoder encoderRight = {'E','F'};
okapi::ADIEncoder encoderBack = {'G','H'};
double positionX = 0;
double positionY = 0;
float dragWheelDiamater = 2.75; // drag wheel radius
double dragWheelCirc = dragWheelDiamater*M_PI;
double gravity = -386.08858267717; // inches per second
void initialize() {

//double M_PI = 3.14159265358979323846264338327950288;

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
    pros::ADIEncoder encoderL (encoderLeftTop_PORT,encoderLeftBottom_PORT);
    pros::ADIEncoder encoderR (encoderRightTop_PORT,encoderRightBottom_PORT);
    pros::ADIEncoder encoderB (encoderBackTop_PORT,encoderBackBottom_PORT);
    encoderL.reset();
    encoderR.reset();
    encoderB.reset();
    pros::delay(500);

  std::shared_ptr<okapi::OdomChassisController> chassis_controller = okapi::ChassisControllerBuilder() 
    .withMotors (
      leftMotors,
      rightMotors
    )
    //		// Gearset | gear ratio | wheel diameter | wheel track (driving) | TPR
	//	.withDimensions({okapi::AbstractMotor::gearset::green, (1./1.)}, {{3.25_in, 15._in + 15._in/16.}, okapi::imev5GreenTPR})
    .withDimensions(
      {okapi::AbstractMotor::gearset::green, 
      (1./1.)}, 
      {{4.125, 
      15.75}, 
      okapi::imev5GreenTPR})
    .withSensors(        
        encoderLeft, // left encoder in ADI ports A & B
        encoderRight,  // right encoder in ADI ports C & D (reversed)
        encoderBack // middle encoder in ADI ports E & F)
    )
		// Tracking wheel diameter | wheel track (tracking) | middle encoder distance | center tracking wheel diameter
    .withOdometry(
      {
        {2.75,
        5.5,
        4,
        2.75}, 
      okapi::quadEncoderTPR})
  	.buildOdometry();
    std::shared_ptr<okapi::SkidSteerModel> chassis_model = std::dynamic_pointer_cast<okapi::SkidSteerModel>(chassis_controller -> getModel());
	  //chassis_max_vel = chassis_model -> getMaxVelocity();
    // EXT_GyroTurret.calibrate;
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello PROS User!");

  //   chassis_controller -> setState(0,0,0);

  /*Launch Math*/


  // Polycarb = 0.1941916032lbs
  // flywheel Weight = 0.24 lbs
  // flywheel = 0.26 lbs 
  /*
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
  middleLineXY = 140.02; */

}


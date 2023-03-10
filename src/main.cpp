#include "main.h"
#include "time.h"
#include "initialization.hpp"
//#include "odom.h"
#define _USE_MATH_DEFINES
pros::ADIDigitalOut indexer (indexer_PORT);
pros::Motor flyWheel (flyWheel_PORT);
pros::Motor intake (intakeMotor_PORT);
pros::Motor leftFrontMotor (leftFrontMotor_PORT);
pros::Motor leftBackMotor (leftBackMotor_PORT);
pros::Motor rightFrontMotor (rightFrontMotor_PORT);
pros::Motor rightBackMotor (rightBackMotor_PORT);
pros::ADIDigitalOut expansion (expansion_PORT);
pros::Motor turretMotor (turretMotor_PORT);




// Begin project code
/* TODO:
Get the robots current velocity either in odom loop or somewhere else
Create movement safezones. (Autonomous line and potentially the low goal L)
Have the ability to switch
Merge flywheel thread and odom thread
Account for change of angle in the turret while moving
Varible flywheel positon change*** after comp probs
Fix turret PID
Fix PIDS
*/


//double M_PI = 3.14159265358979323846264338327950288;


float trackLength = 15;
float inchesToDegrees(double inches, double n = 0) {
  double circ = ((4.125*M_PI)/inches)/360;
  n = circ;
  return(n);
}


/*Launch Math*/


/* Display */
bool thread1On = false;
bool thread2On = false;
bool thread3On = false;
bool thread4On = false;
bool compReady = false;
// Misc
float gameTime = 105;
int null = 0;
int target;
bool activePID;

/* Threads */
/* How to make threads
void thread1() {
int threadLoopCount = 0;
while(true) {
Brain.Screen.setCursor(2, 1);
Brain.Screen.print("Thread #1 Iterations (250ms loop): %d", threadLoopCount);
threadLoopCount += 1;
pros::c::delay(250, msec);
}
}
thread myThread1 = thread(thread1); // how to execute in code
*/
bool redTeam = true;
bool aimBot = true;
bool autoFire = true;
bool lockOn = false;
double distfeet = 0;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

//Autonomous functions 
/*
void movePiD(double X, double Y) {
  pros::Motor leftFrontMotor (leftFrontMotor_PORT);
  pros::Motor leftBackMotor (leftBackMotor_PORT);
  pros::Motor rightFrontMotor (rightFrontMotor_PORT);
  pros::Motor rightBackMotor (rightBackMotor_PORT);
  pros::Motor turretMotor (turretMotor_PORT);
  double factorP = 0.5;
  double factorI = 0.001;
  double factorD = 0.25;
  double error = findDistance(positionX,positionY,X,Y);
  double proportional = 0;
  double intergral = 0;
  double derivitave = 0;
  double lastError = error;
  double fixing = 0;
  double errorAverage = error;
  double lastErrorAverage = 0;
  double loopCount = 0;
  while (abs(error) >= 0) {

    proportional = error*factorP;
    intergral = intergral + error;
    derivitave = error - lastError;
    fixing = proportional+(intergral*factorI)+(derivitave*factorD);
    leftFrontMotor.move_velocity(fixing);
    leftBackMotor.move_velocity(fixing);
    rightFrontMotor.move_velocity(fixing);
    rightBackMotor.move_velocity(fixing);
    lastError = error;
    pros::c::delay(10);
    error = error-dist;
    errorAverage = (errorAverage+error+lastError)/3;
    loopCount += 1;


    if (loopCount > 75) { // This is so previous larger errors don't break the infinite loop fix
      lastErrorAverage = errorAverage;
      errorAverage = 0;
    }


  }
  leftFrontMotor.brake();
  leftBackMotor.brake();
  rightFrontMotor.brake();
  rightBackMotor.brake();
}

void turnPiD(double degr) {
double factorP = 0.625;
double factorI = 0.005;
double factorD = 0.25;
double error = degHead - degr;
double intergral = 0;
double derivitave = 0;
double pError = 1;
double fixing = 0;
double errorAverage = 0;
double pErrorAverage = 0;
double loopCount = 0;
  pros::Motor leftFrontMotor (leftFrontMotor_PORT);
  pros::Motor leftBackMotor (leftBackMotor_PORT);
  pros::Motor rightFrontMotor (rightFrontMotor_PORT);
  pros::Motor rightBackMotor (rightBackMotor_PORT);
while (abs(error) <= 0) {
  if ((abs(pErrorAverage) - 0.01) < (abs(pError) < (abs(pErrorAverage) + 0.01))) { // preventing infinite correct loop
    break;
  }
  intergral = intergral + error;
  derivitave = error - pError;
  fixing = (error*factorP)+(intergral*factorI)+(derivitave*factorD);
  if (degr > 180) { // if it is better to turn left it will, this is to make the function more effecient
    leftFrontMotor.move_velocity(fixing);
    leftBackMotor.move_velocity(fixing);
    rightFrontMotor.move_velocity(fixing);
    rightBackMotor.move_velocity(fixing);
  }
  else {
    leftFrontMotor.move_velocity(fixing);
    leftBackMotor.move_velocity(fixing);
    rightFrontMotor.move_velocity(fixing);
    rightBackMotor.move_velocity(fixing);
  }
  pError = error;
    pros::c::delay(10);
  error = degHead - degr;
  errorAverage = (errorAverage+error+pError)/3;
  loopCount += 1;
  if (loopCount > 100) { // This is so previous larger errors don't break the infinite loop fix
    pErrorAverage = errorAverage;
    errorAverage = 0;
  }
}
leftFrontMotor.brake();
leftBackMotor.brake();
rightFrontMotor.brake();
rightBackMotor.brake();
}
void moveToPoint(double X,double Y,double finalDeg) {
if (findAngleMove(X,Y) != degHead){
  turnPiD(findAngleMove(X,Y));
}
movePiD(X,Y);
if (finalDeg != null) {
  turnPiD(finalDeg);
}
} */
//Control Functiosn
/*void expansionControl() {
expansion = true; // no need to retract this is a one time spring mechanism
} */
pros::ADIDigitalIn intakeSensor ({{expander_PORT,EXT_IntakeSensorPort}});
void triggerControl() {
indexer.set_value(true);
pros::c::delay(250);
indexer.set_value(false);
}
bool threads;
/*
void intakeRead(){
intakeDisc = true;
discLoad += 1;
pros::Task::delay_until((intakeSensor.get_value() == false),10);
intakeDisc = false;
}
void turretRead(){
turretDisc = true;
if ((lockOn =true ) && (autoFire = true)) {
  triggerControl();
}
pros::c::delayUntil(!turretSensor.pressing());
turretDisc = false;
}
*/


void displayThread() { // Controller screen thread
thread2On = true;
while(true) {
  pros::Controller controller1 (pros::E_CONTROLLER_MASTER);
  pros::Controller controller2 (pros::E_CONTROLLER_PARTNER);
  pros::ADIGyro EXT_GyroTurret ({{expander_PORT,EXT_GyroTurretPort}});
  /* Controller Screen Example
  _____________________________
  /                             \
  | Position X: 19 Y: 18        |
  | Heading: 45                 |
  | Power: 89%                  |
  \_____________________________/
  */
  //controller1.clear();
  //pros::c::task_delay(250);                                                                                                                                                                                                                                                                                                                                                                                                                              
  controller2.clear();
  pros::c::task_delay(50);
  controller2.print(0,0,"D: %f", distfeet);
  pros::c::task_delay(50);
  /*
  controller1.print(0,0,"X: %f",positionX);
  pros::c::task_delay(50);
  controller1.print(1,0,"Y: %f",positionY);
    pros::c::task_delay(50);
  controller1.print(2,0,"H: %f", degHead);
    pros::c::task_delay(50); */

 // controller2.print(0,0,"D: %f", distfeet);
   // pros::c::task_delay(50);
    //pros::c::task_delay(50);
  //controller2.print(1,0,"TurretDeg: ", EXT_GyroTurret.get_value());
  //pros::c::task_delay(250);
  /*
  Brain.Screen.setFont(vex::fontType::mono20);
  //Brain.Screen.print(PotentiometerA.angle(degrees));
  Brain.Screen.setFont(vex::fontType::mono60);
 */  
  pros::screen::erase();
  if ((thread1On = true)) {
      pros::screen::set_pen(COLOR_PURPLE);
     pros::screen::print(pros::E_TEXT_LARGE, 1, "3");
  }
  else if ((thread2On = true)) {
    pros::screen::set_pen(COLOR_GREEN);
    pros::screen::print(pros::E_TEXT_LARGE, 1, "2");
  }
  else if ((thread3On = true)) {
    pros::screen::set_pen(COLOR_PURPLE);
    pros::screen::print(pros::E_TEXT_LARGE, 1, "4");
  }
  else if ((thread4On = true)) {
    pros::screen::set_pen(COLOR_PURPLE);
    pros::screen::print(pros::E_TEXT_LARGE, 1, "9");
  }
  else if ((compReady = true)) {
    pros::screen::set_pen(COLOR_DARK_RED);
    pros::screen::print(pros::E_TEXT_LARGE, 1, "V");
    pros::screen::print(pros::E_TEXT_LARGE_CENTER, 2, "Infrared");
  }
  //Brain.Screen.print("3249V");


}
}

int thread4() { // auto Fire Thread
pros::Motor intake (intakeMotor_PORT);
intake.move_velocity(200);
while (true) {
  //intakeSensor.pressed(intakeRead);
  //turretSensor.pressed(turretRead);
  if (discLoad != 3) {
    intake.move_velocity(200);
  }
  else {
    intake.move_velocity(0);
  }
  //if ((discLoad = !3)){
  /* }
  else {
    intake.brake();
  } */
    pros::c::task_delay(10);
}
}
void threadCheck() {
if (thread1On == true && thread2On == true && thread3On == true && thread4On == true){
  compReady = true;
}
}
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */



/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be brakeped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

 
void autonomous() {
  pros::ADIDigitalOut indexer (indexer_PORT);
  pros::Motor flyWheel (flyWheel_PORT);
  pros::Motor intake (intakeMotor_PORT);
  pros::Motor leftFrontMotor (leftFrontMotor_PORT);
  pros::Motor leftBackMotor (leftBackMotor_PORT);
  pros::Motor rightFrontMotor (rightFrontMotor_PORT);
  pros::Motor rightBackMotor (rightBackMotor_PORT);
  pros::ADIDigitalOut expansion (expansion_PORT);
  pros::Motor turretMotor (turretMotor_PORT);
  pros::IMU turretSensor (turretSensor_PORT);
  pros::Motor rollerMotor (rollerMotor_PORT);
  //flywheelPIDFF();
  
  //flyWheel.move_voltage(950);
  /*    

  //leftFrontMotor.tare_position();
  //leftFrontMotor.tare_position();
  //leftFrontMotor.tare_position();
  //leftFrontMotor.tare_position();
  positionX = 0;
  positionY = 0;
  degHead = 0;
  flyWheel.move_velocity(180);
  pros::delay(2500);
  indexer.set_value(true);
  pros::delay(250);
  indexer.set_value(false);
  pros::delay(250);
  intake.move_velocity(200);
  pros::delay(5000);
  intake.brake();
  pros::delay(250);
  indexer.set_value(true);
  pros::delay(250);
  indexer.set_value(false); */
  //movePiD(0,4);
  /*
  intake.move_velocity(200);
  leftFrontMotor.move(50);
  leftBackMotor.move(50);
  rightFrontMotor.move(50);
  rightBackMotor.move(50);
  pros::delay(2500);
  leftFrontMotor.move(0);
  leftBackMotor.move(0);
  rightFrontMotor.move(0);
  rightBackMotor.move(0);
  turretMotor.move(80);
  pros::delay(500);
  indexer.set_value(true);
  pros::delay(250);
  indexer.set_value(false);
  intake.brake();
  flyWheel.brake();
  pros::delay(36000);
  expansion.set_value(true); */
  //pros::delay(250);
  //turretMotor.move(127);
  //pros::delay(1000);




  /* SKILLS PSUEDO CODE
   Starting X = 30  Y = 10.5 ?? = 45 
   Spin turret 90deg to roller
   move roller by 200 degrees
   Move bot 25.5 inches forward
   Spin Turret -90 deg to roller
   move roller by 200 degrees
   Turn chassis 55 degrees
   Move 142.5 inches
   Turn chassis 40 degrees 
   Turn turret -140 deg to roller
   Spin Roller by 200 degrees
   Go forward 28.5 inches
   Turn Turret 90 deg to roller 
   Spin roller 200 Degrees
   Turn Chassis 50 degrees 
   Turn Turret 40 Degrees
   Move Chassis 39 Inches
   Turn Chassis -90 Degrees
   Flywheel Match loads 
   Last 8 seconds 
   Move chassis -47 inches
   Turn Turret -135 
   Launch expansion

  */
  
  chassis_controller->moveDistance(-3*okapi::inch);
  pros::delay(1000);
  rollerMotor.move_relative(-180,127);
  pros::delay(2000);
  //chassis_controller->moveDistance(3*okapi::inch);
  
  //rightFrontMotor.move_relative(900,127);
  //pros::delay(2000);

  //expansion.set_value(true);
  //pros::delay(1000);
  //expansion.set_value(false);
  //indexer.set_value(true);
  //pros::delay(250);
  //indexer.set_value(false);
  //
  //pros::delay(1250);
  //pros::delay(42000);
  
  //chassis_controller->moveDistance(16*okapi::inch);
  //rightFrontMotor.move_relative(720,127);
  //chassis_controller->moveDistance(4*okapi::inch); 

 

  //turretMotor.move_voltage(6000);
  //while(!(89 < turretSensor.get_rotation()) && !(91 > turretSensor.get_rotation())) {
  //pros::delay(1);
  //}
  //turretMotor.move_velocity(0);
  /*rollerMotor.move_relative(200,127);
  while (!(199 < rollerMotor.get_position() < 201.1)) {
    pros::delay(1);
  }
  intake.move_velocity(200);
  chassis_controller->moveDistance(25.5*okapi::inch);
  while(!(0.01 < turretSensor.get_rotation() < 1.1)) {
  turretMotor.move_velocity(-100);
  pros::delay(1);
  }
  rollerMotor.move_relative(200,127);
  while (!(399 < rollerMotor.get_position() < 401.1)) {
    pros::delay(1);
  }
  intake.move_velocity(0);
  chassis_controller->turnAngle(-55*okapi::degree);
  chassis_controller->moveDistance(142.5*okapi::inch);
  chassis_controller->turnAngle(90*okapi::degree);
  while(!(219 < turretSensor.get_rotation() < 221.1)) {
  turretMotor.move_velocity(-100);
  pros::delay(1);
  }
  rollerMotor.move_relative(200,127);
  while (!(599 < rollerMotor.get_position() < 601.1)) {
    pros::delay(1);
  }
  chassis_controller->moveDistance(28.5*okapi::inch);
  while(!(309 < turretSensor.get_rotation() < 311.1)) {
  turretMotor.move_velocity(100);
    pros::delay(1);
  }
  rollerMotor.move_relative(200,127);
  while (!(799 < rollerMotor.get_position() < 801.1)) {
    pros::delay(1);
  }
  chassis_controller->turnAngle(50*okapi::degree);
  while(!(269 < turretSensor.get_rotation() < 271.1)) {
  turretMotor.move_velocity(100);
    pros::delay(1);
  }
  chassis_controller->moveDistance(39*okapi::inch);
  chassis_controller->turnAngle(-95*okapi::degree);
  
  while (pros::c::millis() < 51999) {
    flywheel_controller->setTarget(52);
    intake.move_velocity(100);
    indexer.set_value(true);
    pros::delay(250);
    indexer.set_value(false);
    pros::delay(1250);
  }
  
  chassis_controller->moveDistance(-47*okapi::inch);
  while(!(134 < turretSensor.get_rotation() < 136.1)) {
  turretMotor.move_velocity(100);
  }
  
  expansion.set_value(true);
  pros::delay(250);
  expansion.set_value(false);
  pros::delay(250);
  expansion.set_value(true);
  pros::delay(250);
  expansion.set_value(false);
  pros::delay(250);
  expansion.set_value(true);
  pros::delay(250);
  expansion.set_value(false);
  pros::delay(250);
 */
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be brakeped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  pros::Controller controller1 (pros::E_CONTROLLER_MASTER);
  pros::Controller controller2 (pros::E_CONTROLLER_PARTNER);
  pros::Motor leftFrontMotor (leftFrontMotor_PORT);
  pros::Motor leftBackMotor (leftBackMotor_PORT);
  pros::Motor rightFrontMotor (rightFrontMotor_PORT);
  pros::Motor rightBackMotor (rightBackMotor_PORT);
  pros::Motor turretMotor (turretMotor_PORT);
  pros::Motor intake (intakeMotor_PORT);
  pros::ADIDigitalOut expansion (expansion_PORT);
  pros::ADIDigitalOut indexer (indexer_PORT);
  pros::Motor flyWheel (flyWheel_PORT);
  pros::ADIGyro EXT_GyroTurret ({{expander_PORT,EXT_GyroTurretPort}});
  pros::Motor rollerMotor (rollerMotor_PORT);
  //pros::Task my_task2(displayThread);
  float curve = 0.25;
  float left;
  float right;
  float power;
  float turn;
  double discSpeed = 0;
  distfeet = 0;
  double vFinal = 0;
  double flywheelRPM = 0;
  double turretMove = 0;
  float varChange = 2000;
  while (true) {
    turretMove = controller2.get_analog(ANALOG_RIGHT_X);
    power = controller1.get_analog(ANALOG_LEFT_Y);
    turn = controller1.get_analog(ANALOG_RIGHT_X);
    left = power + turn;
    right = power - turn;
    leftFrontMotor.move(left); // Conners Move
    leftBackMotor.move(left);
    rightFrontMotor.move(right);
    rightBackMotor.move(right);
    turretMotor.move(turretMove);
    //turretMotor.move_voltage(varChange);
    intake.move(controller2.get_analog(ANALOG_LEFT_Y));
    //controller2.clear();
    //pros::delay(50);
    //controller2.print(1,1,"D: %d", distfeet                                                                                                                                                 );
    // pros::delay(50);
    //discSpeed = sqrt((gravity*pow(distfeet*12,2)/((2*cos(radians(flyWheelAngle)))*(zCoordinates[1]-turretOffsetZ-(12*distfeet)*tan(radians(flyWheelAngle)))))); // inches per second
    //vFinal = sqrt(abs((0.5*discMass*pow(discSpeed,2))/(0.5*massTotal)-(0.5*springForce*flyWheelCompression)/(0.5*massTotal)));
    //flywheelRPM = ((vFinal*60)/(2*M_PI*flyWheelRadius))/18;
    //flywheelVelocity = ((ejectVelocity*60)/(2*M_PI*flyWheelRadius*sqrt(discInertia/(flyWheelInertia+flyWheelInertialIncrease))))/flyWheelGearRatio;


    if (controller2.get_digital(DIGITAL_B)) {
      expansion.set_value(true);
    }
    if (controller2.get_digital(DIGITAL_R2)) {
      indexer.set_value(true);
      pros::delay(250);
      indexer.set_value(false);
    }
    if (controller2.get_digital(DIGITAL_L2)) {
      rollerMotor.move_velocity(100);
      pros::delay(400);
      rollerMotor.move_velocity(0);
    }
    if (controller2.get_digital(DIGITAL_L1)) {
      rollerMotor.move_velocity(-100);
      pros::delay(400);
      rollerMotor.move_velocity(0);
    }
    

 //   controller1.clear();
 //   pros::c::task_delay(250);  
 //   controller1.print(2,0,"H: %f", (EXT_GyroTurret.get_value()/10));
 //  pros::c::task_delay(50);
    //controller1.clear();
    //pros::c::task_delay(50); 
    //controller1.print(0,0,"V: %f",varChange);
    //pros::c::task_delay(50);
    if (controller2.get_digital(DIGITAL_UP)) {
      distfeet = distfeet+10;
      pros::delay(50);
    }
    if (controller2.get_digital(DIGITAL_DOWN)) {
      distfeet = distfeet-10;
      pros::delay(50);
    } 
    pros::delay(50);
  }
}








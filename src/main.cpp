#include "main.h"
#include "config.h"
//#include "odom.h"
#define _USE_MATH_DEFINES


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
/* Odometry */
// PROGRAM USES RADIANS
double degHead; // in degrees
double lastdegHead;
double turretError;
bool turretBreak;
float turretThreashold = 3;
float turretOffsetZ = 11.5; // turret offset height from ground so it can properly aim
float flywheelOffset = 5; // needs to be measured
float distanceFromCorners; // Needs calibration
int discLoad;
bool intakeDisc;
bool turretDisc;
bool flyWheelDisc;
// Starting Offsets is how far the sensor is away from the walls or 0,0 on a grid
double startingOffsetX;
double startingOffsetY;
// Grid is for marking objects and where they are
double positionX;
double positionY;
double velocityX;
double velocityY;
double rotVelocity;
// blue near the red high goal and red is near the blue high goal  0,0 is the bottom RIGHT of the feild
// Object coordinates for object locations from 0,0 in inchesf
// for saftey it can be if X = Y and autonmous = true then break?
float middleLineX1 = 0;
float middleLineX2 = 140.02;
float middleLineY1 = 0;
float middleLineXY = 140.02;
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
float coordinateLocations[2][8] {
{0,140.2,110.98,139,29.43,1,122.63,17.78},
{72.20,72.20,139,110.98,1,29.43,17.78,122.63}
};
/* Id's for goal Height
0: Lowest Goal point
1: Centered Goal point
2: Highest Goal point
*/
float zCoordinates[3] {25,30.5,35.5};
/* Id's for Odometry offsets
0 = distance
1 = angle
0:Left Odom Wheel
1:Right Odom Wheel
2:Back Odom Wheel
*/
float offsets[3] {1.53,1.73,8.72};// THIS NEEDS CALIBRATION ONCE ODOMETRY WHEELS ARE IN THE ROBOT < ------------------------------------------------------------
// Low Z and High Z is for aiming. EX if Turret aim is between Zlow < Yangle < ZHigh then Fire
//double M_PI = 3.14159265358979323846264338327950288;
/*Launch Math*/
double flyWheelMass = 0.49; // pounds (m)
double flyWheelRadius = 2; // inches (r)
double flyWheelCrossArea = M_PI*2*2; // inches (A)
double flyWheelCompression = 0.15; // inches (l)
double flyWheelAngle = 45; // angle in degrees (a)
double flyWheelGearRatio = 18; // multipler for gearing
// Compression Force (F=E*A*l)/r
double flyWheelCompressionForce = (flyWheelCrossArea*flyWheelCompression)/(flyWheelRadius*2); // (f)
// Inertia Calc, m*r^2
double discInertia = 0.121254*pow(5.5/2,2); // i_disc
double flyWheelInertia = flyWheelRadius*pow(2,2); // i_wheel
// Inertial increase from Compression i_delta = (F*r^2)/(3*E)
double flyWheelInertialIncrease = (flyWheelCompressionForce*pow((flyWheelRadius*2),2))/(3); // i_wheelDelta
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
float dragWheelDiamater = 2.630; // drag wheel radius
double dragWheelCirc = dragWheelDiamater*M_PI;
double gravity = -386.08858267717; // inches per second
/* Calculations */
double findAngle(int selector, double x = 0) { // selector uses coordniate list
//m=(y2-y1)/(x2-x1) slope formula
x = (180/M_PI)/(atan((positionY - coordinateLocations[1][selector])/(positionX - coordinateLocations[0][selector])));
return(x);
}
double findAngleMove(double X,double Y,double dest = 0) { // in degrees NOT radains
dest = (180/M_PI)*(atan((Y - positionY)/( X- positionX)));
return(dest);
}
double findDistance(double X1, double Y1, double X2, double Y2, double dist = 0) {
dist = sqrt(pow(Y2-Y1,2)+pow(X2-X1,2));
return(dist );
}
// misc
double abs(double n) {
if (n <= 0) {
  n =+ n*-1;
}
return(n);
}

double radians(double deg, double x = 0) {
x = (deg * (M_PI / 180));
return(x);
}
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
float distfeet;






/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
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
  pros::ADIDigitalIn intakeSensor_initializer ({{expander_PORT,EXT_IntakeSensorPort}});  
  pros::ADIGyro EXT_GyroTurret_initializer ({{expander_PORT,EXT_GyroTurretPort}});
  // EXT_GyroTurret.calibrate;
  pros::lcd::initialize();
  pros::lcd::set_text(1, "Hello PROS User!");


}


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
activePID = true;
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
activePID = false;
turretMotor.brake();
return(1);
}
//Autonomous functions
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
  double distError = findDistance(positionX,positionY,X,Y); 
  double proportional = 0;
  double intergral = 0;
  double derivitave = 0;
  double lastError = error;
  double fixing = 0;
  double errorAverage = error;
  double lastErrorAverage = 0;
  double loopCount = 0;
  while (abs(error) >= 0) {
    /*if ((abs(lastErrorAverage) - 0.01) < (abs(lastError) < (abs(lastErrorAverage) + 0.01))) { // preventing infinite correct loop
    break;
    } */
    proportional = error*factorP;
    intergral = intergral + error;
    derivitave = error - lastError;
    fixing = proportional+(intergral*factorI)+(derivitave*factorD);
    leftFrontMotor.move_velocity(fixing);
    leftBackMotor.move_velocity(fixing);
    rightFrontMotor.move_velocity(-fixing);
    rightBackMotor.move_velocity(-fixing);
    lastError = error;
    pros::c::delay(10);
    error = findDistance(positionX,positionY,X,Y);
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
}
//Control Functiosn
/*void expansionControl() {
expansion = true; // no need to retract this is a one time spring mechanism
} */pros::ADIDigitalOut indexer (indexer_PORT);
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




int thread1() { // Position thread If it ever breaks we dead 

/*
New Psuedocode
Get raw distance travelled. 
send to MOVE pid
move PID work? 
*/

  double diffrence = 0;
  double pDiffrence = 0;
  double vDiffrence = 0;
  double vBackWheelDiffrence = 0;
  double backWheelDiffrence = 0;
  double pBackWheelDiffrence = 0;
  double lastEncoderL = 0;
  double lastEncoderR = 0;
  double lastEncoderB = 0;
  double encoderVL = 0;
  double encoderVR = 0;
  double encoderVB = 0;
  double encoderVVL = 0;
  double encoderVVR = 0;
  double encoderVVB = 0;
  double positionLR = 0;
  double positionB = 0;
  double dragDegrees = (dragWheelCirc/360);
  pros::ADIEncoder encoderL (encoderLeftTop_PORT,encoderLeftBottom_PORT);
  pros::ADIEncoder encoderR (encoderRightTop_PORT,encoderRightBottom_PORT);
  pros::ADIEncoder encoderB (encoderBackTop_PORT,encoderBackBottom_PORT);
  while(true) {
    encoderVL = encoderL.get_value();
    encoderVR = encoderR.get_value();
    encoderVB = encoderB.get_value();
  //  encoderVVL = encoderL.velocity(dps);
  //  encoderVVR = encoderR.velocity(dps);
  //  encoderVVB = encoderB.velocity(dps);
    diffrence = encoderVL - encoderVR;
    vDiffrence = encoderVVL - encoderVVR;
    lastdegHead = degHead;
    degHead += (2*M_PI*offsets[1])/360*(diffrence)*dragDegrees*(180/M_PI);  // tracking offset L and R should be the same no matter what
    rotVelocity = (2*M_PI*offsets[1]/360*(vDiffrence)*dragDegrees)*(180/M_PI);
    //degHead = (2*offsets[0]*diffrence*dragWheelCirc*180)/pow(360,2);
    backWheelDiffrence = encoderVB - offsets[0]*diffrence/offsets[2];
    vBackWheelDiffrence = encoderVVB - offsets[0]*diffrence/offsets[2];

    if ((degHead >= 360)) {
    degHead = degHead-360;
    }
    else if ((degHead <= 0)) {
      degHead = degHead+360;
    }
      positionLR = (((encoderVL+encoderVR-diffrence)/2) - (lastEncoderL+lastEncoderR-pDiffrence)/2);
      positionB = (encoderVB+backWheelDiffrence) - (lastEncoderB-pBackWheelDiffrence);
 
    if((180 < degHead)) {
      positionX += (dragDegrees*(cos(radians(degHead))*positionLR+dragDegrees*(sin(radians(degHead))*positionB)))*-1;
      positionY += (dragDegrees*(sin(radians(degHead))*positionLR+dragDegrees*(cos(radians(degHead))*positionB)))*-1;
      velocityX = (dragDegrees*cos(radians(degHead))*(encoderVVL+encoderVVR-vDiffrence)/2+dragDegrees*(sin(radians(degHead))*(encoderVVB)))*-1;
      velocityY = (dragDegrees*sin(radians(degHead))*(encoderVVL+encoderVVR-vDiffrence)/2+dragDegrees*(cos(radians(degHead))*(encoderVVB)))*-1;
    }
    else {
      positionX += dragDegrees*(cos(radians(degHead))*positionLR+dragDegrees*(sin(radians(degHead))*positionB));
      positionY += dragDegrees*(sin(radians(degHead))*positionLR+dragDegrees*(cos(radians(degHead))*positionB));
      velocityX = (dragDegrees*cos(radians(degHead))*(encoderVVL+encoderVVR-vDiffrence)/2+dragDegrees*(sin(radians(degHead))*(encoderVVB)));
      velocityY = (dragDegrees*sin(radians(degHead))*(encoderVVL+encoderVVR-vDiffrence)/2+dragDegrees*(cos(radians(degHead))*(encoderVVB)));
    }
    lastEncoderL = encoderVL;
    lastEncoderR = encoderVR;
    lastEncoderB = encoderVB;
    pDiffrence = diffrence;
    pBackWheelDiffrence = backWheelDiffrence;


    encoderL.reset();
    encoderR.reset();
    encoderB.reset();
    pros::c::task_delay(10);
  // I NEED VELOCITY FOR AIMBOT
  }
}
int thread2() { // Controller screen thread
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
  controller1.clear();
  controller1.print(0,0,"X: ",positionX);
  controller1.print(1,0,"Y: ",positionY);
  controller1.print(2,0,"Heading: ", degHead);
  controller2.print(0,0,"Launch Distance", distfeet);
  controller2.print(1,0,"TurretDeg", EXT_GyroTurret.get_value());
  /*
  Brain.Screen.setFont(vex::fontType::mono20);
  //Brain.Screen.print(PotentiometerA.angle(degrees));
  Brain.Screen.setFont(vex::fontType::mono60);
 */  
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
  pros::c::task_delay(250);
}
}
int thread3() { // auto aim thread
pros::ADIGyro EXT_GyroTurret ({{expander_PORT,EXT_GyroTurretPort}});
//float flyWheelContactAngle = 70; // needs calibration
double totalDistance = 0;
double totalTurretRotation = EXT_GyroTurret.get_value(); // to make sure that we don't go overboard and twist/rip wires
//float maxTotalTurretRotation = 720;
float pneumaticSpeed =  8;// in inches per second idk how this is that fast
double relativeVelocity;
double ejectVelocity;
double flywheelVelocity;
double velocityXsec = velocityX*1000;
double velocityYsec = velocityY*1000;
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
  velocityXsec = velocityX*1000; // inches per ms to inches per second
  velocityYsec = velocityY*1000;
  relativeVelocity = sqrt(pow(velocityXsec,2)+pow(velocityYsec,2)+2*velocityX*velocityYsec*cos(90-findAngle(target)));
  totalDistance = sqrt(pow(findDistance(positionX,positionY,coordinateLocations[0][target],coordinateLocations[1][target]),2)
  +pow(tan(radians(flyWheelAngle))*findDistance(positionX,positionY,coordinateLocations[0][target],coordinateLocations[1][target]),2));
  if((aimBot=true && (totalTurretRotation >= 360*4))) {
    // Turret movement very simple :D
    // it isnt D:
    if ((findAngle(target) - turretThreashold) <= EXT_GyroTurret.get_value() <= (findAngle(target) + turretThreashold)) {
      lockOn = true;
    } // decides if the turret is within acceptable limits
    else if((activePID = false)&&((findAngle(target) - turretThreashold) <=! EXT_GyroTurret.get_value() <=! (findAngle(target) + turretThreashold))) { // prevents infinite PID's
        turretError = findAngle(target);
    }
    // sqrt(g*d^2/(2*cos(a)*)(hg-hr-d*tan(a))
    ejectVelocity = sqrt((gravity*pow(totalDistance,2)/(2*cos(radians(flyWheelAngle)))*(zCoordinates[1]-turretOffsetZ-totalDistance*tan(radians(flyWheelAngle)))))
    -pneumaticSpeed-relativeVelocity; // inches per second
    // RPM = (v*60)/(2*pi*r_wheel*sqrt(i_object/i_wheel+i_deltaWheel))
    flywheelVelocity = ((ejectVelocity*60)/(2*M_PI*flyWheelRadius*sqrt(discInertia/(flyWheelInertia+flyWheelInertialIncrease))))/flyWheelGearRatio;
 
    flyWheel.move_velocity(flywheelVelocity);
   // flyWheel2.move_velocity(flywheelVelocity,rpm);
  }
  else {
  //  EXT_GyroTurret.get_value() = totalTurretRotation;
    turretError = degHead;
  }
    pros::c::task_delay(10);
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
void competition_initialize() {
       pros::Task my_task1(thread1);
       pros::Task my_task2(thread2);
       pros::Task my_task3(thread3);
       pros::Task my_task4(thread4);
       pros::Task my_task5(turretPID);
}




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
void autonomous() {}




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
  float curve = 0.75;
  float left;
  float right;
  float power;
  float turn;
  while (true) {

    power = controller1.get_analog(ANALOG_LEFT_Y);
    turn = controller1.get_analog(ANALOG_RIGHT_X);
    left = power + turn;
    right = power - turn;
    leftFrontMotor.move(100*(((1-curve)*left)/100+(curve*pow(left/100,7)))); // Conners Move
    leftBackMotor.move(100*(((1-curve)*left)/100+(curve*pow(left/100,7))));
    rightFrontMotor.move(100*(((1-curve)*right)/100+(curve*pow(right/100,7))));
    rightBackMotor.move(100*(((1-curve)*right)/100+(curve*pow(right/100,7))));
/*
    leftFrontMotor.spin(forward,(Controller1.Axis3.position() + Controller1.Axis1.position())^2/100,velocityUnits::pct); // Arcade control
    leftBackMotor.spin(forward,(Controller1.Axis3.position() + Controller1.Axis1.position())^2/100,velocityUnits::pct);
    rightFrontMotor.spin(forward,(Controller1.Axis3.position() - Controller1.Axis1.position())^2/100,velocityUnits::pct);
    rightBackMotor.spin(forward,(Controller1.Axis3.position() - Controller1.Axis1.position())^2/100,velocityUnits::pct);
*/
    pros::delay(20);
  }
}






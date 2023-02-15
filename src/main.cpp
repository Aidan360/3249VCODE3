#include "main.h"
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
/* Calculations */ /*
double findAngle(int selector, double x = 0) { // selector uses coordniate list
//m=(y2-y1)/(x2-x1) slope formula
x = (180/M_PI)/(atan((positionY - coordinateLocations[1][selector])/(positionX - coordinateLocations[0][selector])));
return(x);
} */
double findAngleMove(double X,double Y,double dest = 0) { // in degrees NOT radains
dest = (180/M_PI)*(atan((Y - positionY)/( X- positionX)));
return(dest);
} 
/*double findDistance(double X1, double Y1, double X2, double Y2, double dist = 0) {
dist = sqrt(pow(Y2-Y1,2)+pow(X2-X1,2));
return(dist );
} */
// misc
//double abs(double n) {
//if (n <= 0) {
//  n =+ n*-1;
//}
//return(n);
//}

/*
double radians(double deg, double x = 0) {
x = (deg*(M_PI/180));
return(x);
} */
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

/*
int thread1() { // Position thread If it ever breaks we dead



New Psuedocode
Get raw distance travelled.
send to MOVE pid
move PID work?



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
  double positionB = 0;
  double pPositionX = 0;
  double pPositionY = 0;
  double dragDegrees = (dragWheelCirc/360);
  double vDegHead = 0;
  pros::ADIEncoder encoderL (encoderLeftTop_PORT,encoderLeftBottom_PORT);
  pros::ADIEncoder encoderR (encoderRightTop_PORT,encoderRightBottom_PORT);
  pros::ADIEncoder encoderB (encoderBackTop_PORT,encoderBackBottom_PORT);
    encoderL.reset();
    encoderR.reset();
    encoderB.reset();
  while(true) {
  encoderVL = (encoderL.get_value() - lastEncoderL)*dragDegrees;
  encoderVR = (encoderR.get_value() - lastEncoderR)*dragDegrees;
  encoderVB = (encoderB.get_value() - lastEncoderB)*dragDegrees;


  //  encoderVVL = encoderL.velocity(dps);
  //  encoderVVR = encoderR.velocity(dps);
  //  encoderVVB = encoderB.velocity(dps);
 
    //vDiffrence = encoderVVL - encoderVVR;
    lastdegHead = degHead;
    //degHead += (2*M_PI*offsets[1])/360*(diffrence)*dragDegrees*(180/M_PI);  // tracking offset L and R should be the same no matter what
    degHead += (((encoderVL-encoderVR))/leftRightLength)*(180/M_PI);
    vDegHead = degHead - lastdegHead;
    //rotVelocity = (2*M_PI*offsets[1]/360*(vDiffrence)*dragDegrees)*(180/M_PI);
    //degHead = (2*offsets[0]*diffrence*dragWheelCirc*180)/pow(360,2);
    //vBackWheelDiffrence = encoderVVB - offsets[0]*diffrence/offsets[2];

    if ((degHead >= 360)) {
    degHead = degHead-360;
    }
    else if ((degHead <= 0)) {
      degHead = degHead+360;
    } 
 //     dist = ((encoderVR+encoderVL)*sin(radians(degHead)/2))/degHead;
 
//    if((180 < degHead)) {
  //    positionX += ((sin(radians(degHead+vDegHead/2))*dist+(sin(radians(degHead+vDegHead/2))*encoderVB)));
  //    positionY += ((cos(radians(degHead+vDegHead/2))*dist-(cos(radians(degHead+vDegHead/2))*encoderVB)));
 //     velocityX = (dragDegrees*cos(radians(degHead))*(encoderVVL+encoderVVR-vDiffrence)/2+dragDegrees*(sin(radians(degHead))*(encoderVVB)))*-1;
 //     velocityY = (dragDegrees*sin(radians(degHead))*(encoderVVL+encoderVVR-vDiffrence)/2+dragDegrees*(cos(radians(degHead))*(encoderVVB)))*-1;
    //}
  //  else {
    //  positionX += (cos(radians(degHead))*dist+(sin(radians(degHead))*positionB));
     // positionY += (sin(radians(degHead))*dist+(cos(radians(degHead))*positionB));
  //    velocityX = (dragDegrees*cos(radians(degHead))*(encoderVVL+encoderVVR-vDiffrence)/2+dragDegrees*(sin(radians(degHead))*(encoderVVB)));
  //    velocityY = (dragDegrees*sin(radians(degHead))*(encoderVVL+encoderVVR-vDiffrence)/2+dragDegrees*(cos(radians(degHead))*(encoderVVB)));
    //}
    lastEncoderL = encoderL.get_value();
    lastEncoderR = encoderR.get_value();
    lastEncoderB = encoderB.get_value();
    pDiffrence = diffrence;
    pBackWheelDiffrence = backWheelDiffrence;
    velocityX = positionX - pPositionX;
    velocityY = positionY - pPositionY;




  // I NEED VELOCITY FOR AIMBOT
  }
} */
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
  controller1.clear();
  pros::c::task_delay(250);                                                                                                                                                                                                                                                                                                                                                                                                                              
  controller2.clear();
  pros::c::task_delay(250);
  controller1.print(0,0,"X: %f",EXT_GyroTurret.get_value());
  pros::c::task_delay(50);
  controller1.print(1,0,"Y: %f",positionY);
    pros::c::task_delay(50);
  controller1.print(2,0,"H: %f", degHead);
    pros::c::task_delay(50);
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
  //bool intake = false;
  //bool intakeDir = false; // false = forward true = reverse
  //double springForce = (11501.492602*(M_PI*2*flyWheelRadius))/(flyWheelRadius*2);
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
    //turretMotor.move(100*(((1-curve)*turretMove)/100+(curve*pow(turretMove/100,7))));
    turretMotor.move_voltage(varChange);
    intake.move(controller2.get_analog(ANALOG_LEFT_Y));
    //controller2.clear();
    //pros::delay(50);
    //controller2.print(1,1,"D: %d", distfeet                                                                                                                                                 );
    // pros::delay(50);
    //discSpeed = sqrt((gravity*pow(distfeet*12,2)/((2*cos(radians(flyWheelAngle)))*(zCoordinates[1]-turretOffsetZ-(12*distfeet)*tan(radians(flyWheelAngle)))))); // inches per second
    //vFinal = sqrt(abs((0.5*discMass*pow(discSpeed,2))/(0.5*massTotal)-(0.5*springForce*flyWheelCompression)/(0.5*massTotal)));
    //flywheelRPM = ((vFinal*60)/(2*M_PI*flyWheelRadius))/18;
    //flywheelVelocity = ((ejectVelocity*60)/(2*M_PI*flyWheelRadius*sqrt(discInertia/(flyWheelInertia+flyWheelInertialIncrease))))/flyWheelGearRatio;
     if (controller1.get_digital(DIGITAL_B)) {
      turretMotor.move_relative(360,100);
    }


    if (controller2.get_digital(DIGITAL_B)) {
      expansion.set_value(true);
    }
    if (controller2.get_digital(DIGITAL_R2)) {
      indexer.set_value(true);
      pros::delay(250);
      indexer.set_value(false);
    }
    

 //   controller1.clear();
 //   pros::c::task_delay(250);  
 //   controller1.print(2,0,"H: %f", (EXT_GyroTurret.get_value()/10));
 //  pros::c::task_delay(50);
    //controller1.clear();
    //pros::c::task_delay(50); 
    //controller1.print(0,0,"V: %f",varChange);
    //pros::c::task_delay(50);
    if (controller1.get_digital(DIGITAL_UP)) {
      varChange = varChange+10;
      //pros::delay(50);
    }
    if (controller1.get_digital(DIGITAL_DOWN)) {
      varChange = varChange-10;
     // pros::delay(50);
    } 



/*
    leftFrontMotor.spin(forward,(Controller1.Axis3.position() + Controller1.Axis1.position())^2/100,velocityUnits::pct); // Arcade control
    leftBackMotor.spin(forward,(Controller1.Axis3.position() + Controller1.Axis1.position())^2/100,velocityUnits::pct);
    rightFrontMotor.spin(forward,(Controller1.Axis3.position() - Controller1.Axis1.position())^2/100,velocityUnits::pct);
    rightBackMotor.spin(forward,(Controller1.Axis3.position() - Controller1.Axis1.position())^2/100,velocityUnits::pct);
*/
    pros::delay(50);
  }
}








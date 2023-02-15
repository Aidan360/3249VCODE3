extern float dragWheelDiamater; // drag wheel radius
extern double dragWheelCirc;
extern double gravity ; // inches per second
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
// 200:82.758620689655
// 496.5517242 in/s max vel
// 49.65517242 in/s^2 max accel


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

extern float offsets[3];// THIS NEEDS CALIBRATION ONCE ODOMETRY WHEELS ARE IN THE ROBOT < ------------------------------------------------------------
extern float zCoordinates[3];
extern float coordinateLocations[2][8];
extern float leftRightLength; // length between left odom wheel and right odom wheel
// Low Z and High Z is for aiming. EX if Turret aim is between Zlow < Yangle < ZHigh then Fire

extern bool redTeam;
extern int target;
extern bool aimBot;
extern bool flyWheelCalc;
extern bool lockOn;

/*Formulas*/
extern double findDistance(double X1, double Y1, double X2, double Y2, double dist = 0);
extern double radians(double deg, double x = 0);
extern double degrees(double rad, double x = 0);
extern int sgn(int n);
extern double flyWheelVelocityCalc(float target,float totalDistance, float oscilation = 0,float angle = flyWheelAngle);
extern double flyWheelVelocityE(float target,float totalDistance,float occislation = 0, float angle = flyWheelAngle);

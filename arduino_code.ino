* ********************************************************************
* *Version is marked by milestone reached by code. m0 = driving.
* Adapted from:
Complete RC Control with..
Autonomous guidance with simple if statements and variable speeds
3/14/17 by Brian Patton
\* ********************************************************************* */
#include <Servo.h>
#include <Time.h>
//**************************************************************
//*************
Variable Declaration/Definition
**************
//**************************************************************
/* NOTE: check right stick pins and channels */
// These macros can be used as "substitutes." For example:
// later on, where ever you see "RServoPin", treat it as a "2"
#define RServoPin 3
#define LServoPin 2
#define ArmPin 4
#define AutoPin 8
#define RSYPin 11
#define RSXPin 12
#define LSYPin 10
#define LSXPin 9
#define Ch6Pin 7
// arm limit switch pins, hook to signal and ground
#define LimitSwitchPin 31
// prox sensor pins (aka "sharp sensor" or "IR Range sensor")
#define BackProxPin 18
#define FrontProxPin 17
// pins for the photosensors
#define LPhotoPin 16
#define RPhotoPin 15
// pin for the medkit servo
#define medPin 14
// more macros. If you see "RSX" it becomes "Ch1"
#define RSX Ch1
#define RSY Ch2
#define LSX Ch4
#define LSY Ch3
// macros for sending signals. He had these at 1350-1650, but the
// normal 1000-2000 works
#define MAX_SIG 2000
#define MIN_SIG 1000
// Create Variables to hold the Receiver signals. Each
// starts out with the pin number that its hooked up to
int RSX; // right stick x axis
int RSY; // right stick y axis
int LSX; // left stick x axis
int LSY; // left stick y axis
int Ch5; // Auto Control Switch
int Ch6; // "VRB" knob
const int LED = 13; // Onboard LED pin
// time between wheel speed updates
const double cylcedt = 50e-3; // ms*10^-3 = s, must be < ~140 ms
// max torque for the wheels to not slip (Nm)
const double Tmax = 0.239084 + 0.596242; // n/s^2
// moment of inertia of wheels
const double Iwheel = 0.00101173; // kg*m^2
// maximum estimated angular velocity
const double wmax = 4.6/(4.0/12.0); //4.36512/(4.0/12.0); // ft/s * (in/12) = 1/s
// max angular velocity change per cycle (mapped to signal value);
const double dvmax = map(Tmax/Iwheel*cylcedt, 0, wmax, 1500, 2000); // n/s
// "integrator" k value
const double k = 1; // if its sluggish, increase this
// holds right and left wheel speeds (actual)
int Rwheel = 1500, Lwheel = 1500;
// holds the target speeds for the motors
int Rtarg = 1500, Ltarg = 1500;
// holds signal sent to motors, outside for diagnostic purposes
int rSig = 1500, lSig = 1500;
// light sensor values
int lPhotoVal; // Variable to store L photoresistor value
int rPhotoVal; // Variable to store R photoresistor value
// prox sensor values
int backProxVal;
int frontProxVal;
// max distance from medkit drop-off before dropping off
const int dropoffDist = 200;
// tells if medkit door is closed
bool isClosed = true;
// Create Servo Objects as defined in the Servo.h files
Servo L_Motor; // Servo DC Motor Driver (Designed for RC cars)
Servo R_Motor; // Servo DC Motor Driver (Designed for RC cars)
Servo Arm; // Arm servo
Servo medServo; // Medkit door servo
// for auto mode: tells if moved LSY down before trying to go
bool ready = false;
// for auto mode: tells if moved LSY back up
bool go = false;
// for auto wall climb: see function for use
int wallClimbMode = 0;
// for auto wall climb: marks the time the arm has been moving
double armTime;
// for use if I begin to track the time
double t = 0;
// for printing diagnostic data without interrupting control motion
double diagT = 0;
//**************************************************************
//**************************
Macros
**************************
//**************************************************************
/**
* diagnostic mode stuff. If don't want it, comment
* "#define ______" lines out. Its best if you only
* have one uncommented at a time
*/
#define DELAY 1
// DIAGNOSTIC: print values for RC Mode
#ifndef DIAG_RC
//#define DIAG_RC
#endif
// DIAGNOSTIC: print how much time each loop takes
#ifndef DIAG_B
//#define DIAG_B
#endif
// DIAGNOSTIC: print the control parameters
#ifndef DIAG_C
/**
#define DIAG_C
#undef DELAY
#define DELAY 0.1
/**/
#endif
// DIAGNOSTIC: print values of sensors
#ifndef DIAG_S
/**/
#define DIAG_S
#undef DELAY
#define DELAY 0.1
/**/
#endif
//**************************************************************
//***************** Setup & Timing ****************************
//**************************************************************
/**
* gets the current time in seconds.
*/
double time() {
return ((double)millis()) / 1000.0;
}
/**
* setup()
*/
void setup() {
// change mode of transmitter pins to input mode
pinMode(RSXPin, INPUT); // Ch1
pinMode(RSYPin, INPUT); // Ch2
pinMode(LSXPin, INPUT); // Ch3
pinMode(LSYPin, INPUT); // Ch4
pinMode(AutoPin, INPUT); // Ch5
pinMode(Ch6Pin, INPUT); // Ch6
pinMode(LED, OUTPUT); // Onboard LED to output for diagnostics
pinMode(LimitSwitchPin, INPUT_PULLUP); // top arm limit switch
// Attach Speed controller that acts like a servo to the board
R_Motor.attach(RServoPin);
L_Motor.attach(LServoPin);
// Attach the arm servo
Arm.attach(ArmPin);
// Attach medkit servo
medServo.attach(medPin);
//Flash the LED on and Off 10x before entering main loop
for (int i = 0; i < 10; i++) {
digitalWrite(LED, HIGH);
delay(200);
digitalWrite(LED, LOW);
delay(200);
}
// enable USB reading
Serial.begin(9600);
// make sure medkit door is closed
medClose();
#ifdef DIAG_RC
Serial.println("LSX,LSY,RSX,RSY,Ch5,Ch6");
#endif
// DIAGNOSTIC: for Serial Plotting controller data
#ifdef DIAG_C
Serial.println("rw,Rtarg,rSig,lw,Ltarg,lSig");
#endif
// DIAGNOSTIC: for Serial Plotting prox data
#ifdef DIAG_S
Serial.println("limt,LPhoto,RPhoto,FProx,BProx");
#endif
}
/**
* Main Loop
*/
void loop() {
// gets the current time (used in several spots later)
double t2 = time();
// check if mode swap
double oCh5 = Ch5; // old value
// read from photo sensors
// NOTE: darker is higher
rPhotoVal = analogRead(RPhotoPin);
lPhotoVal = analogRead(LPhotoPin);
// read from prox sensors
backProxVal = proxDist(BackProxPin);
frontProxVal = proxDist(FrontProxPin);
// check if should be in auto mode
Ch5 = pulseIn(AutoPin, HIGH, 21000);
Ch6 = pulseIn(Ch6Pin, HIGH, 21000);
if (Ch5 > 1600) { // auto mode
// check for mode swap
if (oCh5 <= 1600) { resetMotion(); }
// use this to tell it to progress
LSY = pulseIn(LSYPin, HIGH, 21000);
// make it so it will only run if you pull LSY down
if (!ready && LSY < 1700) {
ready = true;
} if (ready && LSY > 1300) {
go = true;
}
// check which auto mode it should be in
if (Ch6 > 1500) {// climb the wall vt
digitalWrite(LED, HIGH);
// climb the wall
if (ready && go) { climbWall(); }
else { resetMotion(); }
} else { // deliver the medkit
// do a "heartbeat blink" to signal the mode
if ((int)t2 % 2 == 0 && (int)(4*t2) % 2 == 0) {
digitalWrite(LED, HIGH);
} else {
digitalWrite(LED, LOW);
}
// deliver the medkit
if (ready && go) { deliverMedkit(); }
else { resetMotion(); }
}
} else { // RC mode
// check for mode swap
if (oCh5 > 1600) { resetMotion(); }
// capture from recieve
RSX = pulseIn(RSXPin, HIGH, 21000);
RSY = pulseIn(RSYPin, HIGH, 21000);
LSX = pulseIn(LSXPin, HIGH, 21000);
LSY = pulseIn(LSYPin, HIGH, 21000);
/*if (LSX > 1500) {
medDrop();
} else {
medClose();
}*/
digitalWrite(LED, LOW);
// set the motor targets
setWheelTargets();
}
// only update the motors if enough time has passed
double dt = t2 - t;
if (dt > cylcedt) {
// update the current time
t = t2;
// update the speeds and positions to send to the motors and servos
updateWheelSpeeds();
}
// cause the motors to drive
drive();
// move the arm
moveArm();
double diagT2 = time();
if (diagT2 - diagT > DELAY){
diagT = diagT2;
// DIAGNOSTIC: print how much time each loop takes
#ifdef DIAG_B
bench("loop() ", dt);
#endif
// DIAGNOSTIC: print values for RC Mode
#ifdef DIAG_RC
PrintRC();
#endif
// DIAGNOSTIC: print values of sensors
#ifdef DIAG_S
printSensors();
#endif
#ifdef DIAG_C
printCtrl();
#endif
}
}
/**
* Resets the robot
*/
void resetMotion() {
Rwheel = 0;
Lwheel = 0;
LSY = 0;
ready = false;
go = false;
wallClimbMode = 1;
}
//**************************************************************
//***********************
Drive System
***********************
//**************************************************************
/**
* Sets the target speed for the wheels by reading data
* sent from the controller.
*/
void setWheelTargets() {
int RSY_mod = RSY;
// fix wierd drive issue
int RSX_mod = map(RSX, 1000, 2000, 2000, 1000);
Ltarg = RSY_mod + RSX_mod - 1500;
Rtarg = RSY_mod - RSX_mod + 1500;
// make sure targets are between 1000 and 2000
Ltarg = constrain(Ltarg, 1000, 2000);
Rtarg = constrain(Rtarg, 1000, 2000);
}
/**
* Update Rwheel and Lwheel so that they are closer to Rtarg and
* Ltarg. This should have a delay between calls so the wheels don't
* slip.
*/
void updateWheelSpeeds() {
// Right////////////////////////////////////////////
// find where to cap Rwheel value
int upper = 2000;
int lower = 1000;
if (Rwheel <= Rtarg) { // increasing to Rtarg
upper = Rtarg;
} if (Rwheel >= Rtarg) { // decreasing to Rtarg
lower = Rtarg;
}
// update Rwheel
int dRW = (Rtarg - Rwheel) * k;
Rwheel += constrain(dRW, -dvmax, dvmax);
Rwheel = constrain(Rwheel, lower, upper);
// Left/////////////////////////////////////////////
// find where to cap Lwheel value
upper = 2000;
lower = 1000;
if (Lwheel <= Ltarg) { // increasing to Rtarg
upper = Ltarg;
} if (Lwheel >= Ltarg) { // decreasing to Rtarg
lower = Ltarg;
}
// update Lwheel
int dLW = (Ltarg - Lwheel) * k;
Lwheel += constrain(dLW, -dvmax, dvmax);
Lwheel = constrain(Lwheel, lower, upper);
}
/**
* Causes the motor to drive.
*
* Technically, this updates the current speed of the motor in
* a way that keeps the robot from sliding.
*/
void drive() {
// convert the wheel speeds to signals to send to the motors
lSig = map(Lwheel, 1000, 2000, MAX_SIG, MIN_SIG);
rSig = map(Rwheel, 1000, 2000, MIN_SIG, MAX_SIG);
// send the signals to the motors
R_Motor.writeMicroseconds(rSig);
L_Motor.writeMicroseconds(lSig);
}
//**************************************************************
//***********************
Arm System
*************************
//**************************************************************
/**
* Sets the speed of the arm (with LSY > 1500 being up)
*/
void moveArm() {
// convert the right stick signal value to an arm servo signal
LSY = constrain(LSY, 1000, 2000);
int armSig = map(LSY, 1000, 2000, MIN_SIG, MAX_SIG);
/* Dead zone *
if (LSY > 1500) {
} else {
}
/* end Dead Zone*/
// stop arm if either limit switch is hit
if (digitalRead(LimitSwitchPin) == LOW && LSY > 1500) {
armSig = 1500;
}
// send the signal to the servo
Arm.writeMicroseconds(armSig);
}
/**
* Used to move arm in auto mode, since LSY may be used in other
* cases.
*/
void moveArmAuto(int signal) {
int LSY_mod = LSY;
LSY = signal;
moveArm();
LSY = LSY_mod;
}
//**************************************************************
//*************************
Sensors
**************************
//**************************************************************
/**
* NOTE: analogRead reads a value from 0 to 1023.
*/
// converts voltage to the corresponding analog read value
// (0-1023)
int adc(double volts) {
return (int) map(volts, 0.0, 3.3, 0, 1023);
}
/**
* Prox Sensor: measures 10-80 cm and sends data as an analog
* value (See Digikey part # 1855-1062-ND)
*/
// get distance from a prox sensor and returns it as an int from
// 0 to 5115 (1023*5). See Data curves for exact values.
int proxDist(int proxPin) {
int pv = 0;
for (int i = 0; i < 5; i++) {
pv += analogRead(proxPin);
}
return pv;
}
//**************************************************************
//************************
Auto Mode
*************************
//**************************************************************
/*** NOTE: need auto mode to only set the target speeds/positions
* for the servos and motors.*/
/** * NOTE: want it to perform so it stops one set of wheels at 20
* degrees. */
// WALL CLIMB
// climbs the wall
void climbWall() {
/*** NOTE: step 5: if the arm folds in before it tips, use time (and hope)*/
// check current task based on wall climb mode
switch (wallClimbMode) {
case 0: // make sure at the wall
if (frontProxVal >= 1023) {
wallClimbMode = 1;
} else {
// drive forward
RSY = 2000;
RSX = 1500;
LSY = 2000;
// use arm for small push if need be (just pulse for a bit then go back)
break;
}
case 1: // initial climb up 1st step
// check if done climbing
if (backProxVal >= adc(0.9)) { // found distance of ~12"
// done climbing
wallClimbMode = 2;
} else {
// drive forward
RSY = 2000;
RSX = 1500;
//LSY = 1000;
// use arm for small push if need be (just pulse for a bit then go back)
break;
}
case 2: // driving to position for arm
// check if done driving
if (frontProxVal >= adc(1.65)) { // found distance of ~6"
// done driving
wallClimbMode = 3;
armTime = t;
} else {
// drive forward
RSY = 1700;
RSX = 1500;
//LSY = 2000;
break;
}
// NOTE: if want to check climb fail, do it as first if
case 3: // arm latch
if (t - armTime > 3.0) { // at correct angle
// done driving
wallClimbMode = 4;
} else {
// move arm into position
moveArmAuto(1300);
break;
}
case 4: // final climb
// check if cleared the first step
if (frontProxVal >= 1000) { // cleared
// done driving
wallClimbMode = 5;
} else {
// move arm and drive forward
moveArmAuto(1000);
RSY = 2000;
RSX = 1500;
break;
}
case 5: // getting over it
// go until bottom switch hit
if (backProxVal >= 1000) { // hit
wallClimbMode = 6;
} else {
// move arm and drive forward
moveArmAuto(1000);
RSY = 2000;
RSX = 1500;
break;
}
/** NOTE: watch for flipping **/
case 6: // land
break;
default: // wait
break;
}
}
// MEDKIT DELIVERY
// Find the zero direction and then go to the light
void seekLight() {
// convert them to "distances:" this way, when the sensor
// is far away (and the intensity is low), the value will be
// high, and the opposite when the sensor is cloes.
//int rpv = 1023 - rPhotoVal;
//int lpv = 1023 - lPhotoVal;
int rpv = rPhotoVal; // it works opposite of what I expected
int lpv = lPhotoVal;
// Find a good starting point for the light search by
// finding where the "light distance" is < 700
if (rpv < 700 && lpv < 700){ // play with 700 value
// rotate to find when that's not true
RSY = 1200;
RSX = 1700;
setWheelTargets();
} else {
// set wheel targets: turn left (decrease Ltarg relative)
// to Rtarg) if
Ltarg = map(rpv, 0, 1023, 1500, 1000); // backwards ? 2000 -> 1000
Rtarg = map(lpv, 0, 1023, 1500, 1000);
/* "forward bias method:" *
int bias = map(lpv - rpv, 0, 1023, 200, 0);
Ltarg = map(rpv - bias, -200, 1023, 1000, 2000); // backwards ? 2000 -> 1000
Rtarg = map(lpv + bias, -200, 1023, 1000, 2000);
/* NOTE: Bias' max sets the stopping dist for _Targ:
// rpv and lpv have to == 200 for no motion, This could
// be overcome by a prox/light sensor check, but can't
// be shorter than the distance set by this */
Ltarg = constrain(Ltarg, 1000, 2000);
Rtarg = constrain(Rtarg, 1000, 2000);
// make RSY and RSX follow Ltarg and Rtarg (for monitoring purposes)
int RSY_mod = (Ltarg + Rtarg)/2;
int RSX_mod = (Ltarg - Rtarg + 3000)/2;
RSY = RSY_mod;
RSX = map(RSX_mod, 1000, 2000, 2000, 1000);
}
}
// open med doors to drop the medkit
void medDrop() {
medServo.writeMicroseconds(1000);
isClosed = false;
}
// close med doors
void medClose() {
medServo.writeMicroseconds(1700);
isClosed = true;
}
// returns true if close enough to drop medkit, false otherwise
bool shouldDrop() {
// NOTE: if isn't far enough down, use light sensors
double dist = proxDist(BackProxPin);
return dist < 200;
}
// do medkit process
void deliverMedkit() {
// check if close enough to drop medkit
if (shouldDrop()) {
// wait 3 seconds to make sure stopped
delay(3000);
// drop the medkit
medDrop();
return;
}
// find the light
seekLight();
}
//**************************************************************
//***********************
Diagnostics
************************
//**************************************************************
void printSensors() {
Serial.printf("%d %d %d %d %d\r\n",
digitalRead(LimitSwitchPin)*1023,
lPhotoVal/5, rPhotoVal/5, frontProxVal, backProxVal
);
}
// print Channel values
void PrintRC() {
Serial.printf("%d %d %d %d %d %d\r\n", LSX, LSY, RSX, RSY, Ch5, Ch6);
}
// prints the relevant control parameters
void printCtrl(){
/**/
int rw = map(Rwheel, 1500, 2000, 1500, 2000);
int lw = map(Lwheel, 1500, 2000, 1500, 2000);
Serial.printf("%d %d %d %d %d %d\r\n", rw, Rtarg, rSig, lw, Ltarg, lSig);
/**/
}
void bench(String name, double t) {
Serial.printf("%s took %d seconds\n", name.c_str(), t);
}
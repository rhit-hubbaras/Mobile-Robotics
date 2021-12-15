/*
  NOTE:
   THIS IS THE STANDARD FOR HOW TO PROPERLY COMMENT CODE
   Header comment has program, name, author name, date created
   Header comment has brief description of what program does
   Header comment has list of key functions and variables created with decription
   There are sufficient in line and block comments in the body of the program
   Variables and functions have logical, intuitive names
   Functions are used to improve modularity, clarity, and readability
***********************************
  Lab2.ino
  Andrew Hubbard, Nithin Saravanapandian; 12/5/21

  This program introduces using the stepper motor library to create motion algorithms for the robot.
  The motions will be go to angle, go to goal, move in a circle, square, figure eight and teleoperation (stop, forward, spin, reverse, turn)
  The primary functions created are
  moveCircle - given the diameter in inches and direction of clockwise or counterclockwise, move the robot in a circle with that diameter
  moveFigure8 - given the diameter in inches, use the moveCircle() function with direction input to create a Figure 8
  forward - both wheels move with same velocity, same direction
  pivot- one wheel stationary, one wheel moves forward or back
  spin - both wheels move with same velocity opposite direction
  turn - both wheels move with same direction different velocity
  stop -both wheels stationary

  Interrupts
  https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
  https://www.arduino.cc/en/Tutorial/CurieTimer1Interrupt
  https://playground.arduino.cc/code/timer1
  https://playground.arduino.cc/Main/TimerPWMCheatsheet
  http://arduinoinfo.mywikis.net/wiki/HOME

  Hardware Connections:
  pin mappings: https://www.arduino.cc/en/Hacking/PinMapping2560
  digital pin 13 - enable LED on microcontroller
  digital pin 48 - enable PIN on A4988 Stepper Motor Driver StepSTICK
  digital pin 50 - right stepper motor step pin
  digital pin 51 - right stepper motor direction pin
  digital pin 52 - left stepper motor step pin
  digital pin 53 - left stepper motor direction pin

  digital pin 5 - red LED in series with 220 ohm resistor
  digital pin 6 - green LED in series with 220 ohm resistor
  digital pin 7 - yellow LED in series with 220 ohm resistor


  INSTALL THE LIBRARY
  Sketch->Include Library->Manage Libraries...->AccelStepper->Include
  OR
  Sketch->Include Library->Add .ZIP Library...->AccelStepper-1.53.zip
*/

#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library

//define pin numbers
const int rtStepPin = 50; //right stepper motor step pin (pin 44 for wireless)
const int rtDirPin = 51;  // right stepper motor direction pin (pin 49 for wireless)
const int ltStepPin = 52; //left stepper motor step pin (pin 46 for wireless)
const int ltDirPin = 53;  //left stepper motor direction pin (no change in pin for wireless)
const int stepTime = 500; //delay time between high and low on step pin
const int ftIRPin = 0;
const int bkIRPin = 1;
const int ltIRPin = 2;
const int rtIRPin = 3;
const int ltSonarPin = 4;
const int rtSonarPin = 5;

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);//create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);//create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;//create instance to control multiple steppers at the same time

#define stepperEnable 48    //stepper enable pin on stepStick 
#define enableLED 13        //stepper enabled LED
#define redLED 5           //red LED for displaying states
#define grnLED 6         //green LED for displaying states
#define ylwLED 7        //yellow LED for displaying states
#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor

#define pauseTime 3000 //time before robot moves

void setup()
{
  pinMode(rtStepPin, OUTPUT);//sets pin as output
  pinMode(rtDirPin, OUTPUT);//sets pin as output
  pinMode(ltStepPin, OUTPUT);//sets pin as output
  pinMode(ltDirPin, OUTPUT);//sets pin as output
  pinMode(stepperEnable, OUTPUT);//sets pin as output
  digitalWrite(stepperEnable, stepperEnFalse);//turns off the stepper motor driver
  pinMode(enableLED, OUTPUT);//set enable LED as output
  digitalWrite(enableLED, LOW);//turn off enable LED
  pinMode(redLED, OUTPUT);//set red LED as output
  pinMode(grnLED, OUTPUT);//set green LED as output
  pinMode(ylwLED, OUTPUT);//set yellow LED as output
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  delay(pauseTime / 5); //wait 0.5 seconds
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  digitalWrite(grnLED, LOW);//turn off green LED


  stepperRight.setMaxSpeed(1000);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(10);//set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(1000);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(10);//set desired acceleration in steps/s^2
  steppers.addStepper(stepperRight);//add right motor to MultiStepper
  steppers.addStepper(stepperLeft);//add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);//turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);//turn on enable LED
  delay(pauseTime); //always wait 2.5 seconds before the robot moves
  //Serial.begin(9600); //start serial communication at 9600 baud rate for debugging

  stepperRight.move(1000000);
  stepperLeft.move(1000000);
  stepperRight.stop();
  stepperLeft.stop();
  
  digitalWrite(redLED, HIGH);
  digitalWrite(grnLED, LOW);
  digitalWrite(ylwLED, LOW);

}
// Front IR Calibration : dist (in) = 28*exp(-0.0116*reading) + 2
const float wlRadius = 1.66; //wheel radius in inches
const float wbRadius = 3.76; //wheelbase radius in inches
void loop(){
  int avg=0;
  for (int i = 0; i < 9 ; i++) {
  avg = avg + analogRead(ftIRPin);
  }
  avg = avg/10;
  float frIRdist = 28*exp(-0.0116*avg) + 2;
  if(frIRdist < 5){
    
    stepperRight.stop();
    stepperLeft.stop();
  }else{
    
  
  stepperRight.setSpeed(700);
  stepperLeft.setSpeed(700);
    
  for (int i = 0; i < 19 ; i++) {
  stepperRight.runSpeed();
  stepperLeft.runSpeed();
  }
  }
}

/*This function, runToStop(), will run the robot until the target is achieved and
   then stop it
*/
void runToStop ( void ) {
  int runNow = 1;
  int rightStopped = 0;
  int leftStopped = 0;

  while (runNow) {
    if (!stepperRight.run()) {
      rightStopped = 1;
      stepperRight.stop();//stop right motor
    }
    if (!stepperLeft.run()) {
      leftStopped = 1;
      stepperLeft.stop();//stop ledt motor
    }
    if (rightStopped && leftStopped) {
      runNow = 0;
    }
  }
}

/*function to run both wheels to a position at speed*/
void runAtSpeedToPosition() {
  stepperRight.runSpeedToPosition();
  stepperLeft.runSpeedToPosition();
}

/*function to run both wheels continuously at a speed*/
void runAtSpeed ( void ) {
  while (stepperRight.runSpeed() || stepperLeft.runSpeed()) {
  }
}

// Front IR  Calibration : dist (in) = 24.4*exp(-0.00948*reading) + 1.66
// Back  IR  Calibration : dist (in) = 30.4*exp(-0.01009*reading) + 1.75
// Left  IR  Calibration : dist (in) = 55.4*exp(-0.00758*reading) + 1.48
// Right IR  Calibration : dist (in) = 41.7*exp(-0.00694*reading) + 1.43
// Left  SNR Calibration : dist (in) = 0.00694*reading - 1.292
// Right SNR Calibration : dist (in) = 0.00714*reading - 1.158

/*Functions to read sensor data in inches*/
float readFrontIR() { 
  int avg=0;
  for (int i = 0; i < 9 ; i++) {
  avg = avg + analogRead(ftIRPin);
  }
  avg = avg/10;
  float dist = 24.4*exp(-0.00948*avg) + 1.66;
  return dist;
}

float readBackIR() { 
  int avg=0;
  for (int i = 0; i < 9 ; i++) {
  avg = avg + analogRead(bkIRPin);
  }
  avg = avg/10;
  float dist = 30.4*exp(-0.01009*avg) + 1.75;
  return dist;
}

float readLeftIR() { 
  int avg=0;
  for (int i = 0; i < 9 ; i++) {
  avg = avg + analogRead(ltIRPin);
  }
  avg = avg/10;
  float dist = 55.4*exp(-0.00758*avg) + 1.48;
  return dist;
}

float readRightIR() { 
  int avg=0;
  for (int i = 0; i < 9 ; i++) {
  avg = avg + analogRead(rtIRPin);
  }
  avg = avg/10;
  float dist = 41.7*exp(-0.00694*avg) + 1.43;
  return dist;
}

float readLeftSonar(){
  long value=0;
  for (int i = 0; i < 9 ; i++) {
      
    pinMode(ltSonarPin, OUTPUT);//set the PING pin as an output
    digitalWrite(ltSonarPin, LOW);//set the PING pin low first
    delayMicroseconds(2);//wait 2 us
    digitalWrite(ltSonarPin, HIGH);//trigger sonar by a 2 us HIGH PULSE
    delayMicroseconds(5);//wait 5 us
    digitalWrite(ltSonarPin, LOW);//set pin low first again
    pinMode(ltSonarPin, INPUT);//set pin as input with duration as reception time
    value = value + pulseIn(ltSonarPin, HIGH);
  }
  value=value/10;
  float dist = 0.00694*value - 1.292;
  return dist;
}

float readRightSonar(){
  long value=0;
  for (int i = 0; i < 9 ; i++) {
      
    pinMode(rtSonarPin, OUTPUT);//set the PING pin as an output
    digitalWrite(rtSonarPin, LOW);//set the PING pin low first
    delayMicroseconds(2);//wait 2 us
    digitalWrite(rtSonarPin, HIGH);//trigger sonar by a 2 us HIGH PULSE
    delayMicroseconds(5);//wait 5 us
    digitalWrite(rtSonarPin, LOW);//set pin low first again
    pinMode(rtSonarPin, INPUT);//set pin as input with duration as reception time
    value = value + pulseIn(rtSonarPin, HIGH);
  }
  value=value/10;
  float dist = 0.00714*value - 1.158;
  return dist;
}

  
/*
  Rotate around a static wheel.
  direction - true:CCW, false:CW
  distance - ticks
*/
void pivot(bool direction, int distance) {

  if (direction) { //choose which wheel to move
    stepperLeft.move(0);
    stepperRight.move(distance);
  }
  else {
    stepperLeft.move(distance);
    stepperRight.move(0);
  }
  runAtSpeedToPosition();
  runToStop();
}

/*
  Rotate around the wheelbase center.
  distance - ticks; positive:CCW, negative:CW
*/
void spin(int distance) {

  stepperLeft.move(-distance);
  stepperRight.move(distance);
  runAtSpeedToPosition();
  runToStop();
}

/*
  Move in a circular arc. Speeds must be set beforehand
  distance - distance to move in ticks, as measured for the wheelbase center
*/
void turn(int distance) {

  float lSpeed = abs(stepperLeft.maxSpeed());
  float rSpeed = abs(stepperRight.maxSpeed());
  float lDistance = 2 * distance * lSpeed / (lSpeed + rSpeed);
  float rDistance = 2 * distance * rSpeed / (lSpeed + rSpeed);

  stepperLeft.move(lDistance);
  stepperRight.move(rDistance);
  runAtSpeedToPosition();
  runToStop();

}
/*
  Move in a straight line.
  distance - distance to move in ticks; negative values move backwards
*/
void forward(int distance) {


  stepperLeft.move(distance);
  stepperRight.move(distance);
  runAtSpeedToPosition();
  runToStop();


}
/*
  Move backwards. For compatibility; use forward() with a negative argument.
*/
void reverse(int distance) {
  forward(-distance);
}

/*
  Stop both wheels
*/
void stop() {
  stepperLeft.stop();
  stepperRight.stop();
}




/*
  Rotate to a given angle
  angle - angle to rotate in radians. Positive : CCW, negative : CW
*/
void goToAngle(float angle) {

  float dist = angle / 2 / PI * 800 * wbRadius / wlRadius;
  stepperLeft.setMaxSpeed(300);
  stepperRight.setMaxSpeed(300);
  spin(dist);
}

/*
  Move in a straight line to a given target. Coordinate system is oriented with x-axis forward and y-axis to the left.
  x - x-coordinate of target, in inches
  y - y-coordinate of target, in inches
*/
void goToGoal(float x, float y) {

  float distIn = sqrt(x * x + y * y);
  float dist = distIn / 2 / PI / wlRadius * 800;
  float angle = atan2(y, x);
  goToAngle(angle);
  stepperLeft.setMaxSpeed(1000);
  stepperRight.setMaxSpeed(1000);
  forward(dist);

}


/*
  Drive in a circle
  diam - diameter of circle, in inches, as measured to the wheelbase center
  dir - direction of circle. true : CCW, false : CW
*/
void moveCircle(int diam, bool dir) {


  float radius = diam / 2;
  float speedOuter = 500 * (wbRadius + radius) / radius;
  float speedInner = 500 * (-wbRadius + radius) / radius;
  float distIn = 2 * PI * radius;
  float dist = distIn / 2 / PI / wlRadius * 800;
  if (dir) { //choose speeds based on direction
    stepperLeft.setMaxSpeed(speedInner);
    stepperRight.setMaxSpeed(speedOuter);
  } else {
    stepperLeft.setMaxSpeed(speedOuter);
    stepperRight.setMaxSpeed(speedInner);
  }
  turn(dist);

}

/*
  The moveFigure8() function takes the diameter in inches as the input. It uses the moveCircle() function
  twice with 2 different direcitons to create a figure 8 with circles of the given diameter.
*/
void moveFigure8(int diam) {


  moveCircle(diam, true);
  moveCircle(diam, false);
}

/*
  Drive in a square.
  side - length in inches of side
*/
void moveSquare(int side) {

  goToGoal(side, 0);
  goToGoal(0, side);
  goToGoal(0, side);
  goToGoal(0, side);

}

/************************************
  Final.ino
  Andrew Hubbard, Nithin Saravanapandian; 1/24/22
  This program introduces photosensitivity to the robot. It introduces the braitenburg bot behaviors, and uses a state
  machine to incorperate them with avoidance and wander behaviors. It combines braitenburg bots and wall following to
  make a homing behavior, where the robot leaves the wall to seek out a light source, and returns to the wall after
  investigating.

  New methods:
  HomingStateMachine - main method responsible for wall following and light homing/docking behavior
  BraitenburgStateMachine - main method responsible for braitenburg with obstacle avoidance behavior
  braitenburg - moves following the rules of the braitenburg bots. bot type selected by two true/false parameters
  calibratePhoto - calibrate photosensors
  readPhotoSensors - read photosensor data
  normalizePhotoSensors - reduce photosensor data to usable range
  remapPhotoSensors - remap for backwards driving
  remapIRSensors - remap for backwards driving
*/

#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include <SoftwareSerial.h>


/*
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
  digital pin 40 - bluetooth reciever
  digital pin 41 - bluetooth transmitter

*/
//define pin numbers
const int rtStepPin = 50; //right stepper motor step pin (pin 44 for wireless)
const int rtDirPin = 51;  // right stepper motor direction pin (pin 49 for wireless)
const int ltStepPin = 52; //left stepper motor step pin (pin 46 for wireless)
const int ltDirPin = 53;  //left stepper motor direction pin (no change in pin for wireless)
const int btRecPin = 40;
const int btTransPin = 41;
const int stepTime = 500; //delay time between high and low on step pin
const int ftIRPin = 0;
const int bkIRPin = 1;
const int ltIRPin = 2;
const int rtIRPin = 3;
const int ltSonarPin = 2;
const int rtSonarPin = 3;
const int ltPhotoPin = 6;
const int rtPhotoPin = 7;

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);//create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);//create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;//create instance to control multiple steppers at the same time
//SoftwareSerial MyBlue(0, 1);

#define stepperEnable 48    //stepper enable pin on stepStick 
#define enableLED 13        //stepper enabled LED
#define redLED 5           //red LED for displaying states
#define grnLED 6         //green LED for displaying states
#define ylwLED 7        //yellow LED for displaying states
#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor

#define pauseTime 3000 //time before robot moves


//Initialize position tracking variables
float RobotPos[3];
long stepperPos[2];
int backupCounter = 0;
float pastErr = 0;
float angleAtFollow = 0;
float angleAtTurn = 0;
float returnPos[3];

//Initialize photoresistor sensors
int lBright = 980;
int rBright = 980;
int lAmbiant = 650;
int rAmbiant = 500;

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


  stepperRight.setMaxSpeed(600);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(10000);//set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(600);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(10000);//set desired acceleration in steps/s^2
  steppers.addStepper(stepperRight);//add right motor to MultiStepper
  steppers.addStepper(stepperLeft);//add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);//turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);//turn on enable LED


  Serial.begin(115200);
  //  MyBlue.begin(9600);
  //  pinMode(0, INPUT);
  //  pinMode(1, OUTPUT);
  //  Serial.println("Serial Communitaion Active");
  delay(pauseTime); //always wait 2.5 seconds before the robot moves
  //  Serial.begin(9600); //start serial communication at 9600 baud rate for debugging
  randomSeed(analogRead(0));

  //Initialize position tracking variables

  RobotPos[0] = 0;
  RobotPos[1] = 0;
  RobotPos[2] = 0;
  stepperPos[0] = 0;
  stepperPos[1] = 0;

  calibratePhoto();
  localize();
}

//State machine states
volatile byte state = 0;
#define idle         0
#define topo         1

volatile byte topoState = 0;
#define topoStart       0
#define topoFollowL     1
#define topoFollowR     2
#define topoTurnL       3
#define topoTurnR       4
#define topoStraight    5
#define topoStop        6
#define topoErr         7


volatile byte metricState = 0;
#define metricStart       0
#define metricTurnL       3
#define metricTurnR       4
#define metricStraight    5
#define metricStop        6
#define metricErr         7

volatile byte wallState = 0;
int stateCounter = 0;

//Braitenburg state machine
volatile byte stateBrait = 0;
#define randWand      0
#define avoid         1
#define brait         2

String topoDirections = "LRST";
int topoPointer = 0;

//metric path planning global variables
String metricDirections = "";
String start = "";
String finalgoal = "";
int rownodelist[];
int colnodelist[];
int nodecount = 1;

String message = "";
char command = '0';


/*
   MAIN LOOP CALL
*/
void loop() {
  localize();

  float sensors[6];
  readIRSensors(sensors);
  remapIRSensors(sensors);

  float photoSensors[2];
  readPhotoSensors(photoSensors);
  normalizePhotoSensors(photoSensors);
  remapPhotoSensors(photoSensors);

  //read bluetooth
  char msg;
  if (Serial.available()) {
    msg = Serial.read();
    if (command == 'm') {
      metricDirections = msg;
    } else if (msg == '\n') {
      if (command == 's') {
        start = msg;
      }
      else if (command == 'f') {
        finalgoal = msg;
      }
    } else {
      message = message + msg;
    }
  }


  //  if(Serial.available()){
  //    msg = Serial.read();
  //    if(command == '0'){
  //      command = msg;
  //    }else if(msg == '\n'){
  //      if(command == 't'){
  //        topoDirections = message;
  //        topoPointer = 0;
  //        topoState = topoStart;
  //        state = topo;
  //      }
  //      command = '0';
  //      message = "";
  //    }else{
  //      message = message + msg;
  //    }
  //    Serial.println(message);
  //  }

  //state machine including task/idle
  //  switch(state){
  //    case idle:
  //      break;
  //    case topo:
  //      topoFollowing(sensors);
  //      break;
}
}


const float wlRadius = 1.66; //wheel radius in inches
const float wbRadius = 3.76; //wheelbase radius in inches


void metricFollowing(float sensors[]) {
  int avoidDist = 0;
  int wallDetectDist = 12;
  int frontDetectDist = 4;

  if (sensors[0] < avoidDist || sensors[1] < avoidDist || sensors[2] < avoidDist || sensors[3] < avoidDist) {
    shyKid2(sensors, avoidDist); //avoid obstacles
  }
  else {

  }
}

void metricPlanning(String metricDirections, String start, String finalgoal) {

  int beg = 0;
  int mend = 1;
  String wallmap[4][4] = {};
  for y = 0 : 3{
  for x = 0 : 3 {
    wallmap[x][y] = metricDirections.substring(beg, mend);
      beg = beg + 1;
      mend = mend + 1;
    }
  }

  int startx = start.charAt(0);
  int starty = start.charAt(1);
  int goalx = finalgoal.charAt(0);
  int goaly = finalgoal.charAt(1);


  wallmap[startx][starty] = "S";
  wallmap[goalx][goaly] = "G";

  int row = goalx;
  int col = goaly;

  // defining pathmap with steps from goal to start
  int stepmap[4][4] = {};
  stepmap[startx][starty] = 99;
  stepmap[goalx][goaly] = 0;
  int gridstep = 1;
  int value = 0;


  drow = [-1, 1, 0, 0];
  dcol = [0, 0, 1, -1];

  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 4; c++) {
      if (stepmap[r][c] == value) {
        for (i = 0; i < 4; i++) {
          int rrow = row + drow[i];
          int ccol = col + dcol[i];

          String walltype = wallmap[rrow][ccol];
          int wallcheck = walltype.toInt();

          if (rrow < 0 || ccol < 0 || rrow >= 4 || ccol >= 4) {
            null;
          }
          // always null regardless of direction
          else if (wallcheck = 15) {
            stepmap[rrow][ccol] = 98;
            rownodelist(nodecount) = rrow;
            colnodelist(nodecount) = ccol;
            nodecount += 1;
          }
          //wall check from looking up
          else if ( i = 0 && wallcheck == 5 || wallcheck == 6 || wallcheck == 7 || wallcheck == 12 || wallcheck == 13 || wallcheck == 14) {
            stepmap[rrow][ccol] = null;
            rownodelist(nodecount) = rrow;
            colnodelist(nodecount) = ccol;
            nodecount += 1;
          }
          //wall check looking down
          else if ( i = 1 && (wallcheck % 2) != 0) {
            stepmap[rrow][ccol] = null;
            rownodelist(nodecount) = rrow;
            colnodelist(nodecount) = ccol;
            nodecount += 1;
          }
          //wall check looking right
          else if ( i = 2 && wallcheck > 7) {
            stepmap[rrow][ccol] = null;
            rownodelist(nodecount) = rrow;
            colnodelist(nodecount) = ccol;
            nodecount += 1;
          }
          //wall check looking left
          else if ( i = 0 && wallcheck == 0 || wallcheck == 3 || wallcheck == 4 || wallcheck == 7 || wallcheck == 8 || wallcheck == 12 || wallcheck == 13) {
            stepmap[rrow][ccol] = null;
            rownodelist(nodecount) = rrow;
            colnodelist(nodecount) = ccol;
            nodecount += 1;
          }
          //previous node/grid check
          else if (rrow == rownodelist(nodecount) && ccol == colnodelist(nodecount)) {
            null;
          }
          else {
            stepmap[rrow][ccol] = gridstep;
            rownodelist(nodecount) = rrow;
            colnodelist(nodecount) = ccol;
            nodecount += 1;
          }
        }
        //outside neighboring grid loop
        value = gridstep;
        gridstep = gridstep + 1;

      }
    }
  }
}

}


void topoFollowing(float sensors[]) {
  int avoidDist = 0;
  int wallDetectDist = 12;
  int frontDetectDist = 4;

  if (sensors[0] < avoidDist || sensors[1] < avoidDist || sensors[2] < avoidDist || sensors[3] < avoidDist) {
    shyKid2(sensors, avoidDist); //avoid obstacles
  }
  else {
    char current = '0';
    if (topoPointer < topoDirections.length()) {
      current = topoDirections[topoPointer];
    }
    //Serial.println(current);
    //Serial.println(topoState);
    switch (topoState) {
      case topoStart:
        topoPointer = 0;
        if (current == 'L') {
          topoState = topoFollowL;
        } else if (current == 'R') {
          topoState = topoFollowR;
        } else if (current == 'S') {
          topoState = topoStraight;
        } else if (current == 'T') {
          topoState = topoStop;
        } else {
          topoState = topoErr;
        }
        topoPointer = topoPointer + 1;
        break;

      case topoFollowL: //Follow left wall

        digitalWrite(redLED, HIGH);
        digitalWrite(grnLED, LOW);
        digitalWrite(ylwLED, LOW);
        //change states according to state diagram
        if (sensors[0] < frontDetectDist) {
          topoState = topoErr;
        }
        else if (sensors[2] > wallDetectDist) {
          angleAtTurn = RobotPos[2];
          topoState = topoTurnL;
        }
        else { //keep following left wall

          //float err = sensors[2]-5;
          //bangBang(err);
          //PDcontrol(0);
          drive(-400);
        }
        break;


      case topoFollowR: //Follow right wall

        digitalWrite(redLED, LOW);
        digitalWrite(grnLED, HIGH);
        digitalWrite(ylwLED, LOW);
        //change states according to state diagram
        if (sensors[0] < frontDetectDist) {
          topoState = topoErr;
        }
        else if (sensors[3] > wallDetectDist) {
          angleAtTurn = RobotPos[2]; //right wall lost, turn right
          topoState = topoTurnR;
        }
        else { //keep following right wall
          //float err = 5-sensors[3];
          //bangBang(err);
          //PDcontrol(0);
          drive(-400);
        }
        break;


      case topoStraight: //follow center/hallway

        digitalWrite(redLED, HIGH);
        digitalWrite(grnLED, HIGH);
        digitalWrite(ylwLED, LOW);
        //change states according to state diagram
        if (sensors[0] < 6) {
          stepperRight.setMaxSpeed(400);
          stepperLeft.setMaxSpeed(400);

          stepperRight.setSpeed(400);
          stepperLeft.setSpeed(400);
          if (current == 'S') {
            topoState = topoStraight;
          } else if (current == 'L') {
            //goToAngle(PI/2);
            //reverse(inToSteps(12));
            topoState = topoFollowL;
          } else if (current == 'R') {
            //goToAngle(-PI/2);
            //reverse(inToSteps(12));
            topoState = topoFollowR;
          } else if (current == 'T') {
            topoState = topoStop;
          } else {
            topoState = topoErr;
          }
          topoPointer = topoPointer + 1;
        }
        else {
          float err = 0;
          if (sensors[2] > wallDetectDist && sensors[3] > wallDetectDist) {
            err = 0;
          } else if (sensors[2] < wallDetectDist && sensors[3] > wallDetectDist) {
            err = sensors[2] - 5;
          } else if (sensors[2] > wallDetectDist && sensors[3] < wallDetectDist) {
            err = 5 - sensors[3];
          } else {
            err = (sensors[2] - sensors[3]) / 2;
          }
          //bangBang(err);
          //PDcontrol(0);
          drive(-400);
        }
        break;


      case topoTurnL: //make outside left turn
        digitalWrite(redLED, HIGH);
        digitalWrite(grnLED, LOW);
        digitalWrite(ylwLED, LOW);
        //change states according to state diagram
        if (1 || abs(RobotPos[2] - angleAtTurn) > PI / 2) { //turned 90 degrees


          stepperRight.setMaxSpeed(400);
          stepperLeft.setMaxSpeed(400);

          stepperRight.setSpeed(400);
          stepperLeft.setSpeed(400);
          reverse(inToSteps(11));
          goToAngle(PI / 2);
          reverse(inToSteps(9));
          if (current == 'S') {
            topoState = topoStraight;
          } else if (current == 'L') {
            topoState = topoFollowL;
          } else if (current == 'R') {
            topoState = topoFollowR;
          } else if (current == 'T') {
            topoState = topoStop;
          } else {
            topoState = topoErr;
          }
          topoPointer = topoPointer + 1;
        }
        else { //keep following outside corner
          outsideTurn(1);
        }
        break;


      case topoTurnR: //make outside right turn
        digitalWrite(redLED, LOW);
        digitalWrite(grnLED, HIGH);
        digitalWrite(ylwLED, LOW);
        //change states according to state diagram
        if (1 || abs(RobotPos[2] - angleAtTurn) > PI / 2) { //turned 90 degrees

          stepperRight.setMaxSpeed(400);
          stepperLeft.setMaxSpeed(400);

          stepperRight.setSpeed(400);
          stepperLeft.setSpeed(400);
          reverse(inToSteps(11));
          goToAngle(-PI / 2);
          reverse(inToSteps(9));
          if (current == 'S') {
            topoState = topoStraight;
          } else if (current == 'L') {
            topoState = topoFollowL;
          } else if (current == 'R') {
            topoState = topoFollowR;
          } else if (current == 'T') {
            topoState = topoStop;
          } else {
            topoState = topoErr;
          }
          topoPointer = topoPointer + 1;
        }
        else { //keep following outside corner
          outsideTurn(-1);
        }
        break;


      case topoStop: //turn back around
        digitalWrite(redLED, LOW);
        digitalWrite(grnLED, LOW);
        digitalWrite(ylwLED, HIGH);
        //lights and done with dask
        state = idle;
        break;


      case topoErr:
        digitalWrite(redLED, HIGH);
        digitalWrite(grnLED, LOW);
        digitalWrite(ylwLED, HIGH);
        //lights, error, and done with dask
        state = idle;
        break;

    }
  }
}

/*
   braitenburg vehicles
   photoSensors - array of normalized photoresistor readings
   same - false if right photo controls left side; true if same side
   phphobic - false if light causes forward, true if light causes stop

   love - same, phobic
   aggression - not same, not phobic
   explorer - not same, phobic
   fear - same, not phobic
*/
void braitenburg(float photoSensors[], bool same, bool phobic) {
  float lPhoto = 0;
  float rPhoto = 0;
  if (same) {
    lPhoto = photoSensors[1];
    rPhoto = photoSensors[0];
  } else {
    lPhoto = photoSensors[0];
    rPhoto = photoSensors[1];
  }

  if (phobic) {
    lPhoto = 1 - lPhoto; //pow(1-sqrt(abs(lPhoto)),2)*sign(lPhoto);
    rPhoto = 1 - rPhoto; //pow(1-sqrt(abs(rPhoto)),2)*sign(rPhoto);
  }
  float lSpeed = -500 * lPhoto;
  float rSpeed = -500 * rPhoto;

  stepperRight.setMaxSpeed(rSpeed);
  stepperLeft.setMaxSpeed(lSpeed);

  stepperRight.setSpeed(rSpeed);
  stepperLeft.setSpeed(lSpeed);

  stepperRight.move(10000 * sign(rSpeed));
  stepperLeft.move(10000 * sign(lSpeed));
  int steps = 0;
  long Time = millis();
  while (steps < 50 && millis() - Time < 1000) {

    if (stepperRight.runSpeed()) {
      steps++;
    }
    if (stepperLeft.runSpeed()) {
      steps++;
    }
  }
}

/*
   state machine for braitenburg bot with randomWander and avoidance behavior
   sensors - IR sensor array
   photoSensors - photo sensor array
*/
void BraitenburgStateMachine(float sensors[], float photoSensors[]) {
  int avoidDist = 3;
  float lightDetect = 0.3;

  switch (state) {
    case randWand: //random wander

      digitalWrite(redLED, LOW);
      digitalWrite(grnLED, HIGH);
      digitalWrite(ylwLED, LOW);
      //change states according to state diagram
      if (sensors[0] < avoidDist || sensors[1] < avoidDist || sensors[2] < avoidDist || sensors[3] < avoidDist) {
        state = avoid; //avoid obstacles
      }
      else if (photoSensors[0] > lightDetect || photoSensors[1] > lightDetect) {
        state = brait; //light found, follow light
      }
      else { //stay in random wander
        randomWander();
      }
      break;


    case avoid: //Shy kid obstacle avoidance

      digitalWrite(redLED, HIGH);
      digitalWrite(grnLED, LOW);
      digitalWrite(ylwLED, LOW);
      //change states according to state diagram
      if (sensors[0] > avoidDist && sensors[1] > avoidDist && sensors[2] > avoidDist && sensors[3] > avoidDist) {
        state = randWand; //obstacle cleared, return to top
      }
      else { //stay in obstacle avoidance
        shyKid2(sensors, avoidDist); //use real sensor values for obstacle avoidance
      }
      break;


    case brait: //Follow left wall

      digitalWrite(redLED, LOW);
      digitalWrite(grnLED, HIGH);
      digitalWrite(ylwLED, HIGH);
      //change states according to state diagram
      if (sensors[0] < avoidDist || sensors[1] < avoidDist || sensors[2] < avoidDist || sensors[3] < avoidDist) {
        state = avoid; //avoid obstacles
      }
      else if (photoSensors[0] < lightDetect && photoSensors[1] < lightDetect) {
        stepperRight.move(0);
        stepperLeft.move(0);
        state = randWand; //light lost, return to random wander
      }
      else { //keep following light
        braitenburg(photoSensors, true, true);
      }
      break;
  }
}

/*
   state machine for wall following with light homing/docking behavior
   sensors - IR sensor array
   photoSensors - photo sensor array
*/
/*
  void HomingStateMachine(float sensors[],float photoSensors[]){
  int avoidDist = 1;
  int wallDetectDist = 12;
  int frontDetectDist = 4;
  float lightDetectLevel = 0.3;
  if(sensors[0] < avoidDist || sensors[1] < avoidDist || sensors[2] < avoidDist || sensors[3] < avoidDist){
      shyKid2(sensors,avoidDist); //avoid obstacles
    }
  else{
    switch(state){
      case randWand: //random wander

        digitalWrite(redLED, LOW);
        digitalWrite(grnLED, LOW);
        digitalWrite(ylwLED, LOW);
        //change states according to state diagram
        if(sensors[2] < wallDetectDist){
          state = followL; //wall found; follow left wall
        }
        else if(sensors[3] < wallDetectDist){
          state = followR; //wall found; follow left wall
        }
        else{ //stay in random wander
          randomWander();
        }
        break;


      case avoid: //Shy kid obstacle avoidance
        //obselete; handled by subsumption

        break;


      case followL: //Follow left wall

        digitalWrite(redLED, LOW);
        digitalWrite(grnLED, HIGH);
        digitalWrite(ylwLED, LOW);
        //change states according to state diagram
        if(sensors[0] < frontDetectDist){
          state = turnRin; //front wall found, turn right
        }
        else if(sensors[3] < wallDetectDist){
          state = followC; //right wall found, follow center
        }
        else if(photoSensors[0]>lightDetectLevel || photoSensors[1]>lightDetectLevel){//light found
          returnPos[0] = RobotPos[0];
          returnPos[1] = RobotPos[1];
          returnPos[2] = RobotPos[2];
          wallState = state;
          state = lightFollow;
        }
        else if(sensors[2] > wallDetectDist){
          angleAtTurn = RobotPos[2]; //left wall lost, turn left
          state = turnLout;
        }
        else if(0){//followed too long
          //turn away
          state = randWand;
        }
        else{ //keep following left wall
          float err = sensors[2]-5;
          //bangBang(err);
          PDcontrol(err);
        }
        break;


      case followR: //Follow right wall

        digitalWrite(redLED, LOW);
        digitalWrite(grnLED, HIGH);
        digitalWrite(ylwLED, LOW);
        //change states according to state diagram
        if(sensors[0] < frontDetectDist){
          state = turnLin; //front wall found, turn left
        }
        else if(sensors[2] < wallDetectDist){
          state = followC; //left wall found, follow center
        }
        else if(photoSensors[0]>lightDetectLevel || photoSensors[1]>lightDetectLevel){//light found
          returnPos[0] = RobotPos[0];
          returnPos[1] = RobotPos[1];
          returnPos[2] = RobotPos[2];
          wallState = state;
          state = lightFollow;
        }
        else if(sensors[3] > wallDetectDist){
          angleAtTurn = RobotPos[2]; //right wall lost, turn right
          state = turnRout;
        }
        else if(0){//too much follow
          //turn away
          state = randWand;
        }
        else{ //keep following right wall
          float err = 5-sensors[3];
          //bangBang(err);
          PDcontrol(err);
        }
        break;


      case followC: //follow center/hallway

        digitalWrite(redLED, HIGH);
        digitalWrite(grnLED, HIGH);
        digitalWrite(ylwLED, LOW);
        //change states according to state diagram
        if(sensors[0] < 6){
          state = turnB; //front wall found, turn around
        }
        else if(sensors[2] > wallDetectDist){
          state = followR; //left wall lost, follow right wall
        }
        else if(sensors[3] > wallDetectDist){
          state = followL; //right wall lost, follow left wall
        }
        else{ //keep following center
          float err = (sensors[2]-sensors[3])/2;
          //bangBang(err);
          PDcontrol(err);
        }
        break;


      case turnLin: //make inside left turn

        digitalWrite(redLED, HIGH);
        digitalWrite(grnLED, HIGH);
        digitalWrite(ylwLED, LOW);
        goToAngle(PI/2); //turn left, return to following left wall
        state = followL;
        break;


      case turnRin: //make inside right turn

        digitalWrite(redLED, HIGH);
        digitalWrite(grnLED, HIGH);
        digitalWrite(ylwLED, LOW);
        goToAngle(-PI/2); //turn right, return to following right wall
        state = followR;
        break;


      case turnLout: //make outside left turn
        digitalWrite(redLED, LOW);
        digitalWrite(grnLED, HIGH);
        digitalWrite(ylwLED, LOW);
        //change states according to state diagram
        if(sensors[2] < wallDetectDist){
          state = followL; //found & follow left wall
        }
        else if(photoSensors[0]>lightDetectLevel || photoSensors[1]>lightDetectLevel){//light found
          returnPos[0] = RobotPos[0];
          returnPos[1] = RobotPos[1];
          returnPos[2] = RobotPos[2];
          wallState = state;
          state = lightFollow;
        }
        else if(abs(RobotPos[2]-angleAtTurn)>PI/2){//turned 90 degrees
          goToAngle(-PI/2); //lost wall; turn away and random wander
          state = randWand;
        }
        else{//keep following outside corner
          outsideTurn(1);
        }
        break;


      case turnRout: //make outside right turn
        digitalWrite(redLED, LOW);
        digitalWrite(grnLED, HIGH);
        digitalWrite(ylwLED, LOW);
        //change states according to state diagram
        if(sensors[3] < wallDetectDist){
          state = followR; //found & follow right wall
        }
        else if(photoSensors[0]>lightDetectLevel || photoSensors[1]>lightDetectLevel){//light found
          returnPos[0] = RobotPos[0];
          returnPos[1] = RobotPos[1];
          returnPos[2] = RobotPos[2];
          wallState = state;
          state = lightFollow;
        }
        else if(abs(RobotPos[2]-angleAtTurn)>PI/2){//turned 90 degrees
          goToAngle(PI/2); //lost wall; turn away and random wander
          state = randWand;
        }
        else{//keep following outside corner
          outsideTurn(-1);
        }
        break;


      case turnB: //turn back around
        digitalWrite(redLED, HIGH);
        digitalWrite(grnLED, HIGH);
        digitalWrite(ylwLED, LOW);
        goToAngle(PI);//turn around and return to following center
        state = followC;
        break;

      case lightFollow: //follow light
        digitalWrite(redLED, LOW);
        digitalWrite(grnLED, LOW);
        digitalWrite(ylwLED, HIGH);
        //change states according to state diagram
        if(photoSensors[0]>lightDetectLevel && photoSensors[1]>lightDetectLevel && sensors[0]<frontDetectDist){//light seen and front wall
          for(int i=0;i<3;i++){
            digitalWrite(redLED, HIGH); //blink lights :D
            digitalWrite(grnLED, HIGH);
            digitalWrite(ylwLED, HIGH);
            delay(500);
            digitalWrite(redLED, LOW);
            digitalWrite(grnLED, LOW);
            digitalWrite(ylwLED, LOW);
            delay(500);
          }
          state = returnWall;
        }
        else if(photoSensors[0]<lightDetectLevel && photoSensors[1]<lightDetectLevel){//light lost
          stateCounter = 0;
          state = lightLost;
        }
        else{ //"love" braitenburg light following
          braitenburg(photoSensors,true,true);
        }
        break;

      case lightLost: //look for light in case of momentary drop-out
        digitalWrite(redLED, HIGH);
        digitalWrite(grnLED, LOW);
        digitalWrite(ylwLED, HIGH);
        //change states according to state diagram
        if(stateCounter>20){//waited too long
          state = returnWall;
        }
        else if(photoSensors[0]>lightDetectLevel || photoSensors[1]>lightDetectLevel){//light found
          state = lightFollow;
        }
        else{ //wait
          stateCounter++;
          delay(50);
        }
        break;
      case returnWall: //return to position where it left wall
        digitalWrite(redLED, HIGH);
        digitalWrite(grnLED, LOW);
        digitalWrite(ylwLED, LOW);
        bool atGoal = goToGoal(returnPos[0],returnPos[1]); //go to position where it left wall
        if(atGoal){//at location
          goToAngle(returnPos[2]-RobotPos[2]); //go to original direction
          state = wallState; //return to state where it left the wall
        }
    }
  }
  }
*/

/*
   turn in an attempt to follow outside corner
   dir: 1=left -1=right
*/
void outsideTurn(int dir) {
  if (dir == 1) {
    stepperRight.setMaxSpeed(-250);
    stepperLeft.setMaxSpeed(-500);

    stepperRight.setSpeed(-250);
    stepperLeft.setSpeed(-500);
  } else {
    stepperRight.setMaxSpeed(-500);
    stepperLeft.setMaxSpeed(-250);

    stepperRight.setSpeed(-500);
    stepperLeft.setSpeed(-250);
  }
  stepperRight.move(-10000);
  stepperLeft.move(-10000);
  int steps = 0;
  while (steps < 50) {

    if (stepperRight.runSpeed()) {
      steps++;
    }
    if (stepperLeft.runSpeed()) {
      steps++;
    }
  }
}

/*
   Use PD control to follow the path that minimizes err
   err is the distance to the right of the desired path
*/
void PDcontrol(float err) {
  float kp = 40;
  float kd = 80;
  float adjust = kp * err + kd * (err - pastErr);
  pastErr = err;
  int centerSpeed = 400;

  //correction for driving backwards
  centerSpeed = -centerSpeed;

  float lSpeed = centerSpeed - adjust;
  float rSpeed = centerSpeed + adjust;


  stepperRight.setMaxSpeed(rSpeed);
  stepperLeft.setMaxSpeed(lSpeed);

  stepperRight.setSpeed(rSpeed);
  stepperLeft.setSpeed(lSpeed);

  stepperRight.move(-10000);
  stepperLeft.move(-10000);


  int steps = 0;
  while (steps < 20) {

    if (stepperRight.runSpeed()) {
      steps++;
    }
    if (stepperLeft.runSpeed()) {
      steps++;
    }
  }
}

void drive(int Speed) {

  stepperRight.setMaxSpeed(Speed);
  stepperLeft.setMaxSpeed(Speed);

  stepperRight.setSpeed(Speed);
  stepperLeft.setSpeed(Speed);

  stepperRight.move(-10000);
  stepperLeft.move(-10000);


  int steps = 0;
  while (steps < 40) {

    if (stepperRight.runSpeed()) {
      steps++;
    }
    if (stepperLeft.runSpeed()) {
      steps++;
    }
  }
}

/*
   Use bang-bang control to follow a path that keeps err less than 1 inch
   err is the distance to the right of the desired path
*/
void bangBang(float err) {

  //Serial.println(ltIRdist);

  if (err < -1) { //if too close, turn away

    digitalWrite(redLED, LOW);
    digitalWrite(grnLED, LOW);
    digitalWrite(ylwLED, HIGH);
    stepperRight.stop();
    stepperLeft.stop();

    stepperRight.setMaxSpeed(300);
    stepperLeft.setMaxSpeed(300);
    pivot(false, inToSteps(1));

    forward(inToSteps(3.5));
    pivot(true, inToSteps(0.5));
    stepperRight.stop();
    stepperLeft.stop();


  } else if (err > 1) { //if too far, turn in
    digitalWrite(redLED, HIGH);
    digitalWrite(grnLED, LOW);
    digitalWrite(ylwLED, LOW);
    stepperRight.stop();
    stepperLeft.stop();

    stepperRight.setMaxSpeed(300);
    stepperLeft.setMaxSpeed(300);
    pivot(true, inToSteps(1));

    forward(inToSteps(2.5));
    pivot(false, inToSteps(0.5));
    stepperRight.stop();
    stepperLeft.stop();

  }
  else { //otherwise move forward
    digitalWrite(redLED, LOW);
    digitalWrite(grnLED, LOW);
    digitalWrite(ylwLED, LOW);

    stepperRight.setMaxSpeed(700);
    stepperLeft.setMaxSpeed(700);
    stepperRight.setSpeed(700);
    stepperLeft.setSpeed(700);

    forward(inToSteps(2.5));

  }
}

/*
   go to goal functionality, utilizes local-global coordinate transform to allow for interruption.
   returns true if at desired goal

*/
bool goToGoal(float x, float y) {
  localize();
  float dtheta = atan2(y - RobotPos[1], x - RobotPos[0]) - RobotPos[2] - PI;

  if (abs(dtheta) > PI / 12) { //look toward goal
    goToAngle(dtheta);
  } else { //move forward
    float distToGo = sqrt(pow(y - RobotPos[1], 2) + pow(x - RobotPos[0], 2)) / (wlRadius * 2 * PI) * 800;
    if (distToGo > 50) {

      stepperRight.setMaxSpeed(-700);
      stepperLeft.setMaxSpeed(-700);

      stepperRight.setSpeed(-700);
      stepperLeft.setSpeed(-700);


      stepperRight.move(-10000);
      stepperLeft.move(-10000);

      int steps = 0;
      while (steps < min(distToGo * 2, 200)) {

        if (stepperRight.runSpeed()) {
          steps++;
        }
        if (stepperLeft.runSpeed()) {
          steps++;
        }
      }
      return false;
    }
    else {
      return true;
    }
  }

}

/*
   Update global coordinates based on stepper moter data
*/
void localize() {
  long lPos = stepperLeft.currentPosition();
  long rPos = stepperRight.currentPosition();
  long dl = lPos - stepperPos[0];
  long dr = rPos - stepperPos[1];

  float dtheta = (dr - dl) / 2.0 / 800 * wlRadius * 2 * PI / wbRadius;
  float ddist = (dr + dl) / 2.0 / 800 * wlRadius * 2 * PI;

  RobotPos[0] = RobotPos[0] + ddist * cos(RobotPos[2] + dtheta / 2);
  RobotPos[1] = RobotPos[1] + ddist * sin(RobotPos[2] + dtheta / 2);
  RobotPos[2] = RobotPos[2] + dtheta;

  stepperPos[0] = stepperLeft.currentPosition();
  stepperPos[1] = stepperRight.currentPosition();
}

/*
   Lab 2 new movement patterns

   random wander
   agressive kid
   shy kid
*/

/*
   random wander
   randomly chooses a direction and distance
*/
void randomWander() {

  digitalWrite(redLED, LOW);
  digitalWrite(grnLED, HIGH);
  digitalWrite(ylwLED, LOW);

  //correction for driving backwards
  if (stepperRight.distanceToGo() >= 0 && stepperLeft.distanceToGo() >= 0) { //if at target distance, turn at rendom angle and choose random distance
    stepperRight.stop();
    stepperLeft.stop();
    float x = random(0, 1000000) / 1000000.0;
    float angle = (x - 0.5) * PI * 2 / 3;
    if (angle > 0) {
      angle = angle + PI / 6;
    }
    else {
      angle = angle - PI / 12;
    }
    stepperRight.setMaxSpeed(300);
    stepperLeft.setMaxSpeed(300);
    delay(150);
    goToAngle(angle);
    stepperRight.stop();
    stepperLeft.stop();
    delay(150);
    x = random(0, 1000000) / 1000000.0;

    //correction for driving backwards
    float dist = -((24 - 2) * x + 2);
    Serial.println(dist);
    stepperRight.move(dist / (wlRadius * 2 * PI) * 800);
    stepperLeft.move(dist / (wlRadius * 2 * PI) * 800);

  } else { //keep moving until target distance
    //correction for driving backwards
    int driveSpeed = -700;

    stepperRight.setMaxSpeed(driveSpeed);
    stepperLeft.setMaxSpeed(driveSpeed);

    stepperRight.setSpeed(driveSpeed);
    stepperLeft.setSpeed(driveSpeed);


    int steps = 0;
    while (steps < 200) {

      if (stepperRight.runSpeed()) {
        steps++;
      }
      if (stepperLeft.runSpeed()) {
        steps++;
      }
    }
  }
}

/*
   agressive kid
   backs up from walls directly ahead,
   moves forward otherwise
*/
void agressiveKid(float frIRdist) {
  digitalWrite(redLED, HIGH);
  digitalWrite(grnLED, LOW);
  digitalWrite(ylwLED, LOW);
  if (frIRdist < 3) { //if too close, stop

    stepperRight.stop();
    stepperLeft.stop();
    delay(500);
    stepperRight.setMaxSpeed(300);
    stepperLeft.setMaxSpeed(300);
    forward(-4 / (wlRadius * 2 * PI) * 800);
    delay(200);


  } else { //otherwise move forward

    stepperRight.setMaxSpeed(700);
    stepperLeft.setMaxSpeed(700);
    stepperRight.setSpeed(700);
    stepperLeft.setSpeed(700);


    int steps = 0;
    while (steps < 20) {

      if (stepperRight.runSpeed()) {
        steps++;
      }
      if (stepperLeft.runSpeed()) {
        steps++;
      }
    }
  }
}


/*
   shy kid v2
   determines which IR sensors are "triggered"
   impliments a logical retreat based on triggered sensors
*/
void shyKid2(float sensorArray[], float triggerDist) {

  float fvectormag = 0;
  float fvectorx = 0;
  float fvectory = 0;
  float fvectorangle = 0;
  stepperRight.setMaxSpeed(400);
  stepperLeft.setMaxSpeed(400);
  //    stepperRight.setSpeed(700);
  //    stepperLeft.setSpeed(700);

  int triggers[4];
  triggers[0] = (sensorArray[1] < triggerDist); //updated for backwards driving
  triggers[1] = (sensorArray[0] < triggerDist);
  triggers[2] = (sensorArray[3] < triggerDist);
  triggers[3] = (sensorArray[2] < triggerDist);

  if (triggers[0] == triggers[1] && triggers[2] == triggers[3] && triggers[1] == triggers[2]) { //if all sides are the same, do nothing
    stepperLeft.stop();
    stepperRight.stop();
  }
  else if (triggers[0] > triggers[1] && triggers[2] == triggers[3]) { //if blocked in front move back
    //reverse(800);
    reverse(400);
    if (backupCounter++ >= 3) { //if blocked multiple times, try to move around obstacle
      goToAngle(PI / 2);
      forward(1600);
      goToAngle(-PI / 2);
      forward(400);
      backupCounter = 0;
    }
    //Serial.println(backupCounter);
  }
  else if (triggers[0] == 0 && triggers[2] == triggers[3]) { //if blocked sides or back and not front, escape forward
    forward(800);
  }
  else if (triggers[0] == triggers[1] && 0 == triggers[3]) { //if blocked on front/back or left but not right, escape right
    goToAngle(PI / 2);
    reverse(800);
  }
  else if (triggers[0] == triggers[1] && triggers[2] < triggers[3]) { //if blocked on right but not left, escape left
    goToAngle(-PI / 2);
    reverse(800);
  }
  else if (triggers[0] && triggers[2]) { //if blocked by corner, move diagonally
    goToAngle(PI / 4);
    reverse(800);
  }
  else if (triggers[0] && triggers[3]) {
    goToAngle(-PI / 4);
    reverse(800);
  }
  else if (triggers[1] && triggers[2]) {
    goToAngle(-PI / 4);
    forward(800);
  }
  else if (triggers[1] && triggers[3]) {
    goToAngle(PI / 4);
    forward(800);
  } else {

    stepperLeft.stop();
    stepperRight.stop();
  }
}

/*
   calibrate "low" light threshold for photosensors
*/
void calibratePhoto() {
  float rAvg = 0;
  float lAvg = 0;
  for (int i = 0; i <= 4 ; i++) {
    float add = readRightPhoto();
    //Serial.println(add);
    rAvg = rAvg + add;
    lAvg = lAvg + readLeftPhoto();
  }
  rAvg = rAvg / 5;
  lAvg = lAvg / 5;
  rAmbiant = rAvg;
  lAmbiant = lAvg;
  //Serial.println(rAmbiant);
}

/*
   read all photoresistor sensors
*/
void readPhotoSensors(float (& sensorArray) [2]) {
  sensorArray[0] = readLeftPhoto();
  sensorArray[1] = readRightPhoto();
}


/*
   use calibration to convert sensor values to a number in [0,1] (nominally)
*/
void normalizePhotoSensors(float (& sensorArray) [2]) {
  sensorArray[0] = pow((sensorArray[0] - lAmbiant) / (lBright - lAmbiant), 1);
  sensorArray[1] = pow((sensorArray[1] - rAmbiant) / (rBright - rAmbiant), 1);
}

/*
   remap photosensor values for backwards driving
*/
void remapPhotoSensors(float (& sensorArray) [2]) {
  float temp = sensorArray[0];
  sensorArray[0] = sensorArray[1];
  sensorArray[1] = temp;
}

/*
   read left photosensor brightness
*/
float readLeftPhoto() {
  int avg = 0;
  for (int i = 0; i <= 24 ; i++) {
    avg = avg + analogRead(ltPhotoPin);
  }
  avg = avg / 25.0;
  //float dist = 41.7*exp(-0.00694*avg) + 1.43;
  return avg;//dist*49/50;
}

/*
   read right photosensor brightness
*/
float readRightPhoto() {
  int avg = 0;
  for (int i = 0; i <= 24 ; i++) {
    avg = avg + analogRead(rtPhotoPin);
  }
  avg = avg / 25.0;
  //float dist = 41.7*exp(-0.00694*avg) + 1.43;
  return avg;//dist*49/50;
}

/*
   Read all IR (and sonar) sensor values
*/
void readIRSensors(float (& sensorArray) [6]) {
  //float sensorArray[6];
  sensorArray[0] = readFrontIR();
  sensorArray[1] = readBackIR();
  sensorArray[2] = readLeftIR();
  sensorArray[3] = readRightIR();
  sensorArray[4] = 25;//readLeftSonar();
  sensorArray[5] = 25;//readRightSonar();
}

/*
   remap IR sensor values for backwards driving
*/
void remapIRSensors(float (& sensorArray) [6]) {
  float temp = sensorArray[0];
  sensorArray[0] = sensorArray[1];
  sensorArray[1] = temp;
  temp = sensorArray[2];
  sensorArray[2] = sensorArray[3];
  sensorArray[3] = temp;
}

// Calibration equations:
// Front IR  Calibration : dist (in) = 24.4*exp(-0.00948*reading) + 1.66
// Back  IR  Calibration : dist (in) = 30.4*exp(-0.01009*reading) + 1.75
// Left  IR  Calibration : dist (in) = 55.4*exp(-0.00758*reading) + 1.48
// Right IR  Calibration : dist (in) = 41.7*exp(-0.00694*reading) + 1.43
// Left  SNR Calibration : dist (in) = 0.00694*reading - 1.292
// Right SNR Calibration : dist (in) = 0.00714*reading - 1.158

/*
   Functions to read sensor data in inches
*/

/*
   Read front IR data in inches
*/
float readFrontIR() {
  int avg = 0;
  for (int i = 0; i <= 9 ; i++) {
    avg = avg + analogRead(ftIRPin);
  }
  avg = avg / 10;
  float dist = 24.4 * exp(-0.00948 * avg) + 1.66;
  return dist * 49 / 50;
}

/*
   Read rear IR data in inches
*/
float readBackIR() {
  int avg = 0;
  for (int i = 0; i <= 9 ; i++) {
    avg = avg + analogRead(bkIRPin);
  }
  avg = avg / 10;
  float dist = 30.4 * exp(-0.01009 * avg) + 1.75;
  return dist * 49 / 50;
}

/*
   Read left IR data in inches
*/
float readLeftIR() {
  int avg = 0;
  for (int i = 0; i <= 9 ; i++) {
    avg = avg + analogRead(ltIRPin);
  }
  avg = avg / 10;
  float dist = 55.4 * exp(-0.00758 * avg) + 1.48;
  return dist * 49 / 50;
}

/*
   Read right IR data in inches
*/
float readRightIR() {
  int avg = 0;
  for (int i = 0; i <= 9 ; i++) {
    avg = avg + analogRead(rtIRPin);
  }
  avg = avg / 10;
  float dist = 41.7 * exp(-0.00694 * avg) + 1.43;
  return dist * 49 / 50;
}

/*
   Read left sonar data in inches
*/
float readLeftSonar() {
  long value = 0;
  int n = 0;
  for (int i = 0; i <= 0 ; i++) {
    n++;
    pinMode(ltSonarPin, OUTPUT);//set the PING pin as an output
    digitalWrite(ltSonarPin, LOW);//set the PING pin low first
    delayMicroseconds(2);//wait 2 us
    digitalWrite(ltSonarPin, HIGH);//trigger sonar by a 2 us HIGH PULSE
    delayMicroseconds(5);//wait 5 us
    digitalWrite(ltSonarPin, LOW);//set pin low first again
    pinMode(ltSonarPin, INPUT);//set pin as input with duration as reception time
    int reading = pulseIn(ltSonarPin, HIGH, 4000);
    if (reading == 0) {
      reading = 4000;
    }
    value = value + reading;
  }
  value = value / (n);
  float dist = 0.00694 * value - 1.292;
  return dist * 49 / 50;
}

/*
   Read right sonar data in inches
*/
float readRightSonar() {
  long value = 0;
  int n = 0;
  for (int i = 0; i <= 0 ; i++) {
    n++;
    pinMode(rtSonarPin, OUTPUT);//set the PING pin as an output
    digitalWrite(rtSonarPin, LOW);//set the PING pin low first
    delayMicroseconds(2);//wait 2 us
    digitalWrite(rtSonarPin, HIGH);//trigger sonar by a 2 us HIGH PULSE
    delayMicroseconds(5);//wait 5 us
    digitalWrite(rtSonarPin, LOW);//set pin low first again
    pinMode(rtSonarPin, INPUT);//set pin as input with duration as reception time
    int reading = pulseIn(rtSonarPin, HIGH, 4000);
    if (reading == 0) {
      reading = 4000;
    }
    value = value + reading;
  }
  value = value / (n);
  float dist = 0.00714 * value - 1.158;
  return dist * 49 / 50;
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
  localize();

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
  localize();

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
  if (dist > 0) {
    stepperLeft.setMaxSpeed(-300);
    stepperRight.setMaxSpeed(300);
    stepperLeft.setSpeed(-300);
    stepperRight.setSpeed(300);
  } else {

    stepperLeft.setMaxSpeed(300);
    stepperRight.setMaxSpeed(-300);
    stepperLeft.setSpeed(300);
    stepperRight.setSpeed(-300);
  }
  spin(dist);
}


/*
   convert distance in inches to number of steps
*/
float inToSteps(float dist) {
  return dist / 2.0 / PI / wlRadius * 800;
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

/*
   returhs the sign of the passed nubmer
*/
int sign(int number) {
  if (number > 0) {
    return 1;
  } else if (number < 0) {
    return -1;
  } else {
    return 0;
  }
}

/************************************
  Final.ino
  Andrew Hubbard, Nithin Saravanapandian; Moravec 2/18/22
  This program introduces bluetooth communication functionality to the robot. For a 4x4 world, it also introduces 
  topological path following, metric path planning, world mapping, and localization. This program is designed to communicate
  with the program GUI_Final.pde for the purpose of sending commands to and recieving information from the robot.

  New methods:
  readBluetooth - read data from bluetooth; run code when command phrases are sent
  topoFollowing - robot follows topological path
  metricPlanning - calculate route to nearest goal space
  metricFollowing - follow calculated route to goal
  mappingStateMachine - state machine implimenting world mapping behavior
  localizeStateMachine - state machine implimenting robot localization behavior
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

const float wlRadius = 1.66; //wheel radius in inches
const float wbRadius = 3.76; //wheelbase radius in inches

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
#define metPP        2
#define mapping      3
#define localizing   4

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
#define metricSwitch      1
#define metricLeft        2
#define metricRight       3
#define metricStraight    4
#define metricStop        5
#define metricErr         6

volatile byte mappingState = 0;
#define mappingStart       0
#define mappingMeasure     1
#define mappingUpdateMap   2
#define mappingPlan        3
#define mappingMove        4
#define mappingStop        5
#define mappingErr         6

volatile byte localizeState = 0;
#define localizeStart       0
#define localizePlan        1
#define localizeMeasure     2
#define localizeUpdateMap   3
#define localizeMove        4
#define localizeStop        5
#define localizeErr         6


//topological path following global variables
String topoDirections = "LRST";
int topoPointer = 0;

//metric path planning global variables
byte Tmap[4][4] = {};
int startx = -1;
int starty = -1;
int goalx = -1;
int goaly = -1;
int currx = 0;
int curry = 0;
int facing = 0;
String metricDirections = "";
int metricPointer = 0;

//mapping global variables
float sensorSums[4];
int readings = 0;

//localization global variables
int featurex[16];
int featurey[16];
int feature[16];
int featureLen = 0;
int tempx = 0;
int tempy = 0;
int tempFacing = 0;

//bluetooth global variables
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

  //read commands from bluetooth
  readBluetooth();
  
  //select behavior function based on state
  switch(state){
    case idle:
      break;
    case topo:
      topoFollowing(sensors);
      break;
    case metPP:
      metricFollowing(sensors);
      break;
    case mapping:
      mappingStateMachine(sensors);
      break;
    case localizing:
      localizeStateMachine(sensors);
      break;
  }
}


/*
 * Read data and commands from bluetooth module
 * command finished at \n character
 * Commands:
 * 0 - waiting for command
 * t - start topological path following
 * m - recieving map
 * s - recieving start position
 * f - recieving goal position
 * g - start metric path planning/go-to-goal
 * n - start mapping
 * l - start localization
 * d - manual drive
 * c - stop all functions
 * Data is usually an integer between -1 and 16, so data is offset by 65 (char 'A') to avoid issues 
 * caused by data sharing the character address as \n or other special characters.
 */
void readBluetooth(){
  char msg;

  if(Serial.available()){
    msg = Serial.read();
    if(command == '0'){
      command = msg;
    }else if(msg == '\n'){
      //topological path following
      if(command == 't'){
        topoDirections = message;
        topoPointer = 0;
        topoState = topoStart;
        state = topo;
      }
      
      //recieving map
      else if(command == 'm'){
        for(int i=0; i<4; i++){
          for(int j=0; j<4; j++){
            Tmap[i][j] = message.charAt(j+4*i)-'A';
          }
        }
        Serial.println("sMap Recieved");
      }
      
      //recieving start position
      else if(command == 's'){
        startx = message.charAt(0)-'0';
        starty = message.charAt(1)-'0';
        currx = startx;
        curry = starty;
        Serial.println("sStart Recieved");
      }
      
      //recieving goal position
      else if(command == 'f'){
        goalx = message.charAt(0)-'0';
        goaly = message.charAt(1)-'0';
        Serial.println("sGoal Recieved");
      }
      
      //start metric path following
      else if(command == 'g'){
        int paramGoalx[1]; paramGoalx[0] = goalx;
        int paramGoaly[1]; paramGoaly[0] = goaly;
        metricPlanning(paramGoalx,paramGoaly,1);
        metricState = metricStart;
        state = metPP;
      }
      
      //start mapping
      else if(command == 'n'){
        mappingState = mappingStart;
        state = mapping;
      }
      
      //start mapping
      else if(command == 'l'){
        localizeState = localizeStart;
        state = localizing;
      }
      
      //manual driving control
      else if(command == 'd'){
        Serial.println("sManual Driving");
        int driveCommand = message.charAt(0)-'A';
        manualDrive(driveCommand);
      }
      
      //emergency stop
      else if(command == 'c'){
        Serial.println("sStopped");
        state = idle;
      }

      //reset bluetooth message
      command = '0';
      message = "";
    }else{
      message = message + msg;
    }
  }
}

/*
 * State machine controlling localization behavior
 * Requires:
 * - accurate map of world
 * - robot placed facing North (future versions may allow localization with other starting configurations)
 * 
 */
void localizeStateMachine(float sensors[]){
  int avoidDist = 0;
  int wallDetectDist = 12;
  if (sensors[0] < avoidDist || sensors[1] < avoidDist || sensors[2] < avoidDist || sensors[3] < avoidDist) {
    shyKid2(sensors, avoidDist); //avoid obstacles
  }
  
  else {
    switch(localizeState){
      case localizeStart:{
        //reset localization variables
        tempx = 0;
        tempy = 0;
        tempFacing = 0;
        featureLen = 0;
        localizeState = localizePlan;
        break;
      }

      case localizePlan:{
        //determine whether the robot is in a new location
        bool newPlace = true;
        for(int i=0;i<featureLen;i++){
          if(tempx == featurex[i] && tempy == featurey[i]){
            newPlace = false;
            break;
          }
        }
        
        if(newPlace){  
          //If in a new location, collect wall data
          localizeState = localizeMeasure;
        }else{
          if(tempx == 0 && tempy == 0 && tempFacing == 0){
            //If returned to original position, stuck in a loop, impossible to localize
            localizeState = localizeErr;
          }else{
            //Otherwise, keep looking for new information
            localizeState = localizeMove;
          }
        }
      }

      case localizeMeasure:{
        //Take 20 readings of IR sensors
        if(readings == 20){
          localizeState = localizeUpdateMap;
        }else{
          for(int i=0;i<4;i++){
            sensorSums[i] = sensorSums[i] + sensors[i];
          }
          readings++;
        }
        break;
      }
        
      case localizeUpdateMap:{ 
        //take average of readings
        float sensorArray[4];
        for(int i=0;i<4;i++){
          sensorArray[i] = sensorSums[i]/readings;
          sensorSums[i] = 0;
        }
        readings = 0;

        //determine which walls exist
        bool wallDetected[4];
        for(int i=0;i<4;i++){
          wallDetected[i] = (sensorArray[i] < wallDetectDist);
        }
        //first index - robot facing
        //second index - 0-N/1-E/2-S/3-W
        int indexes[4][4] = {{0,3,1,2},{2,0,3,1},{1,2,0,3},{3,1,2,0}};

        //get 4-bit code for wall structure
        int walls = 0;
        for(int i=0;i<4;i++){
          if(wallDetected[indexes[tempFacing][i]]){
            walls = walls + (1 << i);
          }
        }
        
        //add wall layout to feature list
        featurex[featureLen] = tempx;
        featurey[featureLen] = tempy;
        feature[featureLen] = walls;
        featureLen++;
        
        //check all possible start locations
        int minx = 0;
        int maxx = 0;
        int miny = 0;
        int maxy = 0;
        for(int i=0;i<featureLen;i++){
          if(featurex[i]>maxx){
            maxx = featurex[i];
          }
          if(featurex[i]<minx){
            minx = featurex[i];
          }
          if(featurey[i]>maxy){
            maxy = featurey[i];
          }
          if(featurey[i]<miny){
            miny = featurey[i];
          }
        }

        //loop through possible start locations
        int numValid = 0;
        int possStartx = 0;
        int possStarty = 0;
        for(int i = -minx;i<4-maxx;i++){
          for(int j = -miny;j<4-maxy;j++){
            bool valid = true;
            //loop through observed features. If any don't match, rule out start position
            for(int k=0;k<featureLen;k++){
              if(Tmap[i+featurex[k]][j+featurey[k]] != feature[k]){
                valid = false;
              }
            }
            //if start is valid, add to counter and save start position
            if(valid){
              numValid++;
              possStartx = i;
              possStarty = j;
            }
          }
        }

        if(numValid==0){
          //Localization is impossible, measurements or reference map may be wrong.
          localizeState = localizeErr;
        }else if (numValid ==1){
          //Start location uniquely identified. Send information to GUI and halt.
          Serial.write('t'); Serial.write(possStartx+'A'); Serial.write(possStarty+'A'); Serial.write('\n');
          startx = possStartx;
          starty = possStarty;
          currx = tempx+possStartx;
          curry = tempy+possStarty;
          Serial.write('r'); Serial.write(currx+'A'); Serial.write(curry+'A'); Serial.write(facing+'A'); Serial.write('\n');
          
          localizeState = localizeStop;
        }else if (numValid >1){
          //Start location not uniquely identified. Move to new location.
          localizeState = localizeMove; 
        }else{localizeState = localizeErr;}
        break;
      }
        
      case localizeMove:{
        //Move robot somewhere else in the world
        //Robot moves one space at a time, using a right-wall following algorithm
        //Keeps track of robot position and rotation, compared to start location.
        
        bool wallDetected[4];
        for(int i=0;i<4;i++){
          wallDetected[i] = (sensors[i] < wallDetectDist);
        }
        if(!wallDetected[3]){
          goToAngle(-PI/2);
          tempFacing = (tempFacing + 1) % 4;
        }else if(!wallDetected[0]){
          //no turn
        }else if(!wallDetected[2]){
          goToAngle(PI/2);
          tempFacing = (tempFacing + 3) % 4;
        }else if(!wallDetected[1]){
          goToAngle(PI);
          tempFacing = (tempFacing + 2) % 4;
        }else{
          //error state?
          break;
        }
        //go forward
        reverse(inToSteps(18));
        if(tempFacing == 0){tempy = tempy - 1;}
        if(tempFacing == 1){tempx = tempx + 1;}
        if(tempFacing == 2){tempy = tempy + 1;}
        if(tempFacing == 3){tempx = tempx - 1;}

        localizeState = localizePlan;
        break;
      }
        
      case localizeStop:{
        //Localization finished; halt
        Serial.println("sDone Localizing");
        state = idle;
        break;
      }
        
      case localizeErr:{
        //Localization impossible; halt
        Serial.println("sError");
        state = idle;
        break;
      }
        
    }
  }
}

/*
 * State machine controlling mapping behavior
 * Reauires:
 * - accurate start position
 * - robot rotation is consistent with GUI representation
 */
void mappingStateMachine(float sensors[]){
  int avoidDist = 0;
  int wallDetectDist = 12;
  if (sensors[0] < avoidDist || sensors[1] < avoidDist || sensors[2] < avoidDist || sensors[3] < avoidDist) {
    shyKid2(sensors, avoidDist); //avoid obstacles
  }
  else {
    switch(mappingState){
      case mappingStart:{
        //reset mapping variables
        mappingState = mappingMeasure;
        break;
      }

      case mappingMeasure:{
        //take 20 measurements from IR sensors
        if(readings == 20){
          mappingState = mappingUpdateMap;
        }else{
          for(int i=0;i<4;i++){
            sensorSums[i] = sensorSums[i] + sensors[i];
          }
          readings++;
        }
        break;
      }
        
      case mappingUpdateMap:{ 
        //Average sensor measurements
        float sensorArray[4];
        for(int i=0;i<4;i++){
          sensorArray[i] = sensorSums[i]/readings;
          sensorSums[i] = 0;
        }
        readings = 0;

        //Determine which walls are present
        bool wallDetected[4];
        for(int i=0;i<4;i++){
          wallDetected[i] = (sensorArray[i] < wallDetectDist);
        }
        //first index - robot facing
        //second index - 0-N/1-E/2-S/3-W
        int indexes[4][4] = {{0,3,1,2},{2,0,3,1},{1,2,0,3},{3,1,2,0}};

        //calculate 4-bit wall representation
        int walls = 0;
        for(int i=0;i<4;i++){
          if(wallDetected[indexes[facing][i]]){
            walls = walls + (1 << i);
          }
        }
        
        //write wall representation to map at current position
        Tmap[currx][curry] = walls;

        //send current map to GUI
        Serial.write('m');
        for(int i=0;i<4;i++){
          for(int j=0;j<4;j++){
            Serial.write(Tmap[i][j]+'A');
          }
        }
        Serial.write('\n');
        mappingState = mappingPlan;
        break;
      }
        
      case mappingPlan:{
        //identify all unmapped cells
        int unmappedx[16];
        int unmappedy[16];
        int unmappedLen = 0;
        for(int i=0;i<4;i++){
          for(int j=0;j<4;j++){
            if((Tmap[i][j] & 16)>0){
              unmappedx[unmappedLen] = i;
              unmappedy[unmappedLen] = j;
              unmappedLen = unmappedLen + 1;
            }
          }
        }
        //use metric path planning to find route to unmapped space
        bool moreToMap = metricPlanning(unmappedx,unmappedy,unmappedLen);
        metricState = metricStart;
        if(moreToMap){
          //if path to unmapped space found, move
          mappingState = mappingMove;
        }else{
          //otherwise, halt
          mappingState = mappingStop;
        }
        break;
      }
        
      case mappingMove:{
        if(metricState == metricStop){
          //if at target location, pause and measure
          delay(1000);
          mappingState = mappingMeasure;
        }else{
          //follow metric path planning
          metricFollowing(sensors);
        }
        break;
      }
        
      case mappingStop:{
        //all reachable spaces mapped; halt
        Serial.println("sDone Mapping");
        state = idle;
        break;
      }
        
      case mappingErr:{
        //some error occured; halt
        Serial.println("sError");
        state = idle;
        break;
      }
        
    }
  }
}

/*
 * Follow path determined by metric path planning
 * Recieves commands to move one space forward, turn 90 degrees left, and turn 90 degrees left
 */
void metricFollowing(float sensors[]) {
  int avoidDist = 0;
  int wallDetectDist = 12;
  int frontDetectDist = 4;

  if (sensors[0] < avoidDist || sensors[1] < avoidDist || sensors[2] < avoidDist || sensors[3] < avoidDist) {
    shyKid2(sensors, avoidDist); //avoid obstacles
  }
  else {
    switch(metricState){
      case metricStart:
        //reset path following variables
        Serial.println("sMoving To Goal");
        metricPointer = 0;
        metricState = metricSwitch;
        break;
        
      case metricSwitch:
        //read path directions and choose next state
        Serial.write('r'); Serial.write(currx+'A'); Serial.write(curry+'A'); Serial.write(facing+'A'); Serial.write('\n');
        delay(50);
        if(metricPointer >= metricDirections.length()){
          metricState = metricStop;
        }else{
          char command = metricDirections.charAt(metricPointer);
          if(command == 'L'){
            metricState = metricLeft;
          }else if(command == 'R'){
            metricState = metricRight;
          }else if(command == 'S'){
            metricState = metricStraight;
          }
          metricPointer = metricPointer + 1;
        }
        break;
        
      case metricLeft:
        //Turn 90 degrees left, update rotation
        goToAngle(PI/2);
        facing = (facing + 3) % 4;
        metricState = metricSwitch;
        break;
        
      case metricRight:
        //Turn 90 degrees right, update rotation
        goToAngle(-PI/2);
        facing = (facing + 1) % 4;
        //Serial.println("r" + char(currx+'A') + char(curry+'A') + char(facing+'A'));
        metricState = metricSwitch;
        break;
        
      case metricStraight:
        //Drive one space forward, update position
        reverse(inToSteps(18));
        if(facing == 0){curry = curry - 1;}
        if(facing == 1){currx = currx + 1;}
        if(facing == 2){curry = curry + 1;}
        if(facing == 3){currx = currx - 1;}
        //Serial.println("r" + char(currx+'A') + char(curry+'A') + char(facing+'A'));
        metricState = metricSwitch;
        break;
        
      case metricStop:
        //Reached end of directions, halt
        Serial.println("p");
        Serial.println("sMoved");
        state = idle;
        break;
        
      case metricErr:
        //Error occurred, halt
        Serial.println("p");
        Serial.println("sError");
        state = idle;
        break;
    }
  }
}

/*
 * Use grassfire expansion path planning to plan route to a goal space
 * Parameters:
 * - goalx: x-coordinates of goal locations
 * - goaly: y-coordinates of goal locations
 * - goalLen: number of goal locations
 * 
 * Returns a boolean if a path is found from the current location to a goal location
 * Writes to GUI regarding found path
 * Writes metric directions for path following
 */
bool metricPlanning(int goalx[], int goaly[], int goalLen){

  Serial.println("sStarting Path Planning");
  
  // defining pathmap with steps from goal to start
  // Values:
  // -1: undetermined
  // 0:  goal location
  // 1+: number of steps to nearest goal location

  //new stepmap
  int stepmap[4][4];
  for(int i=0;i<4;i++){
    for(int j=0;j<4;j++){
      stepmap[i][j]=-1;
    }
  }

  //place goal locations
  for(int i=0;i<goalLen;i++){
    stepmap[goalx[i]][goaly[i]] = 0;
  }


  int keeplooping = 1; //0: no more changes, stop/ 1: changes, keep going/ 2: found start, stop
  int value = 0; //cell value to look for on current iteration

  //Fill stepmap with values as defined above using grassfire expansion
  while(keeplooping == 1){
    keeplooping = 0;
    int updates = 0;
    //loop through all squares
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        //if the current square has the current value:
        if (stepmap[i][j] == value) {
          //if the current square is the robot's location, stop grassfire expansion
          if(i==currx && j==curry){
            keeplooping = 2;
          }

          //if North wall is absent, and South wall of northern space is absent, and northern space is undetermined, update northern space to next value
          if((Tmap[i][j] & 1)==0){
            if((Tmap[i][j-1] & 4)==0){
              if(stepmap[i][j-1] ==-1){
                stepmap[i][j-1] = value+1;
                updates++;
              }
            }
          }
          //if East wall is absent, and West wall of eastern space is absent, and eastern space is undetermined, update eastern space to next value
          if((Tmap[i][j] & 2)==0){
            if((Tmap[i+1][j] & 8) ==0){
              if(stepmap[i+1][j] ==-1){
                stepmap[i+1][j] = value+1;
                updates++;
              }
            }
          }
          //if South wall is absent, and North wall of southern space is absent, and southern space is undetermined, update southern space to next value
          if((Tmap[i][j] & 4)==0){
            if((Tmap[i][j+1] & 1) ==0){
              if(stepmap[i][j+1] ==-1){
                stepmap[i][j+1] = value+1;
                updates++;
              }
            }
          }
          //if West wall is absent, and East wall of western space is absent, and western space is undetermined, update western space to next value
          if((Tmap[i][j] & 8)==0){
            if((Tmap[i-1][j] & 2) ==0){
              if(stepmap[i-1][j] ==-1){
                stepmap[i-1][j] = value+1;
                updates++;
              }
            }
          }
  
        }
      }
    }
    //increment value
    value = value + 1;
    //if robot location not reached and some cells were updated, keep looping
    if(updates>0 && keeplooping==0){
      keeplooping = 1;
    }
  }
 
  //Determine number of spaces from robot to goal
  int extent = stepmap[currx][curry];
  if(extent<1){
    //If no path exists or already at goal, halt
    metricState = metricErr;
    return false;
  }

  //use stepmap to trace path from robot to goal
  int x = currx;
  int y = curry;
  int pathx[extent+1];
  int pathy[extent+1];
  int dirs[extent+1];
  dirs[0] = facing;
  for(int i = 0; i<=extent; i++){
    //value decreases from robot square to zero
    int value = extent - i;
    //record cursor position
    pathx[i] = x;
    pathy[i] = y;

    //check directions for: 
    // - no wall blocking
    // - no wall in neighbor's cell blocking
    // - neighbor's cell is 1 space closer to goal
    //if found:
    // - move cursor in direction
    // - record path direction
    // - stop checking directions
    bool keeplooking = true;
    //check North
    if((Tmap[x][y] & 1)==0 && keeplooking){
      if((Tmap[x][y-1] & 4) ==0){
        if(stepmap[x][y-1] == value-1){
          x = x;
          y = y - 1;
          dirs[i+1]=0;
          keeplooking = false;
        }
      }
    }
    //check East
    if((Tmap[x][y] & 2)==0 && keeplooking){
      if((Tmap[x+1][y] & 8) ==0){
        if(stepmap[x+1][y] == value-1){
          x = x + 1;
          y = y;
          dirs[i+1]=1;
          keeplooking = false;
        }
      }
    }
    //check South
    if((Tmap[x][y] & 4)==0 && keeplooking){
      if((Tmap[x][y+1] & 1) ==0){
        if(stepmap[x][y+1] == value-1){
          x = x;
          y = y + 1;
          dirs[i+1]=2;
          keeplooking = false;
        }
      }
    }
    //check West
    if((Tmap[x][y] & 8)==0 && keeplooking){
      if((Tmap[x-1][y] & 2) ==0){
        if(stepmap[x-1][y] == value-1){
          x = x - 1;
          y = y;
          dirs[i+1]=3;
          keeplooking = false;
        }
      }
    }
  }

  //Decode path directions into robot instructions
  metricDirections = "";
  metricPointer = 0;
  //loop though directions
  for(int i=0; i<extent; i++){
    int dir1 = dirs[i];
    int dir2 = dirs[i+1];
    //calculate difference in direction between steps
    int changeDir = (4 + dir2 - dir1)%4;
    //depending on direction difference, turn left/right/about/hold direction and drive forward
    if(changeDir==0){
      //no change
    }else if(changeDir==1){
      metricDirections = metricDirections + "R";
    }else if(changeDir==2){
      metricDirections = metricDirections + "LL";
    }else if(changeDir==3){
      metricDirections = metricDirections + "L";
    }
    metricDirections = metricDirections + "S";
  }

  //send path coordinates to GUI
  String message = "p";
  for(int i=0; i<=extent; i++){
    message = message + char(pathx[i] + 'A');
    message = message + char(pathy[i] + 'A');
  }
  Serial.println(message);
  return true;
}

/*
 * Topological path following behavior
 * Directions:
 * S - Drive straight until a wall is in front, then turn accourding to next command 
 * L - Drive until left wall missing, then turn
 * R - Drive until right wall missing, then turn
 * T - terminate
 */
void topoFollowing(float sensors[]) {
  int avoidDist = 0;
  int wallDetectDist = 12;
  int frontDetectDist = 4;

  if (sensors[0] < avoidDist || sensors[1] < avoidDist || sensors[2] < avoidDist || sensors[3] < avoidDist) {
    shyKid2(sensors, avoidDist); //avoid obstacles
  }
  else {
    //Identify current command - determines next state
    char current = '0';
    if (topoPointer < topoDirections.length()) {
      current = topoDirections[topoPointer];
    }
    switch (topoState) {
      case topoStart:  //reset variables to initial state, choose next state
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
        if (sensors[0] < frontDetectDist) {
          // front wall detected, cannot continue following left wall
          topoState = topoErr;
        }
        else if (sensors[2] > wallDetectDist) {
          //turn left
          angleAtTurn = RobotPos[2];
          topoState = topoTurnL;
        }
        else { //keep following left wall

          //float err = sensors[2]-5;
          //PDcontrol(err);
          reverse(inToSteps(18));
        }
        break;


      case topoFollowR: //Follow right wall

        digitalWrite(redLED, LOW);
        digitalWrite(grnLED, HIGH);
        digitalWrite(ylwLED, LOW);
        if (sensors[0] < frontDetectDist) {
          // front wall detected, cannot continue following right wall
          topoState = topoErr;
        }
        else if (sensors[3] > wallDetectDist) {
          //right wall lost, turn right
          angleAtTurn = RobotPos[2]; 
          topoState = topoTurnR;
        }
        else { //keep following right wall
          //float err = 5-sensors[3];
          //PDcontrol(err);
          reverse(inToSteps(18));
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
          if (current == 'S') {//goto next state
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
          //PD control for driving removed
//          float err = 0;
//          if (sensors[2] > wallDetectDist && sensors[3] > wallDetectDist) {
//            err = 0;
//          } else if (sensors[2] < wallDetectDist && sensors[3] > wallDetectDist) {
//            err = sensors[2] - 5;
//          } else if (sensors[2] > wallDetectDist && sensors[3] < wallDetectDist) {
//            err = 5 - sensors[3];
//          } else {
//            err = (sensors[2] - sensors[3]) / 2;
//          }
          //PDcontrol(err);
          reverse(inToSteps(18));
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
          //reverse(inToSteps(11));
          goToAngle(PI / 2);
          //reverse(inToSteps(9));
          if (current == 'S') {//goto next state
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
          //reverse(inToSteps(11));
          goToAngle(-PI / 2);
          //reverse(inToSteps(9));
          if (current == 'S') {//goto next state
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


      case topoStop: //Reached termination direction, halt
        digitalWrite(redLED, LOW);
        digitalWrite(grnLED, LOW);
        digitalWrite(ylwLED, HIGH);
        //lights and done with dask
        state = idle;
        break;


      case topoErr: //Invalid direction, halt
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
 * Manual drive from GUI
 * Parameter command recieved via bluetooth
 * 3-bit number
 * first bit determines speed
 * - 0: small adjustments
 * - 1: move entire square, 90 degree turns
 * last 2 bits determine direction
 * - 0: drive forward
 * - 1: drive reverse
 * - 2: turn left
 * - 3: turn right
 */
void manualDrive(int command){
  int dir = command & 3;
  if((command & 4)>0){
    if(dir == 0){
      reverse(inToSteps(18));
    }else if(dir == 1){
      forward(inToSteps(18));
    }else if(dir == 2){
      goToAngle(-PI/2);
    }else if(dir == 3){
      goToAngle(PI/2);
    }
  }else{
    if(dir == 0){
      reverse(inToSteps(0.5));
    }else if(dir == 1){
      forward(inToSteps(0.5));
    }else if(dir == 2){
      goToAngle(-PI/30);
    }else if(dir == 3){
      goToAngle(PI/30);
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

/*
   Update global coordinates based on stepper moter data
   Used for keeping track of robot position, not related to localization behavior
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

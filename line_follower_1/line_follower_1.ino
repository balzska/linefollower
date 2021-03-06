#include <Servo.h>

#include <SoftwareSerial.h>
#include <ArduinoJson.h>

//LEGS
#define A_MotorEnable 2
#define A_MotorDir1 3
#define A_MotorDir2 4

#define B_MotorEnable 7
#define B_MotorDir1 6
#define B_MotorDir2 5

#define sonarEchoPin 9
#define sonarTrigPin 8

#define Encoder_A 19
#define Encoder_B 18

#define ServoPin 13

//SERVO
Servo servo;
int servoAngle = 0;

//TIMER
int timerCount = 0;

//IR
const int DEFAULT_SENSOR_DISTANCE = 900;
int lineError = 0;
int previousLineError = 0;
int sensorValues[5] = {0, 0, 0, 0, 0};
int changeLineDelayCounter = 0;
const int LINE_ERROR_WINDOW_SIZE = 20;
const int LINE_ERROR_WINDOW_SAFE = 3;
int lineErrorWindow [LINE_ERROR_WINDOW_SIZE];

//MOTOR
int A_RPM_Correction = 0;
int B_RPM_Correction = 0;
const int RPM = 80;
const int TURN_CONSTANT = 10;
bool isMoving = false;

//SONAR
const int sonarMaximumRange = 200;
const int sonarMinimumRange = 0;
const int sonarDistanceRestriction = 20;

long sonarDuration, sonarDistance;

int sonarWindow[5];

//ENCODER
//A-right, B-left
//B: 81 ~ half turn; 162 ~ full turn
//A: 82 ~ half turn; 164 ~ full turn
volatile int encoderACounter, encoderBCounter = 0;
volatile int encoderACorrectionCounter, encoderBCorrectionCounter = 0;
volatile bool isReadingEncoder = false;
bool isEncoderCorrection = false;

//SPECIAL
bool isSpecialTaskRunning = false;
bool isTurning = false;
bool isMovingForward = false;
bool isFindingLine = false;
bool isScriptRunning = false;
bool isObstacleCourse = false;
int scriptTaskCounter = 0;

//LANECHANGE
bool isLaneChangeLeft = false;
bool isLaneChangeRight = false;
int laneChangeTimeCounter = 0;

//OBSTACLE COURSE
int OCUnitSize = 10;
bool OCIsPreferedSideRight = false;
int OCDisplacementInUnit = 0;
bool OCIsWorking = false;

//BT
SoftwareSerial mySerial(10, 11); //RX, TX

//-------------------------------------------//

void setup() {
  Serial.begin(38400);
  Serial.println("Hello server!!");

  mySerial.begin(9600);
  mySerial.println("Hello client!!");

  //sonar
  pinMode(sonarTrigPin, OUTPUT);
  pinMode(sonarEchoPin, INPUT);
  SonarInit();

  //Encoder
  pinMode(Encoder_A, INPUT_PULLUP);
  pinMode(Encoder_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(Encoder_A), EncoderAHit, HIGH);
  attachInterrupt(digitalPinToInterrupt(Encoder_B), EncoderBHit, HIGH);

  //calibrate
  CalibrateSensors();

  //line error window
  LineErrorWindowSetup();  

  //timer
  Timer1Init();

  //motor
  isMoving = true;

  //servo
  servo.attach(ServoPin);
  servo.write(90);  
}
long directionConstant = 0;

String dir = "";
int mapSensorValues[7];
void loop() {
  //delayMicroseconds(1000);
  //IRRead();
  //Serial.println(GetSensorError());
  //Serial.println(SonarRead());
  //MoveRobot();
  //MoveStraightWithSonar();
  dir = BTComm();
  if(directionConstant > 0) {
    B_RPM_Correction = -15;
    A_RPM_Correction += 15; 
  } else if (directionConstant < 0) {
    A_RPM_Correction = -15;
    B_RPM_Correction += 15;
  } else {
    A_RPM_Correction = 0;
    B_RPM_Correction = 0;
  }
  if(dir == "bwd") {
    MoveForward();
  } else if(dir == "fwd") {
    MoveBackward();
  } else if (dir == "scn") {
    servo.write(45);
    delay(1000);
    mapSensorValues[0] = int(SimpleSonarRead());
    servo.write(60);
    delay(1000);
    mapSensorValues[1] = int(SimpleSonarRead());
    servo.write(75);
    delay(1000);
    mapSensorValues[2] = int(SimpleSonarRead());
    servo.write(90);
    delay(1000);
    mapSensorValues[3] = int(SimpleSonarRead());
    servo.write(105);
    delay(1000);
    mapSensorValues[4] = int(SimpleSonarRead());
    servo.write(120);
    delay(1000);
    mapSensorValues[5] = int(SimpleSonarRead());
    servo.write(135);
    delay(1000);
    mapSensorValues[6] = int(SimpleSonarRead());
    servo.write(90);
    delay(1000);
    mySerial.println(String(mapSensorValues[0]) + " " + String(mapSensorValues[1]) + " " + String(mapSensorValues[2]) + " " + String(mapSensorValues[3]) + " " + String(mapSensorValues[4]) + " " + String(mapSensorValues[5]) + " " + String(mapSensorValues[6]));
  } else {
    MotorStop();
  }
}

//-------------------------------------------//

//FUNCTIONS

String inputString;

String BTComm() {
  const char* move = "";
  long turn = 0;
  if (mySerial.available()) {
    /*Serial.write(mySerial.read());
    while(mySerial.available()) {
      inputString = mySerial.readString();
      
    }
    if (inputString.length() >0) {
      Serial.println(inputString);
      inputString = "";
    }*/

    
    DynamicJsonBuffer jsonBuffer(1024);
    JsonObject& root = jsonBuffer.parseObject(mySerial);
    move = root["move"];
    turn = root["turn"];
  }

  directionConstant = turn;

  return String(move);
}

void Timer1Init(){
  noInterrupts();

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  //maszkolas
  // |= vagy
  // WGM12 bit 1re allitasa
  //CS12 es CS10 bitek egyre allitasa
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12) | (1 << CS10);

  //masodpercenkenti tick beallitasa
  //
  //16000000 / 1024 = 15625 (HZ)
  //periodusido = 1/15625 = 0,064ms (ennyi ido alatt inkrementalodik a szamlalo alapbol)
  //ha egy masodpercet akarunk (1000ms): 1000/0,064 = 15625
  //0tol szamol, ezert kivonunk belole egyet: 15624
  OCR1A = 780;
  // OCR1A = 1560;

  //vege jelzo interrupt bekapcsolasa
  //ha elerte a szamlalo a megadott erteket, az OCIE1A flag bebillen es megtortenik a magszakitas
  TIMSK1 |= (1 << OCIE1A);

  //megszakitasok ujra engedelyezese
  interrupts();
}

//ISR(TIMER1_COMPA_vect){
//  //ISR beepitett fuggveny
//  //akkor fut le, ha megtortenik a timer megszakitas
//
//  //BTComm();
//}

// ISR(TIMER1_COMPA_vect){
//   //ISR beepitett fuggveny
//   //akkor fut le, ha megtortenik a timer megszakitas

//   Serial.println("ISR START");

//   isLaneChangeLeft = IsChangeLaneLeft();
//   isLaneChangeRight = IsChangeLaneRight();

//   if(!isScriptRunning && !isObstacleCourse) {
//     /*float sonarValue = SonarRead();
//     if(sonarValue < sonarDistanceRestriction && sonarValue >= sonarMinimumRange) {
//       MotorStop();
//       isMoving = false;
//     } else if(sonarValue > sonarDistanceRestriction && sonarValue <= sonarMaximumRange) {
//       //isMoving = true;
//     }*/

//     if (isLaneChangeLeft)
//     {
//       while (laneChangeTimeCounter <= 6)
//       {
//       TurnLeft();
//       laneChangeTimeCounter++;
//       }
//     }

//     if (isLaneChangeRight)
//     {
//       while (laneChangeTimeCounter <= 6)
//       {
//       TurnRight();
//       laneChangeTimeCounter++;
//       }
//     }

//     if(isMoving && !isFindingLine  && !isObstacleCourse) {
//       if(!isSpecialTaskRunning) {
//         IRRead();
//         int sensorError = GetSensorError();
//       }
//       MoveRobot();
//     }

//     if(isMoving && isFindingLine) {
//       IRRead();
//       int sensorError = GetSensorError();
//       MoveRobotAndFindLine();
//     }
//   }

//   if(isScriptRunning) {
//     isMoving = false;
//     DoSlalom();
//   }

//   if(isObstacleCourse) {
//     if(!OCIsWorking)
//       DoObstacleCourse();
//    // Serial.println("daaasasd");
//   }

//   if(isEncoderCorrection) {
//     timerCount++;
//     if(timerCount == 100){
//       correctMotorError(encoderACorrectionCounter, encoderBCorrectionCounter);
//       ResetEncoderCorrection();
//       Serial.println(A_RPM_Correction);
//       Serial.println(B_RPM_Correction);

//       timerCount = 0;
//     }
//   }
// }

void DoObstacleCourse() {
  MotorStop();
  Serial.println("OBSTACLE");
  OCIsWorking = true;
  IRRead();
  int sensorError = GetSensorError();
  if(sensorError != 5) {
    
    //reached the edge of the map, try something else
    MotorStop();

    if (sensorError == -4)
    {
      MoveBackwardWithEncoder2(10);
      OCDisplacementInUnit--;
      TurnRightWithEncoder2(50);
     
    }

    if (sensorError == 4)
    {
      MoveBackwardWithEncoder2(10);
      OCDisplacementInUnit--;
      TurnLeftWithEncoder2(40);   
    }

    else
    {
    }

    OCIsWorking = false;
    return;
  }
  Serial.println(sensorError);

  long sonarValue = SimpleSonarRead();
  Serial.println(sonarValue);
  if(sonarValue < sonarDistanceRestriction && sonarValue >= sonarMinimumRange) {
    //turn prefered direction
    //if the path is clear 
    //  move forward one unit
    //  log movement
    //  turn opposite prefered direction
    //  return
    //else 
    //  turn opposite prefered direction
    //  change prefered direction
    //  return
    OCTurnPreferedDirection();
    sonarValue = SimpleSonarRead();

    if(sonarValue > sonarDistanceRestriction) {
      OCMoveForwardOneUnit();
      OCDisplacementInUnit++;
      OCTurnOppositePreferedDirection();
      OCIsWorking = false;
      return;
    } else {
      OCTurnOppositePreferedDirection();
      OCIsPreferedSideRight = !OCIsPreferedSideRight;
      OCIsWorking = false;
      return;
    }
    
  } else if(sonarValue > sonarDistanceRestriction) {
    //move forward one unit
    //if movement log 0
    //  return
    //else
    //  turn opposite prefered direction
    //  if path is clear
    //    move forward one unit
    //    remove movement log
    //    turn prefered direction
    //    return
    //  else
    //    turn prefered direction
    //    return
    OCMoveForwardOneUnit();

    OCMoveForwardOneUnit();

    OCMoveForwardOneUnit();

    OCMoveForwardOneUnit();
    if(OCDisplacementInUnit == 0) {
      OCIsWorking = false;
      return;
    } else {
      OCTurnOppositePreferedDirection();
      sonarValue = SimpleSonarRead();
      if(sonarValue > sonarDistanceRestriction) {
        OCMoveForwardOneUnit();
        OCDisplacementInUnit--;
        OCTurnPreferedDirection();
        OCIsWorking = false;
        return;
      } else {
        OCTurnPreferedDirection();
        OCIsWorking = false;
        return;
      }
    }
  }
  OCIsWorking = false;
}

void OCTurnPreferedDirection() {
  OCIsWorking = true;
  if(OCIsPreferedSideRight)
    TurnRightWithEncoder2(50);
  else
    TurnLeftWithEncoder2(40);
  OCIsWorking = false;
}

void OCTurnOppositePreferedDirection() {
  OCIsWorking = true;
  if(!OCIsPreferedSideRight)
    TurnRightWithEncoder2(50);
  else
    TurnLeftWithEncoder2(40);
  OCIsWorking = false;
}

void OCTurnLeft(){
  OCIsWorking = true;  
  TurnLeftWithEncoder2(40);
  OCIsWorking = false;
}

void OCTurnRight(){
  OCIsWorking = true;
  TurnRightWithEncoder2(50);
  OCIsWorking = false;
}

void OCMoveForwardOneUnit() {
  OCIsWorking = true;
  MoveForwardWithEncoder2(OCUnitSize);
  OCIsWorking = false;
}

void DoSlalom() {
  switch (scriptTaskCounter) {
    case 0:
      MoveForwardWithEncoder(30);
      break;
    case 1:
      TurnRightWithEncoder(25);
      break;
    case 2:
      MoveForwardWithEncoder(30);
      break;
    case 3:
      TurnLeftWithEncoder(25);
      break;
    case 4:
      MoveForwardWithEncoder(30);
      break;
    case 5:
      TurnLeftWithEncoder(25);
      break;
    case 6:
      MoveForwardWithEncoder(30);
      break;
    case 7:
      TurnRightWithEncoder(25);
      break;
    case 8:
      MoveForwardWithEncoder(30);
      break;
    case 9:
      TurnRightWithEncoder(25);
      break;
    case 10:
      MoveForwardWithEncoder(30);
      break;
    case 11:
      TurnLeftWithEncoder(25);
      break;
    case 12:
      isScriptRunning = false;
      isMoving = true;
  }
}

void CalibrateSensors(){
  //TODO: implement method
}

void EncoderAHit(){
  Serial.println("HIT");
  if(isReadingEncoder)
  {
  noInterrupts();
    encoderACounter++;
    interrupts();
  }
  if(isEncoderCorrection)
    encoderACorrectionCounter++;
}

void EncoderBHit(){
  Serial.println("HITB");
  if(isReadingEncoder)
  {
    noInterrupts();
      encoderBCounter++;
      interrupts();
    }
  if(isEncoderCorrection)
    encoderBCorrectionCounter++;
}

void ResetEncoderCorrection() {
  encoderACorrectionCounter = 0;
  encoderBCorrectionCounter = 0;
}

void ResetEncoder() {
  encoderACounter = 0;
  encoderBCounter = 0;
}

void EnableEncoderCorrection() {
  //ResetEncoderCorrection();
  isReadingEncoder = true;
  isEncoderCorrection = true;
}

void DisableEncoderCorrection() {
  ResetEncoderCorrection();
  isEncoderCorrection = false;
}

void ActOnEncoderCounters(){
  Serial.println(encoderACounter);
  Serial.println(encoderBCounter);

  encoderACounter = 0;
  encoderBCounter = 0;
}

void MoveRobot(){
  //Serial.println(lineError);
  LineErrorWindowPush(lineError);
  //Serial.println(LineErrorWindowProcess());
  lineError = LineErrorWindowProcess();
  

    laneChangeTimeCounter = 0;

    Serial.println(lineError);
     
    if(lineError == 9) 
    {
        isObstacleCourse = true;
        Serial.println("anyadat");
    } 
    
    else if(lineError == 0)
  {
      MoveForward();
  } 
  else if (lineError == 5) 
  {
    MoveForward();
  }
  else if(lineError < 0 && lineError >= -4)
  {
    MotorStop();
    TurnLeft();
  } 
  else if (lineError > 0 && lineError <= 4)
  {
    MotorStop();
    TurnRight();
  } 
  else if(lineError == 6) 
  {
    TurnRightWithEncoder(30);
  } 
  else if(lineError == -6) 
  {
    TurnLeftWithEncoder(30);
  } 
  else if(lineError == 7) 
  {
    isScriptRunning = true;
  } 
  else if(lineError == 8)
   {
    MoveForward();
  } 
}

void MoveRobotAndFindLine() {
  //LineErrorWindowPush(lineError);
  //lineError = LineErrorWindowProcess();
  if(lineError == 0){
    isFindingLine = false;
    MoveForward();
  } else if (lineError == 5) {
    MoveForward();
  } else if(lineError < 0 && lineError >= -4){
    MotorStop();
    TurnRight();
  } else if (lineError > 0 && lineError <= 4) {
    MotorStop();
    TurnLeft();
  }
}

void MoveRobot2(){
  if(lineError == 0){
    MoveForward();
  } else if (lineError == 5) {
    MoveForward();
  } else if(lineError < 0 && lineError >= -4){
    MoveForward();
    changeLineDelayCounter++;
    if(previousLineError != 0 && previousLineError > lineError ) {
      changeLineDelayCounter = 0;
      MotorStop();
      TurnLeft();
    }
    if(changeLineDelayCounter >= 20) {
      //stop
      //turn left 45 deg
      //move forward
    }
  } else if (lineError > 0 && lineError <= 4) {
    MoveForward();
    changeLineDelayCounter++;
    if(previousLineError != 0 && previousLineError < lineError ) {
      changeLineDelayCounter = 0;
      MotorStop();
      TurnRight();
    }
    if(changeLineDelayCounter >= 20) {
      //stop
      //turn right 45 deg
      //move forward
    }
  }
  previousLineError = lineError;
}

void IRRead() {
  Serial.println();
  for (int i = 0; i < 5; i++) {
    sensorValues[i] = analogRead(i);
    Serial.print(sensorValues[i]);
    Serial.print(" ");
  }
}

void LineErrorWindowSetup() {
  for(int i = 0; i < LINE_ERROR_WINDOW_SIZE - 1; i++)
    lineErrorWindow[i] = 0;
}

int LineErrorWindowProcess() {
  /*if(IsLineErrorChangePermanent()) {
    if(lineErrorWindow[0] < 0)
      return -6;
    else if(lineErrorWindow[0] > 0 && lineErrorWindow[0] < 5)
      return 6;
  } else*/ if (IsScriptStarting()) {
    return 7;
  } else if (lineErrorWindow[0] == 7) {
    return 0;
  } else if(IsObstacleCourseStarting()) {
    return 9;
  }
  return lineErrorWindow[0];
}

void LineErrorWindowPush(int errorValue) {
  for(int i = LINE_ERROR_WINDOW_SIZE - 1; i > 0; i--) {
    lineErrorWindow[i] = lineErrorWindow[i - 1];
  }
  lineErrorWindow[0] = errorValue;
}

bool IsLineErrorChanged() {
  return lineErrorWindow[0] != lineErrorWindow[1];
}

bool IsChangeLaneLeft() {
  int temp = 0;
  for(int i = 0; i < LINE_ERROR_WINDOW_SIZE - 1; i++) {
    if(lineErrorWindow[i] == 15)
      temp++;
    if(temp >= 2)
      return true;
  }
  return false;
}

bool IsChangeLaneRight() {
  if(lineErrorWindow[0] != 16)
    return false;
  int temp = 0;
  for(int i = 0; i < LINE_ERROR_WINDOW_SIZE - 1; i++) {
    if(lineErrorWindow[i] == 16)
      temp++;
    if(temp >= 4)
      return true;
  }
  return false;
}

int LineErrorChangeAmount() {
  int changeAmount = 0;
  for(int i = 0; i < LINE_ERROR_WINDOW_SIZE - 1; i++) {
    if(lineErrorWindow[i] != lineErrorWindow[i + 1])
      changeAmount++;
  }
  return changeAmount;
}

bool IsLineErrorSafe() {
  for(int i = 0; i < LINE_ERROR_WINDOW_SAFE; i++) {
    if(lineErrorWindow[i] != lineErrorWindow[i + 1])
      return false;
  }
  return true;
}

bool IsLineErrorNormalizing() {
  return abs(lineErrorWindow[0]) < abs(lineErrorWindow[1]);
}

bool IsObstacleCourseStarting() {
  int obstacleCourseCounter = 0;
  for(int i = 0; i < LINE_ERROR_WINDOW_SIZE - 1; i++) {
    if(lineErrorWindow[i] == 8) {
      obstacleCourseCounter++;
    }
    if(obstacleCourseCounter >= 3)
     {
      return true;
     }
  }
  return false;
}

bool IsScriptStarting() {
  if(lineErrorWindow[0] != 7)
    return false;
  int scriptCounter = 0;
  for(int i = 0; i < LINE_ERROR_WINDOW_SIZE - 1; i++) {
    if(lineErrorWindow[i] == 7)
      scriptCounter++;
    if(scriptCounter >= 4)
      return true;
  }
  return false;
}

bool IsLineErrorChangePermanent() {
  int oldLineError = lineErrorWindow[LINE_ERROR_WINDOW_SIZE - 1];
  int errorCounter = 0;
  if(lineErrorWindow[LINE_ERROR_WINDOW_SIZE - 1] == lineErrorWindow[LINE_ERROR_WINDOW_SIZE - 2])
    return false;
  /*if(lineErrorWindow[0] == lineErrorWindow[LINE_ERROR_WINDOW_SIZE - 1]) {
    for(int i = 0; i < LINE_ERROR_WINDOW_SIZE - 1; i++) {
      if(lineErrorWindow[i] != lineErrorWindow[i + 1])
        errorCounter++;
      if(errorCounter >= LINE_ERROR_WINDOW_SIZE * 0.1)
        return false;
    }
  }*/
  for(int i = 0; i < LINE_ERROR_WINDOW_SIZE - 1; i++) {
    if(lineErrorWindow[i] != lineErrorWindow[i + 1])
      return false;
  }
  return true;
}

void SonarInit(){
  long initDur, initDist;
  
  digitalWrite(sonarTrigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(sonarTrigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(sonarTrigPin, LOW);
  initDur = pulseIn(sonarEchoPin, HIGH);

  initDist = initDur / 58.2;
  
  for(int i = 0; i < 5; i++){
    sonarWindow[i] = initDist;
  }
}

long SimpleSonarRead() {
  digitalWrite(sonarTrigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(sonarTrigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(sonarTrigPin, LOW);
  sonarDuration = pulseIn(sonarEchoPin, HIGH);

  sonarDistance = sonarDuration / 58.2;

  return sonarDistance;
}

float SonarRead(){
  float windowMean = 0;
  
  digitalWrite(sonarTrigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(sonarTrigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(sonarTrigPin, LOW);
  sonarDuration = pulseIn(sonarEchoPin, HIGH);

  sonarDistance = sonarDuration / 58.2;

  if(sonarDistance >= sonarMaximumRange || sonarDistance <= sonarMinimumRange){
    return -1;
  } else{
    long windowSum = 0;
    for(int i = 0; i < 4; i++){
      sonarWindow[i + 1] = sonarWindow[i];
      windowSum = windowSum + sonarWindow[i];
    }
    sonarWindow[0] = sonarDistance;
    windowSum = windowSum + sonarWindow[0];

    windowMean = windowSum / 5;
  }
  return windowMean;
}

int GetSensorError(){
  if(
    sensorValues[0] < DEFAULT_SENSOR_DISTANCE &&
    sensorValues[1] < DEFAULT_SENSOR_DISTANCE &&
    sensorValues[2] >= DEFAULT_SENSOR_DISTANCE &&
    sensorValues[3] < DEFAULT_SENSOR_DISTANCE &&
    sensorValues[4] < DEFAULT_SENSOR_DISTANCE
    ) {
      //GO FORWARD
      lineError = 0;
    } else if(
      sensorValues[0] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[1] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[2] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[3] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[4] < DEFAULT_SENSOR_DISTANCE
    ) {
      //HARD RIGHT
      lineError = 4;
    } else if(
      sensorValues[0] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[1] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[2] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[3] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[4] < DEFAULT_SENSOR_DISTANCE
    ) {
      //MEDIUM RIGHT
      lineError = 3;
    } else if(
      sensorValues[0] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[1] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[2] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[3] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[4] < DEFAULT_SENSOR_DISTANCE
    ) {
      //NORMAL RIGHT
      lineError = 2;
    }/* else if(
      sensorValues[0] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[1] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[2] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[3] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[4] < DEFAULT_SENSOR_DISTANCE
    ) {
      //SLIGHT RIGHT
      lineError = 1;
    }*/ else if(
      sensorValues[0] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[1] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[2] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[3] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[4] >= DEFAULT_SENSOR_DISTANCE
    ) {
      //HARD LEFT
      lineError = -4;
    } else if(
      sensorValues[0] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[1] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[2] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[3] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[4] >= DEFAULT_SENSOR_DISTANCE
    ) {
      //MEDIUM LEFT
      lineError = -3;
    } else if(
      sensorValues[0] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[1] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[2] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[3] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[4] < DEFAULT_SENSOR_DISTANCE
    ) {
      //NORMAL LEFT
      lineError = -2;
    } /*else if(
      sensorValues[0] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[1] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[2] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[3] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[4] < DEFAULT_SENSOR_DISTANCE
    ) {
      //SLIGHT LEFT
      lineError = -1;
    } */else if(
      sensorValues[0] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[1] >= DEFAULT_SENSOR_DISTANCE &&
     // sensorValues[2] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[3] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[4] < DEFAULT_SENSOR_DISTANCE
    ) {
      //hurok
      lineError = 1;
    } /*else if(
      sensorValues[0] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[1] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[2] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[3] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[4] < DEFAULT_SENSOR_DISTANCE
    ) {
      //hurok
      lineError = 1;
    } else if(
      sensorValues[0] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[1] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[2] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[3] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[4] >= DEFAULT_SENSOR_DISTANCE
    ) {
      //hurok
      lineError = 1;
    } else if(
      sensorValues[0] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[1] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[2] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[3] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[4] >= DEFAULT_SENSOR_DISTANCE
    ) {
      //hurok
      lineError = 1;
    }*/ else if(
      sensorValues[0] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[1] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[2] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[3] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[4] >= DEFAULT_SENSOR_DISTANCE
    ) {
      //p fordulo, vagy akadalypalya
      lineError = 8;
    }
    else if (
      sensorValues[0] >= DEFAULT_SENSOR_DISTANCE &&
  //    sensorValues[1] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[2] >= DEFAULT_SENSOR_DISTANCE &&
     // sensorValues[3] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[4] >= DEFAULT_SENSOR_DISTANCE
    )
  {
    //szlalom
    lineError=7;
  }
    else if(
      sensorValues[0] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[1] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[2] >= DEFAULT_SENSOR_DISTANCE &&
    //  sensorValues[3] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[4] >= DEFAULT_SENSOR_DISTANCE
    ) 
    {
      lineError = 15;
    }
    else if(
      sensorValues[0] >= DEFAULT_SENSOR_DISTANCE &&
     // sensorValues[1] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[2] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[3] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[4] < DEFAULT_SENSOR_DISTANCE
    ) 
    {
      lineError = 16;
    } 
     else {
      //LOST LINE
      lineError = 5; 
    }
    return lineError;
}

void FindLine(){
  //TODO: implement method
  MotorStop();
}

void MoveForwardWithEncoder2(int amountInMm) {
  noInterrupts();
  isReadingEncoder = true;
  interrupts();
  while(encoderACounter < amountInMm || encoderBCounter < amountInMm) {
    MoveForward();
    Serial.println(encoderACounter);
    Serial.println(encoderBCounter);
  }
  MotorStop();
  noInterrupts();
  isReadingEncoder = false;
  interrupts();
  ResetEncoder();
  return;
}

void MoveBackwardWithEncoder2(int amountInMm) {
  noInterrupts();
  isReadingEncoder = true;
  interrupts();
  while(encoderACounter < amountInMm || encoderBCounter < amountInMm) {
    MoveBackward();
    Serial.println(encoderACounter);
    Serial.println(encoderBCounter);
  }
  MotorStop();
  noInterrupts();
  isReadingEncoder = false;
  interrupts();
  ResetEncoder();
  return;
}

void MoveForwardWithEncoder(int amountInMm) {
  if(!isMoving && !isTurning) {
    isMoving = true;
    isReadingEncoder = true;
    MoveForward();
    EnableEncoderCorrection();
  }
  if(isMoving) {
    double distanceInEncoderHits = amountInMm / 0.52;
    if(encoderACounter >= distanceInEncoderHits || encoderBCounter >= distanceInEncoderHits) {
      MotorStop();
      DisableEncoderCorrection();
      isMoving = false;
      isReadingEncoder = false;
      encoderACounter = 0;
      encoderBCounter = 0;
      if(isScriptRunning)
        scriptTaskCounter++;
    }
  }
}

void measureMoveForwardDistanceByEncoder() {

}

void correctMotorError(int encoderACountInterval, int encoderBCountInterval) {
  if(encoderACountInterval > encoderBCountInterval) {
    A_RPM_Correction--;
  }
  if(encoderBCountInterval > encoderACountInterval) {
    B_RPM_Correction--;
  }
}

void TurnLeftWithEncoder2(int amountInDegrees) {
  noInterrupts();
  isReadingEncoder = true;
  interrupts();
  while(encoderBCounter < amountInDegrees) {
    TurnLeft();
  }
  MotorStop();
  noInterrupts();
  isReadingEncoder = false;
  interrupts();
  ResetEncoder();
  return;
}

void TurnLeftWithEncoder(int amountInDegrees) {
  if(!isSpecialTaskRunning) {
    MotorStop();
    isReadingEncoder = true;
    encoderBCounter = 0;
    isSpecialTaskRunning = true;
    TurnLeft();
  }
  if(encoderBCounter >= (amountInDegrees)) {
    MotorStop();
    ResetEncoder();
    isReadingEncoder = false;
    isSpecialTaskRunning = false;
    isFindingLine = true;

    if(isScriptRunning)
      scriptTaskCounter++;
  }
}

void TurnRightWithEncoder2(int amountInDegrees) {
  noInterrupts();
  isReadingEncoder = true;
  interrupts();
  while(encoderACounter < amountInDegrees) {
    TurnRight();
  }
  MotorStop();
  noInterrupts();
  isReadingEncoder = false;
  interrupts();
  ResetEncoder();
  return;
}

void TurnRightWithEncoder(int amountInDegrees) {
  if(!isSpecialTaskRunning) {
    MotorStop();
    isReadingEncoder = true;
    encoderACounter = 0;
    isSpecialTaskRunning = true;
    TurnRight();
  }
  if(encoderACounter >= (amountInDegrees)) {
    MotorStop();
    ResetEncoder();
    isReadingEncoder = false;
    isSpecialTaskRunning = false;
    isFindingLine = true;

    if(isScriptRunning)
      scriptTaskCounter++;
  }
}

void Turn(int leftTurnValue, int rightTurnValue) {
  A_motorF();
  analogWrite(A_MotorEnable, RPM + A_RPM_Correction + leftTurnValue);
  B_motorF();
  analogWrite(B_MotorEnable, RPM + B_RPM_Correction + rightTurnValue);
}

void TurnRight(){
  A_motorF();
  analogWrite(A_MotorEnable, RPM + A_RPM_Correction);
  B_motorStop();
}

void TurnLeft(){
  B_motorF();
  analogWrite(B_MotorEnable, RPM + B_RPM_Correction);
  A_motorStop();
}

void MoveForward(){
  A_motorF();
  analogWrite(A_MotorEnable, RPM + A_RPM_Correction);
  B_motorF();
  analogWrite(B_MotorEnable, RPM + B_RPM_Correction);
}

void MoveBackward(){
  A_motorR();
  analogWrite(A_MotorEnable, RPM + A_RPM_Correction);
  B_motorR();
  analogWrite(B_MotorEnable, RPM + B_RPM_Correction);
}

void MotorStop(){
  //isMoving = false;
  A_motorStop();
  B_motorStop();
}

void A_motorF(){
  digitalWrite(A_MotorDir1,LOW);
  digitalWrite(A_MotorDir2,HIGH);
}
void A_motorR() {
  digitalWrite(A_MotorDir1, HIGH);
  digitalWrite(A_MotorDir2, LOW);
}

void A_motorStop(){
  digitalWrite(A_MotorDir1, LOW);
  digitalWrite(A_MotorDir2, LOW);
  digitalWrite(A_MotorEnable, LOW);
}

void B_motorF(){
  digitalWrite(B_MotorDir1,LOW);
  digitalWrite(B_MotorDir2,HIGH);
}

void B_motorR() {
  digitalWrite(B_MotorDir1, HIGH);
  digitalWrite(B_MotorDir2, LOW);
}

void B_motorStop(){
  digitalWrite(B_MotorDir1, LOW);
  digitalWrite(B_MotorDir2, LOW);
  digitalWrite(B_MotorEnable, LOW);
}


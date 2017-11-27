//LEGS
#define A_MotorEnable 2
#define A_MotorDir1 3
#define A_MotorDir2 4

#define B_MotorEnable 5
#define B_MotorDir1 6
#define B_MotorDir2 7

#define sonarEchoPin 9
#define sonarTrigPin 8

#define Encoder_A 19
#define Encoder_B 18

//TIMER
int timerCount = 0;

//IR
const int DEFAULT_SENSOR_DISTANCE = 900;
int lineError = 0;
int previousLineError = 0;
int sensorValues[5] = {0, 0, 0, 0, 0};
int changeLineDelayCounter = 0;
const int LINE_ERROR_WINDOW_SIZE = 5;
int lineErrorWindow [LINE_ERROR_WINDOW_SIZE] = {0, 0, 0, 0, 0};

//MOTOR
int A_RPM_Correction = 0;
int B_RPM_Correction = 0;
const int RPM = 80;
const int TURN_CONSTANT = 10;
bool isMoving = false;
bool isTurning = false;

//SONAR
const int sonarMaximumRange = 200;
const int sonarMinimumRange = 0;

long sonarDuration, sonarDistance;

int sonarWindow[5];

//ENCODER
//A-right, B-left
//B: 81 ~ half turn; 162 ~ full turn
//A: 82 ~ half turn; 164 ~ full turn
int encoderACounter, encoderBCounter = 0;
int encoderACorrectionCounter, encoderBCorrectionCounter = 0;
bool isReadingEncoder = false;
bool isEncoderCorrection = false;

//-------------------------------------------//

void setup() {
  Serial.begin(9600);

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

  //timer
  Timer1Init();

  //motor
  isMoving = true;
}

void loop() {
  //delayMicroseconds(1000);
  //IRRead();
  //Serial.println(GetSensorError());
  //Serial.println(SonarRead());
  //MoveRobot();
  //MoveStraightWithSonar();
}

//-------------------------------------------//

//FUNCTIONS

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

  //vege jelzo interrupt bekapcsolasa
  //ha elerte a szamlalo a megadott erteket, az OCIE1A flag bebillen es megtortenik a magszakitas
  TIMSK1 |= (1 << OCIE1A);

  //megszakitasok ujra engedelyezese
  interrupts();
}

ISR(TIMER1_COMPA_vect){
  //ISR beepitett fuggveny
  //akkor fut le, ha megtortenik a timer megszakitas

  if(isMoving) {
    IRRead();
    int sensorError = GetSensorError();

    //float sonarValue = SonarRead();
    //if(sonarValue < 10) {
    //  MotorStop();
    //}

    MoveRobot();
  }

  if(isEncoderCorrection) {
    timerCount++;
    if(timerCount == 50){
      correctMotorError(encoderACorrectionCounter, encoderBCorrectionCounter);
      ResetEncoderCorrection();
      timerCount = 0;
    }
  }
}

void LeftSlalom() {
  isMoving = false;
  MotorStop();
  TurnLeftWithEncoder(45);
  MotorStop();
  MoveForwardWithEncoder2(300);
  MotorStop();
  TurnRightWithEncoder(90);
  MotorStop();
  MoveForwardWithEncoder2(300);
  MotorStop();
  TurnLeftWithEncoder(45);
  isMoving = true;
}

void CalibrateSensors(){
  //TODO: implement method
}

void EncoderAHit(){
  if(isReadingEncoder)
    encoderACounter++;
  if(isEncoderCorrection)
    encoderACorrectionCounter++;
}

void EncoderBHit(){
  if(isReadingEncoder)
    encoderBCounter++;
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
  ResetEncoderCorrection();
  //isReadingEncoder = true;
  isEncoderCorrection = true;
}

void ActOnEncoderCounters(){
  Serial.println(encoderACounter);
  Serial.println(encoderBCounter);

  encoderACounter = 0;
  encoderBCounter = 0;
}

void MoveRobot(){
  Serial.println(lineError);
  if(lineError == 0){
    MoveForward();
  } else if (lineError == 5) {
    MoveForward();
  } else if(lineError < 0 && lineError >= -4){
    MotorStop();
    TurnLeft();
  } else if (lineError > 0 && lineError <= 4) {
    MotorStop();
    TurnRight();
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
  for (int i = 0; i < 5; i++) {
    sensorValues[i] = analogRead(i);

    //Debug
    Serial.print(sensorValues[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void LineErrorWindowPush(int errorValue) {
  for(int i = 0; i < LINE_ERROR_WINDOW_SIZE - 1; i++) {
    lineErrorWindow[i + 1] = lineErrorWindow[i];
  }
  lineErrorWindow[0] = errorValue;
}

bool IsLineErrorChanged() {
  return lineErrorWindow[0] == lineErrorWindow[1];
}

int LineErrorChangeAmount() {
  int changeAmount = 0;
  for(int i = 0; i < LINE_ERROR_WINDOW_SIZE - 1; i++) {
    if(lineErrorWindow[i] != lineErrorWindow[i + 1])
      changeAmount++;
  }
  return changeAmount;
}

bool IsLineErrorChangePermanent() {
  int oldLineError = lineErrorWindow[LINE_ERROR_WINDOW_SIZE - 1];
  for(int i = 0; i < LINE_ERROR_WINDOW_SIZE - 2; i++) {
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
    } else if(
      sensorValues[0] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[1] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[2] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[3] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[4] < DEFAULT_SENSOR_DISTANCE
    ) {
      //SLIGHT RIGHT
      lineError = 1;
    } else if(
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
    } else if(
      sensorValues[0] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[1] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[2] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[3] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[4] < DEFAULT_SENSOR_DISTANCE
    ) {
      //SLIGHT LEFT
      lineError = -1;
    } else if(
      sensorValues[0] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[1] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[2] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[3] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[4] < DEFAULT_SENSOR_DISTANCE
    ) {
      //hurok
      lineError = 1;
    } else if(
      sensorValues[0] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[1] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[2] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[3] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[4] >= DEFAULT_SENSOR_DISTANCE
    ) {
      //p fordulo
      lineError = 0;
    } else if(
      sensorValues[0] < DEFAULT_SENSOR_DISTANCE &&
      sensorValues[1] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[2] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[3] >= DEFAULT_SENSOR_DISTANCE &&
      sensorValues[4] < DEFAULT_SENSOR_DISTANCE
    ) {
      //szlalom, vagy akadalypalya

    } else {
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
  //EnableEncoderCorrection();
  MoveForward();

  double distanceInEncoderHits = amountInMm / 0.52;
  if(encoderACounter >= distanceInEncoderHits || encoderBCounter >= distanceInEncoderHits) {
    MotorStop();
    isReadingEncoder = false;
    ResetEncoder();
  }
}

void MoveForwardWithEncoder(int amountInMm) {
  if(!isMoving && !isTurning) {
    isMoving = true;
    isReadingEncoder = true;
    MoveForward();
  }
  if(isMoving) {
    double distanceInEncoderHits = amountInMm / 0.52;
    if(encoderACounter >= distanceInEncoderHits || encoderBCounter >= distanceInEncoderHits) {
      MotorStop();
      isMoving = false;
      isReadingEncoder = false;
      encoderACounter = 0;
      encoderBCounter = 0;
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

void TurnLeftWithEncoder(int amountInDegrees) {
  if(!isTurning && !isMoving) {
    isTurning = true;
    MotorStop();
    isMoving = false;
    isReadingEncoder = true;
    encoderBCounter = 0;

    TurnLeft();
  }
  if(encoderBCounter >= (amountInDegrees * 2.2)) {
    MotorStop();
    ResetEncoder();
    isReadingEncoder = false;
    isTurning = false;
    //isMoving = true;
  }
}

void TurnRightWithEncoder(int amountInDegrees) {
  if(!isTurning && !isMoving) {
    isTurning = true;
    MotorStop();
    isMoving = false;
    isReadingEncoder = true;
    encoderACounter = 0;

    TurnRight();
  }
  if(encoderACounter >= (amountInDegrees * 2.2)) {
    MotorStop();
    ResetEncoder();
    isReadingEncoder = false;
    isTurning = false;
    //isMoving = true;
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


//LEGS
#define A_MotorEnable 2
#define A_MotorDir1 3
#define A_MotorDir2 4

#define B_MotorEnable 5
#define B_MotorDir1 6
#define B_MotorDir2 7

#define sonarEchoPin 9
#define sonarTrigPin 8

#define Encoder_A 18
#define Encoder_B 19

//TIMER
int timerCount = 0;

//IR
const int DEFAULT_SENSOR_DISTANCE = 900;
int lineError = 0;
int previousLineError = 0;
int sensorValues[5] = {0, 0, 0, 0, 0};
int changeLineDelayCounter = 0;

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
//B-right, A-left
//B: 81 ~ half turn; 162 ~ full turn
//A: 82 ~ half turn; 164 ~ full turn
int encoderACounter, encoderBCounter = 0;
bool isReadingEncoder = false;

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

  //if(isMoving) {
  //  IRRead();
  //  int sensorError = GetSensorError();
  //  float sonarValue = SonarRead();
  //}

  //MoveRobot();

  //timerCount++;
  //if(timerCount == 50){
  //  //ActOnEncoderCounters();
  //  timerCount = 0;
  //}
}

void LeftSlalom() {
  TurnLeftWithEncoder(45);
  MoveForwardWithEncoder(300);
  TurnRightWithEncoder(90);
  MoveForwardWithEncoder(300);
}

void CalibrateSensors(){
  //TODO: implement method
}

void EncoderAHit(){
  if(isReadingEncoder)
    encoderACounter++;
}

void EncoderBHit(){
  if(isReadingEncoder)
    encoderBCounter++;
}

void ActOnEncoderCounters(){
  Serial.println(encoderACounter);
  Serial.println(encoderBCounter);

  encoderACounter = 0;
  encoderBCounter = 0;
}

void MoveRobot(){
  if(isMoving){
    if(lineError == 0){
      MoveForward();
    } else if (lineError == 5) {
      MoveForward();
    } else if(lineError < 0 && lineError >= -4){
      if(previousLineError != 0 && abs(previousLineError) < abs(lineError) ) {
        //stop
        //turn back straight
        //turn left 45 deg
        //move forward
      }
      MotorStop();
      TurnLeft();
    } else if (lineError > 0 && lineError <= 4) {
      if(previousLineError != 0 && abs(previousLineError) < abs(lineError) ) {
        //stop
        //turn back straight
        //turn right 45 deg
        //move forward
      }
      MotorStop();
      TurnRight();
    }
    previousLineError = lineError;
  } else if (lineError = 6) {
    TurnRightWithEncoder(45);
  } else if(lineError = -6) {
    TurnLeftWithEncoder(45);
  } else {
    MotorStop();
  }
}

void MoveRobot2(){
  if(isMoving){
    if(lineError == 0){
      MoveForward();
    } else if (lineError == 5) {
      MoveForward();
    } else if(lineError < 0 && lineError >= -4){
      MoveForward();
      changeLineDelayCounter++;
      if(previousLineError != 0 && previousLineError > lineError ) {
        MotorStop();
        TurnLeft();
      }
      if(changeLineDelayCounter >= 20) {
        //stop
        //turn back straight
        //turn left 45 deg
        //move forward
      }
    } else if (lineError > 0 && lineError <= 4) {
      MoveForward();
      changeLineDelayCounter++;
      if(previousLineError != 0 && previousLineError < lineError ) {
        MotorStop();
        TurnRight();
      }
      if(changeLineDelayCounter >= 20) {
        //stop
        //turn back straight
        //turn right 45 deg
        //move forward
      }
    }
    previousLineError = lineError;
  } else {
    MotorStop();
  }
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

void TurnLeftWithEncoder(int amountInDegrees) {
  if(!isTurning && !isMoving) {
    isTurning = true;
    MotorStop();
    isMoving = false;
    isReadingEncoder = true;

    TurnLeft();
  }
  if(encoderACounter >= (amountInDegrees * 2.2)) {
    MotorStop();
    isReadingEncoder = false;
    encoderACounter = 0;
    isTurning = false;
    isMoving = true;
  }
}

void TurnRightWithEncoder(int amountInDegrees) {
  if(!isTurning && !isMoving) {
    MotorStop();
    isMoving = false;
    isReadingEncoder = true;
    encoderBCounter = 0;

    TurnRight();
  }
  if(encoderBCounter >= (amountInDegrees * 2.2)) {
    MotorStop();
    isReadingEncoder = false;
    encoderBCounter = 0;
    isTurning = false;
    isMoving = true;
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


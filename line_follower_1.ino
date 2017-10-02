//IR
const int IR_SENSOR_0 = 0;
const int IR_SENSOR_1 = 1;
const int IR_SENSOR_2 = 2;
const int IR_SENSOR_3 = 3;
const int IR_SENSOR_4 = 4;

const int DEFAULT_SENSOR_DISTANCE = 1000;

int lineError = 0;
int sensorValues[5] = {0, 0, 0, 0, 0};

//MOTOR
int A_MotorEnable = 2;
int A_MotorDir1 = 3;
int A_MotorDir2 = 4;
int A_RPM_Correction = 0;

int B_MotorEnable = 5;
int B_MotorDir1 = 6;
int B_MotorDir2 = 7;
int B_RPM_Correction = 0;

const int RPM = 100;
const int TURN_CONSTANT = 10;

void setup() {
  Serial.begin(9600);
}

void loop() {
  //delayMicroseconds(1000);
  //IRRead();
  //Serial.println(GetSensorError());
  MoveRobot();
}

void MoveRobot(){
  IRRead();
  GetSensorError();
  
  if(lineError == 0){
    MoveForward();
  } else if(lineError < 0){
    MoveWithLeftTurn(-lineError*TURN_CONSTANT);
  } else if (lineError > 0) {
    MoveWithRightTurn(lineError*TURN_CONSTANT);
  }
  delay(50);
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
    } else {
      //LOST LINE
      lineError = 5; 
      FindLine();
      Serial.println("lost line");
    }

    return lineError;
}

void FindLine(){
  //TODO: implement method
  MotorStop();
}

void MoveWithLeftTurn(int turnValue){
  A_motorF();
  analogWrite(A_MotorEnable, RPM + A_RPM_Correction + turnValue);
  B_motorF();
  analogWrite(B_MotorEnable, RPM + B_RPM_Correction);
}

void MoveWithRightTurn(int turnValue){
  A_motorF();
  analogWrite(A_MotorEnable, RPM + A_RPM_Correction);
  B_motorF();
  analogWrite(B_MotorEnable, RPM + B_RPM_Correction + turnValue);
}

void MoveForward(){
  A_motorF();
  analogWrite(A_MotorEnable, RPM + A_RPM_Correction);
  B_motorF();
  analogWrite(B_MotorEnable, RPM + B_RPM_Correction);
}

void MotorStop(){
  digitalWrite(A_MotorDir1, LOW);
  digitalWrite(A_MotorDir2, LOW);
  digitalWrite(A_MotorEnable, LOW);

  digitalWrite(B_MotorDir1, LOW);
  digitalWrite(B_MotorDir2, LOW);
  digitalWrite(B_MotorEnable, LOW);
}

void A_motorF(){
  //TEST NEEDED
  digitalWrite(A_MotorDir1,LOW);
  digitalWrite(A_MotorDir2,HIGH);
}
void A_motorR() {
  //TEST NEEDED
  digitalWrite(A_MotorDir1, HIGH);
  digitalWrite(A_MotorDir2, LOW);
}

void B_motorF(){
  //TEST NEEDED
  digitalWrite(B_MotorDir1,LOW);
  digitalWrite(B_MotorDir2,HIGH);
}
void B_motorR() {
  //TEST NEEDED
  digitalWrite(B_MotorDir1, HIGH);
  digitalWrite(B_MotorDir2, LOW);
}


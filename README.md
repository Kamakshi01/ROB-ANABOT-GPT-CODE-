# ROB-ANABOT-GPT-SOURCECODE-

#include <AFMotor.h>

//defining pins and variables
#define left A0
#define right A1

//defining motors
AF_DCMotor motor1(1, MOTOR12_1KHZ); 
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);



void setup() {
  //declaring pin types
  pinMode(left,INPUT);
  pinMode(right,INPUT);
  //begin serial communication
  Serial.begin(9600);
  
}

void loop(){
  //printing values of the sensors to the serial monitor
  Serial.println(digitalRead(left));
  
  Serial.println(digitalRead(right));

  //line detected by both
  if(digitalRead(left)==0 && digitalRead(right)==0){
    //Forward
    motor1.run(FORWARD);
    motor1.setSpeed(150);
    motor2.run(FORWARD);
    motor2.setSpeed(150);
    motor3.run(FORWARD);
    motor3.setSpeed(150);
    motor4.run(FORWARD);
    motor4.setSpeed(150);
  }
  //line detected by left sensor
  else if(digitalRead(left)==0 && !analogRead(right)==0){
    //turn left
    motor1.run(FORWARD);
    motor1.setSpeed(200);
    motor2.run(FORWARD);
    motor2.setSpeed(200);
    motor3.run(BACKWARD);
    motor3.setSpeed(200);
    motor4.run(BACKWARD);
    motor4.setSpeed(200);
    
  }
  //line detected by right sensor
  else if(!digitalRead(left)==0 && digitalRead(right)==0){
    //turn right
    motor1.run(BACKWARD);
    motor1.setSpeed(200);
    motor2.run(BACKWARD);
    motor2.setSpeed(200);
    motor3.run(FORWARD);
    motor3.setSpeed(200);
    motor4.run(FORWARD);
    motor4.setSpeed(200);
   
  }
  //line detected by none
  else if(!digitalRead(left)==0 && !digitalRead(right)==0){
    //stop
    motor1.run(RELEASE);
    motor1.setSpeed(0);
    motor2.run(RELEASE);
    motor2.setSpeed(0);
    motor3.run(RELEASE);
    motor3.setSpeed(0);
    motor4.run(RELEASE);
    motor4.setSpeed(0);
   
  }
  
}


IR SENSORS

// Define pins for motor control
const int motor2Pin1 = 3; // Motor 1 positive0 terminal connected to digital pin 2
const int motor2Pin2 = 2; // Motor 1 negative terminal connected to digital pin 3
const int motor1Pin1 = 5; // Motor 2 positive terminal connected to digital pin 4
const int motor1Pin2 = 4; // Motor 2 negative terminal connected to digital pin 5

// Define pins for IR sensor
const int leftSensorPin = A0; // Left IR sensor input connected to digital pin 6
const int rightSensorPin = A1; // Right IR sensor input connected to digital pin 7

void setup() {
  // Initialize motor control pins as outputs
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  
  // Initialize IR sensor pins as inputs
  pinMode(leftSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);
  Serial.begin(9600);
}

void loop() {


  // Read sensor inputs
  int leftSensorValue = digitalRead(leftSensorPin);
  int rightSensorValue = digitalRead(rightSensorPin);
  if (leftSensorValue ==1) {
    leftSensorValue = 0;
  }
  else if (leftSensorValue ==0) {
    leftSensorValue = 1;
  }
  if (rightSensorValue ==1) {
    rightSensorValue = 0;
  }
  else if (rightSensorValue ==0) {
    rightSensorValue = 1;
  }
  
  Serial.print(leftSensorValue);
  Serial.print(rightSensorValue);
  Serial.print(" ");
  
  // If both sensors detect the line, move forward
  if (leftSensorValue == HIGH && rightSensorValue == HIGH) {
    moveForward();
  }
  // If only left sensor detects the line, turn right
  else if (leftSensorValue == HIGH && rightSensorValue == LOW) {
    turnRight();
  }
  // If only right sensor detects the line, turn left
  else if (leftSensorValue == LOW && rightSensorValue == HIGH) {
    turnLeft();
  }
  // If both sensors don't detect the line, stop
  else {
    stopMotors();
  }
}

void moveForward() {
  Serial.println("fw");

  analogWrite(9, 100);
  analogWrite(10, 100);

  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
}

void turnLeft() {
  Serial.println("l");

  analogWrite(9, 100);
  analogWrite(10, 100);

  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
}

void turnRight() {
  Serial.println("r");

  analogWrite(9, 100);
  analogWrite(10, 100);

  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
}

void stopMotors() {
  Serial.println("stop");

  analogWrite(9, 0);
  analogWrite(10, 0);

  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
}



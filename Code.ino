#include <NewPing.h>
#include <Servo.h>
#include <AFMotor.h>

//hc-sr04 sensor
#define TRIGGER_PIN A2
#define ECHO_PIN A3
#define max_distance 50

//ir sensor
#define irLeft A0
#define irRight A1

//motor
#define MAX_SPEED 200
#define MAX_SPEED_OFFSET 20

Servo servo;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, max_distance);

AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);


int distance = 0;
int leftDistance;
int rightDistance;
boolean object;

void setup() {
  Serial.begin(9600);
  pinMode(irLeft, INPUT);
  pinMode(irRight, INPUT);
  servo.attach(10);
  servo.write(90);

  motor1.setSpeed(160);
  motor2.setSpeed(160);
  motor3.setSpeed(160);
  motor4.setSpeed(160);
}

void loop() {
  if (digitalRead(irLeft) == 1 && digitalRead(irRight) == 1 ) {
    objectAvoid();
    //forword
  }
  else if (digitalRead(irLeft) == 1 && digitalRead(irRight) == 0 ) {
    
    Serial.println("TL");
    //leftturn
    moveLeft();
    delay(500);
    
  }
  else if (digitalRead(irLeft) == 0 && digitalRead(irRight) == 1 ) {
    
    Serial.println("TR");
    //rightturn
    moveRight();
    delay(500);
    
    
  }
  else if (digitalRead(irLeft) == 0 && digitalRead(irRight) == 0 ) {
    objectStop();
    Serial.println("STOP");
   
  }

     objectAvoid();
}

void objectStop() {
  distance = getDistance();
  if (distance <= 20) {
    //stop
    Stop();
    delay(5000);
    Serial.println("Stop");
  }
  else {
    Serial.println("moveforword");
    moveForward();
  }
  
}

void objectAvoid() {
  distance = getDistance();
  if (distance <= 20) {
    //stop
    Stop();
    Serial.println("Stop");
    if (digitalRead(irLeft) == 0 && digitalRead(irRight) == 0 ) {
    objectStop();
    Serial.println("STOP");
   
    }
    else{
    lookLeft();
    lookRight();
    delay(100);
    
    if (rightDistance <= leftDistance) {
      //left
      object = true;
      turn();
      Serial.println("moveLeft");
    } else {
      //right
      object = false;
      turn();
      Serial.println("moveRight");
    }
    delay(100);
  }
  }
  else {
    //forword
    Serial.println("moveforword");
    moveForward();
  }
}

int getDistance() {
  delay(50);
  int cm = sonar.ping_cm();
  if (cm == 0) {
    cm = 100;
  }
  return cm;
}

int lookLeft () {
  //lock left
  for (int i=90;i<=150;i=i+5){
    servo.write(i);
    delay(100);
    }
//  servo.write(150);
  delay(500);
  leftDistance = getDistance();
  delay(100);
  servo.write(90);
  Serial.print("Left:");
  Serial.print(leftDistance);
  return leftDistance;
  delay(100);
}

int lookRight() {
  //lock right
  for (int i=90;i>=40;i=i-5){
    servo.write(i);
    delay(100);
    }
//  servo.write(30);
  delay(500);
  rightDistance = getDistance();
  delay(100);
  servo.write(90);
  Serial.print("   ");
  Serial.print("Right:");
  Serial.println(rightDistance);
  return rightDistance;
  delay(100);
}
void Stop() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}
void moveForward() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}
void moveBackward() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);

}
void turn() {
  if (object == false) {
    Serial.println("turn Right");
    moveRight();
    delay(1300);
    moveForward();
    delay(2800);
    moveLeft();
    delay(1300);
    if (digitalRead(irRight) == 0) {
      loop();
    } else {
      moveForward();
   }
  }
  else {
    Serial.println("turn left");
    moveLeft();
    delay(1300);
    moveForward();
    delay(2500);
    moveRight();
    delay(1300);
    if (digitalRead(irLeft) == 0) {
      loop();
    } else {
      moveForward();
   }
  }
}
void moveRight() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}
void moveLeft() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

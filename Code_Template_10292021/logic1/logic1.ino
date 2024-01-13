#include <Servo.h>
#include <math.h>

#define TRIGGER_PIN_FRONT1 2
#define ECHO_PIN_FRONT1 3
#define TRIGGER_PIN_FRONT2 4
#define ECHO_PIN_FRONT2 5
#define TRIGGER_PIN_RIGHT1 6
#define ECHO_PIN_RIGHT1 7
#define TRIGGER_PIN_RIGHT2 8
#define ECHO_PIN_RIGHT2 9
#define TRIGGER_PIN_LEFT1 10
#define ECHO_PIN_LEFT1 11
#define TRIGGER_PIN_LEFT2 12
#define ECHO_PIN_LEFT2 13
// #define TRIGGER_PIN_BACK1 A0
// #define ECHO_PIN_BACK1 A1
// #define TRIGGER_PIN_BACK2 A2
// #define ECHO_PIN_BACK2 A3

#define SERVO_PIN 50
#define BACK_MOTOR_PIN1 44
#define BACK_MOTOR_PIN2 45

Servo steeringServo;
int currentSteeringAngle = 90;
int minSteeringAngle = 60;
int maxSteeringAngle = 120;
int steeringSensitivity = 10;

int backMotorSpeed = 200;

void setup() {
  Serial.begin(9600);
  pinMode(TRIGGER_PIN_FRONT1, OUTPUT);
  pinMode(ECHO_PIN_FRONT1, INPUT);
  pinMode(TRIGGER_PIN_FRONT2, OUTPUT);
  pinMode(ECHO_PIN_FRONT2, INPUT);
  pinMode(TRIGGER_PIN_RIGHT1, OUTPUT);
  pinMode(ECHO_PIN_RIGHT1, INPUT);
  pinMode(TRIGGER_PIN_RIGHT2, OUTPUT);
  pinMode(ECHO_PIN_RIGHT2, INPUT);
  pinMode(TRIGGER_PIN_LEFT1, OUTPUT);
  pinMode(ECHO_PIN_LEFT1, INPUT);
  pinMode(TRIGGER_PIN_LEFT2, OUTPUT);
  pinMode(ECHO_PIN_LEFT2, INPUT);
//   pinMode(TRIGGER_PIN_BACK1, OUTPUT);
//   pinMode(ECHO_PIN_BACK1, INPUT);
//   pinMode(TRIGGER_PIN_BACK2, OUTPUT);
//   pinMode(ECHO_PIN_BACK2, INPUT);

  steeringServo.attach(SERVO_PIN);
  steeringServo.write(currentSteeringAngle);

  pinMode(BACK_MOTOR_PIN1, OUTPUT);
  pinMode(BACK_MOTOR_PIN2, OUTPUT);
}

long getDistance(int triggerPin, int echoPin) {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  long distance = duration / 58.2;
  return distance;
}

void adjustSteering() {
  int frontDistanceDiff = getDistance(TRIGGER_PIN_FRONT1, ECHO_PIN_FRONT1) - getDistance(TRIGGER_PIN_FRONT2, ECHO_PIN_FRONT2);
  int rightDistanceDiff = getDistance(TRIGGER_PIN_RIGHT1, ECHO_PIN_RIGHT1) - getDistance(TRIGGER_PIN_RIGHT2, ECHO_PIN_RIGHT2);
  int leftDistanceDiff = getDistance(TRIGGER_PIN_LEFT1, ECHO_PIN_LEFT1) - getDistance(TRIGGER_PIN_LEFT2, ECHO_PIN_LEFT2);
//   int backDistanceDiff = getDistance(TRIGGER_PIN_BACK1, ECHO_PIN_BACK1) - getDistance(TRIGGER_PIN_BACK2, ECHO_PIN_BACK2);

  if (frontDistanceDiff < -20) {  // Obstacle in front left
    currentSteeringAngle = constrain(currentSteeringAngle + steeringSensitivity, minSteeringAngle, maxSteeringAngle);
  } else if (frontDistanceDiff > 20) {  // Obstacle in front right
    currentSteeringAngle = constrain(currentSteeringAngle - steeringSensitivity, minSteeringAngle,maxSteeringAngle);
      } else if (rightDistanceDiff < -20) {  // Obstacle on right
    currentSteeringAngle = constrain(currentSteeringAngle - steeringSensitivity, minSteeringAngle, maxSteeringAngle);
  }else if (leftDistanceDiff > 20) {  // Obstacle on left
    currentSteeringAngle = constrain(currentSteeringAngle + steeringSensitivity, minSteeringAngle, maxSteeringAngle);
  }

//   if (backDistanceDiff < -20) {  // Obstacle behind, back up
//     digitalWrite(BACK_MOTOR_PIN1, HIGH);
//     digitalWrite(BACK_MOTOR_PIN2, LOW);
//     delay(1000);
//   } else {  // Move forward
 
    digitalWrite(BACK_MOTOR_PIN1, LOW);
    digitalWrite(BACK_MOTOR_PIN2, HIGH);
  }

  steeringServo.write(currentSteeringAngle);
}

void loop() {
  adjustSteering();
}


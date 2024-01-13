#include <Servo.h>
#include <math.h>

// NEW
#define trigPin1 11  // Trig for sensor 1 ()
#define echoPin1 10  // Echo for sensor 1 ()
#define trigPin2 9   // Trig for sensor 2 ()
#define echoPin2 8   // Echo for sensor 2 ()
#define trigPin3 7   // Trig for sensor 3 ()
#define echoPin3 6   // Echo for sensor 3 ()
#define trigPin4 5   // Trig for sensor 4 ()
#define echoPin4 4   // Echo for sensor 4 ()
#define trigPin5 3   // Trig for sensor 5 ()
#define echoPin5 2   // Echo for sensor 5 ()
#define trigPin6 23  // Trig for sensor 6 ()
#define echoPin6 22  // Echo for sensor 6

#define drivePin 12
#define steerPin 13

Servo ssm; //define that ssm (steering servo motor) is a servo
Servo esc; //define that esc is (similar to) a servo

Servo steeringServo;
int currentSteeringAngle = 90;
int minSteeringAngle = 60;
int maxSteeringAngle = 120;
int steeringSensitivity = 10;

int backMotorSpeed = 200;

void setup() {
  pinMode(trigPin1, OUTPUT);  //Set the trigPin1 as output
  pinMode(echoPin1, INPUT);   //Set the echoPin1 as input
  pinMode(trigPin2, OUTPUT);  //Set the trigPin2 as output
  pinMode(echoPin2, INPUT);   //Set the echoPin2 as input
  pinMode(trigPin3, OUTPUT);  //Set the trigPin3 as output
  pinMode(echoPin3, INPUT);   //Set the echoPin3 as input
  pinMode(trigPin4, OUTPUT);  //Set the trigPin4 as output
  pinMode(echoPin4, INPUT);   //Set the echoPin4 as input
  pinMode(trigPin5, OUTPUT);  //Set the trigPin5 as output
  pinMode(echoPin5, INPUT);   //Set the echoPin5 as input
  pinMode(trigPin6, OUTPUT);  //Set the trigPin6 as output
  pinMode(echoPin6, INPUT);   //Set the echoPin6 as input

  ssm.attach(13); //define that ssm is connected at pin 12, steering
  esc.attach(12); //define that esc is connected at pin 13, driving

  //****************** Vehicle Initialization ********************//
  //***************** Do not change below part *****************//
  esc.write(90);
  delay(1000);
  //***************** Do not change above part *****************//
  Serial.begin(9600); // Starts the serial communication, Serial communications at 57600 bps
}

void loop() {
  adjustSteering();
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
  long pin3Distance = getDistance(trigPin3, echoPin3);
  long pin4Distance = getDistance(trigPin4, echoPin4);
  long pin5Distance = getDistance(trigPin5, echoPin5);
  long pin6Distance = getDistance(trigPin6, echoPin6);
  long pin1Distance = getDistance(trigPin1, echoPin1);
  long pin2Distance = getDistance(trigPin2, echoPin2);

  // Debug
  Serial.print(pin3Distance);
  Serial.print("cm [3], ");
  Serial.print(pin4Distance);
  Serial.print("cm [4], ");
  Serial.print(pin5Distance);
  Serial.print("cm [5], ");
  Serial.print(pin6Distance);
  Serial.print("cm [6], ");
  Serial.print(pin5Distance);
  Serial.print("cm [1], ");
  Serial.print(pin6Distance);
  Serial.print("cm [2]");
  Serial.println();

  int frontDistanceDiff = pin3Distance - pin4Distance;
  int rightDistanceDiff = pin5Distance - pin6Distance;
  int leftDistanceDiff = pin1Distance - pin2Distance;

  // Obstacle at front left
  if (frontDistanceDiff < -20) {
    currentSteeringAngle = constrain(currentSteeringAngle + steeringSensitivity, minSteeringAngle, maxSteeringAngle);
  }
  // Obstacle at front right
  else if (frontDistanceDiff > 20) {
    currentSteeringAngle = constrain(currentSteeringAngle - steeringSensitivity, minSteeringAngle, maxSteeringAngle);
  }
  // Obstacle at right
  else if (rightDistanceDiff < -20) {
    currentSteeringAngle = constrain(currentSteeringAngle - steeringSensitivity, minSteeringAngle, maxSteeringAngle);
  }
  // Obstacle at left
  else if (leftDistanceDiff > 20) {
    currentSteeringAngle = constrain(currentSteeringAngle + steeringSensitivity, minSteeringAngle, maxSteeringAngle);
  }

  esc.write(97);
  ssm.write(currentSteeringAngle);
}

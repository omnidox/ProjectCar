// Test steering of the car
// Objective: Turn front wheels left and right continuously

#include "Servo.h"

#define STEER_SMOOTHENING 5
#define STRAIGHT 90
#define MAX_LEFT 0
#define MAX_RIGHT 180

#define STEER_PIN 13

Servo ssm; // Steering control

int currentSteerAngle = 90;

void setup() {
  ssm.attach(STEER_PIN);
  Serial.begin(9600);
}

void loop() {
  steer(MAX_RIGHT);
  steer(MAX_LEFT);
  steer(STRAIGHT);
  delay(1000); // Pause for one second
}

void steer(int targetAngle) {
  if (targetAngle < currentSteerAngle) {
    steerLeft(targetAngle);
  } else {
    steerRight(targetAngle);
  }
}

void steerLeft(int targetAngle) {
  // Guard condition (avoid infinite loops)
  if (targetAngle >= currentSteerAngle) return;

  while (--currentSteerAngle >= targetAngle && currentSteerAngle >= MAX_LEFT) {
    ssm.write(currentSteerAngle);

    Serial.print(currentSteerAngle);
    Serial.println("deg: Steering left");
    delay(STEER_SMOOTHENING);
  }
}

void steerRight(int targetAngle) {
  // Guard condition
  if (targetAngle <= currentSteerAngle) return;

  while (++currentSteerAngle <= targetAngle && currentSteerAngle <= MAX_RIGHT) {
    ssm.write(currentSteerAngle);

    Serial.print(currentSteerAngle);
    Serial.println("deg: Steering right");
    delay(STEER_SMOOTHENING);
  }
}

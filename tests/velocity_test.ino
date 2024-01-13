// Test motor that controls the velocity of the car
// Objective: Increase and decrease the velocity

#include "Servo.h"

#define VELOCITY_SMOOTHENING 10
#define STATIONARY 90
#define MIN_VELOCITY 85
#define MAX_VELOCITY 95

#define DRIVE_PIN 12

Servo esc; // Velocity control

int currentVelocity = STATIONARY;

void setup() {
  esc.attach(DRIVE_PIN);


  //****************** Vehicle Initialization ********************//
  //***************** Do not change below part *****************//
  esc.write(STATIONARY);
  delay(1000);
  //***************** Do not change above part *****************//

  
  Serial.begin(9600);
}

void loop() {
  drive(MAX_VELOCITY);
  drive(MIN_VELOCITY);
  drive(STATIONARY);
  delay(10000);
}

void drive(int targetVelocity) {
  if (targetVelocity < currentVelocity) {
    decreaseVelocity(targetVelocity);
  } else {
    increaseVelocity(targetVelocity);
  }
}

void increaseVelocity(int targetVelocity) {
  if (targetVelocity <= currentVelocity) return;

  while (++currentVelocity <= targetVelocity && currentVelocity <= MAX_VELOCITY) {
    esc.write(currentVelocity);

    Serial.print(currentVelocity);
    Serial.println(": increasing velocity");
    delay(VELOCITY_SMOOTHENING);
  }
}

void decreaseVelocity(int targetVelocity) {
  if (targetVelocity >= currentVelocity) return;

  while (--currentVelocity >= targetVelocity && currentVelocity >= MIN_VELOCITY) {
    esc.write(currentVelocity);

    Serial.print(currentVelocity);
    Serial.println(": decreasing velocity");
    delay(VELOCITY_SMOOTHENING);
  }
}

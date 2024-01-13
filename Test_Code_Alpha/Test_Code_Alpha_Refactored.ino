#include <Servo.h> //define the servo library
#include <math.h>

#define trigPin1 11 // Trig for sensor 1
#define echoPin1 10 // Echo for sensor 1
#define trigPin2 9  // Trig for sensor 2
#define echoPin2 8  // Echo for sensor 2
#define trigPin3 7  // Trig for sensor 3
#define echoPin3 6  // Echo for sensor 3
#define trigPin4 5  // Trig for sensor 4
#define echoPin4 4  // Echo for sensor 4
#define trigPin5 3  // Trig for sensor 5
#define echoPin5 2  // Echo for sensor 5
#define trigPin6 23 // Trig for sensor 6
#define echoPin6 22 // Echo for sensor 6

// Steering
#define STRAIGHT 90
#define MAX_STEER 180
#define MIN_STEER 0

// Velocity
#define STATIONARY 90
#define FORWARD 97

// Tuning
#define STOP_DISTANCE 20
#define LEFT_TIGHTNESS 0.4
#define RIGHT_TIGHTNESS 0.6

Servo ssm; // Declare that ssm (steering servo motor) is a servo
Servo esc; // Declare that esc is (similar to) a servo

// Distance Calculation
float duration,
      distance;

// Vehicle Control
int steering = 90,
    velocity = 100;

// Initialization
int driving_trig = 1;

// Struct here for convenience
struct Ultrasonic {
    int trigPin;
    int echoPin;
    float distance;
};

// I added an array here for convenience to hold ultrasonic sensor pin information
// distance1 would be sensor[0].distance
// ...
// distance6 would be sensor[5].distance
Ultrasonic sensors[] = {
    Ultrasonic {trigPin1, echoPin1, 0},
    Ultrasonic {trigPin2, echoPin2, 0},
    Ultrasonic {trigPin3, echoPin3, 0},
    Ultrasonic {trigPin4, echoPin4, 0},
    Ultrasonic {trigPin5, echoPin5, 0},
    Ultrasonic {trigPin6, echoPin6, 0},
};

void setup() {
    for (Ultrasonic sensor : sensors) {
        pinMode(sensor.trigPin, OUTPUT);  //Set the trigPin as output
        pinMode(sensor.echoPin, INPUT);   //Set the echoPin as input
    }

    ssm.attach(13); // define that ssm is connected at pin 13, steering
    esc.attach(12); // define that esc is connected at pin 12, driving
    driving_trig = 1;

    //****************** Vehicle Initialization ********************//
    //***************** Do not change below part *****************//
    esc.write(90);
    delay(1000);
    //***************** Do not change above part *****************//
    Serial.begin(9600); // Starts the serial communication, Serial communications at 57600 bps
}

void loop() {
    //***************** Do not change below part *****************//
    if (driving_trig == 1) {
        setVehicle(120, 95);
        delay(1000);
        driving_trig = 10;
    }
    //***************** Do not change above part *****************//
    //*******************************************************************************************
    //******!!!!!!!!!Please don't change any code above!!!!!!!***********************************
    //******!!!!!!!!!Please don't change any code above!!!!!!!***********************************
    //*******************************************************************************************
    //*******************************************************************************************
    //*******************************************************************************************
    //******Please input the code of your approaches/algorithms in the below area*****************//

    // Calculate the distance for each ultra sonic sensor
    calculate_distances();
    // View distance information from sensors (useful for debugging)
    debug_distance();

    // Calculate distances to determine ratios
    float leftDistance = sensors[0].distance + sensors[1].distance;
    float rightDistance = sensors[4].distance + sensors[5].distance;
    float s2_s6_distance = sensors[1].distance + sensors[5].distance;
    float totalDistance = leftDistance + rightDistance;
    float frontDistance = (sensors[2].distance + sensors[3].distance) / 2;

    float ratioA = leftDistance / totalDistance;
    float ratioB = s2_s6_distance / totalDistance;

    // Demo code for safety distance to stop the vehicle
    if (frontDistance <= STOP_DISTANCE) {
        velocity = STATIONARY;
        steering = STRAIGHT;
    }
    // Greater than safety distance, run the vehicle
    else {
        velocity = FORWARD;
        steering = calculate_steering(ratioA, ratioB);
    }

    setVehicle(steering, velocity);
}

float calculate_steering(float ratioA, float ratioB) {
    float dynamicSteering;

    // Steer right (Car is angled leftward)
    if (ratioA <= LEFT_TIGHTNESS) {
        dynamicSteering = (1 - ratioA) * 180;
    }
    // Steer left (Car is close to right wall)
    else if (ratioA >= RIGHT_TIGHTNESS) {
        dynamicSteering = (1 - ratioA) * 180;
    }
    // Steer left (Car is angled rightward)
    else if (ratioB <= LEFT_TIGHTNESS) {
        dynamicSteering = (1 - ratioB) * 180;
    }
    // Steer right (Car is close to left wall)
    else if (ratioB >= RIGHT_TIGHTNESS) {
        dynamicSteering = (1 - ratioB) * 180;
    }
    else {
        dynamicSteering = STRAIGHT;
    }

    return dynamicSteering;
}

void calculate_distances() {
    FindRange(sensors[0].trigPin, sensors[0].echoPin);
    sensors[0].distance = distance;
    FindRange(sensors[1].trigPin, sensors[1].echoPin);
    sensors[1].distance = distance;
    FindRange(sensors[2].trigPin, sensors[2].echoPin);
    sensors[2].distance = distance;
    FindRange(sensors[3].trigPin, sensors[3].echoPin);
    sensors[3].distance = distance;
    FindRange(sensors[4].trigPin, sensors[4].echoPin);
    sensors[4].distance = distance;
    FindRange(sensors[5].trigPin, sensors[5].echoPin);
    sensors[5].distance = distance;
}

void debug_distance() {
    Serial.print(sensors[0].distance);
    Serial.print(" cm,  ");
    Serial.print(sensors[1].distance);
    Serial.print(" cm,  ");
    Serial.print(sensors[2].distance);
    Serial.print(" cm,  ");
    Serial.print(sensors[3].distance);
    Serial.print(" cm,  ");
    Serial.print(sensors[4].distance);
    Serial.print(" cm,  ");
    Serial.print(sensors[5].distance);
    Serial.println(" cm  ");
}

//******Please input the code of your approaches/algorithms in the above area*****************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************

//******!!!!!!!!!Please don't change any code below!!!!!!!***********************************
//******!!!!!!!!!Please don't change any code below!!!!!!!***********************************
//*******************************************************************************************
//***************** Do not change below part *****************//
void setVehicle(int s, int v) {
    //********************** Vehicle Control **********************//
    if ((s >= 0 && s <= 180) && (v >= 0 && v <= 180)) {
        ssm.write(s); // write steering value to steering servo
        esc.write(v); // write velocity value to the ESC unit
    }
    else
    {}
}

void FindRange(int trigPin, int echoPin) {
    //Trigger the sensor by sending a HIGH pulse of 10 microseconds. But, before that, give a short LOW pulse to ensure you’ll get a clean HIGH pulse
    digitalWrite(trigPin, LOW); // Send out a Low trigger
    delayMicroseconds(2); // Wait 2 ms
    digitalWrite(trigPin, HIGH); // Send out a High trigger
    delayMicroseconds(10); // Wait 10 ms
    digitalWrite(trigPin, LOW);  //Send out a Low trigger.
    duration = pulseIn(echoPin, HIGH); //Read the HIGH pulse whose duration is the time (in microseconds) from the sending of the ping to the reception of its echo off of an object.
    distance = (duration / 2) * 0.03435; //Sound speed = 0.03435cm/ct
}
//***************** Do not change above part *****************//

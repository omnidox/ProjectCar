// Group ID: 1
// Group Members: Karthik Boddu, Devin Sterling, Rafael Hidalgo, Karen Serrano
// Comprehensive Course Project 1 of CSIT-431/531

#include <Servo.h> //define the servo library
#include <math.h>

Servo ssm; // define that ssm (steering servo motor) is a servo
Servo esc; // define that esc is (similar to) a servo

/** Set echo and trig pin for each sensor **/
#define TRIG_PIN1 11 // Trig for sensor 1
#define ECHO_PIN1 10 // Echo for sensor 1
#define TRIG_PIN2 9
#define ECHO_PIN2 8
#define TRIG_PIN3 7
#define ECHO_PIN3 6
#define TRIG_PIN4 5
#define ECHO_PIN4 4
#define TRIG_PIN5 3
#define ECHO_PIN5 2
#define TRIG_PIN6 23
#define ECHO_PIN6 22

//** Tuning **//
#define LEFT_TIGHTNESS 0.40
#define RIGHT_TIGHTNESS 0.60
#define STOP_DISTANCE 50

//** Steering Control **//
#define STRAIGHT 90

//** Velocity Control **//
#define STATIONARY 90
// May have to adjust speed here
#define FORWARD 100

float duration,
    distance,
    distance1,
    distance2,
    distance3,
    distance4,
    distance5,
    distance6;

int driving_trig = 1;

void setup()
{
    pinMode(TRIG_PIN1, OUTPUT); // Set the TRIG_PIN1 as output
    pinMode(ECHO_PIN1, INPUT);  // Set the ECHO_PIN1 as input
    pinMode(TRIG_PIN2, OUTPUT); // Set the TRIG_PIN2 as output
    pinMode(ECHO_PIN2, INPUT);  // Set the ECHO_PIN2 as input
    pinMode(TRIG_PIN3, OUTPUT); // Set the TRIG_PIN3 as output
    pinMode(ECHO_PIN3, INPUT);  // Set the ECHO_PIN3 as input
    pinMode(TRIG_PIN4, OUTPUT); // Set the TRIG_PIN4 as output
    pinMode(ECHO_PIN4, INPUT);  // Set the ECHO_PIN4 as input
    pinMode(TRIG_PIN5, OUTPUT); // Set the TRIG_PIN5 as output
    pinMode(ECHO_PIN5, INPUT);  // Set the ECHO_PIN5 as input
    pinMode(TRIG_PIN6, OUTPUT); // Set the TRIG_PIN6 as output
    pinMode(ECHO_PIN6, INPUT);  // Set the ECHO_PIN6 as input

    ssm.attach(13); // define that ssm is connected at pin 12, steering
    esc.attach(12); // define that esc is connected at pin 13, driving
    driving_trig = 1;

    //****************** Vehicle Initialization ********************//
    //***************** Do not change below part *****************//
    esc.write(90);
    delay(1000);
    //***************** Do not change above part *****************//
    Serial.begin(9600); // Starts the serial communication, Serial communications at 57600 bps
}

void loop()
{
    //***************** Do not change below part *****************//
    if (driving_trig == 1)
    {
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

    // Calculate the sensor distances
    getDistances();

    // Calculate distances to determine ratios
    float leftDistance = distance1 + distance2;
    float rightDistance = distance5 + distance6;
    float frontDistance = (distance3 + distance4)/2;
    float s2_s6_distance = distance2 + distance6;
    float totalDistance = leftDistance + rightDistance;
    float ratioA = leftDistance / totalDistance;
    float ratioB = s2_s6_distance / totalDistance;

    // Default steering and velocity
    float dynamicSteering = STRAIGHT;
    float velocity = FORWARD;

    // Demo code for safety distance to stop the vehicle
    if (frontDistance <= STOP_DISTANCE)
    {
        // Stop the vehicle; set velocity to 90
        velocity = STATIONARY;
    }
    // Steer right (Car is angled leftward)
    else if (ratioA <= LEFT_TIGHTNESS)
    {
        dynamicSteering = (1 - ratioA) * 180;
    }
    // Steer left (Car is close to right wall)
    else if (ratioA >= RIGHT_TIGHTNESS)
    {
        dynamicSteering = (1 - ratioA) * 180;
    }
    // Steer left (Car is angled rightward)
    else if (ratioB <= LEFT_TIGHTNESS)
    {
        dynamicSteering = (1 - ratioB) * 180;
    }
    // Steer right (Car is close to left wall)
    else if (ratioB >= RIGHT_TIGHTNESS)
    {
        dynamicSteering = (1 - ratioB) * 180;
    }

    setVehicle(dynamicSteering, velocity);
    printDistances();
}

// Calculate the distance for each ultra sonic sensor
void getDistances() {
    FindRange(TRIG_PIN1, ECHO_PIN1);
    distance1 = distance;
    FindRange(TRIG_PIN2, ECHO_PIN2);
    distance2 = distance;
    FindRange(TRIG_PIN3, ECHO_PIN3);
    distance3 = distance;
    FindRange(TRIG_PIN4, ECHO_PIN4);
    distance4 = distance;
    FindRange(TRIG_PIN5, ECHO_PIN5);
    distance5 = distance;
    FindRange(TRIG_PIN6, ECHO_PIN6);
    distance6 = distance;
}

// Print the distance of each ultra sonic sensor
void printDistances() {
    Serial.print("[1]: ");
    Serial.print(distance1);
    Serial.print("cm, [2]: ");
    Serial.print(distance2);
    Serial.print("cm, [3]: ");
    Serial.print(distance3);
    Serial.print("cm, [4]: ");
    Serial.print(distance4);
    Serial.print("cm, [5]: ");
    Serial.print(distance5);
    Serial.println("cm, [6]: ");
    Serial.print(distance6);
}

//******Please input the code of your approaches/algorithms in the above area*****************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************

//******!!!!!!!!!Please don't change any code below!!!!!!!***********************************
//******!!!!!!!!!Please don't change any code below!!!!!!!***********************************
//*******************************************************************************************
//***************** Do not change below part *****************//
void setVehicle(int s, int v)
{
    //********************** Vehicle Control **********************//
    if ((s >= 0 && s <= 180) && (v >= 0 && v <= 180))
    {
        ssm.write(s); // write steering value to steering servo
        esc.write(v); // write velocity value to the ESC unit
    }
    else
    {
    }
}

void FindRange(int trigPin, int echoPin)
{
    // Trigger the sensor by sending a HIGH pulse of 10 microseconds. But, before that, give a short LOW pulse to ensure you’ll get a clean HIGH pulse
    digitalWrite(trigPin, LOW);          // Send out a Low trigger
    delayMicroseconds(2);                // Wait 2 ms
    digitalWrite(trigPin, HIGH);         // Send out a High trigger
    delayMicroseconds(10);               // Wait 10 ms
    digitalWrite(trigPin, LOW);          // Send out a Low trigger.
    duration = pulseIn(echoPin, HIGH);   // Read the HIGH pulse whose duration is the time (in microseconds) from the sending of the ping to the reception of its echo off of an object.
    distance = (duration / 2) * 0.03435; // Sound speed = 0.03435cm/ct
}
//***************** Do not change above part *****************//

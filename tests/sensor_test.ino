// Test ultrasonic sensors
// Objective: Cycle through each sensor and test if distance is measured

// Convenient macro to retrieve the length of an array
#define LEN(X) sizeof(X) / sizeof(*X)

#define SPEED_SOUND 0.03435

// Define some constants for ultrasonics (for convenience)
#define TRIG_PIN1 11    // Trig for sensor 1
#define ECHO_PIN1 10    // Echo for sensor 1
#define TRIG_PIN2 9     // Trig for sensor 2
#define ECHO_PIN2 8     // Echo for sensor 2
#define TRIG_PIN3 7     // Trig for sensor 3
#define ECHO_PIN3 6     // Echo for sensor 3
#define TRIG_PIN4 5     // Trig for sensor 4
#define ECHO_PIN4 4     // Echo for sensor 4
#define TRIG_PIN5 3     // Trig for sensor 5
#define ECHO_PIN5 2     // Echo for sensor 5
#define TRIG_PIN6 23    // Trig for sensor 6
#define ECHO_PIN6 22    // Echo for sensor 6

// Struct here for convenience
struct Ultrasonic {
  int trigPin;
  int echoPin;
};

// Convenience array to hold ultrasonic sensor pin information
Ultrasonic sensors[] = {
  Ultrasonic {TRIG_PIN1, ECHO_PIN1},
  Ultrasonic {TRIG_PIN2, ECHO_PIN2},
  Ultrasonic {TRIG_PIN3, ECHO_PIN3},
  Ultrasonic {TRIG_PIN4, ECHO_PIN4},
  Ultrasonic {TRIG_PIN5, ECHO_PIN5},
  Ultrasonic {TRIG_PIN6, ECHO_PIN6},
};

void setup() {
  for (Ultrasonic sensor : sensors) {
    pinMode(sensor.trigPin, OUTPUT);  //Set the trigPin as output
    pinMode(sensor.echoPin, INPUT);   //Set the echoPin as input
  }

  Serial.begin(9600); // Starts serial communication to retrieve output
}

// What we want to do here for this test is to check if each sensor
// performs as intended. So, we iterate over each one at a time and
// check its status.
void loop() {
  for (int i = 0; i < LEN(sensors); i++) {
    Ultrasonic sensor = sensors[i];
    float distance = getDistance(sensor);

    Serial.print("s");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(distance);
    Serial.print("cm,  ");
  }

  Serial.println();
}

// Returns the traversed distance
float getDistance(Ultrasonic sensor) {
  int trigPin = sensor.trigPin;
  int echoPin = sensor.echoPin;

  // Passing `LOW` here may seem redundant. However,
  // it acts as a precaution to guarantee the pin is at a
  // low voltage state for accurate and consistent results.
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Start triggering sound waves for 10μs
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  // Stop ultrasonic from emitting sound waves
  digitalWrite(trigPin, LOW);

  // Measure duration of `HIGH` pulse
  float duration = pulseIn(echoPin, HIGH);

  // Return calculated distance
  return (duration / 2) * SPEED_SOUND;
}

Initialization:

Include necessary libraries for servo and ultrasonic sensors
Define pins for each of the 8 ultrasonic sensors, servo motor, and two back motors
Set up serial communication with the computer
Set initial steering angle, steering range, and steering sensitivity
Set initial back motor speed

Functions:

getDistance(triggerPin, echoPin): Measures distance using an ultrasonic sensor and returns the distance in centimeters
adjustSteering(): Adjusts the steering angle based on the ultrasonic sensor readings and moves the car accordingly

Loop:

In an infinite loop, call the adjustSteering function repeatedly
In the adjustSteering function:

Measure the distance difference between the two front, two right, two left, and two back ultrasonic sensors
If an obstacle is detected in front left, steer right to avoid it
If an obstacle is detected in front right, steer left to avoid it
If an obstacle is detected on the right, steer left to avoid it
If an obstacle is detected on the left, steer right to avoid it
If an obstacle is detected behind, move the car backward for a short period of time
Otherwise, move the car forward


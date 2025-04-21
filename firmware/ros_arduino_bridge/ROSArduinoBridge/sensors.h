/* Functions for various sensor types */

// Default ultrasonic pins from custom_test.ino
#define ULTRASONIC_TRIG_PIN A1  // outputPin in custom_test
#define ULTRASONIC_ECHO_PIN A0  // inputPin in custom_test

float microsecondsToCm(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per cm.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

// Updated to match the implementation in custom_test.ino
long Ping(int pin) {
  long duration, distance_cm;

  // Using the custom_test.ino implementation
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  
  duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
  distance_cm = duration / 5.8 / 10; // Match custom_test calculation
  
  return distance_cm;
}

// Function to set up the ultrasonic sensor pins
void setupUltrasonicPins() {
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
}


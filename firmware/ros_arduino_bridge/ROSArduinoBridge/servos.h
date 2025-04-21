#ifndef SERVOS_H
#define SERVOS_H

// Only using 1 servo for the ultrasonic sensor, as in custom_test.ino
#define N_SERVOS 1

// Servo sweep delay
int stepDelay [N_SERVOS] = { 20 }; // ms - moderate speed for smooth movement

// Pins - using pin 5 as in custom_test.ino
byte servoPins [N_SERVOS] = { 5 };

// Initial Position - center position
byte servoInitPosition [N_SERVOS] = { 90 }; // [0, 180] degrees


class SweepServo
{
  public:
    SweepServo();
    void initServo(
        int servoPin,
        int stepDelayMs,
        int initPosition);
    void doSweep();
    void setTargetPosition(int position);
    Servo getServo();

  private:
    Servo servo;
    int stepDelayMs;
    int currentPositionDegrees;
    int targetPositionDegrees;
    long lastSweepCommand;
};

SweepServo servos [N_SERVOS];

#endif

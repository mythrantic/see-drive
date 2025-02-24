#include <Servo.h>
int pinLB = 6;        // define pin6 as left back connect with IN1
int pinLF = 9;        // define pin9 as left forward connect with IN2
int pinRB = 10;       // define pin10 as right back connect with IN3
int pinRF = 11;       // define pin11 as right back connect with IN4
int inputPin = A0;    // define ultrasonic receive pin (Echo)
int outputPin = A1;   // define ultrasonic send pin(Trig)
int Fspeedd = 0;      // forward distance
int Rspeedd = 0;      // right distance
int Lspeedd = 0;      // left distance
int directionn = 0;   //
Servo myservo;        // new myservo
int delay_time = 250; // set stable time
int Fgo = 8;
int Rgo = 6;
int Lgo = 4;
int Bgo = 2;
// forward
// turn right
// turn left
// back
void setup()
{
    Serial.begin(9600);
    pinMode(pinLB, OUTPUT);
    pinMode(pinLF, OUTPUT);
    pinMode(pinRB, OUTPUT);
    pinMode(pinRF, OUTPUT);
    pinMode(inputPin, INPUT);
    pinMode(outputPin, OUTPUT);
    myservo.attach(5); // define the servo pin(PWM)
}
void advance(int a) // forward
{
    digitalWrite(pinRB, LOW);
    digitalWrite(pinRF, HIGH);
    digitalWrite(pinLB, LOW);
    digitalWrite(pinLF, HIGH);
    delay(a * 15);
}
void turnR(int d) // turn right
{
    digitalWrite(pinRB, LOW);
    digitalWrite(pinRF, HIGH);
    digitalWrite(pinLB, HIGH);
    digitalWrite(pinLF, LOW);
    delay(d * 50);
}
void turnL(int e) // turn left
{
    digitalWrite(pinRB, HIGH);
    digitalWrite(pinRF, LOW);
    digitalWrite(pinLB, LOW);
    digitalWrite(pinLF, HIGH);
    delay(e * 50);
}
void stopp(int f) // stop
{
    digitalWrite(pinRB, HIGH);
    digitalWrite(pinRF, HIGH);
    digitalWrite(pinLB, HIGH);
    digitalWrite(pinLF, HIGH);
    delay(f * 100);
}
void back(int g) // back
{
    digitalWrite(pinRB, HIGH);
    digitalWrite(pinRF, LOW);
    digitalWrite(pinLB, HIGH);
    digitalWrite(pinLF, LOW);
    delay(g * 300);
}
void detection() // test the distance of different direction
{
    int delay_time = 250; //
    delay(200);
    ask_pin_F();      // read forward distance
    if (Fspeedd < 10) // if distance less then 10
    {
        stopp(1);
        back(2);
    }
    if (Fspeedd < 25) // if distance less then 10
    {
        stopp(1);
        ask_pin_L();
        delay(delay_time);
        ask_pin_R();
        delay(delay_time);
        if (Lspeedd > Rspeedd) // if left distance more than right distance
        {
            directionn = Rgo;
        }
        if (Lspeedd <= Rspeedd) // if left distance not more than right
        // distance
        {
            directionn = Lgo;
        }
        // if left if (Lspeedd < 10 && Rspeedd < 10) distance and right
        // distance both less than 10
        {
            directionn = Bgo;
        }
    }
    else
    {
        directionn = Fgo; // forward go
    }
}
void ask_pin_F() // test forward distance
{
    myservo.write(90);
    digitalWrite(outputPin, LOW);
    delayMicroseconds(2);
    digitalWrite(outputPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(outputPin, LOW);
    float Fdistance = pulseIn(inputPin, HIGH);
    Fdistance = Fdistance / 5.8 / 10;
    Serial.print("F distance:");
    Serial.println(Fdistance);
    Fspeedd = Fdistance;
}
void ask_pin_L() // test left distance
{
    myservo.write(150);
    delay(delay_time);
    digitalWrite(outputPin, LOW);
    delayMicroseconds(2);
    digitalWrite(outputPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(outputPin, LOW);
    float Ldistance = pulseIn(inputPin, HIGH);
    Ldistance = Ldistance / 5.8 / 10;
    Serial.print("L distance:");
    Serial.println(Ldistance);
    Lspeedd = Ldistance;
}
void ask_pin_R() // test right distance
{
    myservo.write(20);
    delay(delay_time);
    digitalWrite(outputPin, LOW);
    delayMicroseconds(2);
    digitalWrite(outputPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(outputPin, LOW);
    float Rdistance = pulseIn(inputPin, HIGH);
    Rdistance = Rdistance / 5.8 / 10;
    Serial.print("R distance:");
    Serial.println(Rdistance);
    Rspeedd = Rdistance;
}
void loop()
{
    myservo.write(90);
    detection();
    if (directionn == 2)
    {
        back(3);
        turnL(2);
        Serial.print(" Reverse ");
    }
    if (directionn == 6)
    {
        back(1);
        turnR(6);
        Serial.print(" Right ");
    }
    if (directionn == 4)
    {
        back(1);
        turnL(6);
        Serial.print(" Left ");
    }
    if (directionn == 8)
    {
        advance(1);
        Serial.print(" Advance ");
        Serial.print(" ");
    }
}

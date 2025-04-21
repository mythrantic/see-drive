/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
#ifdef ARDUINO_ENC_COUNTER
  // Choose pins that don't conflict with your car's setup
  // Avoiding pins 5,6,9,10,11,A0,A1 which are already in use
  #define LEFT_ENC_PIN_A 2   // Digital pin 2
  #define LEFT_ENC_PIN_B 3   // Digital pin 3
  
  // Using digital pins for encoders instead of analog pins 
  // that are used for the ultrasonic sensor
  #define RIGHT_ENC_PIN_A 7  // Digital pin 7
  #define RIGHT_ENC_PIN_B 8  // Digital pin 8
  
  // Define if the right encoder has direct interrupt support
  // #define RIGHT_ENCODER_INTERRUPT
  
  // Interrupt handlers (now declared as regular functions)
  void leftEncoderISR();
  void rightEncoderISR();
  void pollRightEncoder();
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();


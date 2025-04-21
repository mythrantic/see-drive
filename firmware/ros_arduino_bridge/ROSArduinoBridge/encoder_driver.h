/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
#ifdef ARDUINO_ENC_COUNTER
  // Using Arduino pin numbers instead of AVR port names for better compatibility
  // Digital pins for left encoder
  #define LEFT_ENC_PIN_A 2  // Digital pin 2
  #define LEFT_ENC_PIN_B 3  // Digital pin 3
  
  // Analog pins (or digital pin numbers) for right encoder
  #define RIGHT_ENC_PIN_A A4  // Analog pin A4
  #define RIGHT_ENC_PIN_B A5  // Analog pin A5
  
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


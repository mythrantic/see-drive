/* *************************************************************
   Encoder definitions
   ************************************************************ */
   
#ifdef USE_BASE

// Remove AVR-specific interrupt header
// #include <avr/interrupt.h>

#ifdef ROBOGAIA
  /* The Robogaia Mega Encoder shield */
  #include "MegaEncoderCounter.h"

  /* Create the encoder shield object */
  MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return encoders.YAxisGetCount();
    else return encoders.XAxisGetCount();
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT) return encoders.YAxisReset();
    else return encoders.XAxisReset();
  }
#elif defined(ARDUINO_ENC_COUNTER)
  volatile long left_enc_pos = 0L;
  volatile long right_enc_pos = 0L;
  static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup table
  
  // Store previous encoder states
  volatile uint8_t left_enc_state = 0;
  volatile uint8_t right_enc_state = 0;
  
  /* Interrupt service routine for the LEFT encoder */
  void leftEncoderISR() {
    static uint8_t enc_last = 0;
    
    // Read the current state of both encoder pins
    uint8_t current_state = (digitalRead(LEFT_ENC_PIN_B) << 1) | digitalRead(LEFT_ENC_PIN_A);
    
    // Update encoder position
    enc_last <<= 2;  // Shift previous state
    enc_last |= current_state;  // Add current state
    
    left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }
  
  /* Interrupt service routine for the RIGHT encoder */
  void rightEncoderISR() {
    static uint8_t enc_last = 0;
    
    // Read the current state of both encoder pins
    uint8_t current_state = (digitalRead(RIGHT_ENC_PIN_B) << 1) | digitalRead(RIGHT_ENC_PIN_A);
    
    // Update encoder position
    enc_last <<= 2;  // Shift previous state
    enc_last |= current_state;  // Add current state
    
    right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }
  
  /* Function to manually poll the right encoder if direct interrupts aren't available */
  void pollRightEncoder() {
    uint8_t current_state = (digitalRead(RIGHT_ENC_PIN_B) << 1) | digitalRead(RIGHT_ENC_PIN_A);
    
    if (current_state != right_enc_state) {
      uint8_t combined = (right_enc_state << 2) | current_state;
      right_enc_pos += ENC_STATES[combined & 0x0f];
      right_enc_state = current_state;
    }
  }
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    // If using polling for right encoder, check it each time we read
    #ifndef RIGHT_ENCODER_INTERRUPT
    pollRightEncoder();
    #endif
    
    if (i == LEFT) return left_enc_pos;
    else return right_enc_pos;
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      left_enc_pos=0L;
      return;
    } else { 
      right_enc_pos=0L;
      return;
    }
  }
#else
  #error A encoder driver must be selected!
#endif

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

#endif


/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  // Updated pins to match custom_test.ino
  #define LEFT_MOTOR_BACKWARD  6   // pinLB in custom_test
  #define LEFT_MOTOR_FORWARD   9   // pinLF in custom_test
  #define RIGHT_MOTOR_FORWARD  11  // pinRF in custom_test
  #define RIGHT_MOTOR_BACKWARD 10  // pinRB in custom_test
  
  // Remove the enable pins since they don't appear to be used in custom_test.ino
  // If your L298N module needs these, you can add them back
  // #define RIGHT_MOTOR_ENABLE 12
  // #define LEFT_MOTOR_ENABLE 13
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);

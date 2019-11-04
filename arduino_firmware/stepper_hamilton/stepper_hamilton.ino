#include <AccelStepper.h>

AccelStepper LeftBackWheel(1, 2, 5);   // (Type:driver, STEP, DIR) - Stepper1
AccelStepper LeftFrontWheel(1, 4, 7);  // Stepper2
AccelStepper RightBackWheel(1, 3, 6);  // Stepper3
AccelStepper RightFrontWheel(1, 12, 13); // Stepper4


void setup() {
  LeftFrontWheel.setMaxSpeed(3000);
  LeftBackWheel.setMaxSpeed(3000);
  RightFrontWheel.setMaxSpeed(3000);
  RightBackWheel.setMaxSpeed(3000);

  LeftFrontWheel.setSpeed(1500);
  LeftBackWheel.setSpeed(1500);
  RightFrontWheel.setSpeed(1500);
  RightBackWheel.setSpeed(1500);
}
void loop() {
  // Execute the steps
  LeftFrontWheel.runSpeed();
  LeftBackWheel.runSpeed();
  RightFrontWheel.runSpeed();
  RightBackWheel.runSpeed();
  
}

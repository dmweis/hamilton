#include <AccelStepper.h>


AccelStepper RightFrontWheel(1, 2, 5);
AccelStepper LeftFrontWheel(1, 4, 7);
AccelStepper RightBackWheel(1, 3, 6);
AccelStepper LeftBackWheel(1, 12, 13);

void setup()
{

  LeftFrontWheel.setMaxSpeed(300);
  LeftBackWheel.setMaxSpeed(300);
  RightFrontWheel.setMaxSpeed(300);
  RightBackWheel.setMaxSpeed(300);

  LeftFrontWheel.setSpeed(150);
  LeftBackWheel.setSpeed(-150);
  RightFrontWheel.setSpeed(150);
  RightBackWheel.setSpeed(-150);
}
void loop()
{
  LeftFrontWheel.runSpeed();
  LeftBackWheel.runSpeed();
  RightFrontWheel.runSpeed();
  RightBackWheel.runSpeed();
}

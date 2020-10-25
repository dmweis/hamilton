#include <Arduino.h>
#include "BasicStepperDriver.h"

#define MOTOR_STEPS 200
#define RPM 120

#define MICROSTEPS 1

BasicStepperDriver stepper1(MOTOR_STEPS, 5, 2);
BasicStepperDriver stepper2(MOTOR_STEPS, 7, 4);
BasicStepperDriver stepper3(MOTOR_STEPS, 6, 3);
BasicStepperDriver stepper4(MOTOR_STEPS, 13, 12);


void setup() {
    stepper1.begin(RPM, MICROSTEPS);
    stepper2.begin(RPM, MICROSTEPS);
    stepper3.begin(RPM, MICROSTEPS);
    stepper4.begin(RPM, MICROSTEPS);
}

void loop() {
  
    stepper1.enable();
    stepper2.enable();
    stepper3.enable();
    stepper4.enable();
  
    stepper1.rotate(360);
    stepper2.rotate(360);
    stepper3.rotate(360);
    stepper4.rotate(360);

    stepper1.move(-MOTOR_STEPS*MICROSTEPS);
    stepper2.move(-MOTOR_STEPS*MICROSTEPS);
    stepper3.move(-MOTOR_STEPS*MICROSTEPS);
    stepper4.move(-MOTOR_STEPS*MICROSTEPS);

    delay(1000);
}
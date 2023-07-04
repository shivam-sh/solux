#include <AccelStepper.h>

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::FULL4WIRE, 38, 9, 8, 33);

void setup()
{  
  stepper.setMaxSpeed(300);
  stepper.setAcceleration(100);
  stepper.moveTo(1024);
}

void loop()
{
    // If at the end of travel go to the other end
    if (stepper.distanceToGo() == 0)
      stepper.moveTo(-stepper.currentPosition());

    stepper.run();
}
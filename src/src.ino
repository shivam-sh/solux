#include <AccelStepper.h>

typedef struct StepperInfo
{
  AccelStepper *stepper;
  int maxDegLower;
  int maxDegUpper;
};

typedef struct ButtonInfo
{
  int pin;
  bool pressed;
  int num_pressed;
};

AccelStepper azimuth(AccelStepper::FULL4WIRE, 38, 9, 8, 33);
AccelStepper elevation(AccelStepper::FULL4WIRE, 10, 3, 1, 7);

ButtonInfo az_button = { 0, false, 0 };

StepperInfo az_info = { &azimuth, 0, 360 };
StepperInfo el_info = { &elevation, 0, 180 };

// full rotation, from https://lastminuteengineers.com/28byj48-stepper-motor-arduino-tutorial/
const long fullRotation = 2038;
const int lowerElevationLimit = 0;
const int upperElevationLimit = 180;

void IRAM_ATTR buttonIsr() {
  az_button.num_pressed++;
  az_button.pressed = true;
}

void setup() {
   // TODO (npalmar): Implement azimuth calibration
  pinMode(az_button.pin, INPUT);
  attachInterrupt(az_button.pin, buttonIsr, FALLING);
   //  azimuth.setCurrentPosition(0);

  azimuth.setMaxSpeed(200);
  azimuth.setAcceleration(20);
  elevation.setMaxSpeed(200);
  elevation.setAcceleration(20);

  resetElevation(elevation);

  Serial.begin(9600);
}

void loop() {
    azimuth.run();
    elevation.run();

//    Serial.println(elevation.currentPosition());
//    Serial.println(digitalRead(buttonPin));

    
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        Serial.println(input);
    
        if (input.startsWith("az")) {
        float degrees = input.substring(2).toFloat();
        long steps = degreesToSteps(degrees);
//        azimuth.moveTo(steps);
        moveTo(az_info, degrees);
        }
        else if (input.startsWith("el")) {
        float degrees = input.substring(2).toFloat();
        long steps = degreesToSteps(degrees);
//        elevation.moveTo(steps);
        moveTo(el_info, degrees);
        }
        else if (input.startsWith("reset")) {
        resetElevation(elevation);
        }
    }

    if (az_button.pressed && az_button.num_pressed == 1)
    {
      Serial.println("Button pressed once");
      // set the current position to 0
      azimuth.setCurrentPosition(0);
      az_button.pressed = false;
    }
    else if (az_button.pressed)
    {
      Serial.println("Button pressed");
      az_button.pressed = false;
    }
}

long degreesToSteps(float degrees) {
    return (long) (degrees / 360 * fullRotation);
}

// create new definition for stepper move and moveTo 
// absolute position control with limits
void moveTo(StepperInfo &stepper_info, long angleDeg)
{
   if (angleDeg > stepper_info.maxDegUpper)
   {
    angleDeg = stepper_info.maxDegUpper;
   }
   else if (angleDeg < stepper_info.maxDegLower)
   {
    angleDeg = stepper_info.maxDegLower;
   }
   long angleSteps = degreesToSteps(angleDeg);

   stepper_info.stepper->moveTo(angleSteps);
}

// relative position control with limits
void move(StepperInfo &stepper_info, long angleDeg)
{
  // 
}

void resetElevation(AccelStepper &stepper) {
    bool maxReached = false;
    bool foundZero = false;
    const int stopToZeroDistDeg = 18;

    stepper.move(-(fullRotation / 2 +  2 * degreesToSteps(stopToZeroDistDeg)));

    while (!foundZero) {
        stepper.run();

        if (stepper.distanceToGo() == 0) {
            if (!maxReached){   
                maxReached = true;
                stepper.move(degreesToSteps(stopToZeroDistDeg));
            }
            else {
                foundZero = true;
            }
        }
    }

    stepper.setCurrentPosition(0);
}

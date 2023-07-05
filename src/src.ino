#include <AccelStepper.h>

AccelStepper azimuth(AccelStepper::FULL4WIRE, 38, 9, 8, 33);
AccelStepper elevation(AccelStepper::FULL4WIRE, 10, 3, 1, 7);

long fullRotation = 2048;

void setup() {
  azimuth.setMaxSpeed(200);
  azimuth.setAcceleration(20);
  elevation.setMaxSpeed(200);
  elevation.setAcceleration(20);

  resetElevation(elevation);
  azimuth.setCurrentPosition(0);

  Serial.begin(9600);
}

void loop() {
    azimuth.run();
    elevation.run();
    
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        Serial.println(input);
    
        if (input.startsWith("az")) {
        float degrees = input.substring(2).toFloat();
        long steps = degreesToSteps(degrees);
        azimuth.moveTo(steps);
        }
        else if (input.startsWith("el")) {
        float degrees = input.substring(2).toFloat();
        long steps = degreesToSteps(degrees);
        elevation.moveTo(steps);
        }
        else if (input.startsWith("reset")) {
        resetElevation(elevation);
        }
    }
}

long degreesToSteps(float degrees) {
    return (long) (degrees / 360 * fullRotation);
}

void resetElevation(AccelStepper &stepper) {
    bool maxReached = false;
    bool foundZero = false;
    long stopToZeroDist = 50;

    stepper.move(-(fullRotation / 2 +  2 * stopToZeroDist));

    while (!foundZero) {
        stepper.run();

        if (stepper.distanceToGo() == 0) {
            if (!maxReached){   
                maxReached = true;
                stepper.move(stopToZeroDist);
            }
            else {
                foundZero = true;
            }
        }
    }

    stepper.setCurrentPosition(0);
}
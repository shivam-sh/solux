#include <AccelStepper.h>

typedef struct StepperInfo {
    AccelStepper *stepper;
    int maxDegLower;
    int maxDegUpper;
};

typedef struct ButtonInfo {
    int pin;
    bool pressed;
    int num_pressed;
};

AccelStepper azimuth(AccelStepper::FULL4WIRE, 38, 9, 8, 33);
AccelStepper elevation(AccelStepper::FULL4WIRE, 10, 3, 1, 7);

ButtonInfo az_button = {11, false, 0};

StepperInfo az_info = {&azimuth, 0, 360};
StepperInfo el_info = {&elevation, 0, 180};

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

    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        Serial.println(input);

        if (input.startsWith("raz")) {
            float degrees = input.substring(3).toFloat();
            move(az_info, degrees);
        } else if (input.startsWith("rel")) {
            float degrees = input.substring(3).toFloat();
            move(el_info, degrees);
        } else if (input.startsWith("az")) {
            float degrees = input.substring(2).toFloat();
            moveTo(az_info, degrees);
        } else if (input.startsWith("el")) {
            float degrees = input.substring(2).toFloat();
            moveTo(el_info, degrees);
        } else if (input.startsWith("reset")) {
            resetElevation(elevation);
        }
    }

    if (az_button.pressed && az_button.num_pressed == 1) {
        Serial.println("Button pressed once");
        azimuth.setCurrentPosition(0);
        az_button.pressed = false;
    } else if (az_button.pressed) {
        Serial.println("Button pressed");
        az_button.pressed = false;
    }
}

//
// Helper functions
//

/// @brief Converts degrees to steps for the stepper motor
long degreesToSteps(float degrees) { return (long)(degrees / 360 * fullRotation); }

/// @brief Converts steps to degrees for the stepper motor
float stepsToDegrees(long steps) { return (float)( (float) steps / fullRotation * 360); }

/// @brief Moves the stepper motor to the given angle
/// @param stepper_info The stepper motor to move
/// @param angleDeg The angle to move the stepper motor to in degrees
void moveTo(StepperInfo &stepper_info, long angleDeg) {
    if (angleDeg > stepper_info.maxDegUpper) {
        angleDeg = stepper_info.maxDegUpper;
    } else if (angleDeg < stepper_info.maxDegLower) {
        angleDeg = stepper_info.maxDegLower;
    }
    long steps = degreesToSteps(angleDeg);

    stepper_info.stepper->moveTo(steps);
}

/// @brief Moves the stepper motor relateive to its current position by the given angle
/// @param stepper_info The stepper motor to move
/// @param angleDeg The angle to move the stepper motor by in degrees
void move(StepperInfo &stepper_info, long angleDeg) {
    float currentAngle = stepsToDegrees(stepper_info.stepper->currentPosition());
    angleDeg = currentAngle + angleDeg;

    moveTo(stepper_info, angleDeg);
}

/// @brief Resets the elevation stepper motor to the zero position
/// @param stepper The stepper motor to reset
void resetElevation(AccelStepper &stepper) {
    bool maxReached = false;
    bool foundZero = false;
    const int stopToZeroDistDeg = 18;

    stepper.move(-(fullRotation / 2 + 2 * degreesToSteps(stopToZeroDistDeg)));

    while (!foundZero) {
        stepper.run();

        if (stepper.distanceToGo() == 0) {
            if (!maxReached) {
                maxReached = true;
                stepper.move(degreesToSteps(stopToZeroDistDeg));
            } else {
                foundZero = true;
            }
        }
    }

    stepper.setCurrentPosition(0);
}

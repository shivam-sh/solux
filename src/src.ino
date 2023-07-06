#include <AccelStepper.h>

typedef struct SensorCluster {
    int num_sensors;
    int sensors[];
};

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

typedef struct Position {
    float azimuth;
    float elevation;
};

AccelStepper azimuth(AccelStepper::FULL4WIRE, 38, 9, 8, 33);
AccelStepper elevation(AccelStepper::FULL4WIRE, 10, 3, 1, 7);

ButtonInfo az_button = {11, false, 0};

StepperInfo az_info = {&azimuth, 0, 360};
StepperInfo el_info = {&elevation, 0, 180};

SensorCluster coarseSensor = {5, {5, 6, 12, 14, 18}};
// SensorCluster fineSensor = {4, {_, _, _, _}};

// full rotation, from https://lastminuteengineers.com/28byj48-stepper-motor-arduino-tutorial/
const long fullRotation = 2038;
const int lowerElevationLimit = 0;
const int upperElevationLimit = 180;

void IRAM_ATTR buttonIsr() {
    az_button.num_pressed++;
    az_button.pressed = true;
}

void setup() {
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    delay(500);
    // TODO (npalmar): Implement azimuth calibration
    pinMode(az_button.pin, INPUT);
    attachInterrupt(az_button.pin, buttonIsr, FALLING);

    azimuth.setMaxSpeed(200);
    azimuth.setAcceleration(20);
    elevation.setMaxSpeed(200);
    elevation.setAcceleration(20);

    resetElevation(elevation);

    Serial.begin(9600);

    digitalWrite(13, LOW);
}

void loop() {
    azimuth.run();
    elevation.run();

    int *coarseReadings = getSensorReadings(coarseSensor);
    Position angleEstimate = getAngleEstimate(coarseReadings);

    if (azimuth.distanceToGo() == 0 && elevation.distanceToGo() == 0) {
        azimuth.disableOutputs();
        elevation.disableOutputs();

        Serial.printf("%d\t\t%d\t\t%d\t\t%d\t\t%d\t\tAz:%f\t\tEl:%f\n", coarseReadings[0], coarseReadings[1],
                      coarseReadings[2], coarseReadings[3], coarseReadings[4], angleEstimate.azimuth,
                      angleEstimate.elevation);

        // azimuth.stop();
        // moveTo(az_info, angleEstimate.azimuth);
        // elevation.stop();
        // moveTo(el_info, angleEstimate.elevation);
    }
    delay(50);

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

/// @brief Gets sensor readings from the given sensor cluster
/// @param sensor_cluster The sensor cluster to read from
/// @return The sensor readings
int *getSensorReadings(SensorCluster &sensor_cluster) {
    int *sensor_readings = new int[sensor_cluster.num_sensors];

    for (int i = 0; i < sensor_cluster.num_sensors; i++) {
        sensor_readings[i] = analogRead(sensor_cluster.sensors[i]);
    }

    return sensor_readings;
}

/// @brief Gets an angle estimate from coarse sensor readings (elevation and azimuth)
/// @param coarse_readings The coarse sensor readings
/// @return The angle estimate
Position getAngleEstimate(int *coarse_readings) {
    Position angle_estimate = {0, 0};

    // find the two highest readings
    uint8_t highest_horizontal_index = 1;
    uint8_t second_highest_horizontal_index = 1;
    uint8_t lowest_horizontal_index = 0;

    for (int i = 1; i < coarseSensor.num_sensors; i++) {
        if (coarse_readings[i] > coarse_readings[highest_horizontal_index]) {
            second_highest_horizontal_index = highest_horizontal_index;
            highest_horizontal_index = i;
        } else if (coarse_readings[i] > coarse_readings[second_highest_horizontal_index]) {
            second_highest_horizontal_index = i;
        } else if (coarse_readings[i] < coarse_readings[lowest_horizontal_index]) {
            lowest_horizontal_index = i;
        }
    }

    if (second_highest_horizontal_index == highest_horizontal_index) {
        second_highest_horizontal_index = second_highest_horizontal_index + 1 % coarseSensor.num_sensors + 1;
    }

    // azimuth
    int highest_reading = coarse_readings[highest_horizontal_index] > coarse_readings[0]
                              ? coarse_readings[highest_horizontal_index]
                              : coarse_readings[0];
    int lowest_reading = coarse_readings[lowest_horizontal_index] < coarse_readings[0]
                             ? coarse_readings[lowest_horizontal_index]
                             : coarse_readings[0];
    int diff = highest_reading - lowest_reading;

    if (diff < 100) {
        return angle_estimate;
    }

    float normalized_readings[5] = {
        (float)(coarse_readings[0] - lowest_reading) / diff, (float)(coarse_readings[1] - lowest_reading) / diff,
        (float)(coarse_readings[2] - lowest_reading) / diff, (float)(coarse_readings[3] - lowest_reading) / diff,
        (float)(coarse_readings[4] - lowest_reading) / diff,
    };

    Serial.printf("%.2f  \t\t%.2f  \t\t%.2f  \t\t%.2f  \t\t%.2f\n", normalized_readings[0], normalized_readings[1],
                  normalized_readings[2], normalized_readings[3], normalized_readings[4]);

    int avg_horizontal_peak =
        (normalized_readings[highest_horizontal_index] + normalized_readings[second_highest_horizontal_index]);

    if (highest_horizontal_index == 1 && second_highest_horizontal_index == 2 ||
        highest_horizontal_index == 2 && second_highest_horizontal_index == 1) {
        angle_estimate.azimuth = map(normalized_readings[1] - normalized_readings[0], -1, 1, 0, 90);
    } else if (highest_horizontal_index == 2 && second_highest_horizontal_index == 3 ||
               highest_horizontal_index == 3 && second_highest_horizontal_index == 2) {
        angle_estimate.azimuth = map(normalized_readings[2] - normalized_readings[1], -1, 1, 90, 180);
    } else if (highest_horizontal_index == 3 && second_highest_horizontal_index == 4 ||
               highest_horizontal_index == 4 && second_highest_horizontal_index == 3) {
        angle_estimate.azimuth = map(normalized_readings[3] - normalized_readings[2], -1, 1, 180, 270);
    } else if (highest_horizontal_index == 1 && second_highest_horizontal_index == 4 ||
               highest_horizontal_index == 4 && second_highest_horizontal_index == 1) {
        angle_estimate.azimuth = map(normalized_readings[1] - normalized_readings[4], -1, 1, 270, 360);
    }

    // elevation
    int elevation_reading = normalized_readings[0];
    angle_estimate.elevation = map(elevation_reading - avg_horizontal_peak, -1, 1, 0, 90);

    // Serial.printf("elevation: %d\t\tAvg max: %d\n", elevation_reading, avg_horizontal_peak);

    return angle_estimate;
}

/// @brief Converts degrees to steps for the stepper motor
long degreesToSteps(float degrees) { return (long)(degrees / 360 * fullRotation); }

/// @brief Converts steps to degrees for the stepper motor
float stepsToDegrees(long steps) { return (float)((float)steps / fullRotation * 360); }

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

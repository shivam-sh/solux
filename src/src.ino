#include <AccelStepper.h>

typedef enum {
    AZ,
    EL,
    // TODO: add the fine sensor types here
} SensType;

typedef struct PhotoTransistor {
    SensType type;
    int pin;
    int azPosition;
    int elPosition;
    int intensity;
};

typedef struct SensorCluster {
    int num_sensors;
    PhotoTransistor sensors[];
};

typedef struct SafeStepper {
    AccelStepper *stepper;
    int maxDegLower;
    int maxDegUpper;
};

typedef struct ButtonInfo {
    int pin;
    bool pressed;
    int numPressed;
};

typedef struct Position {
    float azimuth;
    float elevation;
};

AccelStepper azimuth(AccelStepper::FULL4WIRE, 38, 9, 8, 33);
AccelStepper elevation(AccelStepper::FULL4WIRE, 10, 3, 1, 7);

ButtonInfo azButton = {11, false, 0};

SafeStepper azSafe = {&azimuth, 0, 360};
SafeStepper elSafe = {&elevation, 0, 90};

SensorCluster coarseSensor = {
    5, {{EL, 6, -1, 90, 0}, {AZ, 12, 0, 0, 0}, {AZ, 14, 90, 0, 0}, {AZ, 18, 180, 0, 0}, {AZ, 17, 270, 0, 0}}};

// full rotation, from https://lastminuteengineers.com/28byj48-stepper-motor-arduino-tutorial/
const long fullRotation = 2038;
const int lowerElevationLimit = 0;
const int upperElevationLimit = 180;

void IRAM_ATTR buttonIsr() {
    azButton.numPressed++;
    azButton.pressed = true;
}

void setup() {
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    delay(500);
    // TODO (npalmar): Implement azimuth calibration
    pinMode(azButton.pin, INPUT);
    attachInterrupt(azButton.pin, buttonIsr, FALLING);

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

    // TODO (npalmar): change sensor readings to take the average or median over a 10 second period (avoids noise/outliers)
    // getSensorReadings(coarseSensor);
    // Position coarseAngleEstimate = getCoarseAngleEstimate(coarseSensor);

    // Serial.printf("%d\t\t%d\t\t%d\t\t%d\t\t%d\t\tAz:%f\t\tEl:%f\n", coarseSensor.sensors[0].intensity,
    //                   coarseSensor.sensors[1].intensity, coarseSensor.sensors[2].intensity,
    //                   coarseSensor.sensors[3].intensity, coarseSensor.sensors[4].intensity, coarseAngleEstimate.azimuth,
    //                   coarseAngleEstimate.elevation);

    if (azSafe.stepper->distanceToGo() == 0 && elSafe.stepper->distanceToGo() == 0) {
        Position rollingAngleEstimates[50] = {};

        for (int i = 0; i < 50; i++) {
            getSensorReadings(coarseSensor);
            rollingAngleEstimates[i] = getCoarseAngleEstimate(coarseSensor);
            delay(20);
        }

        Position averageAngleEstimate = {0, 0};

        for (int i = 0; i < 50; i++) {
            averageAngleEstimate.azimuth += rollingAngleEstimates[i].azimuth;
            averageAngleEstimate.elevation += rollingAngleEstimates[i].elevation;
        }

        averageAngleEstimate.azimuth /= 50;
        averageAngleEstimate.elevation /= 50;

        Serial.printf("%d\t\t%d\t\t%d\t\t%d\t\t%d\t\tAz:%f\t\tEl:%f\n", 0, 0, 0, 0, 0, averageAngleEstimate.azimuth,
                      averageAngleEstimate.elevation);

        // azimuth.stop();
        // moveTo(azSafe, coarseAngleEstimate.azimuth);
        elevation.stop();
        moveTo(elSafe, averageAngleEstimate.elevation);
    }

    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        Serial.println(input);

        if (input.startsWith("raz")) {
            float degrees = input.substring(3).toFloat();
            move(azSafe, degrees);
        } else if (input.startsWith("rel")) {
            float degrees = input.substring(3).toFloat();
            move(elSafe, degrees);
        } else if (input.startsWith("az")) {
            float degrees = input.substring(2).toFloat();
            moveTo(azSafe, degrees);
        } else if (input.startsWith("el")) {
            float degrees = input.substring(2).toFloat();
            moveTo(elSafe, degrees);
        } else if (input.startsWith("reset")) {
            resetElevation(elevation);
        } else if (input.startsWith("dis")) {
            azimuth.disableOutputs();
            elevation.disableOutputs();
        }
    }

    if (azButton.pressed && azButton.numPressed == 1) {
        Serial.println("Button pressed once");
        azimuth.setCurrentPosition(0);
        azButton.pressed = false;
    } else if (azButton.pressed) {
        Serial.println("Button pressed");
        azButton.pressed = false;
    }
}

//
// Helper functions
//

/// @brief Gets sensor readings from the given sensor cluster
/// @param sensor_cluster The sensor cluster to read from and write values to
void getSensorReadings(SensorCluster &sensor_cluster) {
    for (int i = 0; i < sensor_cluster.num_sensors; i++) {
        sensor_cluster.sensors[i].intensity = analogRead(sensor_cluster.sensors[i].pin);
    }
}

// compare function for qsort
/// @brief Sorts the sensor readings in descending order of intensity but with the elevation sensor at the end
/// @param cmp1 The first sensor reading
/// @param cmp2 The second sensor reading
int sort_desc(const void *cmp1, const void *cmp2) {
    PhotoTransistor p1 = (*(PhotoTransistor *)cmp1);
    PhotoTransistor p2 = (*(PhotoTransistor *)cmp2);

    if (p1.type == EL) {
        return 1;
    } else if (p2.type == EL) {
        return -1;
    }

    return p2.intensity - p1.intensity;
}

/// @brief Linear interpolation between two values
int lerp(int a, int b, float t) { return a + t * (b - a); }

/// @brief Gets an angle estimate from coarse sensor readings (elevation and azimuth)
/// @param coarseSensor The coarse sensor data structure
/// @return The angle estimate
Position getCoarseAngleEstimate(SensorCluster &coarseSensor) {
    Position angle_estimate = {0, 0};

    qsort(coarseSensor.sensors, coarseSensor.num_sensors, sizeof(coarseSensor.sensors[0]), sort_desc);

    int lowest_intensity = coarseSensor.sensors[coarseSensor.num_sensors - 2].intensity <
                                   coarseSensor.sensors[coarseSensor.num_sensors - 1].intensity
                               ? coarseSensor.sensors[coarseSensor.num_sensors - 2].intensity
                               : coarseSensor.sensors[coarseSensor.num_sensors - 1].intensity;

    for (uint8_t i = 0; i < coarseSensor.num_sensors; i++) {
        coarseSensor.sensors[i].intensity = coarseSensor.sensors[i].intensity - lowest_intensity;
    }

    float azWeight = 1.0 - (((float)coarseSensor.sensors[0].intensity) /
                            (coarseSensor.sensors[0].intensity + coarseSensor.sensors[1].intensity));

    int azSensorDist = coarseSensor.sensors[0].azPosition - coarseSensor.sensors[1].azPosition;

    int azIntensity = lerp(coarseSensor.sensors[0].intensity, coarseSensor.sensors[1].intensity, azWeight);

    if (abs(azSensorDist) == 180) {
        angle_estimate.azimuth = coarseSensor.sensors[0].azPosition;
        azIntensity = coarseSensor.sensors[0].intensity;

    } else if (azSensorDist == 270) {
        angle_estimate.azimuth = lerp(270, 360, azWeight);
    } else if (azSensorDist == -270) {
        angle_estimate.azimuth = lerp(360, 270, azWeight);
    } else if (abs(azSensorDist) == 90) {
        angle_estimate.azimuth = lerp(coarseSensor.sensors[0].azPosition, coarseSensor.sensors[1].azPosition, azWeight);
    }

    float elWeight = 1.0 - (((float)coarseSensor.sensors[4].intensity) / (coarseSensor.sensors[4].intensity + azIntensity));

    angle_estimate.elevation = lerp(coarseSensor.sensors[4].elPosition, coarseSensor.sensors[0].elPosition, elWeight);

    return angle_estimate;
}

/// @brief Converts degrees to steps for the stepper motor
long degreesToSteps(float degrees) { return (long)(degrees / 360 * fullRotation); }

/// @brief Converts steps to degrees for the stepper motor
float stepsToDegrees(long steps) { return (float)((float)steps / fullRotation * 360); }

/// @brief Moves the stepper motor to the given angle
/// @param stepper_info The stepper motor to move
/// @param angleDeg The angle to move the stepper motor to in degrees
void moveTo(SafeStepper &stepper_info, long angleDeg) {
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
void move(SafeStepper &stepper_info, long angleDeg) {
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

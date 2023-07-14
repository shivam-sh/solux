#include <AccelStepper.h>
#include <Adafruit_NeoPixel.h>

#define PIN 40
#define NUMPIXELS 1

typedef enum {
    AZ,
    EL,
    // TODO: add the fine sensor types here
} SensType;

typedef enum { WAITING, MEASURING, SET_MOVING, MOVING, TESTING } PanelState;

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

Adafruit_NeoPixel pixel(NUMPIXELS, PIN, NEO_RGB + NEO_KHZ800);

PanelState currentState = WAITING;
unsigned long lastMove = 0;
const int movePeriodSec = 10;

unsigned long lastMeasure = 0;
const int measurePeriodSec = 2;
int numReading = 0;
const int totalNumReadings = 5;

AccelStepper azimuth(AccelStepper::FULL4WIRE, 38, 9, 8, 33);
AccelStepper elevation(AccelStepper::FULL4WIRE, 10, 3, 1, 7);

ButtonInfo azButton = {11, false, 0};

SafeStepper azSafe = {&azimuth, 0, 360};
SafeStepper elSafe = {&elevation, 0, 90};

SensorCluster coarseSensor = {
    5, {{EL, 6, -1, 90, 0}, {AZ, 12, 0, 0, 0}, {AZ, 14, 90, 0, 0}, {AZ, 18, 180, 0, 0}, {AZ, 17, 270, 0, 0}}};

int sensorIntensity[5][100] = {0};

// full rotation, from https://lastminuteengineers.com/28byj48-stepper-motor-arduino-tutorial/
const long fullRotation = 2038;
const int lowerElevationLimit = 0;
const int upperElevationLimit = 180;

void IRAM_ATTR buttonIsr() {
    azButton.numPressed++;
    azButton.pressed = true;
}

void setup() {
    pixel.setPixelColor(0, pixel.Color(0, 100, 0));
    pixel.show();

    pinMode(13, OUTPUT);
    // TODO (npalmar): Implement azimuth calibration
    pinMode(azButton.pin, INPUT);
    attachInterrupt(azButton.pin, buttonIsr, FALLING);

    azimuth.setMaxSpeed(200);
    azimuth.setAcceleration(20);
    elevation.setMaxSpeed(200);
    elevation.setAcceleration(20);

    resetElevation(elevation);

    Serial.begin(9600);
    pixel.begin();
}

void loop() {
    azimuth.run();
    elevation.run();
    pixel.clear();

    if (currentState == WAITING) {
        pixel.setPixelColor(0, pixel.Color(0, 100, 0));
        pixel.show();
        if ((millis() - lastMove) * 0.001 > movePeriodSec) {
            currentState = MEASURING;
        }
    }

    if (currentState == MEASURING) {
        pixel.setPixelColor(0, pixel.Color(0, 100, 100));
        pixel.show();
        if (numReading >= totalNumReadings) {
            currentState = SET_MOVING;
            numReading = 0;
        }

        else if ((millis() - lastMeasure) * 0.001 > measurePeriodSec) {
            getSensorReadings(coarseSensor);
            for (uint8_t i = 0; i < coarseSensor.num_sensors; i++) {
                sensorIntensity[i][numReading] = coarseSensor.sensors[i].intensity;
            }
            numReading++;
            lastMeasure = millis();
        }
    }

    if (currentState == SET_MOVING) {
        pixel.setPixelColor(0, pixel.Color(100, 0, 0));
        pixel.show();
        // get the median intensity from sensorIntensity and set in coarseSensor
        for (uint8_t i = 0; i < coarseSensor.num_sensors; i++) {
            qsort(sensorIntensity[i], totalNumReadings, sizeof(sensorIntensity[i][0]), sort_desc2);
            // take the median of even numbers
            if (totalNumReadings % 2 == 0) {
                coarseSensor.sensors[i].intensity =
                    (float)(sensorIntensity[i][totalNumReadings / 2] + sensorIntensity[i][totalNumReadings / 2 - 1]) / 2;
            } else {
                coarseSensor.sensors[i].intensity = sensorIntensity[i][totalNumReadings / 2];
            }
        }
        Serial.printf("%d\t\t%d\t\t%d\t\t%d\t\t%d\t\t\n", coarseSensor.sensors[0].intensity,
                      coarseSensor.sensors[1].intensity, coarseSensor.sensors[2].intensity,
                      coarseSensor.sensors[3].intensity, coarseSensor.sensors[4].intensity);

        // calculate the position to move
        Position angleEstimate = {0, 0};
        angleEstimate = getCoarseAngleEstimate(coarseSensor);
        Serial.printf("Az:%f\t\tEl:%f\n", angleEstimate.azimuth, angleEstimate.elevation);
        azimuth.stop();
        moveTo(azSafe, angleEstimate.azimuth);
        elevation.stop();
        moveTo(elSafe, angleEstimate.elevation);

        currentState = MOVING;
    }

    if (currentState == MOVING) {
        pixel.setPixelColor(0, pixel.Color(0, 0, 200));
        pixel.show();
        // check when moving is done to put it back into waiting state
        if (azSafe.stepper->distanceToGo() == 0 && elSafe.stepper->distanceToGo() == 0) {
            lastMove = millis();
            currentState = WAITING;
        }
    }

    if (currentState == TESTING) {
        pixel.setPixelColor(0, pixel.Color(100, 100, 100));
        pixel.show();
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
/// @brief Sorts the sensor readings (between different sensors) in descending order of intensity but with the elevation
/// sensor at the end
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

// compare function for qsort
/// @brief Sorts the intensity readings for a single sensor
/// @param cmp1 The first sensor reading
/// @param cmp2 The second sensor
int sort_desc2(const void *cmp1, const void *cmp2) {
    int i1 = *(int *)cmp1;
    int i2 = *(int *)cmp2;

    return i2 - i1;
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

    if (coarseSensor.sensors[0].intensity - coarseSensor.sensors[3].intensity < 1000) {
      angle_estimate.azimuth = 0;
      angle_estimate.elevation = 90;
      return angle_estimate;
    } else if (abs(azSensorDist) == 180) {
        angle_estimate.azimuth = coarseSensor.sensors[0].azPosition;
    } else if (azSensorDist == 270) {
        angle_estimate.azimuth = lerp(270, 360, azWeight);
    } else if (azSensorDist == -270) {
        angle_estimate.azimuth = lerp(360, 270, azWeight);
    } else if (abs(azSensorDist) == 90) {
        angle_estimate.azimuth = lerp(coarseSensor.sensors[0].azPosition, coarseSensor.sensors[1].azPosition, azWeight);
    }

    float elWeight = 1.0 - (((float)coarseSensor.sensors[4].intensity) /
                            (coarseSensor.sensors[4].intensity + coarseSensor.sensors[0].intensity));

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

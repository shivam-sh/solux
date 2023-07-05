#include <AccelStepper.h>

AccelStepper azimuth(AccelStepper::FULL4WIRE, 38, 9, 8, 33);
AccelStepper elevation(AccelStepper::FULL4WIRE, 10, 3, 1, 7);

long maxAzimuth = 2048;
long maxElevation = 512;

void setup() {
  azimuth.setMaxSpeed(200);
  azimuth.setAcceleration(20);
  elevation.setMaxSpeed(300);
  elevation.setAcceleration(50);

  azimuth.moveTo(maxAzimuth);
  elevation.moveTo(maxElevation);

  Serial.begin(9600);
}

void loop() {
    if (azimuth.distanceToGo() == 0 && azimuth.targetPosition() == maxAzimuth)
        azimuth.moveTo(0);

    if (elevation.distanceToGo() == 0 && elevation.targetPosition() == maxElevation)
        elevation.moveTo(0);

    azimuth.run();
    elevation.run();

    Serial.printf("%d\t\t%d\n", elevation.currentPosition(), azimuth.currentPosition());
}
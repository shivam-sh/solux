#include <Adafruit_INA219.h>
#include <Wire.h>

Adafruit_INA219 ina219;

// Define pins
int base = 18;

// Reporting frequency
float freq = 0.1;  // Hz (measures once every 10 seconds)
// Delay after changing state of transistor
int del = 10;

// Sensor variables
float current_mA = 0;
float voltage = 0;
float power_mW = 0;

// Tracking time
unsigned long last = 0;
float t = 0;

// expected power on panel
// get this value from a python script using an average intensity during the time
float max_power_mW = 40;

// LED indicator
uint8_t bar_graph[10] = {13, 12, 14, 27, 26, 25, 33, 32, 15, 2};
unsigned long graph_update_time = 0;

typedef struct cmyLED {
    int cyan_pin;
    int magenta_pin;
    int yellow_pin;
    unsigned long last_time;
    int hold_time;
};

cmyLED indicator_led = {17, 16, 4, 0, 2000};

void ledSetup(cmyLED led) {
    pinMode(led.cyan_pin, OUTPUT);
    pinMode(led.magenta_pin, OUTPUT);
    pinMode(led.yellow_pin, OUTPUT);
}

void ledOff(cmyLED led) {
    digitalWrite(led.cyan_pin, HIGH);
    digitalWrite(led.magenta_pin, HIGH);
    digitalWrite(led.yellow_pin, HIGH);
}

void makeRed(cmyLED led) {
    // clear any previous colour and make red
    ledOff(led);
    delay(10);
    digitalWrite(led.cyan_pin, LOW);
}

void makeYellow(cmyLED led) {
    ledOff(led);
    delay(10);
    digitalWrite(led.magenta_pin, LOW);
    digitalWrite(led.cyan_pin, LOW);
}

void makeGreen(cmyLED led) {
    ledOff(led);
    delay(10);
    digitalWrite(led.magenta_pin, LOW);
}

// Run once
void setup(void) {
    pinMode(base, OUTPUT);
    pinMode(2, OUTPUT);

    ledSetup(indicator_led);
    delay(100);
    ledOff(indicator_led);

    for (int i = 0; i < 10; i++) {
        pinMode(bar_graph[i], OUTPUT);
        digitalWrite(bar_graph[i], HIGH);
    }

    Serial.begin(115200);
    while (!Serial) {
        // will pause Zero, Leonardo, etc until serial console opens
        delay(1);
    }

    Serial.println("Hello!");

    // Initialize the INA219.
    // By default the initialization will use the largest range (32V, 2A).  However
    // you can call a setCalibration function to change this range (see comments).
    if (!ina219.begin()) {
        Serial.println("Failed to find INA219 chip");
        while (1) {
            delay(10);
        }
    }
    // To use a slightly lower 32V, 1A range (higher precision on amps):
    // ina219.setCalibration_32V_1A();
    // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
    ina219.setCalibration_16V_400mA();

    Serial.println("Measuring voltage and current with INA219 ...");

    last = millis();
    graph_update_time = millis();
}

// Run indefinitely
void loop(void) {
    if (millis() - last > 1000.0 / freq) {
        last = millis();
        t = last / 1000.0;  // Time in seconds

        // measure current by turning on the transistor to measure I_sc
        digitalWrite(base, HIGH);
        delay(del);
        current_mA = ina219.getCurrent_mA();

        // measure voltage by turning transistor low for V_oc
        digitalWrite(base, LOW);
        delay(del);
        voltage = ina219.getBusVoltage_V();

        // *** END OF POWER COLLECTION

        // Max power (mW) is at roughly 0.7x the voltage
        power_mW = (0.7 * voltage) * current_mA;

        // output a colour depending on the power being measured
        if (power_mW / max_power_mW > 0.7) {
            makeGreen(indicator_led);
        } else if (power_mW / max_power_mW > 0.3) {
            makeYellow(indicator_led);
        } else {
            makeRed(indicator_led);
        }
        indicator_led.last_time = millis();

        // Format: Time, Voltage, Current, Estimated Power
        Serial.print(t);
        Serial.print(", ");
        Serial.print(voltage);
        Serial.print(", ");
        Serial.print(current_mA);
        Serial.print(", ");
        Serial.println(power_mW);
    }

    if (millis() - graph_update_time > 1000 / freq) {
        int num_on = ((millis() % 11000) / 1000 % 11);

        for (int i = 0; i < 11; i++) {
            digitalWrite(bar_graph[i], HIGH);
        }

        for (int i = num_on; i < 11; i++) {
            digitalWrite(bar_graph[i], LOW);
        }

        // graph_update_time = millis();
    }

    if (millis() - indicator_led.last_time > indicator_led.hold_time) {
        ledOff(indicator_led);
    }
}

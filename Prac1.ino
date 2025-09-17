#include <Arduino.h>
// pins
const int TRIG_PIN = 14;
const int ECHO_PIN = 27;
const int RED_LED_PIN = 26;
const int GREEN_LED_PIN = 33;
const int YELLOW_LED_PIN = 25;
const int BLUE_LED_PIN = 32;

class Led {
public:
    Led(int pin_arg) : pin(pin_arg), state(LOW), blinking(false), lastToggleTime(0), blinkInterval(500) {
        pinMode(pin, OUTPUT);
        off();
    }

    void on() {
        state = HIGH;
        blinking = false; 
        digitalWrite(pin, state);
    }

    void off() {
        state = LOW;
        blinking = false; 
        digitalWrite(pin, state);
    }

    void blink(unsigned long interval = 500) {
        blinking = true;
        blinkInterval = interval;
    }

    void update() {
        if (blinking) {
            unsigned long currentTime = millis();
            if (currentTime - lastToggleTime >= blinkInterval) {
                lastToggleTime = currentTime;
                state = !state; // Invierte el estado.
                digitalWrite(pin, state);
            }
        }
    }

private:
    int pin;
    bool state;
    bool blinking;
    unsigned long lastToggleTime;
    unsigned long blinkInterval;
};

class UltrasonicSensor {
public:
    UltrasonicSensor(int trigPin_arg, int echoPin_arg) : trigPin(trigPin_arg), echoPin(echoPin_arg) {
        pinMode(trigPin, OUTPUT);
        pinMode(echoPin, INPUT);
    }

    float getDistanceCm() {
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);

        long duration = pulseIn(echoPin, HIGH);
        if (duration == 0) {
            return 999.0; 
        }
        return duration * 0.034 / 2.0;
    }

private:
    int trigPin;
    int echoPin;
};

class DistanceAlertSystem {
public:
    DistanceAlertSystem(int trigPin, int echoPin, int redLedPin, int greenLedPin, int yellowLedPin, int blueLedPin)
        : sensor(trigPin, echoPin),
          redLed(redLedPin),
          greenLed(greenLedPin),
          yellowLed(yellowLedPin),
          blueLed(blueLedPin) {}

    void setup() {
        Serial.begin(115200);
    }

    void loop() {
        float currentDistance = sensor.getDistanceCm();
        
        updateLedState(currentDistance);

        redLed.update();
        greenLed.update();
        yellowLed.update();
        blueLed.update();

        delay(100);
    }

private:
    void updateLedState(float distance) {
        String status = "Unknown";

        if (distance < 20) {
            redLed.blink();
            greenLed.off();
            yellowLed.off();
            blueLed.off();
            status = " -- Red is blinking";
        }
        else if (distance >= 20 && distance < 40) {
            yellowLed.blink();
            redLed.off();
            greenLed.off();
            blueLed.off();
            status = " -- Yellow is blinking";
        }
        else if (distance >= 40 && distance < 80) {
            greenLed.on();
            redLed.off();
            yellowLed.off();
            blueLed.off();
            status = " -- Green is on";
        }
        else if (distance >= 80 && distance <= 100) {
            blueLed.on();
            redLed.off();
            greenLed.off();
            yellowLed.off();
            status = " -- Blue is on";
        }
        else if (distance <= 200){
            redLed.on();
            yellowLed.on();
            greenLed.off();
            blueLed.off();
            status = " -- Red & Yellow are on";
        }
        else if (distance > 200){
            redLed.off();
            yellowLed.off();
            greenLed.off();
            blueLed.off();
            status = " Out of specified range";
        }
        
        Serial.print("Distance: ");
        Serial.print(distance, 2);
        Serial.print(" cm, Status: ");
        Serial.println(status);
    }

    UltrasonicSensor sensor;
    Led redLed;
    Led greenLed;
    Led yellowLed;
    Led blueLed;
};


DistanceAlertSystem mySystem(TRIG_PIN, ECHO_PIN, RED_LED_PIN, GREEN_LED_PIN, YELLOW_LED_PIN, BLUE_LED_PIN);

void setup() {
    mySystem.setup();
}

void loop() {
    mySystem.loop();
}
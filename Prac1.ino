#include <Arduino.h>

class Led {
public:
    Led(int pin) : _pin(pin), _state(LOW), _blinking(false), _lastToggleTime(0), _blinkInterval(500) {
        pinMode(_pin, OUTPUT);
        off();
    }

    void on() {
        _state = HIGH;
        _blinking = false;
        digitalWrite(_pin, _state);
    }

    void off() {
        _state = LOW;
        _blinking = false;
        digitalWrite(_pin, _state);
    }

    void blink(unsigned long interval = 500) {
        _blinking = true;
        _blinkInterval = interval;
    }

    void update() {
        if (_blinking) {
            unsigned long currentTime = millis();
            if (currentTime - _lastToggleTime >= _blinkInterval) {
                _lastToggleTime = currentTime;
                _state = !_state;
                digitalWrite(_pin, _state);
            }
        }
    }

private:
    int _pin;
    bool _state;
    bool _blinking;
    unsigned long _lastToggleTime;
    unsigned long _blinkInterval;
};

class UltrasonicSensor {
public:
    UltrasonicSensor(int trigPin, int echoPin) : _trigPin(trigPin), _echoPin(echoPin) {
        pinMode(_trigPin, OUTPUT);
        pinMode(_echoPin, INPUT);
    }

    float getDistanceCm() {
        digitalWrite(_trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(_trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(_trigPin, LOW);

        long duration = pulseIn(_echoPin, HIGH);
        if (duration == 0) {
            return 999.0; 
        }
        return duration * 0.034 / 2.0;
    }

private:
    int _trigPin;
    int _echoPin;
};

class DistanceAlertSystem {
public:
    DistanceAlertSystem(int trigPin, int echoPin, int redLedPin, int greenLedPin, int yellowLedPin, int blueLedPin)
        : _sensor(trigPin, echoPin),
          _redLed(redLedPin),
          _greenLed(greenLedPin),
          _yellowLed(yellowLedPin),
          _blueLed(blueLedPin) {}

    void setup() {
        Serial.begin(115200);
    }

    void loop() {
        float currentDistance = _sensor.getDistanceCm();
        updateLedState(currentDistance);
        _redLed.update();
        _greenLed.update();
        _yellowLed.update();
        _blueLed.update();
        delay(100);
    }

private:
    void updateLedState(float distance) {
        if (distance < 20) {
            _redLed.blink();
            _greenLed.off();
            _yellowLed.off();
            _blueLed.off();
        } 
        else if (distance >= 20 && distance < 40) {
            _redLed.off();
            _greenLed.off();
            _yellowLed.on();
            _blueLed.off();
        } 
        else if (distance >= 40 && distance < 80) {
            _redLed.off();
            _greenLed.on();
            _yellowLed.off();
            _blueLed.off();
        } 
        else if (distance >= 80 && distance <= 100) {
            _redLed.off();
            _greenLed.off();
            _yellowLed.off();
            _blueLed.on();
        } 
        else {
            _redLed.on();
            _yellowLed.on();
            _greenLed.off();
            _blueLed.off();
        }
        
        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.print(" cm, ");
        Serial.println(status);
    }

    UltrasonicSensor _sensor;
    Led _redLed;
    Led _greenLed;
    Led _yellowLed;
    Led _blueLed;
};

//  PINS
const int TRIG_PIN = 14;
const int ECHO_PIN = 27;
const int RED_LED_PIN = 26;
const int GREEN_LED_PIN = 33;
const int YELLOW_LED_PIN = 25;
const int BLUE_LED_PIN = 32;

DistanceAlertSystem mySystem(TRIG_PIN, ECHO_PIN, RED_LED_PIN, GREEN_LED_PIN, YELLOW_LED_PIN, BLUE_LED_PIN);

void setup() {
    mySystem.setup();
}

void loop() {
    mySystem.loop();
}
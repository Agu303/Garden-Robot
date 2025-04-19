#include <Wire.h>

// Define Motor A (Right Motor) Pins
#define PWM_A 9
#define A1 7
#define A2 6

// Define Motor B (Left Motor) Pins
#define PWM_B 10
#define B1 11
#define B2 12

// Receiver input pins for motor control
const int steeringPin = 2;
const int throttlePin = 3;

// RC input channels
#define CHANNEL_PIN 5   // FS-i6X channel for pump control
const int actuatorPin = 13; // Actuator control signal
const int motorPin = 4;     // 12V motor control signal

// Input pulse range (FS-i6X default)
#define PULSE_MIN 980
#define PULSE_MAX 1980

// Motor speed range
#define SPEED_MIN -255
#define SPEED_MAX 255

// Deadzone threshold
#define DEADZONE 20

// Timing for non-blocking execution
unsigned long lastUpdate = 0;
const int updateInterval = 50;

void setup() {
    Serial.begin(9600);
    Wire.begin();  // Start I2C as master

    // Set motor control pins as outputs
    pinMode(PWM_A, OUTPUT);
    pinMode(A1, OUTPUT);
    pinMode(A2, OUTPUT);
    pinMode(PWM_B, OUTPUT);
    pinMode(B1, OUTPUT);
    pinMode(B2, OUTPUT);

    // Set receiver input pins
    pinMode(steeringPin, INPUT);
    pinMode(throttlePin, INPUT);
    pinMode(CHANNEL_PIN, INPUT);
    pinMode(actuatorPin, INPUT);
    pinMode(motorPin, INPUT);
}

void loop() {
    if (millis() - lastUpdate >= updateInterval) {
        lastUpdate = millis();

        int throttle, steering, pumpSignal;
        readInputs(throttle, steering, pumpSignal);

        int rightMotorSpeed, leftMotorSpeed;
        calculateMotorSpeeds(throttle, steering, rightMotorSpeed, leftMotorSpeed);

        applyMotorSpeeds(rightMotorSpeed, leftMotorSpeed);

        // Read signals and send to slave arduino
        int actuatorSignal = pulseIn(actuatorPin, HIGH);
        int motorSignal = pulseIn(motorPin, HIGH);
        sendToSlave(actuatorSignal, motorSignal, pumpSignal);

        debugOutput(throttle, steering, rightMotorSpeed, leftMotorSpeed, pumpSignal, actuatorSignal, motorSignal);
    }
}

void readInputs(int &throttle, int &steering, int &pumpSignal) {
    throttle = pulseIn(throttlePin, HIGH);
    steering = pulseIn(steeringPin, HIGH);
    pumpSignal = pulseIn(CHANNEL_PIN, HIGH);

    throttle = map(throttle, PULSE_MIN, PULSE_MAX, SPEED_MIN, SPEED_MAX);
    steering = map(steering, PULSE_MIN, PULSE_MAX, SPEED_MIN, SPEED_MAX);

    throttle = (abs(throttle) < DEADZONE) ? 0 : throttle;
    steering = (abs(steering) < DEADZONE) ? 0 : steering;
}

void calculateMotorSpeeds(int throttle, int steering, int &rightMotorSpeed, int &leftMotorSpeed) {
    if (throttle == 0) {
        rightMotorSpeed = -steering;
        leftMotorSpeed = steering;
    } else {
        rightMotorSpeed = constrain(throttle - steering / 2, SPEED_MIN, SPEED_MAX);
        leftMotorSpeed  = constrain(throttle + steering / 2, SPEED_MIN, SPEED_MAX);

        if (steering == 0) {
            rightMotorSpeed = leftMotorSpeed = throttle;
        }
    }
}

void applyMotorSpeeds(int rightMotorSpeed, int leftMotorSpeed) {
    setMotorSpeed(PWM_A, A1, A2, rightMotorSpeed);
    setMotorSpeed(PWM_B, B1, B2, -leftMotorSpeed);  // Inverted for tank drive
}

void setMotorSpeed(int pwmPin, int pin1, int pin2, int speed) {
    digitalWrite(pin1, speed > 0);
    digitalWrite(pin2, speed < 0);
    analogWrite(pwmPin, abs(speed));
}

// NEW: Send actuator, motor, and pump signals to slave arduino
void sendToSlave(int actuatorSignal, int motorSignal, int pumpSignal) {
    Wire.beginTransmission(8); // Slave address

    Wire.write((byte)(actuatorSignal >> 8));
    Wire.write((byte)(actuatorSignal & 0xFF));
    Wire.write((byte)(motorSignal >> 8));
    Wire.write((byte)(motorSignal & 0xFF));
    Wire.write((byte)(pumpSignal >> 8));
    Wire.write((byte)(pumpSignal & 0xFF));

    Wire.endTransmission();
}

void debugOutput(int throttle, int steering, int rightMotorSpeed, int leftMotorSpeed,
                 int pumpSignal, int actuatorSignal, int motorSignal) {
    Serial.print("Throttle: "); Serial.print(throttle);
    Serial.print(" | Steering: "); Serial.print(steering);
    Serial.print(" | Left Motor: "); Serial.print(leftMotorSpeed);
    Serial.print(" | Right Motor: "); Serial.print(rightMotorSpeed);
    Serial.print(" | Pump: "); Serial.print(pumpSignal);
    Serial.print(" | Actuator: "); Serial.print(actuatorSignal);
    Serial.print(" | Motor: "); Serial.println(motorSignal);
}

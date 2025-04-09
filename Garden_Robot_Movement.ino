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

// Pump control
#define CHANNEL_PIN 5  // FS-i6X receiver channel for pump control
#define MOSFET_PIN 8   // MOSFET Gate control pin

// Input pulse range (FS-i6X default values)
#define PULSE_MIN 980
#define PULSE_MAX 1980

// Motor speed range
#define SPEED_MIN -255
#define SPEED_MAX 255

// Deadzone threshold
#define DEADZONE 20

// Timing for non-blocking execution
unsigned long lastUpdate = 0;
const int updateInterval = 50; // Adjust based on desired responsiveness

void setup() {
    Serial.begin(9600);
    
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

    // Set MOSFET for pump control
    pinMode(MOSFET_PIN, OUTPUT);
    digitalWrite(MOSFET_PIN, LOW);  // Start with pump OFF
}

void loop() {
    if (millis() - lastUpdate >= updateInterval) {
        lastUpdate = millis();

        int throttle, steering, pumpSignal;
        readInputs(throttle, steering, pumpSignal);

        int rightMotorSpeed, leftMotorSpeed;
        calculateMotorSpeeds(throttle, steering, rightMotorSpeed, leftMotorSpeed);

        applyMotorSpeeds(rightMotorSpeed, leftMotorSpeed);

        controlPump(pumpSignal);

        debugOutput(throttle, steering, rightMotorSpeed, leftMotorSpeed, pumpSignal);
    }
}

// Reads input signals and applies deadzone filtering
void readInputs(int &throttle, int &steering, int &pumpSignal) {
    throttle = pulseIn(throttlePin, HIGH);
    steering = pulseIn(steeringPin, HIGH);
    pumpSignal = pulseIn(CHANNEL_PIN, HIGH);

    // Map FS-i6X input range to motor speed range
    throttle = map(throttle, PULSE_MIN, PULSE_MAX, SPEED_MIN, SPEED_MAX);
    steering = map(steering, PULSE_MIN, PULSE_MAX, SPEED_MIN, SPEED_MAX);

    // Apply deadzone filtering
    throttle = (abs(throttle) < DEADZONE) ? 0 : throttle;
    steering = (abs(steering) < DEADZONE) ? 0 : steering;
}

// Computes motor speeds based on throttle and steering input
void calculateMotorSpeeds(int throttle, int steering, int &rightMotorSpeed, int &leftMotorSpeed) {
    if (throttle == 0) {  // In-place turning
        rightMotorSpeed = -steering;
        leftMotorSpeed = steering;
    } else {  // Forward or backward movement with steering adjustment
        rightMotorSpeed = constrain(throttle - steering / 2, SPEED_MIN, SPEED_MAX);
        leftMotorSpeed  = constrain(throttle + steering / 2, SPEED_MIN, SPEED_MAX);

        // Ensure both motors have the same speed when going straight
        if (steering == 0) {
            rightMotorSpeed = leftMotorSpeed = throttle;
        }
    }
}

// Sets motor speed and direction
void applyMotorSpeeds(int rightMotorSpeed, int leftMotorSpeed) {
    setMotorSpeed(PWM_A, A1, A2, rightMotorSpeed);
    setMotorSpeed(PWM_B, B1, B2, -leftMotorSpeed);  // Reverse direction for left motor
}

// Controls motor direction and speed
void setMotorSpeed(int pwmPin, int pin1, int pin2, int speed) {
    digitalWrite(pin1, speed > 0);
    digitalWrite(pin2, speed < 0);
    analogWrite(pwmPin, abs(speed));
}

// Controls water pump activation
void controlPump(int pumpSignal) {
    digitalWrite(MOSFET_PIN, pumpSignal > 1500 ? HIGH : LOW);
}

// Outputs debugging information via Serial Monitor
void debugOutput(int throttle, int steering, int rightMotorSpeed, int leftMotorSpeed, int pumpSignal) {
    Serial.print("Throttle: "); Serial.print(throttle);
    Serial.print(" | Steering: "); Serial.print(steering);
    Serial.print(" | Left Motor: "); Serial.print(leftMotorSpeed);
    Serial.print(" | Right Motor: "); Serial.print(rightMotorSpeed);
    Serial.print(" | Pump Signal: "); Serial.println(pumpSignal);
}

#include <Wire.h>

// === Pin Definitions ===
// Linear Actuator (L298N Driver 2)
#define ACTUATOR_PWM 9
#define ACTUATOR_IN1 7
#define ACTUATOR_IN2 12  

// 12V Motor (L298N Driver 2)
#define MOTOR_IN1 4
#define MOTOR_IN2 2
#define MOTOR_ENA 8

// Pumps (Motor Driver 1)
#define PUMP_A_ENA 10
#define PUMP_B_ENB 6

// === State Variables ===
int prevActSignal = -1;
const int deadzone = 10;

void setup() {
  Serial.begin(9600);
  Wire.begin(8); // I2C Slave Address
  Wire.onReceive(receiveEvent);

  // Set pin modes
  pinMode(ACTUATOR_PWM, OUTPUT);
  pinMode(ACTUATOR_IN1, OUTPUT);
  pinMode(ACTUATOR_IN2, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(PUMP_A_ENA, OUTPUT);
  pinMode(PUMP_B_ENB, OUTPUT);

  // Initialize outputs to LOW
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_ENA, 0);
  digitalWrite(ACTUATOR_IN1, LOW);
  digitalWrite(ACTUATOR_IN2, LOW);
  analogWrite(ACTUATOR_PWM, 0);
  analogWrite(PUMP_A_ENA, 0);
  analogWrite(PUMP_B_ENB, 0);
}

void loop() {
  // I2C events handled in receiveEvent()
}

void receiveEvent(int howMany) {
  if (howMany >= 6) {
    int actSignal = (Wire.read() << 8) | Wire.read();
    int motorSignal = (Wire.read() << 8) | Wire.read();
    int pumpSignal = (Wire.read() << 8) | Wire.read();

    // === Debug Print ===
    Serial.print("Received | Actuator: ");
    Serial.print(actSignal);
    Serial.print(" | Motor: ");
    Serial.print(motorSignal);
    Serial.print(" | Pump: ");
    Serial.println(pumpSignal);

    // === Motor Control ===
    if (motorSignal > 1500) {
      Serial.println("Motor ON");
      digitalWrite(MOTOR_IN1, HIGH);
      digitalWrite(MOTOR_IN2, LOW);
      analogWrite(MOTOR_ENA, 255);
    } else {
      Serial.println("Motor OFF");
      digitalWrite(MOTOR_IN1, LOW);
      digitalWrite(MOTOR_IN2, LOW);
      analogWrite(MOTOR_ENA, 0);
    }

    // === Pump Control ===
    if (pumpSignal > 1500) {
      Serial.println("Pumps ON");
      analogWrite(PUMP_A_ENA, 255);
      analogWrite(PUMP_B_ENB, 255);
    } else {
      Serial.println("Pumps OFF");
      analogWrite(PUMP_A_ENA, 0);
      analogWrite(PUMP_B_ENB, 0);
    }

    // === Actuator Control ===
    if (actSignal < 1010) {
      Serial.println("Actuator FULL Retract");
      digitalWrite(ACTUATOR_IN1, LOW);
      digitalWrite(ACTUATOR_IN2, HIGH);
      analogWrite(ACTUATOR_PWM, 255);
    } else if (actSignal > 1980) {
      Serial.println("Actuator FULL EXTEND");
      digitalWrite(ACTUATOR_IN1, HIGH);
      digitalWrite(ACTUATOR_IN2, LOW);
      analogWrite(ACTUATOR_PWM, 255);
    } else {
      int delta = 0;

      if (prevActSignal == -1) {
        prevActSignal = actSignal;
        Serial.println("Actuator INIT");
      } else {
        delta = actSignal - prevActSignal;

        if (delta > deadzone) {
          Serial.println("Actuator Extending");
          digitalWrite(ACTUATOR_IN1, HIGH);
          digitalWrite(ACTUATOR_IN2, LOW);
          analogWrite(ACTUATOR_PWM, 255);
        } else if (delta < -deadzone) {
          Serial.println("Actuator Retracting");
          digitalWrite(ACTUATOR_IN1, LOW);
          digitalWrite(ACTUATOR_IN2, HIGH);
          analogWrite(ACTUATOR_PWM, 255);
        } else {
          Serial.println("Actuator STOPPED");
          digitalWrite(ACTUATOR_IN1, LOW);
          digitalWrite(ACTUATOR_IN2, LOW);
          analogWrite(ACTUATOR_PWM, 0);
        }

        prevActSignal = actSignal;
      }
    }
  }
}

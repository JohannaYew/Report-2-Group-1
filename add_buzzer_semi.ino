#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <PinChangeInterrupt.h>
#include <math.h>

// Ultrasonic Sensor Pins
const int trigPin = 12;
const int echoPin = 13;

// Motor Pins
const int enA = 3, enB = 11, IN1 = 2, IN3 = 10, IN2 = A3, IN4 = A2;

// Buzzer Pin
const int buzzerPin = A4;

long duration;
int measuredRange;

void setup() {
  // Ultrasonic Sensor Pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Buzzer Pin
  pinMode(buzzerPin, OUTPUT);

  // Serial Monitor for Debugging
  Serial.begin(9600);
  Serial.println("Ultrasonic Sensor HC-SR04 Test");

  // Motor Pins
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stop(); // Ensure motors are stopped initially
}

void loop() {
  // Trigger Ultrasonic Sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure Echo Duration
  duration = pulseIn(echoPin, HIGH);
  measuredRange = duration * 0.034 / 2; // Convert to cm

  // Debugging Output
  Serial.print("Duration: ");
  Serial.println(duration);
  Serial.print("Measured Range: ");
  Serial.println(measuredRange);
  Serial.println(" cm");

  // Control Motors and Buzzer Based on Range
  if (measuredRange > 15) {
    stopBuzzer(); // Ensure buzzer is off
    forward();
  } else {
    Serial.println("Obstacle detected! Activating buzzer.");
    soundBuzzer(); // Activate buzzer
    stop();
    delay(200);
    backward();
    delay(200);
    uturn();
    forward();
  }
}

void forward() {
  analogWrite(enA, 70);
  analogWrite(enB, 70);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("Moving forward...");
}

void stop() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("Motors stopped.");
}

void backward() {
  analogWrite(enA, 70);
  analogWrite(enB, 70);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Reversing...");
}

void uturn() {
  analogWrite(enA, 150);
  analogWrite(enB, 150);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Performing U-turn...");
  delay(1060);
}

// Buzzer Control
void soundBuzzer() {
  digitalWrite(buzzerPin, HIGH); // Turn on the buzzer
  Serial.println("Buzzer ON");
}

void stopBuzzer() {
  digitalWrite(buzzerPin, LOW); // Turn off the buzzer
  Serial.println("Buzzer OFF");
}

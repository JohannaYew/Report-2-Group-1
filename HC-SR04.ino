#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <PinChangeInterrupt.h>
#include <math.h>

const int trigPin = 12; 
const int echoPin = 13;

const int enA = 3, enB = 11, IN1 = 2, IN3 = 10, IN2 = A3, IN4 = A2;

long duration;
int measuredRange;

void setup() {
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT); 
  Serial.begin(9600); 
  Serial.println("Ultrasonic Sensor HC-SR04 Test");
}

void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  measuredRange = duration * 0.034 / 2;
  Serial.print("Measured Range: ");
  Serial.println(measuredRange);
  Serial.println(" cm");

  if (measuredRange > 15) {
    forward();
  } else {
    stop();
  }
}

void forward() {
    analogWrite(enA, 90); 
    analogWrite(enB, 90); 
    digitalWrite(IN1, HIGH); 
    digitalWrite(IN2, HIGH); 
    digitalWrite(IN3, LOW); 
    digitalWrite(IN4, LOW); 
}

void stop() {
    analogWrite(enA, 0); 
    analogWrite(enB, 0); 
    digitalWrite(IN1, LOW); 
    digitalWrite(IN2, LOW); 
    digitalWrite(IN3, LOW); 
    digitalWrite(IN4, LOW); 
}



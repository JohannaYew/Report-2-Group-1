#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <PinChangeInterrupt.h>
#include <math.h>

// LCD Pin configuration
const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Line Sensor Pins
const int sensorPin1 = 13; // Sensor JOHANNA
const int sensorPin2 = 12; // Sensor KAYLEN

// Motor Pins
const int enA = 3, enB = 11, IN1 = 2, IN3 = 10, IN2 = A3, IN4 = A2;

// Encoder Pins
const int encoderPin1 = A1;
const int encoderPin2 = A0;

// Encoder Constants
const float wheelCircumference = 16; // Wheel circumference in cm
const int pulsesPerRevolution = 30;    // Encoder PPR
const float cmPerPulse = wheelCircumference / pulsesPerRevolution;

// Variables for encoders
volatile int counter1 = 0;
volatile int counter2 = 0;
float totalDistance = 0.0; // Accumulated line length

volatile unsigned long lastInterruptTime1 = 0;
volatile unsigned long lastInterruptTime2 = 0;
const unsigned long debounceDelay = 2;

unsigned long startTime = 0;

// MPU-6050 Setup
Adafruit_MPU6050 mpu;
float angle_pitch = 0.0, angle_roll = 0.0;
float angle_pitch_output = 0.0, angle_roll_output = 0.0;
boolean onRamp = false;
boolean startDistanceCalc = false;
boolean resetTilt = false;
boolean rampFinished = false;
long loop_timer;

unsigned long timerStartTime = 0;
bool timerRunning = false;

// Function prototypes
void forward(int speed);
void stop();
void turnRight();
void turnLeft();
void updateDistance();
void handleEncoderPin1();
void handleEncoderPin2();
bool isMoving();

void setup() {
    // Initialize LCD
    lcd.begin(16, 2);

    // Initialize sensors and motors
    pinMode(sensorPin1, INPUT);
    pinMode(sensorPin2, INPUT);
    pinMode(enA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Initialize encoder pins
    pinMode(encoderPin1, INPUT_PULLUP);
    pinMode(encoderPin2, INPUT_PULLUP);

    // Attach interrupts for encoders
    attachPCINT(digitalPinToPCINT(encoderPin1), handleEncoderPin1, CHANGE);
    attachPCINT(digitalPinToPCINT(encoderPin2), handleEncoderPin2, CHANGE);

    // Initialize Serial communication
    Serial.begin(115200);

    // Initialize the MPU6050
    if (!mpu.begin()) {
        Serial.println("Failed to initialize MPU6050!");
        while (1);
    }

    // Configure MPU-6050
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_42_HZ);

    // Initialize timing
    loop_timer = micros();
    startTime = millis();
}

void loop() {
    int sensorValue1 = digitalRead(sensorPin1); // JOHANNA
    int sensorValue2 = digitalRead(sensorPin2); // KAYLEN

    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    // Calculate pitch and roll from gyroscope data
    float gyroX = gyro.gyro.x;
    float gyroY = gyro.gyro.y;
    angle_pitch += gyroX * 58 * 0.004; 
    angle_roll += gyroY * 58 * 0.004;



    // Calculate pitch and roll from accelerometer data
    float acc_total_vector = sqrt((accel.acceleration.x * accel.acceleration.x) +
                                  (accel.acceleration.y * accel.acceleration.y) +
                                  (accel.acceleration.z * accel.acceleration.z));
    float angle_pitch_acc = asin((float)accel.acceleration.y / acc_total_vector) * 58;
    float angle_roll_acc = asin((float)accel.acceleration.x / acc_total_vector) * -58;


    // Complementary filter to combine gyro and accel data
    angle_pitch = angle_pitch * 0.95 + angle_pitch_acc * 0.05;
    angle_roll = angle_roll * 0.95 + angle_roll_acc * 0.05;

    // Apply offsets to the output angles
    angle_pitch_output = angle_pitch_output * 0.9 + (angle_pitch -0.93 ) * 0.1;
    angle_roll_output = angle_roll_output * 0.9 + (angle_roll + 0.46) * 0.1;

    // Calculate total tilt angle
    float total_tilt = sqrt(angle_pitch_output * angle_pitch_output + angle_roll_output * angle_roll_output);

    if (!onRamp && !rampFinished) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Tilt: ");
        lcd.print(total_tilt, 1);

        lcd.setCursor(0, 1);
        lcd.print("Dist: ");
        lcd.print(totalDistance, 1);
        lcd.print(" cm");
    }

    if (total_tilt > 22.5) {
        // If on ramp and ascending
        onRamp = true; // Set onRamp flag to true when ascending
        forward(175);
        Serial.println("Ascending ramp...");
    } 
    else if (onRamp && total_tilt < 22.5 && total_tilt > 2.5) {
        if (onRamp && total_tilt > 15 && total_tilt < 22) {
            if (sensorValue1 == LOW && sensorValue2 == LOW) {
            analogWrite(enA, 1); 
            analogWrite(enB, 255); 
            digitalWrite(IN1, HIGH); 
            digitalWrite(IN2, HIGH); 
            digitalWrite(IN3, LOW); 
            digitalWrite(IN4, LOW); 
            delay(5);
            stop();
            delay(30);
        } else if (sensorValue1 == LOW && sensorValue2 == HIGH) {
            analogWrite(enA, 1); 
            analogWrite(enB, 255); 
            digitalWrite(IN1, HIGH); 
            digitalWrite(IN2, HIGH); 
            digitalWrite(IN3, LOW); 
            digitalWrite(IN4, LOW); 
            delay(5);
            stop();
            delay(30);
        } else if (sensorValue1 == HIGH && sensorValue2 == LOW) {
            analogWrite(enA, 1); 
            analogWrite(enB, 255); 
            digitalWrite(IN1, HIGH); 
            digitalWrite(IN2, HIGH); 
            digitalWrite(IN3, LOW); 
            digitalWrite(IN4, LOW); 
            delay(5);
            stop();
            delay(30);
        } else if (sensorValue1 == HIGH && sensorValue2 == HIGH) {
            analogWrite(enA, 1); 
            analogWrite(enB, 255); 
            digitalWrite(IN1, HIGH); 
            digitalWrite(IN2, HIGH); 
            digitalWrite(IN3, LOW); 
            digitalWrite(IN4, LOW); 
            delay(5);
            stop();
            delay(30);
        }



            
        } else {
            analogWrite(enA, 30); 
            analogWrite(enB, 255); 
            digitalWrite(IN1, HIGH); 
            digitalWrite(IN2, HIGH); 
            digitalWrite(IN3, LOW);      
            digitalWrite(IN4, LOW); 
            delay(460);
            stop();
            delay(4000);

            // Perform 360-degree rotation
            analogWrite(enA, 255);
            analogWrite(enB, 255);
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);

            delay(1650); 
            stop();
            delay(2000);

            forward(80); // Move forward slowly
            delay(2500);
            stop();
            delay(2000);


            resetTilt = true;
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Tilt: ");
            lcd.print(total_tilt, 1);

            lcd.setCursor(0, 1);
            lcd.print("Dist: 10cm");
            onRamp = false;
            rampFinished = true;
            startDistanceCalc = true;

        }
      }
    

    if (!onRamp) {
        if (sensorValue1 == LOW && sensorValue2 == LOW) {
            stop();
            Serial.println("Line lost - stopping...");
        } else if (sensorValue1 == LOW && sensorValue2 == HIGH) {
            turnLeft(); 
            delay(240);
            Serial.println("Correcting left...");
        } else if (sensorValue1 == HIGH && sensorValue2 == LOW) {
            turnRight(); 
            delay(50);
            Serial.println("Correcting right...");
        } else if (sensorValue1 == HIGH && sensorValue2 == HIGH) {
            forward(100);
            Serial.println("Following line...");
        }
        delay(20);
    }

    if (resetTilt) {
        delay(2000);
        angle_pitch = 0.0;
        angle_roll = 0.0;
        angle_pitch_output = 0.0;
        angle_roll_output = 0.0;

        totalDistance = 0.0;
        startDistanceCalc = true;
        resetTilt = false;

    }

    if (startDistanceCalc && isMoving() && onRamp == false && rampFinished) {
       if (!timerRunning) {
            // Set the start time when the condition is met
            timerStartTime = millis();
            timerRunning = true;
            resetTilt = true; // Reset tilt when starting the timer
        }
       
        updateDistance();

        lcd.setCursor(0, 0);
        lcd.print("Seconds: ");
        lcd.setCursor(9,0);
        lcd.print(millis()/1000);
        lcd.print(" s    "); // Clear trailing characters

        lcd.setCursor(0, 1);
        lcd.print("Dist: ");
        lcd.print(totalDistance, 1);
        lcd.print(" cm  "); // Clear trailing characters
      }
    

    // Maintain consistent loop timing
    while (micros() - loop_timer < 4000){
    loop_timer = micros();
    }
}

bool isMoving() {
    return analogRead(enA) > 0 || analogRead(enB) > 0; // Check motor activity
}

void forward(int speed) {
    analogWrite(enA, speed);
    analogWrite(enB, speed);
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

void turnRight() {
    analogWrite(enA, 180);
    analogWrite(enB, 180);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void turnLeft() {
    analogWrite(enA, 109);
    analogWrite(enB, 109);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void handleEncoderPin1() {
    unsigned long interruptTime = millis();
    if (interruptTime - lastInterruptTime1 > debounceDelay) {
        counter1++;
        lastInterruptTime1 = interruptTime;
    }
}

void handleEncoderPin2() {
    unsigned long interruptTime = millis();
    if (interruptTime - lastInterruptTime2 > debounceDelay) {
        counter2++;
        lastInterruptTime2 = interruptTime;
    }
}

void updateDistance() {
    int totalPulses = (counter1 + counter2) / 2;
    float distance = totalPulses * cmPerPulse;
    totalDistance += distance;

    counter1 = 0;
    counter2 = 0;
}


// void calibrateMPU6050() {
//     const int calibration_samples = 2000; // Increase number of samples for better accuracy
//     float pitch_sum = 0.0, roll_sum = 0.0;

//     Serial.println("Calibrating MPU6050...");

//     for (int i = 0; i < calibration_samples; i++) {
//         sensors_event_t accel, gyro, temp;
//         mpu.getEvent(&accel, &gyro, &temp);

//         float acc_total_vector = sqrt((accel.acceleration.x * accel.acceleration.x) +
//                                       (accel.acceleration.y * accel.acceleration.y) +
//                                       (accel.acceleration.z * accel.acceleration.z));

//         float pitch = asin((float)accel.acceleration.y / acc_total_vector) * 58; // Convert to degrees
//         float roll = asin((float)accel.acceleration.x / acc_total_vector) * -58; // Convert to degrees

//         pitch_sum += pitch;
//         roll_sum += roll;

//         delay(5); // Delay to ensure stability in readings
//     }

    // pitch_offset = pitch_sum / calibration_samples;
    // roll_offset = roll_sum / calibration_samples;

//     Serial.print("Pitch Offset: ");
//     Serial.println(pitch_offset, 5); // Print with precision
//     Serial.print("Roll Offset: ");
//     Serial.println(roll_offset, 5); // Print with precision

//     Serial.println("Calibration complete.");
// }
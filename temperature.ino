#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <LiquidCrystal.h>

// Define LCD pins
const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Initialize MPU6050 object
Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);  // Wait for the Serial monitor to open
  
  Serial.println("Adafruit MPU6050 Test!");
  
  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Optional: Configure MPU6050 settings
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Initialize LCD (16x2)
  lcd.begin(16, 2);
}

void loop() {
  // Define sensor events for accelerometer, gyroscope, and temperature
  sensors_event_t a, g, temp;

  // Get new sensor events
  mpu.getEvent(&a, &g, &temp);

  // Retrieve temperature from the sensor events
  float temperature = temp.temperature;
  
  // Print temperature to Serial Monitor
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  // Display temperature on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("TEMPERATURE:");
  lcd.setCursor(0, 1);  // Move to the second row
  lcd.print(temperature, 1);  // Print temperature with 1 decimal place
  lcd.print(" C");

  delay(1000); // Wait 1 second before updating again
}

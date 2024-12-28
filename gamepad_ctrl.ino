const int enA = 3, enB = 11, IN1 = 2, IN3 = 10, IN2 = A3, IN4 = A2; // Motor control pins

int data;
int speedValue = 150; // Default speed (0-255)

void setup() {
  // Set up motor pins as outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Start hardware serial communication with Bluetooth module
  Serial.begin(9600); // Use hardware Serial for communication with HC-05
  
  // Initialize motors to stop
  Stop();
}

void loop() {
  if (Serial.available() > 0) {
    data = Serial.read(); // Read incoming data
    Serial.println(data); // Debug: print received data to Serial Monitor

    // Direction control commands (from left gamepad part)
    if (data == 'F') {
      forward();
    } else if (data == 'B') {
      back();
    } else if (data == 'L') {
      left();
    } else if (data == 'R') {
      right();
    } else if (data == 'T') {
      Stop();
    }

    // Speed control commands (from right gamepad part)
    else if (data == '+') { 
      increaseSpeed();
    } else if (data == '-') { 
      decreaseSpeed();
    }
  }
}

// Motor control functions
void forward() {
  analogWrite(enA, speedValue);
  analogWrite(enB, speedValue);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void back() {
  analogWrite(enA, speedValue);
  analogWrite(enB, speedValue);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
}

void left() {
  analogWrite(enA, speedValue / 2); // Reduce speed for turning
  analogWrite(enB, speedValue);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void right() {
  analogWrite(enA, speedValue);
  analogWrite(enB, speedValue / 2); // Reduce speed for turning
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void Stop() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// Speed control functions
void increaseSpeed() {
  speedValue += 10; // Increase speed by 10
  if (speedValue > 255) speedValue = 255; // Limit speed to max 255
  Serial.print("Speed: ");
  Serial.println(speedValue);
}

void decreaseSpeed() {
  speedValue -= 10; // Decrease speed by 10
  if (speedValue < 0) speedValue = 0; // Limit speed to min 0
  Serial.print("Speed: ");
  Serial.println(speedValue);
}

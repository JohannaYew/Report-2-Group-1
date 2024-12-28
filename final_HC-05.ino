const int enA = 3, enB = 11, IN1 = 2, IN3 = 10, IN2 = A3, IN4 = A2;
int data;

void setup() {
  // Set up motor pins as outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Start hardware serial communication with Bluetooth module
  Serial.begin(9600);  // Use hardware Serial for communication with HC-05
  
  // Initialize motors and LED to off
  Stop();
}

void loop() {
  while (Serial.available() > 0) {  // Use Serial instead of BTserial
    data = Serial.read();
    Serial.println(data);  // Print incoming data to Serial Monitor

    // Motor control commands
    if (data == 'F') {
      forward();
    } else if (data == 'B') {
      back();
    } else if (data == 'L') {
      left();
    } else if (data == 'R') {
      right();
    } else if (data == 'S') {
      Stop();
    }
  }
}

void forward() {
  analogWrite(enA, 200);
  analogWrite(enB, 200);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void back() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  analogWrite(enA, 200);
  analogWrite(enB, 200);
}

void left() {
  analogWrite(enA, 160);
  analogWrite(enB, 210);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(100);
}

void right() {
  analogWrite(enA, 255);
  analogWrite(enB, 255);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(100);
}

void Stop() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  delay(50);
}

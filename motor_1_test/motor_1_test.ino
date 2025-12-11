// --- Left side (Motor A)
#define ENA 27
#define IN1 25
#define IN2 26

// --- Right side (Motor B)
#define ENB 14
#define IN3 12
#define IN4 13

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

// helper functions
void moveForward(int speedVal) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speedVal);
  analogWrite(ENB, speedVal);
}

void moveBackward(int speedVal) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speedVal);
  analogWrite(ENB, speedVal);
}

void turnLeft(int speedVal) {
  // left side backward, right side forward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speedVal);
  analogWrite(ENB, speedVal);
}

void turnRight(int speedVal) {
  // left side forward, right side backward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speedVal);
  analogWrite(ENB, speedVal);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void loop() {
  moveForward(200);
  delay(2000);

  stopMotors();
  delay(1000);

  moveBackward(200);
  delay(2000);

  stopMotors();
  delay(1000);

  turnLeft(200);
  delay(1500);

  turnRight(200);
  delay(1500);

  stopMotors();
  delay(2000);
}
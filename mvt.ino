// Define motor driver pins
const int motor1_in1 = 7;
const int motor1_in2 = 8;
const int motor2_in1 = 11;
const int motor2_in2 = 10;

// Define motor speed PWM pins (connected to L283N's PWM pins)
const int motor1_pwm = 6;
const int motor2_pwm = 9;

void setup() {
  // Set motor driver pins as outputs
  pinMode(motor1_in1, OUTPUT);
  pinMode(motor1_in2, OUTPUT);
  pinMode(motor2_in1, OUTPUT);
  pinMode(motor2_in2, OUTPUT);

  // Set PWM pins as output
  pinMode(motor1_pwm, OUTPUT);
  pinMode(motor2_pwm, OUTPUT);

  // Initialize serial communication for debugging
  Serial.begin(9600);
  Serial.println("Motor Control Ready");
}

void loop() {
  // Example commands:
delay(2000);
  // Move motor 1 forward at half speed
  moveMotor1Forward(1); // 1 represents speed (0-1)
  moveMotor2Forward(1);
  delay(1000);


  // Move motor 2 backward at half speed
  moveMotor1Backward(1);
  moveMotor2Backward(1); // 1 represents speed (0-1)
  delay(1000);

  moveMotor1Backward(1);
  moveMotor2Forward(1); // 1 represents speed (0-1)
  delay(1000);
  moveMotor2Backward(1);
  moveMotor1Forward(1); // 1 represents speed (0-1)
  delay(1000);


   //Stop both motors
  stopMotors();
  delay(2000);


}

// Function to move motor 1 forward
void moveMotor1Forward(float speed) {
  digitalWrite(motor1_in1, HIGH);
  digitalWrite(motor1_in2, LOW);
  analogWrite(motor1_pwm, speed * 255);  // PWM control
  Serial.print("Motor 1 Forward at speed: ");
  Serial.println(speed);
}

// Function to move motor 1 backward
void moveMotor1Backward(float speed) {
  digitalWrite(motor1_in1, LOW);
  digitalWrite(motor1_in2, HIGH);
  analogWrite(motor1_pwm, speed * 255);
  Serial.print("Motor 1 Backward at speed: ");
  Serial.println(speed);
}

// Function to move motor 2 forward
void moveMotor2Forward(float speed) {
  digitalWrite(motor2_in1, HIGH);
  digitalWrite(motor2_in2, LOW);
  analogWrite(motor2_pwm, speed * 255);
  Serial.print("Motor 2 Forward at speed: ");
  Serial.println(speed);
}

// Function to move motor 2 backward
void moveMotor2Backward(float speed) {
  digitalWrite(motor2_in1, LOW);
  digitalWrite(motor2_in2, HIGH);
  analogWrite(motor2_pwm, speed * 255);
  Serial.print("Motor 2 Backward at speed: ");
  Serial.println(speed);
}

// Function to stop both motors
void stopMotors() {
  digitalWrite(motor1_in1, LOW);
  digitalWrite(motor1_in2, LOW);
  digitalWrite(motor2_in1, LOW);
  digitalWrite(motor2_in2, LOW);
  analogWrite(motor1_pwm, 0);
  analogWrite(motor2_pwm, 0);
  Serial.println("Motors Stopped");
}

// Function to rotate motors in opposite directions
void rotateMotors() {
  moveMotor1Forward(1);
  moveMotor2Backward(1);
}

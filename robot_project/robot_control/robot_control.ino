// Define motor driver pins
const int motor1_in1 = 7;
const int motor1_in2 = 8;
const int motor2_in1 = 11;
const int motor2_in2 = 10;

// Define motor speed PWM pins
const int motor1_pwm = 6;
const int motor2_pwm = 9;

// Motor speed (0-255)
const int MOTOR_SPEED = 200;  // Adjust this value as needed

void setup() {
  // Set motor driver pins as outputs
  pinMode(motor1_in1, OUTPUT);
  pinMode(motor1_in2, OUTPUT);
  pinMode(motor2_in1, OUTPUT);
  pinMode(motor2_in2, OUTPUT);

  // Set PWM pins as output
  pinMode(motor1_pwm, OUTPUT);
  pinMode(motor2_pwm, OUTPUT);

  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Robot Control Ready");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    // Process the command
    switch (command) {
      case 'F':  // Forward
        moveForward();
        Serial.println("Moving Forward");
        break;
        
      case 'B':  // Backward
        moveBackward();
        Serial.println("Moving Backward");
        break;
        
      case 'L':  // Left
        turnLeft();
        Serial.println("Turning Left");
        break;
        
      case 'R':  // Right
        turnRight();
        Serial.println("Turning Right");
        break;
        
      case 'S':  // Stop
        stopMotors();
        Serial.println("Stopped");
        break;
        
      default:
        // Ignore invalid commands
        break;
    }
  }
}

void moveForward() {
  digitalWrite(motor1_in1, HIGH);
  digitalWrite(motor1_in2, LOW);
  digitalWrite(motor2_in1, HIGH);
  digitalWrite(motor2_in2, LOW);
  analogWrite(motor1_pwm, MOTOR_SPEED);
  analogWrite(motor2_pwm, MOTOR_SPEED);
}

void moveBackward() {
  digitalWrite(motor1_in1, LOW);
  digitalWrite(motor1_in2, HIGH);
  digitalWrite(motor2_in1, LOW);
  digitalWrite(motor2_in2, HIGH);
  analogWrite(motor1_pwm, MOTOR_SPEED);
  analogWrite(motor2_pwm, MOTOR_SPEED);
}

void turnLeft() {
  digitalWrite(motor1_in1, LOW);
  digitalWrite(motor1_in2, HIGH);
  digitalWrite(motor2_in1, HIGH);
  digitalWrite(motor2_in2, LOW);
  analogWrite(motor1_pwm, MOTOR_SPEED);
  analogWrite(motor2_pwm, MOTOR_SPEED);
}

void turnRight() {
  digitalWrite(motor1_in1, HIGH);
  digitalWrite(motor1_in2, LOW);
  digitalWrite(motor2_in1, LOW);
  digitalWrite(motor2_in2, HIGH);
  analogWrite(motor1_pwm, MOTOR_SPEED);
  analogWrite(motor2_pwm, MOTOR_SPEED);
}

void stopMotors() {
  digitalWrite(motor1_in1, LOW);
  digitalWrite(motor1_in2, LOW);
  digitalWrite(motor2_in1, LOW);
  digitalWrite(motor2_in2, LOW);
  analogWrite(motor1_pwm, 0);
  analogWrite(motor2_pwm, 0);
}

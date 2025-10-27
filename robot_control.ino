// Define motor driver pins
const int motor1_in1 = 7;
const int motor1_in2 = 8;
const int motor2_in1 = 11;
const int motor2_in2 = 10;

// Define motor speed PWM pins
const int motor1_pwm = 6;
const int motor2_pwm = 9;

// Define LED pin for status indication
const int STATUS_LED = 13;  // Built-in LED

// Define Ultrasonic sensor pins
const int TRIG_PIN = 12;     // Trigger pin
const int ECHO_PIN = 13;     // Echo pin

// Define distance thresholds (in cm)
const int DANGER_DISTANCE = 20;   // Distance for emergency stop
const int WARNING_DISTANCE = 40;  // Distance for slowing down

// Motor speed (0-255)
const int MOTOR_SPEED = 200;  // Adjust this value as needed

void setup() {
  // Set motor driver pins as outputs
  pinMode(motor1_in1, OUTPUT);
  pinMode(motor1_in2, OUTPUT);
  pinMode(motor2_in1, OUTPUT);
  pinMode(motor2_in2, OUTPUT);
  
  // Set status LED pin as output
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  // Set PWM pins as output
  pinMode(motor1_pwm, OUTPUT);
  pinMode(motor2_pwm, OUTPUT);

  // Setup Ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Robot Control Ready");
}

// Buffer for incoming commands
String inputBuffer = "";

// Function to measure distance using ultrasonic sensor
int measureDistance() {
  // Clear the trigger pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Send trigger pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Measure the response
  long duration = pulseIn(ECHO_PIN, HIGH);
  
  // Calculate distance in centimeters
  int distance = duration * 0.034 / 2;
  
  return distance;
}

void loop() {
  // Measure distance and send it periodically
  static unsigned long lastDistanceTime = 0;
  if (millis() - lastDistanceTime > 100) {  // Send distance every 100ms
    int distance = measureDistance();
    Serial.print("D:");  // Distance prefix
    Serial.println(distance);
    lastDistanceTime = millis();
  }
  
  while (Serial.available() > 0) {
    char inChar = Serial.read();
    
    // Add character to buffer if it's not a newline
    if (inChar != '\n') {
      inputBuffer += inChar;
    } else {
      // Process the complete command
      char command = inputBuffer.charAt(0);
      inputBuffer = "";  // Clear buffer
    
    // Process the command
    switch (command) {
      case 'F':  // Forward
        moveForward();
        digitalWrite(STATUS_LED, HIGH);
        Serial.println("OK:Forward");
        break;
        
      case 'B':  // Backward
        moveBackward();
        digitalWrite(STATUS_LED, HIGH);
        Serial.println("OK:Backward");
        break;
        
      case 'L':  // Left
        turnLeft();
        digitalWrite(STATUS_LED, HIGH);
        Serial.println("OK:Left");
        break;
        
      case 'R':  // Right
        turnRight();
        digitalWrite(STATUS_LED, HIGH);
        Serial.println("OK:Right");
        break;
        
      case 'S':  // Stop
        stopMotors();
        digitalWrite(STATUS_LED, LOW);
        Serial.println("OK:Stop");
        break;
        
      default:
        // Invalid command received
        Serial.print("ERROR:Invalid command '");
        Serial.print(command);
        Serial.println("'");
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
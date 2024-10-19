#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>

#define BT_RX 10
#define BT_TX 11

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // Create a PWM driver object

// Define motor control channels (0-15)
#define MOTOR1_R_CHANNEL 0  // Front Left Motor Right
#define MOTOR1_L_CHANNEL 1  // Front Left Motor Left
#define MOTOR2_R_CHANNEL 3  // Front Right Motor Right
#define MOTOR2_L_CHANNEL 2  // Front Right Motor Left


#define MOTOR3_R_CHANNEL 4   // Back Left Motor Right
#define MOTOR3_L_CHANNEL 5  // Back Left Motor Left
#define MOTOR4_R_CHANNEL 6  // Back Right Motor Right
#define MOTOR4_L_CHANNEL 7  // Back Right Motor Left

// Speed control
int motorSpeed = 1000; // Max PWM value for speed control (0-4095 for PCA9685)

// Bluetooth input
String dataIn = ""; // For storing string input from Bluetooth

SoftwareSerial bt(BT_RX, BT_TX);

void setup() {
  Serial.begin(38400); // Updated baud rate to 38400 for Bluetooth communication
  bt.begin(38400);

  pwm.begin();
  pwm.setPWMFreq(60); // Set frequency to 60 Hz for servos

  // Set initial motor states to stop
  stopMotors();
}

void loop() {
  // Check if there is data available from the Bluetooth module

//  moveForward();
//  delay(2000);
//  stopMotors();
//  delay(2000);

//  moveBackward();
//  delay(2000);
//  stopMotors();
//  delay(2000);

//  moveLeft();
//  delay(2000);
// stopMotors();
// delay(2000);

//  moveCW();
//  delay(2000);
//  stopMotors();
  //delay(2000);

//  moveCCW();
//  delay(2000);
//  stopMotors();
//  delay(2000);

//  moveDiagonalForwardRight();
//  delay(2000);
//  stopMotors();
//  delay(2000);

//  moveDiagonalForwardLeft();
//  delay(2000);
//  stopMotors();
//  delay(2000);

//  moveDiagonalBackwardRight();
//  delay(2000);
//  stopMotors();
//  delay(2000);

//  moveDiagonalBackwardLeft();
//  delay(2000);
//  stopMotors();
//  delay(2000);

  if (bt.available() > 0) {
    dataIn = bt.readStringUntil('\n'); // Read the full string from Bluetooth
    Serial.println(dataIn);       // Print the received string for debugging

    // Control motors based on the input string
    if (dataIn == "FORWARD") {          // Move forward
      moveForward();
    } else if (dataIn == "BACKWARD") {   // Move backward
      moveBackward();
    } else if (dataIn == "RIGHT") {   // Turn right
      moveRight();
    } else if (dataIn == "LEFT") {   // Turn left
      moveLeft();
    } else if (dataIn == "STOP") {   // Stop
      stopMotors();
    } else if (dataIn == "CW") {   // Clockwise movement
      moveCW();
    } else if (dataIn == "CCW") {   // Counterclockwise movement
      moveCCW();
    } else if (dataIn == "FORWARD_RIGHT") {  // Move forward-right diagonal
      moveDiagonalForwardRight();
    } else if (dataIn == "FORWARD_LEFT") {  // Move forward-left diagonal
      moveDiagonalForwardLeft();
    } else if (dataIn == "BACKWARD_RIGHT") {  // Move backward-right diagonal
      moveDiagonalBackwardRight();
    } else if (dataIn == "BACKWARD_LEFT") {  // Move backward-left diagonal
      moveDiagonalBackwardLeft();
    }
  }
}

void setMotorPWM(int channel, int value) {
  pwm.setPWM(channel, 0, value); // Set the PWM value for the specified motor channel
}

void moveForward() {
  setMotorPWM(MOTOR1_R_CHANNEL, motorSpeed);
  setMotorPWM(MOTOR1_L_CHANNEL, 0);
  setMotorPWM(MOTOR2_R_CHANNEL, motorSpeed);
  setMotorPWM(MOTOR2_L_CHANNEL, 0);
  setMotorPWM(MOTOR3_R_CHANNEL, motorSpeed);
  setMotorPWM(MOTOR3_L_CHANNEL, 0);
  setMotorPWM(MOTOR4_R_CHANNEL, motorSpeed);
  setMotorPWM(MOTOR4_L_CHANNEL, 0);
}

void moveBackward() {
  setMotorPWM(MOTOR1_R_CHANNEL, 0);
  setMotorPWM(MOTOR1_L_CHANNEL, motorSpeed);
  setMotorPWM(MOTOR2_R_CHANNEL, 0);
  setMotorPWM(MOTOR2_L_CHANNEL, motorSpeed);
  setMotorPWM(MOTOR3_R_CHANNEL, 0);
  setMotorPWM(MOTOR3_L_CHANNEL, motorSpeed);
  setMotorPWM(MOTOR4_R_CHANNEL, 0);
  setMotorPWM(MOTOR4_L_CHANNEL, motorSpeed);
}

void moveRight() {
  setMotorPWM(MOTOR1_R_CHANNEL, (motorSpeed*4.095));
  setMotorPWM(MOTOR1_L_CHANNEL, 0);
  setMotorPWM(MOTOR2_R_CHANNEL, 0);
  setMotorPWM(MOTOR2_L_CHANNEL, (motorSpeed*4.095));
  setMotorPWM(MOTOR3_R_CHANNEL, 0);
  setMotorPWM(MOTOR3_L_CHANNEL, (motorSpeed*4.095));
  setMotorPWM(MOTOR4_R_CHANNEL, (motorSpeed*4.095));
  setMotorPWM(MOTOR4_L_CHANNEL, 0);
}

void moveLeft() {
  setMotorPWM(MOTOR1_R_CHANNEL, 0);
  setMotorPWM(MOTOR1_L_CHANNEL, (motorSpeed*4.095));
  setMotorPWM(MOTOR2_R_CHANNEL, (motorSpeed*4.095));
  setMotorPWM(MOTOR2_L_CHANNEL, 0);
  setMotorPWM(MOTOR3_R_CHANNEL, (motorSpeed*4.095));
  setMotorPWM(MOTOR3_L_CHANNEL, 0);
  setMotorPWM(MOTOR4_R_CHANNEL, 0);
  setMotorPWM(MOTOR4_L_CHANNEL, (motorSpeed*4.095));
}

void moveCW() {
  setMotorPWM(MOTOR1_R_CHANNEL, (motorSpeed*1.5));
  setMotorPWM(MOTOR1_L_CHANNEL, 0);
  setMotorPWM(MOTOR2_R_CHANNEL, 0);
  setMotorPWM(MOTOR2_L_CHANNEL, (motorSpeed*1.5));
  setMotorPWM(MOTOR3_R_CHANNEL, (motorSpeed*1.5));
  setMotorPWM(MOTOR3_L_CHANNEL, 0);
  setMotorPWM(MOTOR4_R_CHANNEL, 0);
  setMotorPWM(MOTOR4_L_CHANNEL, (motorSpeed*1.5));
}

void moveCCW() {
  setMotorPWM(MOTOR1_R_CHANNEL, 0);
  setMotorPWM(MOTOR1_L_CHANNEL, (motorSpeed*1.5));
  setMotorPWM(MOTOR2_R_CHANNEL, (motorSpeed*1.5));
  setMotorPWM(MOTOR2_L_CHANNEL, 0);
  setMotorPWM(MOTOR3_R_CHANNEL, 0);
  setMotorPWM(MOTOR3_L_CHANNEL, (motorSpeed*1.5));
  setMotorPWM(MOTOR4_R_CHANNEL, (motorSpeed*1.5));
  setMotorPWM(MOTOR4_L_CHANNEL, 0);
}

// Diagonal Movements
void moveDiagonalForwardRight() {
  setMotorPWM(MOTOR1_R_CHANNEL, (motorSpeed*3));
  setMotorPWM(MOTOR1_L_CHANNEL, 0);
  setMotorPWM(MOTOR2_R_CHANNEL, 0);
  setMotorPWM(MOTOR2_L_CHANNEL, 0);
  setMotorPWM(MOTOR3_R_CHANNEL, 0);
  setMotorPWM(MOTOR3_L_CHANNEL, 0);
  setMotorPWM(MOTOR4_R_CHANNEL, (motorSpeed*3));
  setMotorPWM(MOTOR4_L_CHANNEL, 0);
}

void moveDiagonalForwardLeft() {
  setMotorPWM(MOTOR1_R_CHANNEL, 0);
  setMotorPWM(MOTOR1_L_CHANNEL, 0);
  setMotorPWM(MOTOR2_R_CHANNEL, (motorSpeed*3));
  setMotorPWM(MOTOR2_L_CHANNEL, 0);
  setMotorPWM(MOTOR3_R_CHANNEL, (motorSpeed*3));
  setMotorPWM(MOTOR3_L_CHANNEL, 0);
  setMotorPWM(MOTOR4_R_CHANNEL, 0);
  setMotorPWM(MOTOR4_L_CHANNEL, 0);
}

void moveDiagonalBackwardRight() {
  setMotorPWM(MOTOR1_R_CHANNEL, 0);
  setMotorPWM(MOTOR1_L_CHANNEL, (motorSpeed*3));
  setMotorPWM(MOTOR2_R_CHANNEL, 0);
  setMotorPWM(MOTOR2_L_CHANNEL, 0);
  setMotorPWM(MOTOR3_R_CHANNEL, 0);
  setMotorPWM(MOTOR3_L_CHANNEL, 0);
  setMotorPWM(MOTOR4_R_CHANNEL, 0);
  setMotorPWM(MOTOR4_L_CHANNEL, (motorSpeed*3));
}

void moveDiagonalBackwardLeft() {
  setMotorPWM(MOTOR1_R_CHANNEL, 0);
  setMotorPWM(MOTOR1_L_CHANNEL, 0);
  setMotorPWM(MOTOR2_R_CHANNEL, 0);
  setMotorPWM(MOTOR2_L_CHANNEL, (motorSpeed*3));
  setMotorPWM(MOTOR3_R_CHANNEL, 0);
  setMotorPWM(MOTOR3_L_CHANNEL, (motorSpeed*3));
  setMotorPWM(MOTOR4_R_CHANNEL, 0);
  setMotorPWM(MOTOR4_L_CHANNEL, 0);
}

void stopMotors() {
  setMotorPWM(MOTOR1_R_CHANNEL, 0);
  setMotorPWM(MOTOR1_L_CHANNEL, 0);
  setMotorPWM(MOTOR2_R_CHANNEL, 0);
  setMotorPWM(MOTOR2_L_CHANNEL, 0);
  setMotorPWM(MOTOR3_R_CHANNEL, 0);
  setMotorPWM(MOTOR3_L_CHANNEL, 0);
  setMotorPWM(MOTOR4_R_CHANNEL, 0);
  setMotorPWM(MOTOR4_L_CHANNEL, 0);
}

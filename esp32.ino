// Blynk Template and Auth Configuration
#define BLYNK_TEMPLATE_ID "YourTemplateID"       // Replace with your Blynk Template ID
#define BLYNK_TEMPLATE_NAME "YourTemplateName"   // Replace with your Blynk Template Name
#define BLYNK_AUTH_TOKEN "YourAuthToken"         // Replace with your Blynk Auth Token

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Servo.h>
#include <BluetoothSerial.h>

// Pin Definitions
#define trigPin 13       // Ultrasonic sensor trigger pin
#define echoPin 12       // Ultrasonic sensor echo pin
#define servoPin 14      // Servo motor pin
#define motor1Pin1 25    // Motor 1 control pin 1
#define motor1Pin2 26    // Motor 1 control pin 2
#define motor2Pin1 27    // Motor 2 control pin 1
#define motor2Pin2 32    // Motor 2 control pin 2

// Wi-Fi credentials
char ssid[] = "YourWiFiSSID";  // Replace with your Wi-Fi SSID
char pass[] = "YourWiFiPassword"; // Replace with your Wi-Fi password

// Bluetooth setup
BluetoothSerial SerialBT;

// Servo motor for ultrasonic sensor
Servo myServo;

// Flags for control modes
bool wifiControl = false; // Wi-Fi control flag
bool autoMode = false;    // Autonomous mode flag

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize Bluetooth
  SerialBT.begin("ESP32_Car"); // Name of the Bluetooth device

  // Initialize Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  // Initialize ultrasonic sensor and servo
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  myServo.attach(servoPin);
  myServo.write(90); // Center the servo

  // Initialize motor pins
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  // Stop the car initially
  stopCar();
}

void loop() {
  // Run Blynk for Wi-Fi control
  Blynk.run();

  // Check for Bluetooth commands
  if (SerialBT.available() && !wifiControl) {
    char command = SerialBT.read();
    handleBluetoothCommand(command);
  }

  // Autonomous obstacle avoidance
  if (autoMode) {
    avoidObstacle();
  }
}

// Function to handle Bluetooth commands
void handleBluetoothCommand(char command) {
  switch (command) {
    case 'F': // Move Forward
      moveForward();
      break;
    case 'B': // Move Backward
      moveBackward();
      break;
    case 'L': // Turn Left
      turnLeft();
      break;
    case 'R': // Turn Right
      turnRight();
      break;
    case 'S': // Stop
      stopCar();
      break;
    case 'A': // Toggle Autonomous Mode
      autoMode = !autoMode;
      break;
  }
}

// Function to measure distance using ultrasonic sensor
long getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  long distance = (duration / 2) / 29.1; // Convert to cm
  return distance;
}

// Function to avoid obstacles
void avoidObstacle() {
  long distance = getDistance();
  if (distance < 20) { // If obstacle is within 20 cm
    stopCar();

    // Rotate servo to check left and right
    myServo.write(0); // Look left
    delay(500);
    long leftDistance = getDistance();

    myServo.write(180); // Look right
    delay(500);
    long rightDistance = getDistance();

    myServo.write(90); // Return to center
    delay(500);

    // Decide direction
    if (leftDistance > rightDistance) {
      turnLeft();
    } else {
      turnRight();
    }
    delay(500); // Turn for 0.5 seconds
  } else {
    moveForward(); // Continue moving forward
  }
}

// Motor control functions
void moveForward() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
}

void moveBackward() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
}

void turnLeft() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
}

void turnRight() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
}

void stopCar() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
}

// Blynk Virtual Pin Functions
BLYNK_WRITE(V1) { // Virtual Pin 1 for Forward
  wifiControl = true;
  int value = param.asInt();
  if (value == 1) {
    moveForward();
  } else {
    stopCar();
  }
}

BLYNK_WRITE(V2) { // Virtual Pin 2 for Backward
  wifiControl = true;
  int value = param.asInt();
  if (value == 1) {
    moveBackward();
  } else {
    stopCar();
  }
}

BLYNK_WRITE(V3) { // Virtual Pin 3 for Left
  wifiControl = true;
  int value = param.asInt();
  if (value == 1) {
    turnLeft();
  } else {
    stopCar();
  }
}

BLYNK_WRITE(V4) { // Virtual Pin 4 for Right
  wifiControl = true;
  int value = param.asInt();
  if (value == 1) {
    turnRight();
  } else {
    stopCar();
  }
}

BLYNK_WRITE(V5) { // Virtual Pin 5 for Autonomous Mode
  wifiControl = true;
  autoMode = param.asInt();
}

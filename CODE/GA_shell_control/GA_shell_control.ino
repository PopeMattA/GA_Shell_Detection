#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#define RELAY_PIN 4

#define FLOAT_SWITCH_PIN 8  // Pin connected to the float switch

// Create PWM servo driver instance
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define constants for servo positions
const int SERVOMIN = 150;  // 150 should be the actual min or maybe 0
// Minimum pulse length (0°)
const int SERVOMAX = 600; // Maximum pulse length (180°)

// Define servo channel and positions
const int servoChannel = 0;  // The servo connected to channel 0 on PCA9865
const int position0 = 450;  // 0 degrees
const int position180 = SERVOMAX; // 180 degrees

const int motorPin1 = 9;
const int motorPin2 = 10;
const int motorPWM = 11;

//Define button commands
const int goodSumpButton = 12;  // Button to move servo to 0°
const int badSumpButton = 13;   // Button to move servo to 180°

const int sensorPin = 2;

//const int BUTTON_PIN = 7; // pin of button on Arduino
bool motor_running = false;
bool servo_reset_done = false;

int currentPositionState = 0; // Tracks current state (0 or 180 degrees)

bool relayState = false;
unsigned long lastRelayToggleTime = 0;
//const unsigned long relayInterval = 5000; // 5 seconds
const unsigned long relayOnTime = 3000;  // Relay stays ON for 3 seconds
const unsigned long relayOffTime = 7000; // Relay stays OFF for 7 seconds

int ena = 5;
int in1 = 6;
int in2 = 7;

void setup() {
  Serial.begin(9600); // Initialize Serial communication
  Serial.println("Press 'g' to go to position 0 degrees (Good sump).");
  Serial.println("Press 'b' to go to position 180 degrees (Bad sump).");

  // Initialize the PCA9685
  pwm.begin();
  pwm.setPWMFreq(50); // Set frequency to 50 Hz for servos

  pinMode(goodSumpButton, INPUT_PULLUP);
  pinMode(badSumpButton, INPUT_PULLUP);
  

  pinMode(ena, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(sensorPin, INPUT_PULLUP);  // This enables the internal pull-up resistor

  pinMode(FLOAT_SWITCH_PIN, INPUT_PULLUP);  // Use internal pull-up resistor
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);  // Keep relay OFF initially
}

void loop() {
  reset_servo();
  handleButtons();
  toggleRelay();
  runDC();
  sensorDetect();
}

void reset_servo() {
  if (!servo_reset_done) {
    setServoPosition(600);
    delay(1000);

    setServoPosition(150);
    delay(1500);

    setServoPosition(600);
    delay(1000);

    servo_reset_done = true;
    Serial.println("Servo is reset and you can now dump.");
  }
}

void handleButtons() {
  static bool lastGoodSumpState = LOW;  // NC button starts LOW
  static bool lastBadSumpState = LOW;

  bool goodSumpState = digitalRead(goodSumpButton);
  bool badSumpState = digitalRead(badSumpButton);

  // Detect release (transition from LOW to HIGH)
  if (goodSumpState == HIGH && lastGoodSumpState == LOW) {
    Serial.println("Moving to good sump.");
    setServoPosition(position0);
    currentPositionState = 0;
  }

  if (badSumpState == HIGH && lastBadSumpState == LOW) {
    Serial.println("Moving to bad sump.");
    setServoPosition(position180);
    currentPositionState = 180;
  }

  lastGoodSumpState = goodSumpState;
  lastBadSumpState = badSumpState;
}

void toggleRelay() {
  int floatState = digitalRead(FLOAT_SWITCH_PIN);  // Read float switch state


  if (floatState == LOW) {  // If float switch is activated (closed)
        digitalWrite(RELAY_PIN, HIGH);  // Turn ON relay (active low)
        Serial.println("flow no go");
  } else {
        digitalWrite(RELAY_PIN, LOW); // Turn OFF relay
        Serial.println("flow go");
  }


//   unsigned long currentMillis = millis();

//   // Check if it's time to toggle the relay
//   if (relayState && (currentMillis - lastRelayToggleTime >= relayOnTime)) {
//     relayState = false;  // Turn relay OFF
//     digitalWrite(RELAY_PIN, LOW);
//     lastRelayToggleTime = currentMillis;  // Reset timer
//   }
//   else if (!relayState && (currentMillis - lastRelayToggleTime >= relayOffTime)) {
//     relayState = true;  // Turn relay ON
//     digitalWrite(RELAY_PIN, HIGH);
//     lastRelayToggleTime = currentMillis;  // Reset timer
//   }
// }
}

void runDC() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(ena, 24);
}

// Function to set servo position
void setServoPosition(int position) {
  pwm.setPWM(servoChannel, 0, position); // Set PWM signal for the servo
}

void sensorDetect() {
  if (digitalRead(sensorPin) == LOW) {
    Serial.println("Shell detected! Take a picture!");
  }
}
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define RELAY_PIN 4
#define FLOAT_SWITCH_PIN 8  // Pin connected to the float switch
#define FLOAT_SWITCH_PIN_2 9 // Pin connected to second float switch
#define SENSOR_PIN 2        // NPN sensor connected to digital pin 2
#define CAMERA_PIN 11       // LED/camera trigger on pin 13

// Create PWM servo driver instance
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define constants for servo positions
const int SERVOMIN = 150;  
const int SERVOMAX = 600; 

const int servoChannel = 0;  // The servo connected to channel 0 on PCA9865
const int position0 = 450;   // 0 degrees
const int position180 = SERVOMAX; // 180 degrees

// Define button commands
const int goodSumpButton = 12;  // Button to move servo to 0°
const int badSumpButton = 13;   // Button to move servo to 180°

bool motor_running = false;
bool servo_reset_done = false;
int currentPositionState = 0; 

bool relayState = false;
unsigned long lastRelayToggleTime = 0;

int ena = 5;
int in1 = 6;
int in2 = 7;

// Physical parameters - adjust these to your actual setup
const float shellDiameter = 4;         
const float sensorWidth = 2.5;          
const float distanceToCamera = 200.0;   
const float tubeDiameterAtSensor = 7.0;  
const float tubeDiameterAtCamera = 10;  
const float expansionPoint = 100;      
const float DELAY_CONSTANT = 50; // Time Delay Constant for LED (camera)

// Timing variables
volatile unsigned long detectionStartTime = 0;
volatile unsigned long detectionEndTime = 0;
volatile bool shellDetected = false;
volatile bool measurementComplete = false;
volatile float shellVelocity = 0.0;
volatile float adjustedVelocity = 0.0;
volatile float triggerDelay = 0; 

// Non-blocking timing
unsigned long triggerTime = 0;
unsigned long cameraOffTime = 0;
bool triggerScheduled = false;
bool cameraOn = false;

void setup() {
    Serial.begin(9600);
    Serial.println("System Initialized");

    // Initialize PCA9685 for servo
    pwm.begin();
    pwm.setPWMFreq(50);

    pinMode(goodSumpButton, INPUT_PULLUP);
    pinMode(badSumpButton, INPUT_PULLUP);

    pinMode(ena, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

    pinMode(FLOAT_SWITCH_PIN, INPUT_PULLUP);
    pinMode(FLOAT_SWITCH_PIN_2, INPUT_PULLUP);
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, HIGH);  

    pinMode(SENSOR_PIN, INPUT_PULLUP);
    pinMode(CAMERA_PIN, OUTPUT);
    digitalWrite(CAMERA_PIN, LOW);

    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), sensorChange, CHANGE);
}

void loop() {
    reset_servo();
    //handleButtonsTime();
    handleButtonsManual();
    toggleRelay();
    runDC();
    sensorDetect();
    handleCameraTrigger();
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
        Serial.println("Servo is reset and ready.");
    }
}

void handleButtonsTime() {
    static bool lastGoodSumpState = LOW;
    static bool lastBadSumpState = LOW;
    static unsigned long goodSumpPressedTime = 0;
    static unsigned long badSumpPressedTime = 0;
    static bool goodSumpPending = false;
    static bool badSumpPending = false;
    static float delayTime = 3000;  // Default to 3 seconds if no sensor data

    bool goodSumpState = digitalRead(goodSumpButton);
    bool badSumpState = digitalRead(badSumpButton);
    unsigned long currentMillis = millis();

    // Calculate delay time dynamically based on shell speed
    if (shellVelocity > 0) {
        float switchingDistance = 50.0;  // Distance to switching point (adjust as needed)
        delayTime = (switchingDistance / shellVelocity) * 1000.0;  // Convert to milliseconds
    }

    // Detect button press and start countdown
    if (goodSumpState == HIGH && lastGoodSumpState == LOW) {
        Serial.println("Good sump button pressed, waiting dynamically...");
        goodSumpPressedTime = currentMillis;
        goodSumpPending = true;
    }

    if (badSumpState == HIGH && lastBadSumpState == LOW) {
        Serial.println("Bad sump button pressed, waiting dynamically...");
        badSumpPressedTime = currentMillis;
        badSumpPending = true;
    }

    // Check if calculated time has passed and move servo accordingly
    if (goodSumpPending && (currentMillis - goodSumpPressedTime >= delayTime)) {
        Serial.println("Moving to good sump.");
        setServoPosition(position0);
        currentPositionState = 0;
        goodSumpPending = false; // Reset flag
    }

    if (badSumpPending && (currentMillis - badSumpPressedTime >= delayTime)) {
        Serial.println("Moving to bad sump.");
        setServoPosition(position180);
        currentPositionState = 180;
        badSumpPending = false; // Reset flag
    }

    lastGoodSumpState = goodSumpState;
    lastBadSumpState = badSumpState;
}

void handleButtonsManual() {
    static bool lastGoodSumpState = LOW;
    static bool lastBadSumpState = LOW;

    bool goodSumpState = digitalRead(goodSumpButton);
    bool badSumpState = digitalRead(badSumpButton);

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
    int floatState = digitalRead(FLOAT_SWITCH_PIN);
    int floatState2 = digitalRead(FLOAT_SWITCH_PIN_2);
    if (floatState == LOW || floatState2 == LOW) {  
        digitalWrite(RELAY_PIN, HIGH);  
        //Serial.println("Flow no go");
    } else {
        digitalWrite(RELAY_PIN, LOW);  
        //Serial.println("Flow go");
    }
}

void runDC() {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(ena, 24);
}

void setServoPosition(int position) {
    pwm.setPWM(servoChannel, 0, position);
}

void sensorDetect() {
    if (measurementComplete) {

        sensorChange();
        float shellPassTime = (detectionEndTime - detectionStartTime) / 1000.0;  

        Serial.print("Shell detected! Pass time: ");
        Serial.print(shellPassTime, 2);
        Serial.println(" ms");

        Serial.print("Initial velocity: ");
        Serial.print(shellVelocity);
        Serial.println(" m/s");

        adjustedVelocity = shellVelocity * sq(tubeDiameterAtSensor / tubeDiameterAtCamera);

        Serial.print("Adjusted velocity after tube expansion: ");
        Serial.print(adjustedVelocity);
        Serial.println(" m/s");

        if (expansionPoint < distanceToCamera) {
            float timeToExpansion = expansionPoint / shellVelocity / 1000.0;
            float timeAfterExpansion = (distanceToCamera - expansionPoint) / adjustedVelocity / 1000.0;
            triggerDelay = (timeToExpansion + timeAfterExpansion) * 1000.0;
        } else {
            triggerDelay = ((distanceToCamera + DELAY_CONSTANT) / shellVelocity) * 1000.0;
        }

        Serial.print("Trigger delay: ");
        Serial.print(triggerDelay);
        Serial.println(" ms");

        triggerTime = millis() + triggerDelay;
        triggerScheduled = true;

        measurementComplete = false;
    }
}

void handleCameraTrigger() {
    if (triggerScheduled && millis() >= triggerTime) {
        digitalWrite(CAMERA_PIN, HIGH);
        Serial.println("Camera triggered!");
        cameraOffTime = millis() + 50;  
        cameraOn = true;
        triggerScheduled = false;
    }

    if (cameraOn && millis() >= cameraOffTime) {
        digitalWrite(CAMERA_PIN, LOW);
        cameraOn = false;
    }
}

// Interrupt handler for sensor state changes
void sensorChange() {
    if (digitalRead(SENSOR_PIN) == LOW) {
        detectionStartTime = micros();
        shellDetected = true;
    } else if (shellDetected) {
        detectionEndTime = micros();
        shellDetected = false;

        float timeDiff = (detectionEndTime - detectionStartTime) / 1000000.0;
        float effectiveWidth = shellDiameter + sensorWidth;

        shellVelocity = effectiveWidth / timeDiff / 1000.0;
        measurementComplete = true;
    }
}
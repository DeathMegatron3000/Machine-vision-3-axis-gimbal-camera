/**
 * 3-Axis AI Gimbal Controller
 * Platform: ESP32
 * Motors: NEMA 17 (Pan), MG90S (Tilt), ES08MDII (Roll)
 * Sensor: MPU6500/MPU9250
 */

#include <Wire.h>
#include <MPU9250_WE.h>
#include <ESP32Servo.h>
#include <AccelStepper.h>

// --- PIN DEFINITIONS ---
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define MPU_ADDR 0x68

#define PAN_STEP_PIN 25
#define PAN_DIR_PIN 26
#define TILT_SERVO_PIN 17
#define ROLL_SERVO_PIN 16

// --- TUNING PARAMETERS ---
// Roll Stabilization (PID-ish)
const float ROLL_KP = 1.3;        // Proportional gain for Roll
const float ROLL_OFFSET = 90.0;   // Mechanical center of the servo

// Vision Tracking Gains
// Adjust these if tracking is too slow (increase) or oscillates (decrease)
const float PAN_GAIN = 0.25;      // Stepper steps per pixel error
const float TILT_GAIN = 0.08;     // Servo degrees per pixel error

// --- OBJECTS ---
MPU9250_WE myMPU = MPU9250_WE(MPU_ADDR);
Servo RollServo;
Servo TiltServo;
// Interface 1 = Driver (STEP + DIR)
AccelStepper panStepper(AccelStepper::DRIVER, PAN_STEP_PIN, PAN_DIR_PIN);

// --- VARIABLES ---
float fusedRoll = 0.0;
unsigned long lastImuTime = 0;
float currentTiltAngle = 90.0;    // Start Tilt at middle

// Serial Parsing
String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // 1. Setup IMU
  if (!myMPU.init()) {
    Serial.println("IMU Connection Failed!");
  } else {
    Serial.println("IMU Connected.");
  }
  
  Serial.println("Calibrating IMU (Keep flat)...");
  delay(1000);
  myMPU.autoOffsets();
  
  // IMU Filters
  myMPU.setSampleRateDivider(5);
  myMPU.setAccRange(MPU9250_ACC_RANGE_2G);
  myMPU.enableAccDLPF(true);
  myMPU.setAccDLPF(MPU9250_DLPF_6);

  // 2. Setup Servos
  RollServo.attach(ROLL_SERVO_PIN);
  TiltServo.attach(TILT_SERVO_PIN);
  RollServo.write(ROLL_OFFSET);
  TiltServo.write(currentTiltAngle);

  // 3. Setup Stepper
  // Adjust MaxSpeed based on microstepping (e.g., 1/16 needs higher values)
  panStepper.setMaxSpeed(3000);
  panStepper.setAcceleration(2000);
  panStepper.setMinPulseWidth(20); 

  Serial.println("GIMBAL INITIALIZED");
}

void loop() {
  // --- CRITICAL: STEPPER MOTION ---
  // This must be called as fast as possible in the loop.
  // Do not use delay() inside loop()!
  panStepper.run();

  // --- IMU STABILIZATION (ROLL) ---
  // Run this approx every 10ms (100Hz)
  unsigned long currentTime = millis();
  if ((currentTime - lastImuTime) >= 10) {
    float dt = (currentTime - lastImuTime) / 1000.0;
    lastImuTime = currentTime;

    // Read Sensor
    xyzFloat gyr = myMPU.getGyrValues();
    float accRoll = myMPU.getPitch(); // NOTE: Check if getPitch or getRoll suits your mount
    
    // Complementary Filter
    fusedRoll = 0.98 * (fusedRoll + gyr.y * dt) + 0.02 * accRoll;

    // Calculate Correction (Target = 0 degrees flat)
    float error = 0 - fusedRoll;
    float correction = ROLL_KP * error;
    
    // Write to Servo
    float servoPos = ROLL_OFFSET + correction;
    servoPos = constrain(servoPos, 0, 180);
    RollServo.write(servoPos);
  }

  // --- SERIAL COMMUNICATION ---
  // Read incoming data from Raspberry Pi
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }

  // Parse Command
  if (stringComplete) {
    parseVisionData();
    inputString = "";
    stringComplete = false;
  }
}

void parseVisionData() {
  // Expected Format: "P<pan_error>T<tilt_error>"
  // Example: "P-40T20"
  
  int pIndex = inputString.indexOf('P');
  int tIndex = inputString.indexOf('T');

  if (pIndex != -1 && tIndex != -1) {
    String pStr = inputString.substring(pIndex + 1, tIndex);
    String tStr = inputString.substring(tIndex + 1);

    int panError = pStr.toInt();
    int tiltError = tStr.toInt();

    // 1. Update Pan (Stepper)
    // Relative movement based on error
    if (panError != 0) {
      long targetPos = panStepper.currentPosition() + (panError * PAN_GAIN);
      panStepper.moveTo(targetPos);
    }

    // 2. Update Tilt (Servo)
    if (tiltError != 0) {
      // Invert +/- if tilt moves wrong way
      currentTiltAngle += (tiltError * TILT_GAIN);
      currentTiltAngle = constrain(currentTiltAngle, 10, 170); // Safety limits
      TiltServo.write(currentTiltAngle);
    }
  }
}

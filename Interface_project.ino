#include <Arduino.h>
#include <driver/ledc.h>
#include <Wire.h>
#include <MPU6050.h>
#include <VL53L0X.h>
#include <WiFi.h>
#include <WebServer.h>
#include "BluetoothSerial.h"

// --------------------------
// Bluetooth Configuration
// --------------------------
BluetoothSerial SerialBT;

// --------------------------
// Motor Control Pin Definitions
// --------------------------
#define MOTOR1_IN1 26
#define MOTOR1_IN2 27
#define MOTOR1_ENA 25  // PWM-capable GPIO pin

#define MOTOR2_IN3 14
#define MOTOR2_IN4 12
#define MOTOR2_ENA 13  // PWM-capable GPIO pin

// --------------------------
// Encoder Pin Definitions
// --------------------------
#define ENCODER1_A 32
#define ENCODER1_B 33
#define ENCODER2_A 18
#define ENCODER2_B 19

// IR sensor pins
#define IR_SENSOR_1 23
#define IR_SENSOR_2 4

// I2C pins
#define I2C_SDA 21
#define I2C_SCL 22

// VL53L0X object
VL53L0X lidar;

// --------------------------
// Global Variables
// --------------------------
MPU6050 mpu;

float yaw = 0; // Yaw angle
float Kp = 4.3199, Ki = 2.4, Kd = 2.755;
const float derivativeAlpha = 0.1;
float prev_error = 0;
int Base_PWM = 130; 
float cycleError = 0;
volatile int encoder_left_count = 0; 
volatile int encoder_right_count = 0; 
const float wheel_diameter = 4.4;
const int PPR = 210;              
const float wheel_circumference = 3.14159 * wheel_diameter;

// PID State Variables
float integral   = 0.0;
float derivative = 0.0;
unsigned long lastTime = 0;
const float maxIntegral = 1000.0;

float desiredYawAngle = 0;
float gyroBiasZ = 0;

// *** ADDED *** Maze size, robot state, visited array
static const int MAZE_WIDTH  = 8;
static const int MAZE_HEIGHT = 8;

// Robot heading: 0=N, 1=E, 2=S, 3=W
int robotX = 0;       // Start at leftmost bottom cell => (0,0)
int robotY = 0;       // 
int robotHeading = 0; // 0=N (facing upward in standard 2D grid)
bool visited[MAZE_WIDTH][MAZE_HEIGHT]; // track visited cells

// --------------------------
// LEDC (PWM) Configuration
// --------------------------
void configureLEDC(uint8_t channel, uint8_t pin, uint32_t freq, uint8_t resolution) {
  // Configure LEDC timer
  ledc_timer_config_t ledcTimer = {
    .speed_mode       = LEDC_LOW_SPEED_MODE,
    .duty_resolution  = (ledc_timer_bit_t)resolution,
    .timer_num        = LEDC_TIMER_0,
    .freq_hz          = freq,
    .clk_cfg          = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledcTimer);

  // Configure LEDC channel
  ledc_channel_config_t ledcConfig = {
    .gpio_num       = pin,
    .speed_mode     = LEDC_LOW_SPEED_MODE,
    .channel        = (ledc_channel_t)channel,
    .intr_type      = LEDC_INTR_DISABLE,
    .timer_sel      = LEDC_TIMER_0,
    .duty           = 0,
    .hpoint         = 0
  };
  ledc_channel_config(&ledcConfig);
}

// --------------------------
// LiDAR Functions
// --------------------------
void initLiDAR() {
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!lidar.init()) {
    Serial.println("Failed to initialize VL53L0X!");
    while (1);
  }
  lidar.setTimeout(500);
  lidar.startContinuous();
  Serial.println("LiDAR initialized.");
}

int readLiDAR() {
  int distance = lidar.readRangeContinuousMillimeters();
  if (lidar.timeoutOccurred()) {
    Serial.println("LiDAR timeout!");
    return -1;
  }
  return distance;
}

// --------------------------
// Motor & Encoder
// --------------------------
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  int leftDuty  = abs(leftSpeed);
  int rightDuty = abs(rightSpeed);

  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)0, leftDuty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)0);

  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)1, rightDuty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)1);
}

void updateEncoderLeft()  { encoder_left_count++; }
void updateEncoderRight() { encoder_right_count++; }

void initialize_motors() {
  // Forward direction
  digitalWrite(MOTOR1_IN1, HIGH);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN3, HIGH);
  digitalWrite(MOTOR2_IN4, LOW);
  setMotorSpeed(Base_PWM, Base_PWM);
}

void stopMotors() {
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN3, LOW);
  digitalWrite(MOTOR2_IN4, LOW);
  setMotorSpeed(0, 0);
}

// --------------------------
// MPU (Gyro) Functions
// --------------------------
void initializeMPU6050() {
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    return;
  }
  Serial.println("MPU6050 initialized successfully!");
}

float readYaw() {
  static unsigned long lastTime = 0;
  if (lastTime == 0) lastTime = micros();

  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);

  float gyroZ = (gz / 131.0f) - gyroBiasZ; // Subtract bias

  unsigned long currentTime = micros();
  float deltaTime = (currentTime - lastTime) / 1000000.0f;
  lastTime = currentTime;
  if (deltaTime <= 0.0f || deltaTime > 1.0f) {
    deltaTime = 0.01f;
  }

  yaw += gyroZ * deltaTime;
  // wrap yaw in [0..360)
  if (yaw >= 360.0f) yaw -= 360.0f;
  if (yaw < 0.0f)    yaw += 360.0f;

  SerialBT.printf("Yaw: %.2f, GyroZ: %.2f, DeltaTime: %.6f\n", yaw, gyroZ, deltaTime);
  return yaw;
}

// --------------------------
// Basic PID update
// --------------------------
void updatePID(float error) {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0f;
  if (deltaTime <= 0.0f) deltaTime = 0.01f;

  integral += error * deltaTime;
  integral = constrain(integral, -maxIntegral, maxIntegral);

  float derivativeRaw = (error - prev_error) / deltaTime;
  derivative = derivativeAlpha * derivativeRaw + (1.0f - derivativeAlpha) * derivative;

  prev_error = error;
  lastTime   = currentTime;
}

// --------------------------
// moveForward(...) with ramp & correction
// --------------------------

void moveForward(float target_distance) {

  // Reset encoder counts
  encoder_left_count = 0;
  encoder_right_count = 0;

  // --- Local (Distance) PID State Variables ---
  float distanceIntegral = 0;
  float distancePrevError = 0;
  unsigned long lastTimeDist = millis();
  
  // --- PID Gains (tune these values as needed) ---
  // Distance PID gains
  float Kp_dist = Kp;
  float Ki_dist = Ki;
  float Kd_dist = Kd;
  
  // Heading PID gains (for the correction; these work with your updatePID function)
  float Kp_head = Kp;
  float Ki_head = Ki;
  float Kd_head = Kd;
  
  // --- Reset the global PID state for heading (used by updatePID) ---
  integral = 0;
  prev_error = 0;
  derivative = 0;
  lastTime = millis();
  
  // Start moving forward (initialize motor direction to forward)
  initialize_motors();
  
  while (true) {
    // 1. Calculate distance traveled using the average of the encoder counts
    int avgPulses = (encoder_left_count + encoder_right_count) / 2;
    float distanceTraveled = (avgPulses / (float)PPR) * wheel_circumference;
    
    // Compute distance error (how far remaining)
    float distError = target_distance - distanceTraveled;
    
    // If the error is small enough (within 0.5 cm), we're done.
    if (fabs(distError) < 0.5) {
      break;
    }
    
    // 2. Compute the time step for the distance PID
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTimeDist) / 1000.0f;
    if (dt < 0.01f) dt = 0.01f;
    lastTimeDist = currentTime;
    
    // 3. Distance PID Calculations (local variables)
    distanceIntegral += distError * dt;
    float distanceDerivative = (distError - distancePrevError) / dt;
    float forwardCommand = (Kp_dist * distError) + (Ki_dist * distanceIntegral) + (Kd_dist * distanceDerivative);
    distancePrevError = distError;
    
    // Constrain the forward command (this is the desired base speed)
    forwardCommand = constrain(forwardCommand, 0, 150);
    
    // 4. Heading Correction using your ready PID function:
    // Calculate a heading error based on the difference between left and right encoders.
    float headingError = (float)encoder_left_count - (float)encoder_right_count;
    updatePID(headingError);  // This updates the global variables: integral, prev_error, derivative.
    float headingCorrection = (Kp_head * headingError) + (Ki_head * integral) + (Kd_head * derivative);
    
    // 5. Combine the two: adjust the left/right speeds to maintain heading while moving forward.
    int leftSpeed  = constrain((int)(forwardCommand - headingCorrection), 0, 255);
    int rightSpeed = constrain((int)(forwardCommand + headingCorrection), 0, 255);
    
    setMotorSpeed(leftSpeed, rightSpeed);
    delay(5);
  }
  
  stopMotors();
}

// --------------------------
// Turn Right / Turn Left
// --------------------------

// Helper: smallest angular difference in [-180..180]
float angleDiff(float from, float to) {
  float diff = fmodf((to - from + 540.0f), 360.0f) - 180.0f;
  return diff;
}

void turnRight(float targetAngle) {
  float initialYaw = readYaw();
  float rawTarget = initialYaw - targetAngle;
  if (rawTarget < 0)   rawTarget += 360.0f;
  if (rawTarget >= 360.0f) rawTarget -= 360.0f;

  float currentYaw = initialYaw;

  // Reset PID
  integral = 0;
  prev_error = 0;
  derivative = 0;
  lastTime = millis();

  unsigned long startTime = millis();
  SerialBT.printf("Turning Right: InitialYaw=%.2f, TargetYaw=%.2f\n", initialYaw, rawTarget);

  while (true) {
    if (millis() - startTime > 5000) {
      SerialBT.println("Turn Timeout! Stopping Motors.");
      stopMotors();
      break;
    }

    currentYaw = readYaw();
    float error = angleDiff(currentYaw, rawTarget);

    if (fabs(error) < 3.0f) {
      SerialBT.println("Turn Completed.");
      stopMotors();
      break;
    }

    updatePID(error);
    if (integral > 100)  integral = 100;
    if (integral < -100) integral = -100;

    float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // negative => turn right, positive => turn left
    int leftDir  = (correction > 0) ? LOW : HIGH;
    int rightDir = (correction > 0) ? HIGH : LOW;
    digitalWrite(MOTOR1_IN1, leftDir);
    digitalWrite(MOTOR1_IN2, !leftDir);
    digitalWrite(MOTOR2_IN3, rightDir);
    digitalWrite(MOTOR2_IN4, !rightDir);

    int turnSpeed = 50 + (int)fabs(correction);
    turnSpeed = constrain(turnSpeed, 50, 140);

    setMotorSpeed(turnSpeed, -turnSpeed);
    delay(10);
  }
  
  // *** MODIFIED *** Update heading
  robotHeading = (robotHeading + 1) % 4; 
}

void turnLeft(float targetAngle) {
  float initialYaw = readYaw();
  float rawTarget = initialYaw + targetAngle;
  if (rawTarget < 0)      rawTarget += 360.0f; 
  if (rawTarget >= 360.0f) rawTarget -= 360.0f;

  float currentYaw = initialYaw;

  integral = 0;
  prev_error = 0;
  derivative = 0;
  lastTime = millis();

  unsigned long startTime = millis();
  SerialBT.printf("Turning Left: InitialYaw=%.2f, TargetYaw=%.2f\n", initialYaw, rawTarget);

  while (true) {
    if (millis() - startTime > 5000) {
      SerialBT.println("Turn Timeout! Stopping Motors.");
      stopMotors();
      break;
    }

    currentYaw = readYaw();
    float error = angleDiff(currentYaw, rawTarget);

    if (fabs(error) < 3.0f) {
      SerialBT.println("Turn Completed.");
      stopMotors();
      break;
    }

    updatePID(error);
    if (integral > 100)  integral = 100;
    if (integral < -100) integral = -100;

    float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

    int leftDir  = (correction > 0) ? LOW : HIGH;
    int rightDir = (correction > 0) ? HIGH : LOW;
    digitalWrite(MOTOR1_IN1, leftDir);
    digitalWrite(MOTOR1_IN2, !leftDir);
    digitalWrite(MOTOR2_IN3, rightDir);
    digitalWrite(MOTOR2_IN4, !rightDir);

    int turnSpeed = 50 + (int)fabs(correction);
    turnSpeed = constrain(turnSpeed, 50, 140);

    setMotorSpeed(-turnSpeed, turnSpeed);
    delay(10);
  }

  // *** MODIFIED *** Update heading
  // Turning left is heading = (heading + 3) % 4 
  // (equivalent to heading-1 mod 4)
  robotHeading = (robotHeading + 3) % 4; 
}

// --------------------------
// Maze / Sensor functions
// --------------------------
int API_mazeWidth()  { return 8; }
int API_mazeHeight() { return 8; }

int API_wallFront() {
  int distance = readLiDAR();
  return (distance > 0 && distance < 130);
}

int API_wallRight() {
  int irRight = digitalRead(IR_SENSOR_1);
  // If your IR hardware returns HIGH for "wall" or "no wall," adapt as needed.
  return (irRight == LOW); // e.g. LOW means no wall or vice versa
}

int API_wallLeft() {
  int irLeft = digitalRead(IR_SENSOR_2);
  return (irLeft == LOW);
}

/*Heading 0 => 𝑦++  heading 1 => 𝑥++ heading 2 => 𝑦−− heading 3 => 𝑥−−*/
bool inCenter(int x, int y) {
  // The 4 center cells for an 8x8: (3,3), (3,4), (4,3), (4,4)
  if ((x == 2) && (y == 3)) {
    return true;
  }
  return false;
}

// *** ADDED *** After moving 1 cell forward, update (robotX, robotY)
void updatePosition() {
  // robotHeading: 0=N, 1=E, 2=S, 3=W
  if (robotHeading == 0) {
    robotY = min(robotY + 1, MAZE_HEIGHT - 1);
  } else if (robotHeading == 1) {
    robotX = min(robotX + 1, MAZE_WIDTH - 1);
  } else if (robotHeading == 2) {
    robotY = max(robotY - 1, 0);
  } else if (robotHeading == 3) {
    robotX = max(robotX - 1, 0);
  }
  visited[robotX][robotY] = true; 
}

// --------------------------
// Right-Hand Rule Maze Logic
// --------------------------

void rightHandRule() {
  // *** ADDED *** Mark start as visited
  robotX = 0;
  robotY = 0;
  robotHeading = 0;
  for (int i = 0; i < MAZE_WIDTH; i++) {
    for (int j = 0; j < MAZE_HEIGHT; j++) {
      visited[i][j] = false;
    }
  }
  visited[0][0] = true;

  delay(2000);
  
  while (true) {
    // If we've reached the center, stop.
    if (inCenter(robotX, robotY)) {
      Serial.println("Reached the center goal!");
      stopMotors();
      break;
    }

    bool wallRight  = API_wallRight();
    bool wallFront  = API_wallFront();
    bool wallLeft   = API_wallLeft();

    Serial.print("Pos("); Serial.print(robotX); Serial.print(", ");
    Serial.print(robotY); Serial.print("), heading=");
    Serial.print(robotHeading);
    Serial.print(" | WallRight="); Serial.print(wallRight);
    Serial.print(", WallFront="); Serial.print(wallFront);
    Serial.print(", WallLeft="); Serial.println(wallLeft);

    if (!wallRight && !wallLeft) {
      // Both left and right are open, choose randomly
      int randomChoice = random(2);  // Generates 0 or 1
      if (randomChoice == 0) {
        delay(500);
        turnRight(90);
        updatePosition();
      } 
      else {
        delay(500);
        turnLeft(90);
        updatePosition();
      }
      moveForward(20);
      
    }

    else if (!wallRight) {
      // Turn right, move forward 1 cell
      delay(500);
      Serial.println("Right is open => Turn Right + Forward");
      turnRight(90);
      updatePosition(); // *** ADDED ***
      moveForward(19.6);
    }
    else if (!wallFront) {
      // Move forward
      delay(500);
      Serial.println("Forward is open => Move Forward");
      moveForward(19.6);
      updatePosition(); // *** ADDED ***
    }
    else if (!wallLeft) {
      // Turn left, move forward 1 cell
      delay(500);
      Serial.println("Left is openzz => Turn Left + Forward");
      turnLeft(90);
      updatePosition(); // *** ADDED ***
      moveForward(19.6);
    }
    else {
      // Dead end => turn around 180
      Serial.println("Dead End => Turn around");
      turnRight(180);
      robotHeading == 3;
      // Move forward 1 cell (optional) 
      updatePosition();
      moveForward(19.6);
      
    }
    delay(500);
  }
}

// --------------------------
// Setup & Loop
// --------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN3, OUTPUT);
  pinMode(MOTOR2_IN4, OUTPUT);
  pinMode(MOTOR1_ENA, OUTPUT);
  pinMode(MOTOR2_ENA, OUTPUT);

  configureLEDC(0, MOTOR1_ENA, 5000, 8);
  configureLEDC(1, MOTOR2_ENA, 5000, 8);

  pinMode(ENCODER1_A, INPUT_PULLUP);
  pinMode(ENCODER2_A, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER1_A), updateEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A), updateEncoderRight, RISING);

  // IR sensors
  pinMode(IR_SENSOR_1, INPUT_PULLUP);
  pinMode(IR_SENSOR_2, INPUT_PULLUP);

  initializeMPU6050();
  initLiDAR();
  
}

void loop() {
  // Right-hand rule solver
   rightHandRule();

  // Once done, do nothing
  while (1) {
    delay(1000);
  }
}
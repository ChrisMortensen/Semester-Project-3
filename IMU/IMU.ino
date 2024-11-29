#include <Wire.h>
#include "MPU6050.h"

MPU6050 mpu;

// Variables for angle and position calculation
float gyroX, gyroY, gyroZ;
float angleZ = 0.0; // Angle around the Z-axis
int16_t accX, accY, accZ; // Accelerometer raw data (in raw units)
float velX = 0.0, velY = 0.0; // Velocity in X and Y directions
float posX = 0.0, posY = 0.0; // Position in X and Y directions
unsigned long lastTime = 0; // Last timestamp
float gyroZBias = 0.0; // Gyro Z-axis bias
float dt; // Time delta in seconds

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
  
  Serial.println("MPU6050 connected");

  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

  // Calibrate the gyroscope
  calibrateGyroscope();
  
  lastTime = micros(); // Initialize the timer
}

void loop() {
  // Get current time and calculate time delta
  unsigned long currentTime = micros();
  dt = (currentTime - lastTime) / 1000000.0; // Time in seconds
  lastTime = currentTime;

  calculateRotation();
  calculatePosition();

  printRotationAndPosition();

  delay(2); // Short delay for stability
}

// Function to calculate the rotation
void calculateRotation() {
  // Read gyroscope data
  gyroZ = (mpu.getRotationZ() / 131.0) - gyroZBias; // Get rotation around Z-axis and subtract bias

  // Integrate angular velocity to calculate angle
  angleZ += gyroZ * dt;

  // Normalize angle to -180 to 180 range
  if (angleZ > 180) angleZ -= 360;
  else if (angleZ < -180) angleZ += 360;
}

// Function to calculate the current position
void calculatePosition() {
  // Read accelerometer data
  mpu.getAcceleration(&accX, &accY, &accZ); // Get raw accelerometer data

  velX = accX / 16384.0 * 9.8 * dt + 0.11;
  velY = accY / 16384.0 * 9.8 * dt + 0.01;

  // Update position using velocity (simple integration)
  posX += velX * dt;
  posY += velY * dt;
}

// Function to calibrate the gyroscope
void calibrateGyroscope() {
  Serial.println("Calibrating gyroscope...");
  int numSamples = 500;
  float sumZ = 0;

  for (int i = 0; i < numSamples; i++) {
    sumZ += mpu.getRotationZ() / 131.0; // Read Z-axis rotation
    delay(2); // Small delay between samples
  }

  gyroZBias = sumZ / numSamples; // Calculate the average bias
  Serial.print("Gyroscope Z bias: ");
  Serial.println(gyroZBias);
}

// Output angle, position, and velocity
void printRotationAndPosition() {
  Serial.print("Angle Z: ");
  Serial.print(angleZ);
  Serial.print(" | PosX: ");
  Serial.print(posX);
  Serial.print(" | PosY: ");
  Serial.print(posY);
  Serial.print(" | VelX: ");
  Serial.print(velX);
  Serial.print(" | VelY: ");
  Serial.println(velY);
}
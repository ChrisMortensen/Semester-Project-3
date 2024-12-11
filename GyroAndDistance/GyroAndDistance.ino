#include <Wire.h>
#include "MPU6050.h"

MPU6050 mpu;

// Variables for angle calculation
float gyroX, gyroY, gyroZ;
float angleZ = 0.0; // Angle around the Z-axis
unsigned long lastTime = 0;

// Bias variables
float gyroZBias = 0.0; 


const int trigPin1 = 23;
const int echoPin1 = 22;
const int powerPin1 = 7;
//const int trigPin2 = 11;
//const int echoPin2 = 12;

float duration, distance; //duration2, distance2;

void setup() {
  distanceSetup();
  Serial.begin(9600);
  Wire.begin();
  
  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
  
  //Serial.println("MPU6050 connected");

  // Calibrate the gyroscope
  calibrateGyroscope();
  
  lastTime = micros(); // Initialize the timer
}

void loop() {
  calcRotation();
  calcDistance(trigPin1, echoPin1);
  Serial.print(angleZ);
  Serial.print(" ");
  Serial.println(distance);
  delay(100); // Short delay for stability
}

void distanceSetup(){
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(powerPin1, OUTPUT);
  digitalWrite(powerPin1, HIGH);
  //pinMode(trigPin2, OUTPUT);
  //pinMode(echoPin2, INPUT);
}

void calcDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  distance = (duration*.0343)/2;
  //return distance;
}


// Function to calibrate the gyroscope
void calibrateGyroscope() {
  //Serial.println("Calibrating gyroscope...");
  int numSamples = 500;
  float sumZ = 0;

  for (int i = 0; i < numSamples; i++) {
    sumZ += mpu.getRotationZ() / 131.0; // Read Z-axis rotation
    delay(2); // Small delay between samples
  }

  gyroZBias = sumZ / numSamples; // Calculate the average bias
  //Serial.print("Gyroscope Z bias: ");
  //Serial.println(gyroZBias);
}

void calcRotation(){
  // Get current time
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0; // Time delta in seconds
  lastTime = currentTime;

  // Read gyroscope data (angular velocity in °/s)
  gyroZ = (mpu.getRotationZ() / 131.0) - gyroZBias; // Subtract bias

  // Integrate angular velocity to calculate angle
  angleZ += gyroZ * dt;

  // Normalize angle to 0–360 or -180 to 180
  if (angleZ > 180) angleZ -= 360;
  else if (angleZ < -180) angleZ += 360;

  // Output the angle
  //return angleZ;
}



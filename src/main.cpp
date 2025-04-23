// ESP32 code for monitor orientation detection using MPU-6050
// Supports detecting 0, 90, 180, and 270 degree rotations
// Uses direct I2C communication instead of Adafruit libraries
#include <Wire.h>
#include <WiFi.h>

// I2C pins - fixed back to match your hardware
const int SDA_PIN = 21;
const int SCL_PIN = 22;

// MPU6050 I2C address
const uint8_t MPU_ADDR = 0x68;  // Default address is 0x68

// Orientation values
const int ORIENTATION_0 = 0;    // Normal landscape
const int ORIENTATION_90 = 1;   // Portrait (rotated right)
const int ORIENTATION_180 = 2;  // Upside-down landscape
const int ORIENTATION_270 = 3;  // Portrait (rotated left)

// Thresholds for orientation detection (in degrees)
const float G_THRESHOLD = 0.7;
const float G_CONVERSION = 16384.0; // Sensitivity for ±2g range

// Variables to store monitor state
int currentOrientation = ORIENTATION_0;
float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;

// Serial communication timing
const unsigned long UPDATE_INTERVAL = 5; // seconds


// Function prototypes
int determineOrientation(float x, float y, float z);
void sendOrientationData();
bool setupMPU();
bool readMPUData(int16_t* ax, int16_t* ay, int16_t* az);

void setup() {
  // Initialize serial communication at higher baud rate
  Serial.begin(115200);
  delay(1000); // Give serial time to initialize
  
  Serial.println("Monitor Tilt");
  Serial.println("Initializing...");
  
  // Initialize I2C with specified pins
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // Set I2C clock speed to 400kHz
  
  Serial.println("Wire initialized.");
  
  // Scan for I2C devices
  Serial.println("Scanning I2C bus...");
  byte error, address;
  int deviceCount = 0;
  
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      
      // Try to identify MPU-6050
      if (address == MPU_ADDR) {
        Serial.println(" (MPU-6050)");
      } else {
        Serial.println();
      }
      deviceCount++;
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("No I2C devices found");
  } else {
    Serial.print("Found ");
    Serial.print(deviceCount);
    Serial.println(" device(s)");
  }
  
  Serial.println("Attempting to initialize MPU6050...");
  
  // Initialize the MPU6050
  if (setupMPU()) {
    Serial.println("MPU6050 initialized successfully!");
  } else {
    Serial.println("Failed to initialize MPU6050");
    Serial.println("Continuing with default orientation");
  }
  
  // Short delay after setup
  delay(100);
  Serial.println("Setup complete");
}

void loop() {
  // Try to read accelerometer data
  int16_t ax, ay, az;
  
  if (readMPUData(&ax, &ay, &az)) {
    // Convert raw values to g-force (approximate)
    accelX = ax / G_CONVERSION;
    accelY = ay / G_CONVERSION;
    accelZ = az / G_CONVERSION;
    
    // Determine orientation based on raw accelerometer values
    // Using direct axis values instead of pitch/roll calculation
    int newOrientation = determineOrientation(accelX, accelY, accelZ);
    
    // Check if orientation changed
    if (newOrientation != currentOrientation) {
      currentOrientation = newOrientation;
      sendOrientationData();
    }
  }
  else {
    // If MPU6050 reading failed
    if (millis() % (UPDATE_INTERVAL * 1000) < 10) {
      Serial.println("MPU6050 not responding");
    }
  }
  
  // Periodically send data regardless of changes
  static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime > (UPDATE_INTERVAL * 1000)) {
    sendOrientationData();
    lastSendTime = millis();
  }
  
  // Check for any incoming serial data and echo it back
  // This helps with device detection
  while (Serial.available() > 0) {
    char c = Serial.read();
    // Echo any received character
    //Serial.write(c);
    
    // If we receive a linefeed, also send the MAC address
    if (c == '\n') {
      sendOrientationData();
    }
  }
  
  delay(10);  // Short delay for stability
}

// Setup the MPU6050
bool setupMPU() {
  // Wake up the MPU-6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  byte status = Wire.endTransmission(true);
  
  if (status != 0) {
    Serial.print("MPU6050 not responding, I2C status: ");
    Serial.println(status);
    return false;
  }
  
  // Configure accelerometer range to ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);  // ACCEL_CONFIG register
  Wire.write(0x00);  // ±2g range
  Wire.endTransmission(true);
  
  return true;
}

// Read accelerometer data from MPU6050
bool readMPUData(int16_t* ax, int16_t* ay, int16_t* az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  if (Wire.endTransmission(false) != 0) {
    return false;  // Failed to set register pointer
  }
  
  // Request 6 bytes (2 bytes per axis)
  if (Wire.requestFrom(MPU_ADDR, (size_t) 6, (bool) true) != 6) {
    return false;  // Didn't receive 6 bytes
  }
  
  // Read the values
  *ax = (Wire.read() << 8) | Wire.read();
  *ay = (Wire.read() << 8) | Wire.read();
  *az = (Wire.read() << 8) | Wire.read();
  
  return true;
}

// Determine the orientation based on raw accelerometer values
// Updated to focus on Z-axis orientation
int determineOrientation(float x, float y, float z) {
  if (y > G_THRESHOLD && abs(x) < G_THRESHOLD) {
    return ORIENTATION_270;//3
  }
  
  if (x > G_THRESHOLD && abs(y) < G_THRESHOLD) {
    return ORIENTATION_0;//0
  }
  
  if (y < -G_THRESHOLD && abs(x) < G_THRESHOLD) {
    return ORIENTATION_90;//1
  }
  
  if (x < -G_THRESHOLD && abs(y) < G_THRESHOLD) {
    return ORIENTATION_180;//2
  }
  
  // If no clear orientation is detected, keep the current one
  return currentOrientation;
}

void sendOrientationData() {
  // Get MAC address
  String macAddress = WiFi.macAddress();
  
  // Send JSON data over serial
  Serial.print("{\"mac\":\"");
  Serial.print(macAddress);
  Serial.print("\",\"orientation\":");
  Serial.print(currentOrientation);
  Serial.print(",\"x\":");
  Serial.print(accelX);
  Serial.print(",\"y\":");
  Serial.print(accelY);
  Serial.print(",\"z\":");
  Serial.print(accelZ);
  Serial.println("}");
}
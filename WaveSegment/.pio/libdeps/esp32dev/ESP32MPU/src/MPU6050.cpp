#include <Arduino.h>
#include <Wire.h>
#include <math.h>

//-----------------------
// MPU6050 Definitions
//-----------------------
#define MPU6050_ADDR             0x68  // Default I2C address for MPU6050
#define MPU6050_REG_PWR_MGMT_1   0x6B  // Power management register
#define MPU6050_REG_ACCEL_XOUT_H 0x3B  // Starting register for accelerometer (and gyro) data
#define MPU6050_REG_ACCEL_CONFIG 0x1C  // Accelerometer configuration register

static const char *TAG = "MPU6050";
#define ESP_OK 0

/**
 * Write a single byte to a specific MPU6050 register.
 */
static int mpu6050_write_byte(uint8_t reg_addr, uint8_t data) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg_addr);
  Wire.write(data);
  int ret = Wire.endTransmission();  // 0 on success
  if (ret != ESP_OK) {
      Serial.printf("%s: Failed to write to reg 0x%02X\r\n", TAG, reg_addr);
  }
  return ret;
}

/**
 * Read multiple bytes from the MPU6050 starting at a specific register.
 */
static int mpu6050_read_bytes(uint8_t reg_addr, uint8_t* data, size_t len) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg_addr);
  int ret = Wire.endTransmission(false);  // false = send a repeated start
  if (ret != ESP_OK) {
      Serial.printf("%s: Failed to write register address 0x%02X\r\n", TAG, reg_addr);
      return ret;
  }
  
  size_t bytesRead = Wire.requestFrom(MPU6050_ADDR, (uint8_t)len, true);
  if (bytesRead < len) {
      Serial.printf("%s: Failed to read from reg 0x%02X\r\n", TAG, reg_addr);
      return 1;  // Generic error code
  }
  
  for (size_t i = 0; i < len; i++) {
      data[i] = Wire.read();
  }
  return ESP_OK;
}

/**
 * Initialize the MPU6050 sensor.
 */
int mpu6050_init(void) {
  int ret = mpu6050_write_byte(MPU6050_REG_PWR_MGMT_1, 0);
  if (ret == ESP_OK) {
      Serial.printf("%s: MPU6050 initialization successful.\r\n", TAG);
  } else {
      Serial.printf("%s: MPU6050 initialization failed.\r\n", TAG);
  }
  
  // Explicitly enable and configure the accelerometer (±2g range)
  ret = mpu6050_write_byte(MPU6050_REG_ACCEL_CONFIG, 0x00);
  if (ret == ESP_OK) {
      Serial.printf("%s: Accelerometer enabled (±2g).\r\n", TAG);
  } else {
      Serial.printf("%s: Failed to enable accelerometer.\r\n", TAG);
  }
  
  return ret;
}

/**
 * Read accelerometer and gyroscope data from the MPU6050.
 */
int mpu6050_read_accel_gyro(int16_t* ax, int16_t* ay, int16_t* az,
                            int16_t* gx, int16_t* gy, int16_t* gz) {
  uint8_t buffer[14];
  int ret = mpu6050_read_bytes(MPU6050_REG_ACCEL_XOUT_H, buffer, sizeof(buffer));
  if (ret != ESP_OK) {
      return ret;
  }
  
  *ax = (int16_t)((buffer[0] << 8) | buffer[1]);
  *ay = (int16_t)((buffer[2] << 8) | buffer[3]);
  *az = (int16_t)((buffer[4] << 8) | buffer[5]);
  
  *gx = (int16_t)((buffer[8]  << 8) | buffer[9]);
  *gy = (int16_t)((buffer[10] << 8) | buffer[11]);
  *gz = (int16_t)((buffer[12] << 8) | buffer[13]);

  return ESP_OK;
}

//-----------------------
// Global Variables for Complementary Filter
//-----------------------

// A simple structure to hold IMU-related constants and state.
struct IMUData {
  float mdpsPerLSB;    // millidegrees per second per LSB (e.g., ~7.63 for ±250 dps)
  float gyroODR;       // Gyro output data rate (Hz)
  struct {
    float y;         // current gyro reading for the Y axis
  } g;
  struct {
    float y;         // gyro bias for the Y axis
  } gyroBias;
};

IMUData imu = {7.63, 100.0, {0}, {0}};  // Set conversion factors and initial bias

// Structure to hold Euler angles (pitch angle here is stored in .y)
struct EulerAngles {
  float y;
};

EulerAngles eulerAngles = {0.0};
float prevBias = 0.0;  // For bias correction in the complementary filter

//-----------------------
// Main Application Code
//-----------------------

void setup() {
  Serial.begin(115200);
  Serial.println("MPU6050 Test Starting...");
  
  // Initialize I2C bus with SDA on pin 21 and SCL on pin 22
  Wire.begin(21, 22);
  delay(100);
  
  if (mpu6050_init() != ESP_OK) {
    Serial.println("Error initializing MPU6050");
  }
}

void loop() {
  int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
  
  if (mpu6050_read_accel_gyro(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw) == ESP_OK) {
    // Print raw sensor data
    Serial.printf("Accel: X=%d, Y=%d, Z=%d | Gyro: X=%d, Y=%d, Z=%d\r\n",
                  ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw);
    
    // Convert raw values to floats (the scale cancels out in the ratio for angle)
    float ax = (float)ax_raw;
    float ay = (float)ay_raw;
    float az = (float)az_raw;
    
    // Compute the observed pitch angle from the accelerometer (in degrees)
    // (Assumes the sensor is oriented such that rotation about the Y axis gives pitch)
    float observedAngle = atan2(ax, sqrt(ay * ay + az * az)) * (180.0 / PI);
    
    // Update the gyro reading for the Y axis (for pitch rate)
    // (For proper integration, you might want to apply conversion from raw data to dps.)
    imu.g.y = (float)gy_raw;
    
    // Prediction: integrate the gyro reading over the sample period.
    // Here (1/imu.gyroODR) is used as the time step.
    float dt = 1.0 / imu.gyroODR;
    float predictedAngle = eulerAngles.y + ((imu.mdpsPerLSB / 1000.0) * (imu.g.y - imu.gyroBias.y)) * dt;
    
    // Complementary filter parameters
    float kappa = 0.97;      // weight for the gyro (equivalent to 1 - alpha)
    float epsilon = 0.0001;  // weight for the bias correction
    
    // Combine the prediction and the observed accelerometer angle
    eulerAngles.y = kappa * predictedAngle + (1.0 - kappa) * observedAngle;
    
    // Update the bias estimate
    float bias = prevBias - epsilon * (1.0 / (imu.gyroODR * imu.gyroODR)) * (eulerAngles.y - observedAngle);
    prevBias = bias;
    imu.gyroBias.y = bias;
    
    // Print the computed (filtered) pitch angle
    Serial.printf("Filtered Pitch Angle: %.2f degrees\r\n", eulerAngles.y);
  } else {
    Serial.println("Failed to read sensor data.");
  }
  
  delay(500);
}
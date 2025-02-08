#include "MPU6050.h"
#include <Arduino.h>
#include <Wire.h>

static const char *TAG = "MPU6050";

// Define a success value
#define ESP_OK 0
// Define a timeout (not used directly with Wire, but kept for reference)
#define I2C_TIMEOUT_MS 1000

/**
 Write a single byte to a specific MPU6050 register.
 
 reg_addr The register address to write to.
 data The data byte to write.
 
 int ESP_OK (0) on success, or a nonzero error code otherwise.
 */
static int mpu6050_write_byte(uint8_t reg_addr, uint8_t data)
{
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
Read multiple bytes from the MPU6050 starting at a specific register.
 
 reg_addr The starting register address.
 data Pointer to the buffer to store read data.
 len Number of bytes to read.
 
 int ESP_OK (0) on success, or a nonzero error code otherwise.
 */
static int mpu6050_read_bytes(uint8_t reg_addr, uint8_t* data, size_t len)
{
    // Write the register address with a repeated start (do not release the bus)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg_addr);
    int ret = Wire.endTransmission(false);  // false = send a repeated start
    if (ret != ESP_OK) {
        Serial.printf("%s: Failed to write register address 0x%02X\r\n", TAG, reg_addr);
        return ret;
    }
    
    // Request 'len' bytes from the MPU6050
    size_t bytesRead = Wire.requestFrom(MPU6050_ADDR, (uint8_t)len, true);
    if (bytesRead < len) {
        Serial.printf("%s: Failed to read from reg 0x%02X\r\n", TAG, reg_addr);
        return 1;  // Return a generic error code
    }
    
    for (size_t i = 0; i < len; i++) {
        data[i] = Wire.read();
    }
    return ESP_OK;
}

/**
 Initialize the MPU6050 sensor.
 
 The MPU6050 powers up in sleep mode. Wake it up by writing 0 to PWR_MGMT_1.
 
 int ESP_OK (0) on success, or a nonzero error code otherwise.
 */
int mpu6050_init(void)
{
    // Ensure that Wire.begin() has been called before this function,
    // or uncomment the next line if you wish to initialize I2C here.
    // Wire.begin();
    
    int ret = mpu6050_write_byte(MPU6050_REG_PWR_MGMT_1, 0);
    if (ret == ESP_OK) {
        Serial.printf("%s: MPU6050 initialization successful.\r\n", TAG);
    } else {
        Serial.printf("%s: MPU6050 initialization failed.\r\n", TAG);
    }
    return ret;
}

/**
  Read accelerometer and gyroscope data from the MPU6050.
 
  Reads 14 consecutive bytes starting at register 0x3B:
  - Bytes 0–5: Accelerometer data (X, Y, Z; 2 bytes each)
  - Bytes 6–7: Temperature (skipped here)
  - Bytes 8–13: Gyroscope data (X, Y, Z; 2 bytes each)
 
 ax Pointer to store accelerometer X-axis value.
 ay Pointer to store accelerometer Y-axis value.
 az Pointer to store accelerometer Z-axis value.
 gx Pointer to store gyroscope X-axis value.
 gy Pointer to store gyroscope Y-axis value.
 gz Pointer to store gyroscope Z-axis value.
 
 int ESP_OK (0) on success, or a nonzero error code otherwise.
 */
int mpu6050_read_accel_gyro(int16_t* ax, int16_t* ay, int16_t* az,
                            int16_t* gx, int16_t* gy, int16_t* gz)
{
    uint8_t buffer[14];

    // Read 14 bytes starting at the accelerometer X-axis high register (0x3B)
    int ret = mpu6050_read_bytes(MPU6050_REG_ACCEL_XOUT_H, buffer, sizeof(buffer));
    if (ret != ESP_OK) {
        return ret;
    }

    // Combine high and low bytes for each measurement.
    // Accelerometer data
    *ax = (int16_t)((buffer[0] << 8) | buffer[1]);
    *ay = (int16_t)((buffer[2] << 8) | buffer[3]);
    *az = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    // Note: Temperature data is in buffer[6] and buffer[7] if needed.
    
    // Gyroscope data
    *gx = (int16_t)((buffer[8]  << 8) | buffer[9]);
    *gy = (int16_t)((buffer[10] << 8) | buffer[11]);
    *gz = (int16_t)((buffer[12] << 8) | buffer[13]);

    return ESP_OK;
}
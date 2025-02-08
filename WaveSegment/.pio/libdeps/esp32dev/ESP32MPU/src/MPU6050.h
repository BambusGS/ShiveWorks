#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>

// MPU6050 default I2C address
#define MPU6050_ADDR 0x68

// MPU6050 register definitions
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_ACCEL_XOUT_H  0x3B

/**
 Initialize the MPU6050 sensor.
 
This function wakes up the MPU6050 (which starts in sleep mode)
 by writing 0 to the power management register.
 
 @return esp_err_t ESP_OK on success, or an error code otherwise.
 */
esp_err_t mpu6050_init(void);

/**
 Reads accelerometer and gyroscope data from the MPU6050.
 
Read 14 consecutive bytes starting at register 0x3B:
Bytes 0–5: Accelerometer data for X, Y, Z (2 bytes each)
Bytes 6–7: Temperature (not used here)
Bytes 8–13: Gyroscope data for X, Y, Z (2 bytes each)
----------------
ax Pointer to store accelerometer X-axis value.
ay Pointer to store accelerometer Y-axis value.
az Pointer to store accelerometer Z-axis value.
gx Pointer to store gyroscope X-axis value.
gy Pointer to store gyroscope Y-axis value.
gz Pointer to store gyroscope Z-axis value.
 
esp_err_t ESP_OK on success, or an error code otherwise.
 */
esp_err_t mpu6050_read_accel_gyro(int16_t* ax, int16_t* ay, int16_t* az,
                                  int16_t* gx, int16_t* gy, int16_t* gz);

#endif // MPU6050_H
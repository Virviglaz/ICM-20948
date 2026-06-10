#include "ICM-20948.h"
#include "i2c.h"
#include <exception>
#include <cstdio>
#include <thread>
#include <chrono>

/* I2C interface and device provided by my custom platform-independent implementation */
static I2C_Interface i2c_interface;
static I2C_DeviceBase i2c_device(i2c_interface, 0x68);
static ICM20948_I2C icm20948_ifs(i2c_device);
static ICM20948_DMP icm20948(icm20948_ifs);

int main(int argc, char *argv[])
{
    i2c_interface.Init(argc > 1 ? argv[1] : "/dev/i2c-0");

    int res = 0;
    try {
        icm20948.Reset(); // Reset the ICM-20948 to ensure it's in a known state before initialization
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Wait for 100 ms after reset
        res = icm20948.Init();
        if (!res) {
            icm20948.EnableFifo();
            icm20948.SetAccelAvgRate(ICM20948::AccelAvgRate::AVG_16); // Set accelerometer average rate to 16 samples
            icm20948.SetGyroAvgRate(ICM20948::GyroAvgRate::AVG_16);
            printf("ICM-20948 initialized successfully\n");
            res = icm20948.Calibrate();
            printf("ICM-20948 calibration %s\n", res ? "failed" : "succeeded");
        }
    } catch (const std::exception &e) {
        fprintf(stderr, "Failed to initialize ICM-20948: %s\n", e.what());
        return 1;
    }

    /* Read and print raw sensor data */
    for (int i = 0; i < 10; ++i) {
        //std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Wait for 100 ms after reset
        auto real_data = icm20948.WaitForData();
        printf("Accelerometer: ax=%.2f g, ay=%.2f g, az=%.2f g\n", real_data.GetAccX(), real_data.GetAccY(), real_data.GetAccZ());
        printf("Gyroscope: gx=%.2f °/s, gy=%.2f °/s, gz=%.2f °/s\n", real_data.GetGyroX(), real_data.GetGyroY(), real_data.GetGyroZ());
        printf("Temperature: %.2f °C\n\n", real_data.GetTemperature());
    }

    /* Read and print real-world IMU data from DMP */
    for (int i = 0; i < 10; ++i) {
        auto real_data = icm20948.GetRealIMUData();
        printf("Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°\n", real_data.roll, real_data.pitch, real_data.yaw);
        printf("Gyro: gx=%.2f °/s, gy=%.2f °/s, gz=%.2f °/s\n", real_data.gx, real_data.gy, real_data.gz);
        printf("Linear Acceleration: ax=%.2f m/s², ay=%.2f m/s², az=%.2f m/s²\n\n", real_data.ax_linear, real_data.ay_linear, real_data.az_linear);
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep to simulate a 10 Hz update rate
    }
    return res;
}

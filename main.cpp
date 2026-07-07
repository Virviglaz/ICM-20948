#include "ICM-20948.h"
#include "spi.h"
#include <exception>
#include <cstdio>
#include <thread>
#include <chrono>
#include <cstring>

/* SPI interface and device provided by my custom platform-independent implementation */
static SPI_Interface spi_interface;
static SPI_DeviceBase spi_device(spi_interface);
static ICM20948_SPI icm20948_spi(spi_device);
static ICM20948_DMP icm20948(icm20948_spi);

int main(int argc, char *argv[])
{
    spi_interface.Init(argc > 1 ? argv[1] : "/dev/spidev1.0");

    int res = 0;
    try {
        icm20948.Reset(); // Reset the ICM-20948 to ensure it's in a known state before initialization
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Wait for 100 ms after reset
        res = icm20948.Init();
        if (!res) {
            icm20948.SetAccelAvgRate(ICM20948::AccelAvgRate::AVG_16); // Set accelerometer average rate to 16 samples
            icm20948.SetGyroAvgRate(ICM20948::GyroAvgRate::AVG_16);
            printf("ICM-20948 initialized successfully\n");
            res = icm20948.Calibrate();
            printf("ICM-20948 calibration %s\n", res ? "failed" : "succeeded");
        } else {
            fprintf(stderr, "Failed to initialize ICM-20948: Error code %d (%s)\n", res, strerror(-res));
            return res;
        }

        /* Read and print raw sensor data */
        for (int i = 0; i < 10; ++i) {
            auto raw_data = icm20948.WaitForData();
            printf("Accelerometer: ax=%.2f g, ay=%.2f g, az=%.2f g\n", raw_data.Accel.GetX(), raw_data.Accel.GetY(), raw_data.Accel.GetZ());
            printf("Gyroscope: gx=%.2f °/s, gy=%.2f °/s, gz=%.2f °/s\n", raw_data.Gyro.GetX(), raw_data.Gyro.GetY(), raw_data.Gyro.GetZ());
            printf("Temperature: %.2f °C\n\n", raw_data.GetTemperature());
        }

        /* Read and print real-world IMU data from DMP */
        for (int i = 0; i < 10; ++i) {
            auto real_data = icm20948.GetRealIMUData();
            printf("Roll: %.2f °, Pitch: %.2f °, Yaw: %.2f °\n", real_data.GetRollDeg(), real_data.GetPitchDeg(), real_data.GetYawDeg());
        }
        return res;
    } catch (const std::exception &e) {
        fprintf(stderr, "Failed to initialize ICM-20948: %s\n", e.what());
        return 1;
    }
}

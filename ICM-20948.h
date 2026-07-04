/*
 * This file is provided under a MIT license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * MIT License
 *
 * Copyright (c) 2026 Pavel Nadein
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * ICM-20948 C++ driver header.
 *
 * Contact Information:
 * Pavel Nadein <pavelnadein@gmail.com>
 */

#ifndef ICM_20948_H
#define ICM_20948_H

#include <cstdint>
#include "devices.h"

/**
 * Abstract base class for ICM-20948 interfaces (I2C, SPI, etc.)
 */
class ICM20948_IFS_Base
{
public:
    explicit ICM20948_IFS_Base() = default;
    virtual ~ICM20948_IFS_Base() = default;

    /* Bit manipulation */
    void SetBit(uint8_t reg, uint8_t bit_mask);
    void ClrBit(uint8_t reg, uint8_t bit_mask);

    /* Register access */
    void WriteReg(uint8_t reg, uint8_t value);
    void WriteReg_s16(uint8_t reg, int16_t value);
    int16_t ReadReg_s16(uint8_t reg);
    uint8_t ReadReg(uint8_t reg);
    void Read(uint8_t reg, uint8_t* data, size_t length);
    void Write(uint8_t reg, const uint8_t* data, size_t length);

    virtual uint8_t SPI_IFS_Enabled() { return 0; }

    /* Pure virtual methods to be implemented by derived classes for specific interfaces */
    virtual void Write(const uint8_t *reg_addr,
                       size_t reg_addr_size,
                       const uint8_t *data,
                       size_t length) = 0;

    virtual void Read(const uint8_t *reg_addr,
                      size_t reg_addr_size,
                      uint8_t *data,
                      size_t length) = 0;
};

/**
 * ICM-20948 I2C interface implementation.
 */
class ICM20948_I2C : public ICM20948_IFS_Base
{
public:
    explicit ICM20948_I2C(I2C_DeviceBase& device) : device_(device) {}
    ~ICM20948_I2C() override = default;

    void Write(const uint8_t *reg_addr,
               size_t reg_addr_size,
               const uint8_t *data,
               size_t length) override;

    void Read(const uint8_t *reg_addr,
              size_t reg_addr_size,
              uint8_t *data,
              size_t length) override;
private:
    I2C_DeviceBase& device_;
};

/**
 * ICM-20948 SPI interface implementation.
 */
class ICM20948_SPI : public ICM20948_IFS_Base
{
public:
    explicit ICM20948_SPI(SPI_DeviceBase& device) : device_(device) {}
    ~ICM20948_SPI() override = default;

    void Write(const uint8_t *reg_addr,
               size_t reg_addr_size,
               const uint8_t *data,
               size_t length) override;

    void Read(const uint8_t *reg_addr,
              size_t reg_addr_size,
              uint8_t *data,
              size_t length) override;

    uint8_t SPI_IFS_Enabled() override { return 1 << 4; }
private:
    SPI_DeviceBase& device_;
    uint8_t buffer_[512]; // Temporary buffer for SPI transactions (adjust size as needed)
};

/**
 * ICM20948 class provides high-level methods to interact with the ICM-20948 sensor,
 * including initialization, data reading, and calibration.
 * It uses an ICM20948_IFS_Base interface for low-level register access,
 * allowing it to work with different communication interfaces (I2C, SPI, etc.)
 * through derived classes like ICM20948_I2C and ICM20948_SPI.
 * The class includes methods to set the accelerometer and gyroscope full scale ranges,
 * read raw sensor data, and perform sensor calibration by adjusting hardware offset
 * registers iteratively to minimize sensor errors.
 * The RawData struct encapsulates the raw sensor readings and provides methods to convert
 * them to real-world units based on the current gain settings.
 *
 * @note This class provides a basic implementation for the ICM-20948 sensor.
 */
class ICM20948
{
public:
    explicit ICM20948(ICM20948_IFS_Base& ifs) : ifs_(ifs) {}
    ~ICM20948() = default;

    int Reset();
    virtual int Init();

    enum class AccelFSR : uint8_t
    {
        G_2 = 0,
        G_4 = 1,
        G_8 = 2,
        G_16 = 3
    };

    /**
     * Sets scaling factor for the accelerometer. The default is ±2g (G_2).
     *
     * @param fsr The desired accelerometer full scale range (FSR).
     * Valid values are:
     * - AccelFSR::G_2  : ±2g
     * - AccelFSR::G_4  : ±4g
     * - AccelFSR::G_8  : ±8g
     * - AccelFSR::G_16 : ±16g
     */
    void SetAccelFSR(AccelFSR fsr = AccelFSR::G_2);

    enum class GyroFSR : uint8_t
    {
        DPS_250 = 0,
        DPS_500 = 1,
        DPS_1000 = 2,
        DPS_2000 = 3
    };

    /**
     * Sets scaling factor for the gyroscope. The default is ±250°/s (DPS_250).
     *
     * @param fsr The desired gyroscope full scale range (FSR).
     * Valid values are:
     * - GyroFSR::DPS_250  : ±250°/s
     * - GyroFSR::DPS_500  : ±500°/s
     * - GyroFSR::DPS_1000 : ±1000°/s
     * - GyroFSR::DPS_2000 : ±2000°/s
     */
    void SetGyroFSR(GyroFSR fsr = GyroFSR::DPS_250);

    enum class AccelAvgRate : uint8_t
    {
        AVG_1 = 0,
        AVG_8 = 1,
        AVG_16 = 2,
        AVG_32 = 3
    };

    /**
     * Sets the accelerometer averaging rate. The default is 1 sample (AVG_1).
     *
     * @param rate The desired accelerometer averaging rate.
     * Valid values are:
     * - AccelAvgRate::AVG_1  : 1 sample
     * - AccelAvgRate::AVG_8  : 8 samples
     * - AccelAvgRate::AVG_16 : 16 samples
     * - AccelAvgRate::AVG_32 : 32 samples
     */
    void SetAccelAvgRate(AccelAvgRate rate = AccelAvgRate::AVG_1);

    enum class GyroAvgRate : uint8_t
    {
        AVG_1 = 0,
        AVG_2 = 1,
        AVG_4 = 2,
        AVG_8 = 3,
        AVG_16 = 4,
        AVG_32 = 5,
        AVG_64 = 6,
        AVG_128 = 7
    };

    /**
     * Sets the gyroscope averaging rate. The default is 1 sample (AVG_1).
     *
     * @param rate The desired gyroscope averaging rate.
     * Valid values are:
     * - GyroAvgRate::AVG_1   : 1 sample
     * - GyroAvgRate::AVG_2   : 2 samples
     * - GyroAvgRate::AVG_4   : 4 samples
     * - GyroAvgRate::AVG_8   : 8 samples
     * - GyroAvgRate::AVG_16  : 16 samples
     * - GyroAvgRate::AVG_32  : 32 samples
     * - GyroAvgRate::AVG_64  : 64 samples
     * - GyroAvgRate::AVG_128 : 128 samples
     */
    void SetGyroAvgRate(GyroAvgRate rate = GyroAvgRate::AVG_1);

    enum class IcmOdr {
        ODR_220_HZ,  // Gyro: 220Hz (div 4),  Accel: 225Hz (div 4)
        ODR_100_HZ,  // Gyro: 100Hz (div 10), Accel: 102.2Hz (div 10)
        ODR_50_HZ,   // Gyro: 50Hz (div 21),  Accel: 51.1Hz (div 21)
        ODR_20_HZ    // Gyro: 20Hz (div 54),  Accel: 20.4Hz (div 54)
    };

    void SetSynchronizedSampleRate(IcmOdr target_odr);

    /**
     * RawData struct represents the raw sensor data read from the ICM-20948. It includes raw accelerometer, gyroscope, and temperature readings.
     * The GetTemperature(), GetAccX(), GetAccY(), GetAccZ(), GetGyroX(), GetGyroY(), and GetGyroZ() methods convert the raw readings to
     * real-world units (°C for temperature, g for acceleration, and °/s for angular velocity) based on the current gain settings.
     */
    class RawData {
        friend class ICM20948;
    public:
        RawData(float acc_scale, float gyro_scale)
            : _acc_scale(acc_scale), _gyro_scale(gyro_scale) {}
        float GetAccX() const;
        float GetAccY() const;
        float GetAccZ() const;
        float GetGyroX() const;
        float GetGyroY() const;
        float GetGyroZ() const;
        float GetTemperature() const;
    private:
        int16_t x; /* ICM-20948_RA_ACCEL_XOUT H/L */
        int16_t y; /* ICM-20948_RA_ACCEL_YOUT H/L */
        int16_t z; /* ICM-20948_RA_ACCEL_ZOUT H/L */
        int16_t ax; /* ICM-20948_RA_GYRO_XOUT H/L */
        int16_t ay; /* ICM-20948_RA_GYRO_YOUT H/L */
        int16_t az; /* ICM-20948_RA_GYRO_ZOUT H/L */
        int16_t temp; /* ICM-20948_RA_TEMP_OUT H/L */
        float _acc_scale, _gyro_scale; /* Scaling factors for accelerometer and gyroscope */
    };

    /**
     * Waits for new sensor data to be available and returns the raw data. This method will block until new data is ready.
     * It is recommended to use EnableDataReadyInterrupt() and override IsDataReady() for a more efficient implementation
     * that does not rely on busy-waiting.
     */
    RawData WaitForData();

    /**
     * @brief Wait for new sensor data to be available and returns the raw data.
     *
     * @note This method will block until new data is ready.
     *
     * @return the latest raw sensor data without waiting. The caller should ensure that new data is available before calling
     * this method, either by using EnableDataReadyInterrupt() and checking IsDataReady(), or by implementing their own timing mechanism.
     * This method reads the raw accelerometer, gyroscope, and temperature data from the ICM-20948 and returns it as a RawData
     * struct. The raw values are in big-endian format and are converted to native endianness before being returned.
     */
    RawData GetData();

    /**
     * @brief Performs a calibration of the ICM-20948 sensor.
     *
     * The calibration process adjusts the hardware offset registers
     * iteratively to minimize the error between the sensor readings and the expected values (0 g for accelerometer axes,
     * 0 °/s for gyro axes). The calibration will run for a maximum of max_iterations or until the error is within
     * target_error LSB of the target values.
     * Returns 0 on success, or a negative error code on failure.
     *
     * @param max_iterations The maximum number of calibration iterations to perform.
     * A higher number may yield better results but will take longer.
     * @param target_error The target error threshold in LSB.
     * Calibration will stop when all sensor readings are within this error
     * threshold of the target values (0 for accel X/Y, 16384 for accel Z, 0 for gyro X/Y/Z).
     *
     * @return 0 if calibration succeeded, or a negative error code if calibration failed after max_iterations.
     */
    int Calibrate(int max_iterations = 100, int16_t target_error = 250);

    /**
     * Struct to hold the calibration offsets for the accelerometer and gyroscope.
     * This struct is used to read and write the hardware offset registers of the
     * ICM-20948 during the calibration process.
     * The offsets are stored as 16-bit signed integers, and the struct is packed
     * to ensure it matches the layout of the ICM-20948's offset registers.
     */
    #pragma pack(push, 1)
    struct cal_offsets {
        int16_t acc_x_offset = 0;
        int16_t acc_y_offset = 0;
        int16_t acc_z_offset = 0;
        int16_t gyro_x_offset = 0;
        int16_t gyro_y_offset = 0;
        int16_t gyro_z_offset = 0;
    };
    #pragma pack(pop)
    static_assert(sizeof(cal_offsets) == 12,
        "cal_offsets struct must be exactly 12 bytes to match the ICM-20948 offset register layout");

    /**
     * @brief Reads the current calibration offsets from the ICM-20948 hardware registers
     * and returns them as a cal_offsets struct.
     *
     * The caller can use this method to retrieve the current offsets
     * before starting a calibration process, or to read the offsets after
     * calibration to save them for future use.
     */
    cal_offsets ReadCalibrationOffsets();

    /**
     * @brief Writes the provided calibration offsets to the ICM-20948's hardware offset registers.
     *
     * Writes the given calibration offsets to the ICM-20948's hardware offset registers.
     * The caller can use this method to apply new offsets to the sensor, either as
     * part of a calibration process or to restore previously saved offsets. The offsets
     * should be provided as a cal_offsets struct, which contains the accelerometer and gyroscope
     * offsets as 16-bit signed integers. The method will write the offsets to the appropriate
     * registers on the ICM-20948.
     */
    void WriteCalibrationOffsets(const cal_offsets& offsets);

    /**
     * @brief Enables the FIFO buffer on the ICM-20948.
     *
     * When enabled, sensor data will be stored in the FIFO buffer for later retrieval.
     * This method configures the necessary registers to enable FIFO operation and should be called before attempting to read data from the FIFO.
     */
    virtual void EnableFifo();

    /**
     * @brief Disables the FIFO buffer on the ICM-20948.
     *
     * When disabled, sensor data will no longer be stored in the FIFO buffer.
     * Data ready interrupts will be used instead to indicate when new sensor data is available.
     */
    void DisableFifo();

    /**
     * @brief Check if the FIFO buffer is full.
     *
     * @return true if the FIFO buffer is full, false otherwise.
     */
    bool IsFifoOverflown();

    /**
     * @brief Called when the FIFO buffer is full more than half.
     */
    virtual void FifoOverflowEventHandler() {}

    /**
	 * @brief Reset the FIFO buffer on the ICM-20948.
	 */
    virtual void ResetFIFO();

    /* Replace this with a GPIO read implementation if needed */
    virtual bool IsDataReady();
protected:
    ICM20948_IFS_Base& ifs_;

    uint16_t FifoCount();
    void SwitchMemoryBank(uint8_t bank);
    bool isFifoEnabled = false;

    virtual uint16_t GetPacketSize() const { return 14; } /* 6 bytes accel + 6 bytes gyro + 2 bytes temp */
private:
    int CheckWhoAmI();
    AccelFSR current_accel_fsr_ = AccelFSR::G_2;
    GyroFSR current_gyro_fsr_ = GyroFSR::DPS_250;
    uint8_t current_bank_ = 10; // Invalid bank to force initial switch
};

#if 0 // Magnetometer support is currently disabled in this build.
/**
 * This class extends the ICM20948 base class to implement support
 * for the ICM-20948's magnetometer (AK09916). It provides methods
 * to initialize the magnetometer and read/write its registers
 * through the ICM-20948's internal I2C master interface.
 * The magnetometerInit() method configures the necessary settings
 * to enable communication with the AK09916, including enabling the
 * internal I2C master interface and setting the appropriate clock speed.
 * The write_MagnetometerReg() and read_MagnetometerReg() methods allow
 * writing to and reading from the AK09916's registers, respectively,
 * by sending the appropriate commands through the ICM-20948's I2C master interface.
 */
class ICM20948_MAG : public ICM20948
{
public:
    using ICM20948::ICM20948; // Inherit constructors from ICM20948
    int Init() override;
protected:
    int write_MagnetometerReg(uint8_t reg, uint8_t value);
    int read_MagnetometerReg(uint8_t reg, uint8_t& out_value);
    int CheckWhoAmI();
};
#endif /* MAGNETOMETER_SUPPORT */

class ICM20948_DMP : public ICM20948
{
public:
    explicit ICM20948_DMP(ICM20948_IFS_Base& ifs) : ICM20948(ifs) {}

    /**
     * Initializes the ICM-20948 with DMP (Digital Motion Processor) support.
     * This method uploads the DMP firmware to the sensor, configures the necessary registers,
     * and prepares the sensor for DMP operation. It must be called after the base ICM-20948
     * initialization and before attempting to read DMP data.
     *
     * @return 0 on success, or a negative error code on failure.
     */
    int Init() override;

    /**
     * Struct to hold the real-world IMU data obtained from the DMP.
     * This includes roll, pitch, and yaw angles in degrees,
     * angular velocity (gx, gy, gz) in degrees per second,
     * and linear acceleration (ax_linear, ay_linear, az_linear) in m/s².
     * The GetRealIMUData() method reads raw DMP data from the FIFO buffer,
     * converts it to real-world units, and returns it as a RealIMUData struct.
     * The conversion includes normalizing quaternions, calculating Euler angles,
     * converting gyro data to degrees per second, and calculating linear
     * acceleration by removing the gravity component.
     */
    struct RealIMUData
    {
        float roll, pitch, yaw;                // angles in degrees
    };

    /**
     * Read real-world IMU data from the DMP.
     *
     * @return A RealIMUData struct containing roll, pitch, yaw, angular velocity, and linear acceleration.
     */
    RealIMUData GetRealIMUData();

    /**
     * @brief Reads the raw quaternion data from the DMP FIFO buffer.
     *
     * @return A struct containing the raw quaternion values (w, x, y, z) as 16-bit signed integers.
     */
    RealIMUData WaitForRealIMUData();
protected:
    virtual void ResetFIFO() override;
    virtual void EnableFifo() override;
    virtual uint16_t GetPacketSize() const override { return 16; }
    virtual bool IsMDPDataReady();
};


#endif // ICM_20948_H

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
 * ICM-20948 C++ driver implementation.
 *
 * Contact Information:
 * Pavel Nadein <pavelnadein@gmail.com>
 */

#include "ICM-20948.h"
#include "ICM-20948_registers.h"
#include "bitops.h"
#include <cmath>
#include <errno.h>
#include <cstring>
#include <array>
#include <algorithm>

#define IFS_BUS_TIMEOUT_CYCLES 50000
#ifndef BIT
#define BIT(n) (1U << (n))
#endif

/****************************** ICM20948_IFS_Base *****************************/
void ICM20948_IFS_Base::SetBit(uint8_t reg, uint8_t bit_mask)
{
    uint8_t value;
    Read(&reg, 1, &value, 1);
    value |= bit_mask;
    Write(&reg, 1, &value, 1);
}

void ICM20948_IFS_Base::ClrBit(uint8_t reg, uint8_t bit_mask)
{
    uint8_t value;
    Read(&reg, 1, &value, 1);
    value &= ~bit_mask;
    Write(&reg, 1, &value, 1);
}

void ICM20948_IFS_Base::WriteReg(uint8_t reg, uint8_t value)
{
    Write(&reg, 1, &value, 1);
}

void ICM20948_IFS_Base::WriteReg_s16(uint8_t reg, int16_t value)
{
    int16_t be_value = NativeToBigEndian(value);
    Write(&reg, 1, reinterpret_cast<uint8_t*>(&be_value), 2);
}

int16_t ICM20948_IFS_Base::ReadReg_s16(uint8_t reg)
{
    int16_t value;
    Read(&reg, 1, reinterpret_cast<uint8_t*>(&value), 2);
    return BigEndianToNative(value);
}

uint8_t ICM20948_IFS_Base::ReadReg(uint8_t reg)
{
    uint8_t value;
    Read(&reg, 1, &value, 1);
    return value;
}

void ICM20948_IFS_Base::Write(uint8_t reg, const uint8_t* data, size_t length)
{
    Write(&reg, 1, data, length);
}

void ICM20948_IFS_Base::Read(uint8_t reg, uint8_t* data, size_t length)
{
    Read(&reg, 1, data, length);
}

/****************************** ICM20948 I2C *****************************/
void ICM20948_I2C::Write(const uint8_t *reg_addr,
                         size_t reg_addr_size,
                         const uint8_t *data,
                         size_t length)
{
    device_.Write(reg_addr, reg_addr_size, data, length);
}

void ICM20948_I2C::Read(const uint8_t *reg_addr,
                        size_t reg_addr_size,
                        uint8_t *data,
                        size_t length)
{
    device_.Read(reg_addr, reg_addr_size, data, length);
}

/****************************** ICM20948 SPI *****************************/
void ICM20948_SPI::Write(const uint8_t *reg_addr,
                         size_t reg_addr_size,
                         const uint8_t *data,
                         size_t length)
{
    // SPI write operation: MSB of register address should be 0
    memcpy(buffer_, reg_addr, reg_addr_size);
    memcpy(buffer_ + reg_addr_size, data, length);
    buffer_[0] &= 0x7F; // Update the register address with MSB cleared
    device_.Transfer(buffer_, nullptr, reg_addr_size + length); // Send register address and data
}

void ICM20948_SPI::Read(const uint8_t *reg_addr,
                        size_t reg_addr_size,
                        uint8_t *data,
                        size_t length)
{
    // SPI read operation: MSB of register address should be 1
    memcpy(buffer_, reg_addr, reg_addr_size);
    buffer_[0] |= 0x80; // Update the register address with MSB set
    device_.Transfer(buffer_, buffer_, reg_addr_size + length); // Send register address and read data into buffer
    memcpy(data, buffer_ + reg_addr_size, length); // Copy the read data to the output buffer
}

/******************************** ICM20948 *******************************/
int ICM20948::Reset()
{
    // 1. Ensure we are in Bank 0 to access power management registers
    SwitchMemoryBank(0);
    
    // 2. Trigger the hardware master reset by setting the DEVICE_RESET bit
    ifs_.WriteReg(PWR_MGMT_1, BIT_DEVICE_RESET);
    
    // 3. Poll until the chip clears the DEVICE_RESET bit back to 0
    // This confirms that the internal register map reset sequence is finished
    uint32_t reset_timeout = IFS_BUS_TIMEOUT_CYCLES;
    while (ifs_.ReadReg(PWR_MGMT_1) & BIT_DEVICE_RESET) {
        if (--reset_timeout == 0) {
            return -ETIMEDOUT; // Error: Chip reset timed out (hardware failure)
        }
    }
    
    // 4. Wake up the chip from default SLEEP mode and set clock source to Auto (0x01)
    // Writing 0x01 clears the BIT_SLEEP (0x40) and sets the clock
    ifs_.WriteReg(PWR_MGMT_1, CLK_BEST_AVAIL);

	if (ifs_.ReadReg(PWR_MGMT_1) & 0x40) {
		return -EIO; // Error: Failed to wake up the chip (hardware failure)
	}
    
    return 0; // Reset and wake up completed successfully
}

int ICM20948::Init()
{
    if (int res = CheckWhoAmI()) {
        return res; // Error handling: Device not found or not responding correctly
    }

    SwitchMemoryBank(0);
    ifs_.WriteReg(PWR_MGMT_1, 0x01); // PWR_MGMT_1 register, clear SLEEP bit to wake up the device
    ifs_.WriteReg(PWR_MGMT_2, 0x00); // PWR_MGMT_2 register, enable all sensors
    ifs_.WriteReg(INT_ENABLE_1, 0x01);

    // Explicitly (re)apply the full scale range so the ACCEL_FS_SEL/GYRO_FS_SEL
    // register bits are guaranteed to match current_accel_fsr_/current_gyro_fsr_,
    // which are used to scale raw counts to physical units in GetData().
    SetAccelFSR(current_accel_fsr_);
    SetGyroFSR(current_gyro_fsr_);

    return 0;
}

void ICM20948::SetAccelFSR(AccelFSR fsr)
{
    current_accel_fsr_ = fsr; // Store the current accelerometer full scale range
    SwitchMemoryBank(2); // Switch to user bank 2 to access ACCEL_CONFIG register

    // ACCEL_CONFIG bit layout: FCHOICE[0], ACCEL_FS_SEL[2:1], ACCEL_DLPFCFG[5:3].
    // Only update the FS_SEL bits and preserve FCHOICE/DLPFCFG via read-modify-write,
    // otherwise the DLPF/FCHOICE configuration would be silently reset every call.
    uint8_t reg = ifs_.ReadReg(ACCEL_CONFIG);
    reg &= ~(0x03 << 1);
    reg |= (static_cast<uint8_t>(fsr) & 0x03) << 1;
    ifs_.WriteReg(ACCEL_CONFIG, reg);
}

void ICM20948::SetGyroFSR(GyroFSR fsr)
{
    current_gyro_fsr_ = fsr; // Store the current gyroscope full scale range
    SwitchMemoryBank(2); // Switch to user bank 2 to access GYRO_CONFIG_1 register

    // GYRO_CONFIG_1 bit layout: FCHOICE[0], GYRO_FS_SEL[2:1], GYRO_DLPFCFG[5:3].
    uint8_t reg = ifs_.ReadReg(GYRO_CONFIG_1);
    reg &= ~(0x03 << 1);
    reg |= (static_cast<uint8_t>(fsr) & 0x03) << 1;
    ifs_.WriteReg(GYRO_CONFIG_1, reg);
}

void ICM20948::SetAccelAvgRate(AccelAvgRate rate)
{
    // NOTE: averaging/decimation for the accelerometer lives in ACCEL_CONFIG_2
    // (DEC3_CFG bits [1:0]), NOT in ACCEL_CONFIG (which holds ACCEL_FS_SEL).
    // Writing to ACCEL_CONFIG here would silently corrupt the full scale range
    // set by SetAccelFSR() and desynchronize it from current_accel_fsr_, which
    // is used to convert raw counts to g's in GetData() - this was the source
    // of incorrect scaling and IMU drift.
    SwitchMemoryBank(2); // Switch to user bank 2 to access ACCEL_CONFIG_2 register
    uint8_t reg = ifs_.ReadReg(ACCEL_CONFIG_2);
    reg &= ~0x03;
    reg |= static_cast<uint8_t>(rate) & 0x03;
    ifs_.WriteReg(ACCEL_CONFIG_2, reg);
}

void ICM20948::SetGyroAvgRate(GyroAvgRate rate)
{
    // NOTE: averaging for the gyroscope lives in GYRO_CONFIG_2 (GYRO_AVGCFG
    // bits [2:0]), NOT in GYRO_CONFIG_1 (which holds GYRO_FS_SEL). See note
    // in SetAccelAvgRate() above.
    SwitchMemoryBank(2); // Switch to user bank 2 to access GYRO_CONFIG_2 register
    uint8_t reg = ifs_.ReadReg(GYRO_CONFIG_2);
    reg &= ~0x07;
    reg |= static_cast<uint8_t>(rate) & 0x07;
    ifs_.WriteReg(GYRO_CONFIG_2, reg);
}

void ICM20948::SetSynchronizedSampleRate(IcmOdr target_odr)
{
    uint16_t gyro_div = 0;
    uint16_t accel_div = 0;

    // Select dividers that yield the closest matching frequencies
    switch (target_odr)
    {
        case IcmOdr::ODR_220_HZ:
            gyro_div  = 4;   // 1100 / (1 + 4)  = 220.0 Hz
            accel_div = 4;   // 1125 / (1 + 4)  = 225.0 Hz
            break;

        case IcmOdr::ODR_100_HZ:
            gyro_div  = 10;  // 1100 / (1 + 10) = 100.0 Hz
            accel_div = 10;  // 1125 / (1 + 10) = 102.2 Hz
            break;

        case IcmOdr::ODR_50_HZ:
            gyro_div  = 21;  // 1100 / (1 + 21) = 50.0 Hz
            accel_div = 21;  // 1125 / (1 + 21) = 51.1 Hz
            break;

        case IcmOdr::ODR_20_HZ:
            gyro_div  = 54;  // 1100 / (1 + 54) = 20.0 Hz
            accel_div = 54;  // 1125 / (1 + 54) = 20.4 Hz
            break;
    }

    // Switch to Bank 2 where sample rate registers reside
    SwitchMemoryBank(2);

    // 1. Write Gyro Sample Rate Divider (1 byte)
    ifs_.WriteReg(GYRO_SMPLRT_DIV, static_cast<uint8_t>(gyro_div));

    // 2. Write Accel Sample Rate Divider (2 bytes: High then Low)
    uint8_t accel_div_h = static_cast<uint8_t>((accel_div >> 8) & 0x0F);
    uint8_t accel_div_l = static_cast<uint8_t>(accel_div & 0xFF);

    ifs_.WriteReg(ACCEL_SMPLRT_DIV_1, accel_div_h);
    ifs_.WriteReg(ACCEL_SMPLRT_DIV_2, accel_div_l);
}

ICM20948::RawData& ICM20948::WaitForData()
{
    while (!IsDataReady()) {};
    return GetData();
}

ICM20948::RawData& ICM20948::GetData()
{
    SwitchMemoryBank(0); // Ensure we are in Bank 0 to read sensor data registers

    #pragma pack(push, 1)
    union {
        uint8_t be_buffer[14]; // Buffer to hold big-endian raw data (14 bytes: AccX, AccY, AccZ, Temp, GyroX, GyroY, GyroZ)
        struct {
            int16_t x; /* ICM-20948_RA_ACCEL_XOUT H/L */
            int16_t y; /* ICM-20948_RA_ACCEL_YOUT H/L */
            int16_t z; /* ICM-20948_RA_ACCEL_ZOUT H/L */
            int16_t ax; /* ICM-20948_RA_GYRO_XOUT H/L */
            int16_t ay; /* ICM-20948_RA_GYRO_YOUT H/L */
            int16_t az; /* ICM-20948_RA_GYRO_ZOUT H/L */
            int16_t temp; /* ICM-20948_RA_TEMP_OUT H/L */
        } data_struct; // Struct to access the raw data as individual fields
    } data_union;
    #pragma pack(pop)

	if (isFifoEnabled) {
		ifs_.Read(FIFO_R_W, data_union.be_buffer, sizeof(data_union.be_buffer)); // Read 14 bytes of sensor data from FIFO
	} else {
		ifs_.Read(ACCEL_XOUT_H, data_union.be_buffer,
				sizeof(data_union.be_buffer)); // Read 14 bytes of sensor data starting from ACCEL_XOUT_H
	}

	cached_data = ICM20948::RawData(
    		IMU::RawData16_XYZ( // acceleration data
    				BigEndianToNative(data_union.data_struct.x),
    				BigEndianToNative(data_union.data_struct.y),
					BigEndianToNative(data_union.data_struct.z),
					static_cast<float>(16384.0f / (1 << static_cast<uint8_t>(current_accel_fsr_)))),
			IMU::RawData16_XYZ( // gyroscope data
					BigEndianToNative(data_union.data_struct.ax),
					BigEndianToNative(data_union.data_struct.ay),
					BigEndianToNative(data_union.data_struct.az),
					static_cast<float>(131.0f / (1 << static_cast<uint8_t>(current_gyro_fsr_)))),
			static_cast<float>(BigEndianToNative(data_union.data_struct.temp))
	);

	return cached_data;
}

uint16_t ICM20948::FifoCount()
{
    SwitchMemoryBank(0); // Ensure we are in Bank 0 to read FIFO count registers
    auto fifo_count = ifs_.ReadReg_s16(FIFO_COUNTH); // Read FIFO count as signed 16-bit integer
    if (fifo_count > 512)
    	FifoOverflowEventHandler(); // Handle FIFO overflow if count exceeds 512 bytes

    return static_cast<uint16_t>(fifo_count); // Return FIFO count as unsigned 16-bit integer
}

bool ICM20948::IsDataReady()
{
    if (isFifoEnabled) {
        return FifoCount() >= GetPacketSize(); // Check if FIFO has at least 14 bytes of data (6 bytes for Accel + 6 bytes for Gyro + 2 bytes for Temp)
    }

    SwitchMemoryBank(0);
    return ifs_.ReadReg(INT_STATUS_1) & 0x01; // Check if the Data Ready bit (Bit 0) in the INT_STATUS register is set
}

#ifdef DEBUG
#include <stdio.h>
#endif
    
int ICM20948::Calibrate(int max_iterations, int16_t target_error)
{
    int res = -EFAULT;

    struct cal_offsets offsets = ReadCalibrationOffsets(); // Read current calibration offsets from the sensor

    for (int i = 0; i != max_iterations; i++) {
        RawData data = WaitForData();

        /* Lambda function to divide by 64 with rounding */
        auto divide_by_64_fast = [](int16_t x) -> int16_t {
            return (x + ((x >> 15) & 63)) >> 6;
        };

        offsets.acc_x_offset  -= divide_by_64_fast(data.Accel.GetRawX());
        offsets.acc_y_offset  -= divide_by_64_fast(data.Accel.GetRawY());
        offsets.acc_z_offset  -= divide_by_64_fast(data.Accel.GetRawZ());
        offsets.gyro_x_offset -= divide_by_64_fast(data.Gyro.GetRawX());
        offsets.gyro_y_offset -= divide_by_64_fast(data.Gyro.GetRawY());
        offsets.gyro_z_offset -= divide_by_64_fast(data.Gyro.GetRawZ());

        WriteCalibrationOffsets(offsets); // Apply new offsets to the sensor

        /* Calculate the maximum error between the current readings and the target values */
        auto get_error = [&data]() {
        	std::array<int16_t, 6> results = { data.Accel.GetRawX(), data.Accel.GetRawY(), data.Accel.GetRawZ(), data.Gyro.GetRawX(), data.Gyro.GetRawY(), data.Gyro.GetRawZ() };
            int16_t max_error = 0;
            for (size_t k = 0; k < 6; k++) {
                int16_t error = std::abs(results[k]);
                if (error > max_error)
                    max_error = error;
            }
            return max_error;
        };

        /* Measure the maximum error and break if within target */
        int16_t error = get_error();
#ifdef DEBUG
        printf("Iter %3d: x=%6d y=%6d z=%6d gx=%6d gy=%6d gz=%6d Err=%d\n",
               i + 1,
			   data.Accel.GetRawX(),
			   data.Accel.GetRawY(),
			   data.Accel.GetRawZ(),
			   data.Gyro.GetRawX(),
			   data.Gyro.GetRawY(),
			   data.Gyro.GetRawZ(),
			   error);
#endif
        if (error < target_error) {
            res = 0;
            break; // Calibration successful if all readings are within target_error
        }
    }

    return res;
}

void ICM20948::ResetFIFO()
{
    SwitchMemoryBank(0);
    ifs_.WriteReg(USER_CTRL, ifs_.SPI_IFS_Enabled());
    ifs_.WriteReg(FIFO_RST, BIT(3)); // Reset FIFO
    ifs_.WriteReg(FIFO_RST, 0x00);
    while (ifs_.ReadReg(FIFO_RST) & BIT(3)) {}; // Wait until FIFO reset is complete
    ifs_.WriteReg(USER_CTRL, BIT(6) | ifs_.SPI_IFS_Enabled());
}

void ICM20948::SwitchMemoryBank(uint8_t bank)
{
    // 1. Check if the requested bank is already active; if so, no action is needed
    if (bank == current_bank_) {
        return;
    }

    // 2. Write to the REG_BANK_SEL register to switch to the desired bank
    ifs_.WriteReg(REG_BANK_SEL, bank << 4); // Shift bank number to bits 4-5
    current_bank_ = bank;
}

ICM20948::cal_offsets ICM20948::ReadCalibrationOffsets()
{
    cal_offsets result;

    SwitchMemoryBank(1);
    result.acc_x_offset = ifs_.ReadReg_s16(ACCEL_X_OFFSET_H) >> 1;
    result.acc_y_offset = ifs_.ReadReg_s16(ACCEL_Y_OFFSET_H) >> 1;
    result.acc_z_offset = ifs_.ReadReg_s16(ACCEL_Z_OFFSET_H) >> 1;

    SwitchMemoryBank(2);
    result.gyro_x_offset = ifs_.ReadReg_s16(GYRO_X_OFFSET_H);
    result.gyro_y_offset = ifs_.ReadReg_s16(GYRO_Y_OFFSET_H);
    result.gyro_z_offset = ifs_.ReadReg_s16(GYRO_Z_OFFSET_H);

    return result;
}

void ICM20948::WriteCalibrationOffsets(const cal_offsets& offsets)
{
    SwitchMemoryBank(1);
    ifs_.WriteReg_s16(ACCEL_X_OFFSET_H, offsets.acc_x_offset << 1);
    ifs_.WriteReg_s16(ACCEL_Y_OFFSET_H, offsets.acc_y_offset << 1);
    ifs_.WriteReg_s16(ACCEL_Z_OFFSET_H, offsets.acc_z_offset << 1);
    
    SwitchMemoryBank(2);
    ifs_.WriteReg_s16(GYRO_X_OFFSET_H, offsets.gyro_x_offset);
    ifs_.WriteReg_s16(GYRO_Y_OFFSET_H, offsets.gyro_y_offset);
    ifs_.WriteReg_s16(GYRO_Z_OFFSET_H, offsets.gyro_z_offset);
}

void ICM20948::EnableFifo()
{
    if (isFifoEnabled) {
        return; // FIFO is already enabled, no action needed
    }

    SwitchMemoryBank(0);
    ifs_.WriteReg(FIFO_CONFIG, 0x00); // 0x69 = FIFO_CONFIG, set FIFO mode to stop when full (default) and no watermark
    ifs_.WriteReg(FIFO_EN_1, 0x00); // Do not use external sensor data channels (Aux Slave 0-3) in the FIFO stream
    ifs_.WriteReg(FIFO_EN_2, 0x1F); // Use data from all 5 internal sensor channels (Accel, Gyro, Temp) in the FIFO stream
    ifs_.WriteReg(USER_CTRL, BIT(6) | ifs_.SPI_IFS_Enabled()); // Enable FIFO operation by setting FIFO_EN (Bit 6) in USER_CTRL

    isFifoEnabled = true;

    ResetFIFO();
}

void ICM20948::DisableFifo()
{
    if (!isFifoEnabled) {
        return; // FIFO is already disabled, no action needed
    }

    SwitchMemoryBank(0);
    ifs_.WriteReg(USER_CTRL, ifs_.SPI_IFS_Enabled()); // Disable FIFO operation by clearing FIFO_EN (Bit 6) in USER_CTRL
    ifs_.WriteReg(LP_CONFIG, 0x00);

    isFifoEnabled = false;
}

int ICM20948::CheckWhoAmI()
{
    SwitchMemoryBank(0); // Ensure we are in Bank 0 to read the WHO_AM_I register

    if (ifs_.ReadReg(WHO_AM_I) != 0xEA) { // Expected value for ICM-20948
        return -ENODEV; // Device not found
    }

    return 0; // Device is present and responding correctly
}

bool ICM20948::IsFifoOverflown()
{
    SwitchMemoryBank(0);

    return (ifs_.ReadReg(INT_STATUS_1) & 0x08);
}

#if 0 // Magnetometer support is currently disabled in this build.
/******************************************* ICM20948 MAG ***********************************************/
int ICM20948_MAG::Init()
{
    // Initialize the base ICM20948 functionality first
    int res = ICM20948::Init(); 
    if (res != 0) {
        return res; 
    }

    // STEP 1: Safely enable internal auxiliary I2C Master interface without losing base flags
    SwitchMemoryBank(0);
    ifs_.WriteReg(INT_PIN_CFG, 0x00); // Clear INT_PIN_CFG to disable bypass mode and allow I2C Master control
    ifs_.WriteReg(LP_CONFIG, 0x00); // Clear LP_CONFIG to disable low-power mode and allow I2C Master control
    ifs_.WriteReg(USER_CTRL, BIT(5) | ifs_.SPI_IFS_Enabled()); // Set I2C_MST_EN (Bit 5) to enable the auxiliary I2C Master interface

    // STEP 2: Configure internal I2C clock speed (Must be done in Bank 3)
    SwitchMemoryBank(3);
    ifs_.WriteReg(I2C_MST_CTRL, BIT(4) | 0x0D); 

    // STEP 3: Issue the software reset to the magnetometer via Slave 4
    res = write_MagnetometerReg(AK09916_CNTL3, 0x01);
    if (res != 0) {
        return res; 
    }

    // --- REPLACED delay_ms(10) WITH HARDWARE POLLING ---
    // During soft-reset, the AK09916 pulls its lines high and responds with NACK (I2C_MST_PNA).
    // We poll the read function until the NACK flag clears and I2C_SLV4_DONE is asserted.
    uint32_t reset_timeout = 50000; // Guard against physical bus disconnect
    uint8_t dummy_read_val = 0;
    while (true) {
        // Attempt to read the CNTL3 register. While resetting, this will return -ENXIO due to NACK.
        res = read_MagnetometerReg(AK09916_CNTL3, dummy_read_val);
        
        // If the I2C transaction succeeds (returns 0) and the reset bit auto-clears to 0
        if (res == 0 && (dummy_read_val == 0x00)) {
            break; // Success: The magnetometer is awake, stable, and ready
        }
        
        if (--reset_timeout == 0) {
            return -ETIMEDOUT; // Error: Magnetometer failed to recover from reset (hardware failure)
        }
    }

    // STEP 4: Verify device communication and ID compatibility
    res = CheckWhoAmI();
    if (res != 0) {
        return res; 
    }

    // STEP 5: Set operational mode (Continuous Measurement Mode 4 - 100 Hz)
    res = write_MagnetometerReg(AK09916_CNTL2, 0x08);
    if (res != 0) {
        return res; 
    }
    
    // --- HARDWARE POLLING FOR MODE CHANGE STABILIZATION ---
    // Poll the ST1 (Status 1) register inside AK09916 until DRDY (Bit 0) becomes active
    uint32_t drdy_timeout = 10000;
    uint8_t st1_status = 0;
    while (true) {
        res = read_MagnetometerReg(AK09916_ST1, st1_status);
        if (res == 0 && (st1_status & 0x01)) {
            break; // Success: DRDY bit is high, hardware data pipelines are stable
        }
        if (--drdy_timeout == 0) {
            return -ETIMEDOUT; // Error: Magnetometer failed to stabilize measurements
        }
    }

    // STEP 6: Setup background burst reading (MUST enforce Bank 3 before writing SLV0 registers)
    SwitchMemoryBank(3);
    ifs_.WriteReg(I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80); 
    ifs_.WriteReg(I2C_SLV0_REG, AK09916_ST1);              
    ifs_.WriteReg(I2C_SLV0_CTRL, 0x89);                     // Enable Slave 0 and auto-fetch 9 bytes

    // Explicitly return to Bank 0 so regular sensor data reading functions operate smoothly
    SwitchMemoryBank(0);

    return 0; 
}

int ICM20948_MAG::write_MagnetometerReg(uint8_t reg, uint8_t value)
{
    // 1. Switch to User Bank 3 to configure the Auxiliary I2C Slave 4 controller
    SwitchMemoryBank(3);

    // Set the auxiliary I2C target address and clear MSB for a WRITE operation
    ifs_.WriteReg(I2C_SLV4_ADDR, AK09916_I2C_ADDR);

    // Specify the target register inside the AK09916 magnetometer
    ifs_.WriteReg(I2C_SLV4_REG, reg);

    // Load the data byte to be transmitted into the Slave 4 Data Out register
    ifs_.WriteReg(I2C_SLV4_DO, value);

    // Do NOT set Bit 6 (Interrupt) and leave bits [5:0] as 0 (do not disable register address).
    ifs_.WriteReg(I2C_SLV4_CTRL, BIT(7)); 

    // 2. Switch to User Bank 0 where I2C_MST_STATUS physically resides (Address 0x17)
    SwitchMemoryBank(0);

    // 3. Poll the I2C Master status register until the transaction ends or fails
    uint32_t timeout = 10000; 
    while (true) {
        uint8_t status = ifs_.ReadReg(I2C_MST_STATUS);

        // Check Bit 6: I2C_SLV4_DONE (Transaction completed successfully)
        if (status & 0x40) {
            break; 
        }

        // Check Bit 4: I2C_MST_PNA (Slave did not acknowledge the transfer / NACK)
        if (status & 0x10) {
            return -ENXIO; // Error: Physical connection or address issue
        }

        if (--timeout == 0) {
            return -ETIMEDOUT; // Error: The AUX bus locked up entirely
        }
    }

    return 0; // Magnetometer register written successfully
}

int ICM20948_MAG::read_MagnetometerReg(uint8_t reg, uint8_t& out_value)
{
    // 1. Switch to User Bank 3 to configure the Auxiliary I2C Slave 4 controller
    SwitchMemoryBank(3);

    // Set target address with Bit 7 HIGH (0x80) to signal a READ operation
    ifs_.WriteReg(I2C_SLV4_ADDR, AK09916_I2C_ADDR | 0x80);

    // Specify the target register inside the AK09916 magnetometer
    ifs_.WriteReg(I2C_SLV4_REG, reg);

    // Trigger the single-byte transfer by setting Bit 7 (I2C_SLV4_EN)
    ifs_.WriteReg(I2C_SLV4_CTRL, 0x80);

    // 2. Switch to User Bank 0 where I2C_MST_STATUS physically resides (Address 0x17)
    SwitchMemoryBank(0);

    // 3. Poll the hardware status register until the transaction ends or fails
    uint32_t timeout = 10000;
    while (true) {
        uint8_t status = ifs_.ReadReg(I2C_MST_STATUS);

        // Check Bit 6: I2C_SLV4_DONE (Success)
        if (status & 0x40) {
            break;
        }
        // Check Bit 4: I2C_MST_PNA (Slave NACK / Connection issue)
        if (status & 0x10) {
            return -ENXIO;
        }
        if (--timeout == 0) {
            return -ETIMEDOUT;
        }
    }

    // 4. Switch back to User Bank 3 to fetch the received data byte
    SwitchMemoryBank(3);
    out_value = ifs_.ReadReg(I2C_SLV4_DI);

    return 0;
}

int ICM20948_MAG::CheckWhoAmI()
{
    const uint8_t AK09916_WIA2 = 0x01;       // Company ID / Device ID 2 register [1]
    const uint8_t AK09916_EXPECTED_ID = 0x09; // Fixed value for AK09916 [1]
    uint8_t device_id = 0;

    // Read the Who Am I register from the magnetometer
    int res = read_MagnetometerReg(AK09916_WIA2, device_id);
    if (res != 0) {
        // Return the underlying I2C/SPI or timeout error code
        return res; 
    }

    // Validate the retrieved hardware ID
    if (device_id != AK09916_EXPECTED_ID) {
        return -ENODEV; // Error: No such device (Hardware mismatch)
    }

    return 0; // Success: Magnetometer found and operational
}
#endif /* MAGNETOMETER_SUPPORT */

/******************************************* ICM20948 DMP ***********************************************/
int ICM20948_DMP::Init()
{
    // Initialize the base ICM20948 functionality first
    int res = ICM20948::Init(); 
    if (res != 0) {
        return res; 
    }

    DisableFifo(); // Ensure FIFO is disabled before configuring DMP

    // The DMP formats and pushes its own packets into the FIFO. The autonomous
    // raw-sensor-to-FIFO path (FIFO_EN_1/FIFO_EN_2) must be disabled, otherwise
    // raw register bytes get interleaved with DMP packets and corrupt the stream.
    SwitchMemoryBank(0);
    ifs_.WriteReg(FIFO_EN_1, 0x00);
    ifs_.WriteReg(FIFO_EN_2, 0x00);

    const uint8_t firmware_data[] = {
        #include "ICM-20948-dmp_img.h"
    };
    const uint32_t size = sizeof(firmware_data);
    const uint32_t DMP_PAGE_SIZE = 256;
    const uint32_t I2C_BURST_SIZE = 16;

    // Hardware registers for DMP memory interaction (All inside User Bank 0)
    const uint8_t REG_MEM_BANK_SEL   = 0x7E;
    const uint8_t REG_MEM_START_ADDR = 0x7C;
    const uint8_t REG_MEM_R_W        = 0x7D;

    // Ensure we are operating in User Bank 0 for the entire download process
    SwitchMemoryBank(0);

    uint32_t bytes_written = 0;

    // 1. Stream the firmware data blocks into the DMP SRAM safely
    while (bytes_written < size) {
        uint8_t dmp_bank_page = (uint8_t)(bytes_written >> 8); 
        uint8_t start_address = (uint8_t)(bytes_written & 0xFF); 

        uint32_t bytes_remaining = size - bytes_written;
        uint32_t chunk_size = (bytes_remaining > I2C_BURST_SIZE) ? I2C_BURST_SIZE : bytes_remaining;

        if (start_address + chunk_size > DMP_PAGE_SIZE) {
            chunk_size = DMP_PAGE_SIZE - start_address;
        }

        // CRITICAL INVEN SENSE PATTERN: Set the page and memory offset ONCE per block.
        // Do NOT re-write these registers inside the internal loop, otherwise the SRAM controller resets.
        ifs_.WriteReg(REG_MEM_BANK_SEL, dmp_bank_page);
        ifs_.WriteReg(REG_MEM_START_ADDR, start_address);

        // Stream bytes sequentially into the auto-incrementing R_W register
        ifs_.Write(REG_MEM_R_W, firmware_data + bytes_written, chunk_size); // Write the entire chunk in one go

        bytes_written += chunk_size;
    }

    if (bytes_written != size) {
        return -EFAULT; 
    }

    /* --- 2. DMP Configuration for 6-Axis Quaternion Output (Must be done BEFORE boot!) ---
     * DMP memory offsets are encoded as (bank << 8) | addr, where bank is the
     * high byte written to REG_MEM_BANK_SEL and addr is the low byte written to
     * REG_MEM_START_ADDR. Using the InvenSense DMP memory map:
     *   DATA_OUT_CTL1 = (4  * 16 + 0)  = 0x0040  -> bank 0x00, addr 0x40
     *   ODR_QUAT6     = (11 * 16 + 12) = 0x00BC  -> bank 0x00, addr 0xBC
     * The previous bank/addr values (0x0A/0x10 and 0x00/0x60) pointed at the
     * wrong DMP memory location, so the DMP was never told to output Game
     * Rotation Vector data, and the FIFO stayed empty.
     */
    ifs_.WriteReg(REG_MEM_START_ADDR, 0x40);
    ifs_.WriteReg(REG_MEM_R_W, 0x00); // High Byte (0x40) -> 0x00
    ifs_.WriteReg(REG_MEM_R_W, 0x08); // Low Byte  (0x41) -> 0x08 (Game Rot)

    // Configure DMP output data rate divider (0x01 divider = ~112 Hz output frequency)
    ifs_.WriteReg(REG_MEM_START_ADDR, 0xBC); // ODR_QUAT6
    ifs_.WriteReg(REG_MEM_R_W, 0x00); // High Byte
    ifs_.WriteReg(REG_MEM_R_W, 0x01); // Low Byte

    // 3. Set DMP Program Start Address execution vector.
    ifs_.WriteReg(0x7A, 0x10);
    ifs_.WriteReg(0x7B, 0x00);

    // 4. Final step: Boot up the DMP and enable FIFO data streaming
    ifs_.WriteReg(USER_CTRL, BIT(7) | BIT(6) | ifs_.SPI_IFS_Enabled());

    EnableFifo(); // Start pushing DMP packets into the FIFO

    // 5. Reset the FIFO once more now that the DMP is fully configured and
    // booted, to discard any stray bytes accumulated during firmware upload.
    ResetFIFO();

    return 0;
}

IMU::RealData<double>& ICM20948_DMP::GetRealIMUData()
{
#pragma pack(push, 1)
    struct DMP_QuatPacket
    {
        int32_t x;
        int32_t y;
        int32_t z;
        int32_t w;
    };
#pragma pack(pop)
    static_assert(sizeof(DMP_QuatPacket) == 16, "DMP_QuatPacket must be exactly 16 bytes to match the DMP output format");

    struct Quaternion
    {
        float w, x, y, z;
    };

    DMP_QuatPacket raw_packet;

    ifs_.Read(0x72, reinterpret_cast<uint8_t *>(&raw_packet), GetPacketSize());

    int32_t qx = BigEndianToNative(raw_packet.x);
    int32_t qy = BigEndianToNative(raw_packet.y);
    int32_t qz = BigEndianToNative(raw_packet.z);
    int32_t qw = BigEndianToNative(raw_packet.w);

    const float q_scale = 1073741824.0f; // 2^30
    Quaternion out_quat;
    out_quat.x = static_cast<float>(qx) / q_scale;
    out_quat.y = static_cast<float>(qy) / q_scale;
    out_quat.z = static_cast<float>(qz) / q_scale;
    out_quat.w = static_cast<float>(qw) / q_scale;

    cached_real_data = IMU::RealData<double>(
		static_cast<double>(out_quat.w),
		static_cast<double>(out_quat.x),
		static_cast<double>(out_quat.y),
		static_cast<double>(out_quat.z)
	);

    return cached_real_data;
}

IMU::RealData<double>& ICM20948_DMP::WaitForRealIMUData()
{
	while (!IsMDPDataReady()) {}
	return GetRealIMUData();
}

void ICM20948_DMP::ResetFIFO()
{
    SwitchMemoryBank(0);
    ifs_.WriteReg(USER_CTRL, ifs_.SPI_IFS_Enabled());
    ifs_.WriteReg(FIFO_RST, 0x1F); // Reset FIFO
    ifs_.WriteReg(FIFO_RST, 0x00);
    while (ifs_.ReadReg(FIFO_RST) & BIT(3)) {}; // Wait until FIFO reset is complete
    ifs_.WriteReg(USER_CTRL, BIT(7) | BIT(6) | ifs_.SPI_IFS_Enabled());
}

void ICM20948_DMP::EnableFifo()
{
    SwitchMemoryBank(0);
    ifs_.WriteReg(FIFO_CONFIG, 0x00); // 0x69 = FIFO_CONFIG, set FIFO mode to stop when full (default) and no watermark
    ifs_.WriteReg(FIFO_EN_1, 0x00); // Do not use external sensor data channels (Aux Slave 0-3) in the FIFO stream
    ifs_.WriteReg(FIFO_EN_2, BIT(4)); // only ACCEL_FIFO_ENABLE
    ifs_.WriteReg(USER_CTRL, BIT(7) | BIT(6) | ifs_.SPI_IFS_Enabled()); // Enable FIFO operation by setting FIFO_EN (Bit 6) in USER_CTRL

    isFifoEnabled = false; // Do not use FIFO for RAW data, only for DMP packets

    ResetFIFO();
}

bool ICM20948_DMP::IsMDPDataReady()
{
	return FifoCount() >= GetPacketSize();
}

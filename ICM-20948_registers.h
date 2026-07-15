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
 * ICM-20948 C++ driver registers map description header file.
 *
 * Contact Information:
 * Pavel Nadein <pavelnadein@gmail.com>
 */

#ifndef ICM_20948_REGISTERS_H
#define ICM_20948_REGISTERS_H

const uint8_t WHO_AM_I = 0x00; // WHO_AM_I register address (User Bank 0)
const uint8_t GYRO_CONFIG_1 = 0x01; // Gyroscope Configuration Register 1 (User Bank 2): FCHOICE[0], FS_SEL[2:1], DLPFCFG[5:3]
const uint8_t GYRO_CONFIG_2 = 0x02;  // Gyroscope Configuration Register 2 (User Bank 2): GYRO_AVGCFG[2:0]
const uint8_t ACCEL_CONFIG = 0x14;   // Accelerometer Configuration Register (User Bank 2): FCHOICE[0], FS_SEL[2:1], DLPFCFG[5:3]
const uint8_t ACCEL_CONFIG_2 = 0x15; // Accelerometer Configuration Register 2 (User Bank 2): DEC3_CFG[1:0]

// User Bank 0 registers
const uint8_t PWR_MGMT_1     = 0x06;
const uint8_t PWR_MGMT_2     = 0x07;

// PWR_MGMT_1 register bits
const uint8_t BIT_DEVICE_RESET = 0x80; // Bit 7: Triggers a full chip reset
const uint8_t BIT_SLEEP        = 0x40; // Bit 6: Puts the chip into sleep mode
const uint8_t CLK_BEST_AVAIL   = 0x01; // Bits 2:0: Auto-selects the best clock source

// Bank selection register
const uint8_t REG_BANK_SEL   = 0x7F;

// User Bank 0 registers
const uint8_t USER_CTRL      = 0x03;
const uint8_t LP_CONFIG      = 0x05;
const uint8_t INT_PIN_CFG    = 0x0F;
const uint8_t INT_ENABLE_1   = 0x11;
const uint8_t I2C_MST_STATUS = 0x17;
const uint8_t INT_STATUS     = 0x19;
const uint8_t INT_STATUS_1   = 0x1A;
const uint8_t FIFO_EN_1      = 0x66;
const uint8_t FIFO_EN_2      = 0x67;
const uint8_t FIFO_RST       = 0x68;
const uint8_t FIFO_CONFIG    = 0x69;
const uint8_t FIFO_COUNTH    = 0x70;
const uint8_t FIFO_R_W	     = 0x72;

// I2C_MST_STATUS register bits
const uint8_t I2C_SLV4_DONE  = 0x40; // Bit 6: SLV4 transaction completed

// Sensor data registers (Bank 0)
const uint8_t ACCEL_XOUT_H = 0x2D; // Starting address for accelerometer, gyroscope, and temperature data (14 bytes total)
const uint8_t TEMP_OUT_H   = 0x39; // Starting address for temperature data (2 bytes)
const uint8_t DATA_RDY_STATUS = 0x74; // Data Ready Status register (Bank 0)

// Calibration offset registers (Bank 1)
const uint8_t ACCEL_X_OFFSET_H = 0x14;
const uint8_t ACCEL_Y_OFFSET_H = 0x17;
const uint8_t ACCEL_Z_OFFSET_H = 0x1A;

// Gyroscope offset registers (Bank 2)
const uint8_t GYRO_SMPLRT_DIV  = 0x00;
const uint8_t GYRO_X_OFFSET_H  = 0x03;
const uint8_t GYRO_X_OFFSET_L  = 0x04;
const uint8_t GYRO_Y_OFFSET_H  = 0x05;
const uint8_t GYRO_Y_OFFSET_L  = 0x06;
const uint8_t GYRO_Z_OFFSET_H  = 0x07;
const uint8_t GYRO_Z_OFFSET_L  = 0x08;
const uint8_t ODR_ALIGN_EN     = 0x09;
const uint8_t ACCEL_SMPLRT_DIV_1 = 0x10;
const uint8_t ACCEL_SMPLRT_DIV_2 = 0x11;

// User Bank 3 registers
const uint8_t I2C_MST_ODR_CONFIG = 0x00; // Aux I2C master transaction scheduler ODR/divider config
const uint8_t I2C_MST_CTRL   = 0x01;
const uint8_t I2C_MST_DELAY_CTRL = 0x02; // Per-slave transaction delay/shadow enable bits
const uint8_t I2C_SLV0_ADDR  = 0x03;
const uint8_t I2C_SLV0_REG   = 0x04;
const uint8_t I2C_SLV0_CTRL  = 0x05;
const uint8_t I2C_SLV0_DO    = 0x06;
const uint8_t I2C_SLV4_ADDR  = 0x13;
const uint8_t I2C_SLV4_REG   = 0x14;
const uint8_t I2C_SLV4_CTRL  = 0x15;
const uint8_t I2C_SLV4_DO    = 0x16;

// AK09916 Magnetometer configuration
const uint8_t AK09916_I2C_ADDR = 0x0C;
const uint8_t AK09916_ST1      = 0x10;
const uint8_t AK09916_CNTL2    = 0x31;
const uint8_t AK09916_CNTL3    = 0x32;

// AK09916 Magnetometer registers
const uint8_t AK09916_WIA2     = 0x01; // Who I Am 2 register (Expected value: 0x09)
const uint8_t I2C_SLV4_DI      = 0x17; // Bank 3 register to read input data from SLV4

// User Bank 0 DMP Memory Access Registers
const uint8_t MEM_BANK_SEL      = 0x7C; // Selects the internal memory bank of DMP
const uint8_t MEM_START_ADDR    = 0x7D; // Sets the starting write address within the bank
const uint8_t MEM_R_W           = 0x7E; // Memory read/write data port
const uint8_t REG_MEM_BANK_SEL  = 0x7C; // Alias for MEM_BANK_SEL
const uint8_t REG_MEM_START_ADDR = 0x7D; // Alias for MEM_START_ADDR
const uint8_t REG_MEM_R_W        = 0x7E; // Alias for MEM_R_W
#endif /* ICM_20948_REGISTERS_H */

/* 06/16/2017 Copyright Tlera Corporation
 *
 *  Created by Kris Winer
 *
 Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor,
 getting properly scaled accelerometer, gyroscope, and magnetometer data out.
 Addition of 9 DoF sensor fusion using open source Madgwick and Mahony filter algorithms.
 Sketch runs on the 3.3 V Dragonfly STM32L476 Breakout Board.

 Library may be used freely and without limit with attribution.

*/

#include "MPU9250.h"
#include "i2c.h"
#define delay(x) (HAL_Delay(x))


float _aRes, _gRes, _mRes;
uint8_t _Mmode;

//MPU9250(uint8_t intPin)
//{
//  _intPin = intPin;
//}

uint8_t getMPU9250ID(imu_t *imu)
{
  uint8_t c = readByte(imu->MPU9250ID, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  return c;
}

  uint8_t getAK8963CID(imu_t *imu)
{
//  uint8_t c = readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);  // Read WHO_AM_I register for MPU-9250
  writeByte(imu->MPU9250ID, USER_CTRL, 0x20);    // Enable I2C Master mode
  writeByte(imu->MPU9250ID, I2C_MST_CTRL, 0x0D); // I2C configuration multi-master I2C 400KHz

  writeByte(imu->MPU9250ID, I2C_SLV0_ADDR, (AK8963_ADDRESS | 0x80));    // Set the I2C slave address of AK8963 and set for read.
  writeByte(imu->MPU9250ID, I2C_SLV0_REG, WHO_AM_I_AK8963);           // I2C slave 0 register address from where to begin data transfer
  writeByte(imu->MPU9250ID, I2C_SLV0_CTRL, 0x81);                     // Enable I2C and transfer 1 byte
  delay(10);
  uint8_t c = readByte(imu->MPU9250ID, EXT_SENS_DATA_00);             // Read the WHO_AM_I byte
  return c;
}


float getMres(uint8_t Mscale) {
  switch (Mscale)
  {
   // Possible magnetometer scales (and their register bit settings) are:
  // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
          _mRes = 10.0f*4912.0f/8190.0f; // Proper scale to return milliGauss
          return _mRes;
          break;
    case MFS_16BITS:
          _mRes = 10.0f*4912.0f/32760.0f; // Proper scale to return milliGauss
          return _mRes;
          break;
  }
  return 0;
}

float getGres(uint8_t Gscale) {
  switch (Gscale)
  {
  // Possible gyro scales (and their register bit settings) are:
  // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    case GFS_250DPS:
          _gRes = 250.0/32768.0;
          return _gRes;
          break;
    case GFS_500DPS:
          _gRes = 500.0/32768.0;
          return _gRes;
          break;
    case GFS_1000DPS:
         _gRes = 1000.0/32768.0;
         return _gRes;
         break;
    case GFS_2000DPS:
          _gRes = 2000.0/32768.0;
         return _gRes;
         break;
  }
  return 0;
}

float getAres(uint8_t Ascale) {
  switch (Ascale)
  {
  // Possible accelerometer scales (and their register bit settings) are:
  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
         _aRes = 2.0f/32768.0f;
         return _aRes;
         break;
    case AFS_4G:
         _aRes = 4.0f/32768.0f;
         return _aRes;
         break;
    case AFS_8G:
         _aRes = 8.0f/32768.0f;
         return _aRes;
         break;
    case AFS_16G:
         _aRes = 16.0f/32768.0f;
         return _aRes;
         break;
  }
  return 0;
}



void accelWakeOnMotion(imu_t* imu)
{
  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
  // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  uint8_t c = readByte(imu->MPU9250ID, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | 0x01;  // Set accelerometer rate to 1 kHz and bandwidth to 184 Hz
  writeByte(imu->MPU9250ID, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
   writeByte(imu->MPU9250ID, INT_PIN_CFG, 0x12);  // INT is 50 microsecond pulse and any read to clear
   writeByte(imu->MPU9250ID, INT_ENABLE, 0x41);   // Enable data ready (bit 0) and wake on motion (bit 6)  interrupt

   // enable wake on motion detection logic (bit 7) and compare current sample to previous sample (bit 6)
   writeByte(imu->MPU9250ID, MOT_DETECT_CTRL, 0xC0);

   // set accel threshold for wake up at  mG per LSB, 1 - 255 LSBs == 0 - 1020 mg), pic 0x19 for 25 mg
   writeByte(imu->MPU9250ID, WOM_THR, 0x19);

  // set sample rate in low power mode
  /* choices are 0 == 0.24 Hz, 1 == 0.49 Hz, 2 == 0.98 Hz, 3 == 1.958 Hz, 4 == 3.91 Hz, 5 == 7.81 Hz
   *             6 == 15.63 Hz, 7 == 31.25 Hz, 8 == 62.50 Hz, 9 = 125 Hz, 10 == 250 Hz, and 11 == 500 Hz
   */
  writeByte(imu->MPU9250ID, LP_ACCEL_ODR, 0x02);

  c = readByte(imu->MPU9250ID, PWR_MGMT_1);
  writeByte(imu->MPU9250ID, PWR_MGMT_1, c | 0x20);     // Write bit 5 to enable accel cycling

  gyromagSleep(imu);
  delay(100); // Wait for all registers to reset

}


void gyromagSleep(imu_t* imu)
{
  uint8_t temp = 0;
  writeByte(imu->MPU9250ID, I2C_SLV0_ADDR, (AK8963_ADDRESS | 0x80));        // Set the I2C slave address of AK8963 and set for read.
  writeByte(imu->MPU9250ID, I2C_SLV0_REG,  AK8963_CNTL);                  // I2C slave 0 register address from where to begin data transfer
  writeByte(imu->MPU9250ID, I2C_SLV0_CTRL, 0x81);                         // Enable I2C and transfer 1 byte
  delay(10);
  temp = readByte(imu->MPU9250ID, EXT_SENS_DATA_00);

  writeByte(imu->MPU9250ID, I2C_SLV0_ADDR, AK8963_ADDRESS);               // Set the I2C slave address of AK8963 and set for write.
  writeByte(imu->MPU9250ID, I2C_SLV0_REG,  AK8963_CNTL);                  // I2C slave 0 register address from where to begin data transfer
  writeByte(imu->MPU9250ID, I2C_SLV0_DO, temp & ~(0x0F));                 // Power down AK8963
  writeByte(imu->MPU9250ID, I2C_SLV0_CTRL, 0x81);                         // Enable I2C and transfer 1 byte
  delay(10);

  temp = readByte(imu->MPU9250ID, PWR_MGMT_1);
  writeByte(imu->MPU9250ID, PWR_MGMT_1, temp | 0x10);     // Write bit 4 to enable gyro standby
  delay(10); // Wait for all registers to reset
}


void gyromagWake(imu_t* imu, uint8_t Mmode)
{
  uint8_t temp = 0;
  writeByte(imu->MPU9250ID, I2C_SLV0_ADDR, (AK8963_ADDRESS | 0x80));        // Set the I2C slave address of AK8963 and set for read.
  writeByte(imu->MPU9250ID, I2C_SLV0_REG,  AK8963_CNTL);                  // I2C slave 0 register address from where to begin data transfer
  writeByte(imu->MPU9250ID, I2C_SLV0_CTRL, 0x81);                         // Enable I2C and transfer 1 byte
  delay(10);
  temp = readByte(imu->MPU9250ID, EXT_SENS_DATA_00);

  writeByte(imu->MPU9250ID, I2C_SLV0_ADDR, AK8963_ADDRESS);               // Set the I2C slave address of AK8963 and set for write.
  writeByte(imu->MPU9250ID, I2C_SLV0_REG,  AK8963_CNTL);                  // I2C slave 0 register address from where to begin data transfer
  writeByte(imu->MPU9250ID, I2C_SLV0_DO, temp | Mmode);                   // Reset normal mode for  magnetometer
  writeByte(imu->MPU9250ID, I2C_SLV0_CTRL, 0x81);                         // Enable I2C and transfer 1 byte
  delay(10);
  temp = readByte(imu->MPU9250ID, PWR_MGMT_1);
  writeByte(imu->MPU9250ID, PWR_MGMT_1, 0x01);                            // return gyro and accel normal mode
  delay(10); // Wait for all registers to reset
}


void resetMPU9250(imu_t* imu)
{
  // reset device
  writeByte(imu->MPU9250ID, PWR_MGMT_1, 0x80); // Set bit 7 to reset MPU9250
  delay(100); // Wait for all registers to reset
}

void readMPU9250Data(imu_t* imu)
{
  uint8_t rawData[14];  // x/y/z accel register data stored here
  readBytes(imu->MPU9250ID, ACCEL_XOUT_H, 14, &rawData[0]);  // Read the 14 raw data registers into data array
  imu->MPU9250Data[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  imu->MPU9250Data[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  imu->MPU9250Data[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
  imu->MPU9250Data[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;
  imu->MPU9250Data[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;
  imu->MPU9250Data[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;
  imu->MPU9250Data[6] = ((int16_t)rawData[12] << 8) | rawData[13] ;
}

void readAccelData(imu_t* imu)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(imu->MPU9250ID, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  imu->accelCount[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  imu->accelCount[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  imu->accelCount[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}


void readGyroData(imu_t* imu)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(imu->MPU9250ID, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  imu->gyroCount[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  imu->gyroCount[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  imu->gyroCount[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

uint8_t checkNewMagData(imu_t* imu)
{
  uint8_t test;
//  test = (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01);
  writeByte(imu->MPU9250ID, I2C_SLV0_ADDR, (AK8963_ADDRESS | 0x80));     // Set the I2C slave address of AK8963 and set for read.
  writeByte(imu->MPU9250ID, I2C_SLV0_REG, AK8963_ST1);                 // I2C slave 0 register address from where to begin data transfer
  writeByte(imu->MPU9250ID, I2C_SLV0_CTRL, 0x81);                     // Enable I2C and transfer 1 byte
  delay(2);
  test = (readByte(imu->MPU9250ID, EXT_SENS_DATA_00) & 0x01); // Check data ready status byte
  return test;
}

uint8_t checkNewAccelGyroData(imu_t* imu)
{
  uint8_t test;
  test = (readByte(imu->MPU9250ID, INT_STATUS) & 0x01);
  return test;
}

uint8_t checkWakeOnMotion(imu_t* imu)
{
  uint8_t test;
  test = (readByte(imu->MPU9250ID, INT_STATUS) & 0x40);
  return test;
}


void readMagData(imu_t* imu)
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
//  readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
   writeByte(imu->MPU9250ID, I2C_SLV0_ADDR, (AK8963_ADDRESS | 0x80));    // Set the I2C slave address of AK8963 and set for read.
   writeByte(imu->MPU9250ID, I2C_SLV0_REG, AK8963_XOUT_L);             // I2C slave 0 register address from where to begin data transfer
   writeByte(imu->MPU9250ID, I2C_SLV0_CTRL, 0x87);                     // Enable I2C and read 7 bytes
//   delay(10);
   readBytes(imu->MPU9250ID, EXT_SENS_DATA_00, 7, &rawData[0]);        // Read the x-, y-, and z-axis calibration values
   uint8_t c = rawData[6]; // End data read by reading ST2 register
   if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
   imu->magCount[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
   imu->magCount[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
   imu->magCount[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
   }
}

int16_t readGyroTempData(imu_t* imu)
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(imu->MPU9250ID, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
  return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}


void initAK8963Slave(imu_t* imu, uint8_t Mscale, uint8_t Mmode)
{
   // First extract the factory calibration for each magnetometer axis
   uint8_t rawData[3];  // x/y/z gyro calibration data stored here
   _Mmode = Mmode;

   writeByte(imu->MPU9250ID, I2C_SLV0_ADDR, AK8963_ADDRESS);           // Set the I2C slave address of AK8963 and set for write.
   writeByte(imu->MPU9250ID, I2C_SLV0_REG, AK8963_CNTL2);              // I2C slave 0 register address from where to begin data transfer
   writeByte(imu->MPU9250ID, I2C_SLV0_DO, 0x01);                       // Reset AK8963
   writeByte(imu->MPU9250ID, I2C_SLV0_CTRL, 0x81);                     // Enable I2C and write 1 byte
   delay(50);
   writeByte(imu->MPU9250ID, I2C_SLV0_ADDR, AK8963_ADDRESS);           // Set the I2C slave address of AK8963 and set for write.
   writeByte(imu->MPU9250ID, I2C_SLV0_REG, AK8963_CNTL);               // I2C slave 0 register address from where to begin data transfer
   writeByte(imu->MPU9250ID, I2C_SLV0_DO, 0x00);                       // Power down magnetometer
   writeByte(imu->MPU9250ID, I2C_SLV0_CTRL, 0x81);                     // Enable I2C and write 1 byte
   delay(50);
   writeByte(imu->MPU9250ID, I2C_SLV0_ADDR, AK8963_ADDRESS);           // Set the I2C slave address of AK8963 and set for write.
   writeByte(imu->MPU9250ID, I2C_SLV0_REG, AK8963_CNTL);               // I2C slave 0 register address from where to begin data transfer
   writeByte(imu->MPU9250ID, I2C_SLV0_DO, 0x0F);                       // Enter fuze mode
   writeByte(imu->MPU9250ID, I2C_SLV0_CTRL, 0x81);                     // Enable I2C and write 1 byte
   delay(50);

   writeByte(imu->MPU9250ID, I2C_SLV0_ADDR, (AK8963_ADDRESS | 0x80));    // Set the I2C slave address of AK8963 and set for read.
   writeByte(imu->MPU9250ID, I2C_SLV0_REG, AK8963_ASAX);               // I2C slave 0 register address from where to begin data transfer
   writeByte(imu->MPU9250ID, I2C_SLV0_CTRL, 0x83);                     // Enable I2C and read 3 bytes
   delay(50);
   readBytes(imu->MPU9250ID, EXT_SENS_DATA_00, 3, &rawData[0]);        // Read the x-, y-, and z-axis calibration values
   imu->magCalibration[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;        // Return x-axis sensitivity adjustment values, etc.
   imu->magCalibration[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;
   imu->magCalibration[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f;

   writeByte(imu->MPU9250ID, I2C_SLV0_ADDR, AK8963_ADDRESS);           // Set the I2C slave address of AK8963 and set for write.
   writeByte(imu->MPU9250ID, I2C_SLV0_REG, AK8963_CNTL);               // I2C slave 0 register address from where to begin data transfer
   writeByte(imu->MPU9250ID, I2C_SLV0_DO, 0x00);                       // Power down magnetometer
   writeByte(imu->MPU9250ID, I2C_SLV0_CTRL, 0x81);                     // Enable I2C and transfer 1 byte
   delay(50);

   writeByte(imu->MPU9250ID, I2C_SLV0_ADDR, AK8963_ADDRESS);           // Set the I2C slave address of AK8963 and set for write.
   writeByte(imu->MPU9250ID, I2C_SLV0_REG, AK8963_CNTL);               // I2C slave 0 register address from where to begin data transfer
   // Configure the magnetometer for continuous read and highest resolution
   // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
   // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
   writeByte(imu->MPU9250ID, I2C_SLV0_DO, Mscale << 4 | Mmode);        // Set magnetometer data resolution and sample ODR
   writeByte(imu->MPU9250ID, I2C_SLV0_CTRL, 0x81);                     // Enable I2C and transfer 1 byte
   delay(50);
}


void initMPU9250(imu_t* imu, uint8_t Ascale, uint8_t Gscale, uint8_t sampleRate, uint8_t intMode)
{
 // wake up device
  writeByte(imu->MPU9250ID, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  delay(100); // Wait for all registers to reset

 // get stable time source
  writeByte(imu->MPU9250ID, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200);

 // Configure Gyro and Thermometer
 // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
 // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 // be higher than 1 / 0.0059 = 170 Hz
 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
 // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(imu->MPU9250ID, CONFIG, 0x03);

 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(imu->MPU9250ID, SMPLRT_DIV, sampleRate);  // Use a 200 Hz rate; a rate consistent with the filter update rate
                                                       // determined inset in CONFIG above

 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(imu->MPU9250ID, GYRO_CONFIG); // get current GYRO_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x02; // Clear Fchoice bits [1:0]
  c = c & ~0x18; // Clear AFS bits [4:3]
  c = c | Gscale << 3; // Set full scale range for the gyro
 // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  writeByte(imu->MPU9250ID, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

 // Set accelerometer full-scale range configuration
  c = readByte(imu->MPU9250ID, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | Ascale << 3; // Set full scale range for the accelerometer
  writeByte(imu->MPU9250ID, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(imu->MPU9250ID, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeByte(imu->MPU9250ID, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
   if (intMode == 1) writeByte(imu->MPU9250ID, INT_PIN_CFG, 0x10);  // INT is 50 microsecond pulse and any read to clear
   else writeByte(imu->MPU9250ID, INT_PIN_CFG, 0x22);
   writeByte(imu->MPU9250ID, INT_ENABLE, 0x01);   // Enable data ready (bit 0) interrupt
   delay(100);

  writeByte(imu->MPU9250ID, USER_CTRL, 0x20);          // Enable I2C Master mode
  writeByte(imu->MPU9250ID, I2C_MST_CTRL, 0x1D);       // I2C configuration STOP after each transaction, master I2C bus at 400 KHz
  writeByte(imu->MPU9250ID, I2C_MST_DELAY_CTRL, 0x81); // Use blocking data retreival and enable delay for mag sample rate mismatch
  writeByte(imu->MPU9250ID, I2C_SLV4_CTRL, 0x01);      // Delay mag data retrieval to once every other accel/gyro data sample
}


void magcalMPU9250(imu_t* imu)
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};
  uint8_t rawData[7] = {0, 0, 0, 0, 0, 0, 0}, magCalibration[3] = {0, 0, 0};

//  Serial.print("Mag Calibration for: 0x"); Serial.println(imu->MPU9250ID, HEX); Serial.println("Wave device in a figure eight until done!");
  delay(4000);

// shoot for ~fifteen seconds of mag data
  if(_Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
  if(_Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
  for(ii = 0; ii < sample_count; ii++) {
    writeByte(imu->MPU9250ID, I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80);    // Set the I2C slave address of AK8963 and set for read.
    writeByte(imu->MPU9250ID, I2C_SLV0_REG, AK8963_XOUT_L);             // I2C slave 0 register address from where to begin data transfer
    writeByte(imu->MPU9250ID, I2C_SLV0_CTRL, 0x87);                     // Enable I2C and read 7 bytes
    if(_Mmode == 0x02) delay(125);  // at 8 Hz ODR, new mag data is available every 125 ms
    if(_Mmode == 0x06) delay(10);   // at 100 Hz ODR, new mag data is available every 10 ms
    readBytes(imu->MPU9250ID, EXT_SENS_DATA_00, 7, &rawData[0]);        // Read the x-, y-, and z-axis calibration values
    mag_temp[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;     // Turn the MSB and LSB into a signed 16-bit value
    mag_temp[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;     // Data stored as little Endian
    mag_temp[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;

    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
 }

//    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
//    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
//    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

    writeByte(imu->MPU9250ID, I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80);    // Set the I2C slave address of AK8963 and set for read.
    writeByte(imu->MPU9250ID, I2C_SLV0_REG, AK8963_ASAX);               // I2C slave 0 register address from where to begin data transfer
    writeByte(imu->MPU9250ID, I2C_SLV0_CTRL, 0x83);                     // Enable I2C and read 3 bytes
    delay(50);
    readBytes(imu->MPU9250ID, EXT_SENS_DATA_00, 3, &rawData[0]);        // Read the x-, y-, and z-axis calibration values
    magCalibration[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;        // Return x-axis sensitivity adjustment values, etc.
    magCalibration[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;
    magCalibration[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f;

    imu->magBias[0] = (float) mag_bias[0]*_mRes*magCalibration[0];  // save mag biases in G for main program
    imu->magBias[1] = (float) mag_bias[1]*_mRes*magCalibration[1];
    imu->magBias[2] = (float) mag_bias[2]*_mRes*magCalibration[2];

    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    imu->magScale[0] = avg_rad/((float)mag_scale[0]);
    imu->magScale[1] = avg_rad/((float)mag_scale[1]);
    imu->magScale[2] = avg_rad/((float)mag_scale[2]);

//   Serial.println("Mag Calibration done!");
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU9250(imu_t* imu)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

 // reset device
  writeByte(imu->MPU9250ID, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);

 // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
 // else use the internal oscillator, bits 2:0 = 001
  writeByte(imu->MPU9250ID, PWR_MGMT_1, 0x01);
  writeByte(imu->MPU9250ID, PWR_MGMT_2, 0x00);
  delay(200);

// Configure device for bias calculation
  writeByte(imu->MPU9250ID, INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(imu->MPU9250ID, FIFO_EN, 0x00);      // Disable FIFO
  writeByte(imu->MPU9250ID, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(imu->MPU9250ID, I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(imu->MPU9250ID, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(imu->MPU9250ID, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);

// Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(imu->MPU9250ID, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(imu->MPU9250ID, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(imu->MPU9250ID, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(imu->MPU9250ID, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(imu->MPU9250ID, USER_CTRL, 0x40);   // Enable FIFO
  writeByte(imu->MPU9250ID, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
  writeByte(imu->MPU9250ID, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(imu->MPU9250ID, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(imu->MPU9250ID, FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];

}
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}

// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

// Push gyro biases to hardware registers
  writeByte(imu->MPU9250ID, XG_OFFSET_H, data[0]);
  writeByte(imu->MPU9250ID, XG_OFFSET_L, data[1]);
  writeByte(imu->MPU9250ID, YG_OFFSET_H, data[2]);
  writeByte(imu->MPU9250ID, YG_OFFSET_L, data[3]);
  writeByte(imu->MPU9250ID, ZG_OFFSET_H, data[4]);
  writeByte(imu->MPU9250ID, ZG_OFFSET_L, data[5]);

// Output scaled gyro biases for display in the main program
  imu->gyroBias[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
  imu->gyroBias[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  imu->gyroBias[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(imu->MPU9250ID, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(imu->MPU9250ID, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(imu->MPU9250ID, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for(ii = 0; ii < 3; ii++) {
    if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
//  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
//  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
//  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
//  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
//  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
//  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

// Output scaled accelerometer biases for display in the main program
   imu->accelBias[0] = (float)accel_bias[0]/(float)accelsensitivity;
   imu->accelBias[1] = (float)accel_bias[1]/(float)accelsensitivity;
   imu->accelBias[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void SelfTest(imu_t* imu) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
   uint8_t selfTest[6];
   int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
   float factoryTrim[6];
   uint8_t FS = 0;

  writeByte(imu->MPU9250ID, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  writeByte(imu->MPU9250ID, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(imu->MPU9250ID, GYRO_CONFIG, 1<<FS);  // Set full scale range for the gyro to 250 dps
  writeByte(imu->MPU9250ID, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(imu->MPU9250ID, ACCEL_CONFIG, 1<<FS); // Set full scale range for the accelerometer to 2 g

  for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer

  readBytes(imu->MPU9250ID, ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
  aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    readBytes(imu->MPU9250ID, GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
  gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
  aAvg[ii] /= 200;
  gAvg[ii] /= 200;
  }

// Configure the accelerometer for self-test
   writeByte(imu->MPU9250ID, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
   writeByte(imu->MPU9250ID, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   delay(25);  // Delay a while to let the device stabilize

  for( int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer

  readBytes(imu->MPU9250ID, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    readBytes(imu->MPU9250ID, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
  aSTAvg[ii] /= 200;
  gSTAvg[ii] /= 200;
  }

 // Configure the gyro and accelerometer for normal operation
   writeByte(imu->MPU9250ID, ACCEL_CONFIG, 0x00);
   writeByte(imu->MPU9250ID, GYRO_CONFIG,  0x00);
   delay(25);  // Delay a while to let the device stabilize

   // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
   selfTest[0] = readByte(imu->MPU9250ID, SELF_TEST_X_ACCEL); // X-axis accel self-test results
   selfTest[1] = readByte(imu->MPU9250ID, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
   selfTest[2] = readByte(imu->MPU9250ID, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
   selfTest[3] = readByte(imu->MPU9250ID, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
   selfTest[4] = readByte(imu->MPU9250ID, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
   selfTest[5] = readByte(imu->MPU9250ID, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
   factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
   factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
   factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
   factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
   factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
   factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get percent, must multiply by 100
   for (int i = 0; i < 3; i++) {
     imu->SelfTest[i]   = 100.0f*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.0f;   // Report percent differences
     imu->SelfTest[i+3] = 100.0f*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.0f; // Report percent differences
   }

}


/*
// simple function to scan for I2C devices on the bus
void I2Cscan()
{
  // scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();


    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}
*/

// I2C read/write functions for the MPU9250 sensors

  void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c1, address, subAddress, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

  uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data = 0;                        // `data` will store the register data
  HAL_I2C_Mem_Read(&hi2c1, address, subAddress, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
  return data;                             // Return data read from slave register
}

  void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
    HAL_I2C_Mem_Read(&hi2c1, address, subAddress, I2C_MEMADD_SIZE_8BIT, dest, count, 100);
}


void Madgwick2(imu_t* imu){
  float q1 = imu->q[0], q2 = imu->q[1], q3 = imu->q[2], q4 = imu->q[3];   // short name local variable for readability
  float ax = imu->ax, ay = imu->ay, az = imu->az, mx = imu->mx, my = imu->my, mz = imu->mz;
  float bx = imu->bx, bz = imu->bz;
  float qDot1, qDot2, qDot3, qDot4;
  float qHatDot1, qHatDot2, qHatDot3, qHatDot4;    // estimated direction of Gyroscope error
  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33, J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64; // Jacobian elements!
  float f_1, f_2, f_3, f_4, f_5, f_6;                   // objective function elements
  float g_err_x, g_err_y, g_err_z;
  float gx = imu->gx*PI/180.0f, gy = imu->gy*PI/180.0f, gz = imu->gz*PI/180.0f;
  float hx, hy, hz;               // computed flux in the earth frame
  float norm;

  float half_q1 = 0.5f * q1, half_q2 = 0.5f * q2, half_q3 = 0.5f * q3, half_q4 = 0.5f * q4;
  float _2q1 = 2.0f* q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3, _2q4 = 2.0f * q4;
  float _2bx = 2.0f * bx, _2bz = 2.0f * bz;
  float _2bxq1 = 2.0f * bx * q1, _2bxq2 = 2.0f * bx * q2, _2bxq3 = 2.0f * bx * q3, _2bxq4 = 2.0f * bx * q4;
  float _2bzq1 = 2.0f * bx * q1, _2bzq2 = 2.0f * bz * q2, _2bzq3 = 2.0f * bz * q3, _2bzq4 = 2.0f * bz * q4;
  float q1q2, q1q3 = q1 * q3, q1q4, q2q3, q2q4 = q2 * q4, q3q4;
  float _2mx = 2.0f * mx, _2my = 2.0f * my, _2mz = 2.0f * mz;

  norm = sqrtf(ax * ax + ay * ay + az * az);
  ax/=norm;
  ay/=norm;
  az/=norm;

  norm = sqrtf(mx * mx + my * my + mz * mz);
  mx/=norm;
  my/=norm;
  mz/=norm;

  f_1 = _2q2 *q4 -_2q1 * q3 - ax;
  f_2 = _2q1 * q2 + _2q3 * q4 - ay;
  f_3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
  f_4 = _2bx * (0.5f - q3 * q3 - q4 * q4) + _2bz * (q2q4 - q1q3) -mx;
  f_5 = _2bx * (q2*q3 - q1*q4) + _2bz*(q1*q2 + q3*q4) - my;
  f_6 = _2bx * (q1q3 + q2q4) + _2bz*(0.5f - q2 * q2 - q3 * q3) - mz;

  J_11or24 = _2q3;
  J_12or23 = 2.0f * q4;
  J_13or22 = _2q1;
  J_14or21 = _2q2;
  J_32 = 2.0f * J_14or21;
  J_33 = 2.0f * J_11or24;
  J_41 = _2bzq3;
  J_42 = _2bzq4;
  J_43 = 2.0f * _2bxq3 + _2bzq1;
  J_44 = 2.0f * _2bxq4 - _2bzq2;
  J_51 = _2bxq4 - _2bzq2;
  J_52 = _2bxq3 + _2bzq1;
  J_53 = _2bxq2 + _2bzq4;
  J_54 = _2bxq1 - _2bzq3;
  J_61 = _2bxq3;
  J_62 = _2bxq4 - 2.0f * _2bzq2;
  J_63 = _2bxq1 - 2.0f * _2bzq3;
  J_64 = _2bxq2;

  qHatDot1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
  qHatDot2 = J_12or23 * f_1 + J_13or22 *f_2 -J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 *f_6;
  qHatDot3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
  qHatDot4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 -J_54 * f_5 + J_64 * f_6;

  norm = sqrtf(qHatDot1 * qHatDot1 + qHatDot2 * qHatDot2 + qHatDot3 * qHatDot3 + qHatDot4 * qHatDot4);
  qHatDot1 /= norm;
  qHatDot2 /= norm;
  qHatDot3 /= norm;
  qHatDot4 /= norm;

  g_err_x = _2q1 * qHatDot2 - _2q2 * qHatDot1 - _2q3 * qHatDot4 + _2q4 * qHatDot3;
  g_err_y = _2q1 * qHatDot3 + _2q2 * qHatDot4 - _2q3 * qHatDot1 - _2q4 * qHatDot2;
  g_err_z = _2q1 * qHatDot4 - _2q2 * qHatDot3 + _2q3 * qHatDot2 - _2q4 * qHatDot1;

  imu->g_bx += g_err_x * imu->deltat * imu->zeta;
  imu->g_by += g_err_y * imu->deltat * imu->zeta;
  imu->g_bz += g_err_z * imu->deltat * imu->zeta;
  gx -= imu->g_bx;
  gy -= imu->g_by;
  gz -= imu->g_bz;

  qDot1 = - half_q2 * gx - half_q3 * gy - half_q4 * gz;
  qDot2 = half_q1 * gx + half_q3 * gz - half_q4 * gy;
  qDot3 = half_q1 * gy - half_q2 * gz + half_q4 * gx;
  qDot4 = half_q1 * gz + half_q2 * gy - half_q3 * gx;


  q1 += (qDot1 - (imu->beta * qHatDot1)) * imu->deltat;
  q2 += (qDot2 - (imu->beta * qHatDot2)) * imu->deltat;
  q3 += (qDot3 - (imu->beta * qHatDot3)) * imu->deltat;
  q4 += (qDot4 - (imu->beta * qHatDot4)) * imu->deltat;


  norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  q1 /= norm;
  q2 /= norm;
  q3 /= norm;
  q4 /= norm;


  q1q2 = q1 * q2;
  q1q3 = q1 * q3;
  q1q4 = q1 * q4;
  q3q4 = q3 * q4;
  q2q3 = q2 * q3;
  q2q4 = q2 * q4;

  hx = _2mx *(0.5f - q3 * q3 - q4 * q4) + _2my * (q2q3 - q1q4) + _2mz * (q2q4 + q1q3);
  hy = _2mx *(q2q3 + q1q4) + _2my * (0.5f -q2 * q2 - q4 * q4) + _2mz * (q3q4 - q1q2);
  hz = _2mx * (q2q4 - q1q3) + _2my * (q3q4 + q1q2) + _2mz * (0.5f - q2*q2 - q3*q3);

  imu->bx = sqrtf(hx * hx + hy * hy);
  imu->bz = hz;

  imu->q[0] = q1;
  imu->q[1] = q2;
  imu->q[2] = q3;
  imu->q[3] = q4;

}

void MadgwickQuaternionUpdate(imu_t* imu)
{
  float q1 = imu->q[0], q2 = imu->q[1], q3 = imu->q[2], q4 = imu->q[3];   // short name local variable for readability
  float ax = imu->ax, ay = imu->ay, az = imu->az, mx = imu->mx, my = imu->my, mz = imu->mz;
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;
  float gx = imu->gx*PI/180.0f, gy = imu->gy*PI/180.0f, gz = imu->gz*PI/180.0f;
  // Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrtf(ax * ax + ay * ay + az * az);
//  arm_sqrt_f32(ax * ax + ay * ay + az * az, &norm);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrtf(mx * mx + my * my + mz * mz);
//  arm_sqrt_f32(mx * mx + my * my + mz * mz, &norm);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrtf(hx * hx + hy * hy);
//  arm_sqrt_f32(hx * hx + hy * hy, &_2bx);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
//  arm_sqrt_f32(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4, &norm); // normalise step magnitude
  norm = 1.0f/norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;

  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - imu->beta * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - imu->beta * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - imu->beta * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - imu->beta * s4;

  // Integrate to yield quaternion
  q1 += qDot1 * imu->deltat;
  q2 += qDot2 * imu->deltat;
  q3 += qDot3 * imu->deltat;
  q4 += qDot4 * imu->deltat;
  norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
//  arm_sqrt_f32(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4, &norm); // normalise quaternion
  norm = 1.0f/norm;
  imu->q[0] = q1 * norm;
  imu->q[1] = q2 * norm;
  imu->q[2] = q3 * norm;
  imu->q[3] = q4 * norm;
}

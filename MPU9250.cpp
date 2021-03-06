#include "MPU9250.h"

/// Returns true when the MPU9250 registers have been filled with new data.
bool MPU9250::ready() {
	return (readByte(this->MPU9250_ADDRESS, INT_STATUS) & 0x01) && (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01);
}

/// Fills a MPU9250Dataset structure with the latest sensor data.
void MPU9250::update(MPU9250Dataset& dataset) {
	uint8_t rawData[14]; // x/y/z accel register data stored here
	readBytes(this->MPU9250_ADDRESS, ACCEL_XOUT_H, 14, &rawData[0]); // Read the 14 raw data registers into data array

	int16_t rawValues[7];
	rawValues[0] = ((int16_t)rawData[0] << 8) | rawData[1]; // Turn the MSB and LSB into a signed 16-bit value
	rawValues[1] = ((int16_t)rawData[2] << 8) | rawData[3];
	rawValues[2] = ((int16_t)rawData[4] << 8) | rawData[5];
	rawValues[3] = ((int16_t)rawData[6] << 8) | rawData[7];
	rawValues[4] = ((int16_t)rawData[8] << 8) | rawData[9];
	rawValues[5] = ((int16_t)rawData[10] << 8) | rawData[11];
	rawValues[6] = ((int16_t)rawData[12] << 8) | rawData[13];

	// Now we'll calculate the accleration value into actual g's
	dataset.Ax = (float)rawValues[0] * this->aRes - this->accelBias[0];
	dataset.Ay = (float)rawValues[1] * this->aRes - this->accelBias[1];
	dataset.Az = (float)rawValues[2] * this->aRes - this->accelBias[2];

	// Temperature in degrees Centigrade
	dataset.T = ((float)rawValues[3]) / 333.87 + 21.0;

	// Calculate the gyro value into actual degrees per second
	dataset.Gx = (float)rawValues[4] * this->gRes - this->gyroBias[0];
	dataset.Gy = (float)rawValues[5] * this->gRes - this->gyroBias[1];
	dataset.Gz = (float)rawValues[6] * this->gRes - this->gyroBias[2];
/*
	uint8_t rawMagData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
	int16_t rawMagValues[3];
	if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
		readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawMagData[0]);  // Read the six raw data and ST2 registers sequentially into data array
		uint8_t c = rawMagData[6]; // End data read by reading ST2 register
		if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
			rawMagValues[0] = ((int16_t)rawMagData[1] << 8) | rawMagData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
			rawMagValues[1] = ((int16_t)rawMagData[3] << 8) | rawMagData[2] ;  // Data stored as little Endian
			rawMagValues[2] = ((int16_t)rawMagData[5] << 8) | rawMagData[4] ;
		}
	}

	// Calculate the magnetometer values in milliGauss
	// Include factory calibration per data sheet and user environmental corrections
	dataset.Mx = (float)rawMagValues[0] * this->mRes * this->magCalibration[0] - this->magBias[0];
	dataset.My = (float)rawMagValues[1] * this->mRes * this->magCalibration[1] - this->magBias[1];
	dataset.Mz = (float)rawMagValues[2] * this->mRes * this->magCalibration[2] - this->magBias[2];
	*/
}

/// Initializes the MPU9250 and onboard AK8963 magnetometer, and sets their sensor resolutions. Returns false if we can't communicate with the MPU9250 or AK8963.
bool MPU9250::init(uint8_t Ascale = AFS_4G, uint8_t Gscale = GFS_500DPS, uint8_t Mscale = MFS_16BITS, uint8_t Mmode = MMODE_100HZ, bool set_AD0 = false) {
	if(set_AD0)
		this->MPU9250_ADDRESS = 0x69;
		
	if(readByte(this->MPU9250_ADDRESS, WHO_AM_I_MPU9250) == 0x71) {
		// reset device
		writeByte(this->MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
		delay(100);

		// get stable time source; Auto select clock source to be PLL gyroscope reference if ready
		// else use the internal oscillator, bits 2:0 = 001
		writeByte(this->MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
		writeByte(this->MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
		delay(200);

		// Configure Gyro and Thermometer
		// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
		// minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
		// be higher than 1 / 0.0059 = 170 Hz
		// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
		// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
		writeByte(this->MPU9250_ADDRESS, CONFIG, 0x03);

		// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
		writeByte(this->MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
		// determined inset in CONFIG above

		// Set gyroscope full scale range
		// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
		uint8_t c = readByte(this->MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
		// c = c & ~0xE0; // Clear self-test bits [7:5]
		c = c & ~0x02; // Clear Fchoice bits [1:0]
		c = c & ~0x18; // Clear AFS bits [4:3]
		c = c | Gscale << 3; // Set full scale range for the gyro
		// c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
		writeByte(this->MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

		// Set accelerometer full-scale range configuration
		c = readByte(this->MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
		// c = c & ~0xE0; // Clear self-test bits [7:5]
		c = c & ~0x18;  // Clear AFS bits [4:3]
		c = c | Ascale << 3; // Set full scale range for the accelerometer
		writeByte(this->MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

		// Set accelerometer sample rate configuration
		// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
		// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
		c = readByte(this->MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
		c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
		c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
		writeByte(this->MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

		// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
		// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

		// Configure Interrupts and Bypass Enable
		// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
		// clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
		// can join the I2C bus and all can be controlled by the Arduino as master
		writeByte(this->MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
		writeByte(this->MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
		delay(100);
	} else
		return false;

	// And now for the magnetometer
	if(readByte(AK8963_ADDRESS, AK8963_WHO_AM_I) == 0x48) {
		// First extract the factory calibration for each magnetometer axis
		uint8_t rawData[3];  // x/y/z gyro calibration data stored here
		writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
		delay(10);
		writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
		delay(10);
		readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
		this->magCalibration[0] = (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
		this->magCalibration[1] = (float)(rawData[1] - 128)/256. + 1.;
		this->magCalibration[2] = (float)(rawData[2] - 128)/256. + 1.;
		writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
		delay(10);
		// Configure the magnetometer for continuous read and highest resolution
		// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
		// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
		writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
		delay(10);
	} else
		/*Serial.println((readByte(AK8963_ADDRESS, AK8963_WHO_AM_I) == 0x48));
		Serial.println(readByte(AK8963_ADDRESS, AK8963_WHO_AM_I));
		Serial.println((readByte(this->MPU9250_ADDRESS, WHO_AM_I_MPU9250) == 0x71));
		Serial.println(readByte(this->MPU9250_ADDRESS, WHO_AM_I_MPU9250));*/
		return false;

	switch (Mscale) {
		// Possible magnetometer scales (and their register bit settings) are:
		// 14 bit resolution (0) and 16 bit resolution (1)
		case MFS_14BITS:
			this->mRes = 10.*4912./8190.; // Proper scale to return milliGauss
			break;
		case MFS_16BITS:
			this->mRes = 10.*4912./32760.0; // Proper scale to return milliGauss
			break;
	}

	switch (Gscale) {
		// Possible gyro scales (and their register bit settings) are:
		// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
		case GFS_250DPS:
			this->gRes = 250.0/32768.0;
			break;
		case GFS_500DPS:
			this->gRes = 500.0/32768.0;
			break;
		case GFS_1000DPS:
			this->gRes = 1000.0/32768.0;
			break;
		case GFS_2000DPS:
			this->gRes = 2000.0/32768.0;
			break;
	}

	switch (Ascale) {
		// Possible accelerometer scales (and their register bit settings) are:
		// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
		case AFS_2G:
			this->aRes = 2.0/32768.0;
			break;
		case AFS_4G:
			this->aRes = 4.0/32768.0;
			break;
		case AFS_8G:
			this->aRes = 8.0/32768.0;
			break;
		case AFS_16G:
			this->aRes = 16.0/32768.0;
			break;
	}

	return true;
}

/// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250::self_test(float* results) { // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
	uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
	uint8_t selfTest[6];
	int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
	float factoryTrim[6];
	uint8_t FS = 0;

	writeByte(this->MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
	writeByte(this->MPU9250_ADDRESS, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	writeByte(this->MPU9250_ADDRESS, GYRO_CONFIG, FS<<3);  // Set full scale range for the gyro to 250 dps
	writeByte(this->MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	writeByte(this->MPU9250_ADDRESS, ACCEL_CONFIG, FS<<3); // Set full scale range for the accelerometer to 2 g

	for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer

		readBytes(this->MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
		aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

		readBytes(this->MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
		gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
	}

	for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
		aAvg[ii] /= 200;
		gAvg[ii] /= 200;
	}

	// Configure the accelerometer for self-test
	writeByte(this->MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
	writeByte(this->MPU9250_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	delay(25);  // Delay a while to let the device stabilize

	for( int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer
		readBytes(this->MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
		aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

		readBytes(this->MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
		gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
	}

	for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
		aSTAvg[ii] /= 200;
		gSTAvg[ii] /= 200;
	}

	// Configure the gyro and accelerometer for normal operation
	writeByte(this->MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
	writeByte(this->MPU9250_ADDRESS, GYRO_CONFIG,  0x00);
	delay(25);  // Delay a while to let the device stabilize

	// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	selfTest[0] = readByte(this->MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
	selfTest[1] = readByte(this->MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
	selfTest[2] = readByte(this->MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
	selfTest[3] = readByte(this->MPU9250_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
	selfTest[4] = readByte(this->MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
	selfTest[5] = readByte(this->MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

	// Retrieve factory self-test value from self-test code reads
	factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01, ((float)selfTest[0] - 1.0) ));  // FT[Xa] factory trim calculation
	factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01, ((float)selfTest[1] - 1.0) ));  // FT[Ya] factory trim calculation
	factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01, ((float)selfTest[2] - 1.0) ));  // FT[Za] factory trim calculation
	factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01, ((float)selfTest[3] - 1.0) ));  // FT[Xg] factory trim calculation
	factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01, ((float)selfTest[4] - 1.0) ));  // FT[Yg] factory trim calculation
	factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01, ((float)selfTest[5] - 1.0) ));  // FT[Zg] factory trim calculation

	// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	// To get percent, must multiply by 100
	for (int i = 0; i < 3; i++) {
		results[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.;   // Report percent differences
		results[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.; // Report percent differences
	}
}

/// Collects and averages a FIFO buffer full of at-rest accel and gyro readings.
void MPU9250::calibrate_still_bias(float* newAccelBias, float* newGyroBias) {
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

	// reset device
	writeByte(this->MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	delay(100);

	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready
	// else use the internal oscillator, bits 2:0 = 001
	writeByte(this->MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
	writeByte(this->MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
	delay(200);

	// Configure device for bias calculation
	writeByte(this->MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
	writeByte(this->MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
	writeByte(this->MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
	writeByte(this->MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
	writeByte(this->MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	writeByte(this->MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
	delay(15);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	writeByte(this->MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	writeByte(this->MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	writeByte(this->MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	writeByte(this->MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

// Configure FIFO to capture accelerometer and gyro data for bias calculation
	writeByte(this->MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
	writeByte(this->MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
	writeByte(this->MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	readBytes(this->MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		readBytes(this->MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
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

	if(accel_bias[2] > 0L) {
		accel_bias[2] -= (int32_t) accelsensitivity;   // Remove gravity from the z-axis accelerometer bias calculation
	} else {
		accel_bias[2] += (int32_t) accelsensitivity;
	}
	/*
	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
		data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
		data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
		data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
		data[3] = (-gyro_bias[1]/4)       & 0xFF;
		data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
		data[5] = (-gyro_bias[2]/4)       & 0xFF;

	// Push gyro biases to hardware registers

		writeByte(this->MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
		writeByte(this->MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
		writeByte(this->MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
		writeByte(this->MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
		writeByte(this->MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
		writeByte(this->MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
		*/

// Output scaled gyro biases for display in the main program
	newGyroBias[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
	newGyroBias[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
	newGyroBias[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.
	/*
		int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
		readBytes(this->MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
		accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
		readBytes(this->MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
		accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
		readBytes(this->MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
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
		  writeByte(this->MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
		  writeByte(this->MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
		  writeByte(this->MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
		  writeByte(this->MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
		  writeByte(this->MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
		  writeByte(this->MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
		*/
// Output scaled accelerometer biases for display in the main program
	newAccelBias[0] = (float)accel_bias[0]/(float)accelsensitivity;
	newAccelBias[1] = (float)accel_bias[1]/(float)accelsensitivity;
	newAccelBias[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

/*
void MPU9250::calibrate_mag_bias(float* newMagBias, float* newMagScale, uint8_t Mmode) {
	uint16_t ii = 0, sample_count = 0;
	int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
	int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

//	Serial.println("Mag Calibration: Wave device in a figure eight until done!");
	delay(4000);

	// shoot for ~fifteen seconds of mag data
	if(Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
	if(Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
	for(ii = 0; ii < sample_count; ii++) {
		readMagData(mag_temp);  // Read the mag data
		for (int jj = 0; jj < 3; jj++) {
			if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
			if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
		}
		if(Mmode == 0x02) delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
		if(Mmode == 0x06) delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
	}

//    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
//    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
//    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

	// Get hard iron correction
	mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
	mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
	mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

	newMagBias[0] = (float) mag_bias[0]*mRes*magCalibration[0];  // save mag biases in G for main program
	newMagBias[1] = (float) mag_bias[1]*mRes*magCalibration[1];
	newMagBias[2] = (float) mag_bias[2]*mRes*magCalibration[2];

	// Get soft iron correction estimate
	mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
	mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
	mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	avg_rad /= 3.0;

	newMagScale[0] = avg_rad/((float)mag_scale[0]);
	newMagScale[1] = avg_rad/((float)mag_scale[1]);
	newMagScale[2] = avg_rad/((float)mag_scale[2]);

//	Serial.println("Mag Calibration done!");
}
*/

/// Sets the SOFTWARE biases. Does NOT fill the MPU9250 bias registers.
void MPU9250::set_bias(float* newAccelBias, float* newGyroBias, float* newMagBias) {
	this->accelBias[0] = newAccelBias[0];
	this->accelBias[1] = newAccelBias[1];
	this->accelBias[2] = newAccelBias[2];

	this->gyroBias[0] = newGyroBias[0];
	this->gyroBias[1] = newGyroBias[1];
	this->gyroBias[2] = newGyroBias[2];

	this->magBias[0] = newMagBias[0];
	this->magBias[1] = newMagBias[1];
	this->magBias[2] = newMagBias[2];
}

// Wire.h read and write protocols
void MPU9250::writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

uint8_t MPU9250::readByte(uint8_t address, uint8_t subAddress) {
	uint8_t data; // `data` will store the register data
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);                  // Put slave register address in Tx buffer
	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
	Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

void MPU9250::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) {
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	Wire.write(subAddress);            // Put slave register address in Tx buffer
	Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
	Wire.requestFrom(address, count);  // Read bytes from slave register address
	while (Wire.available()) {
		dest[i++] = Wire.read();
	} // Put read results in the Rx buffer
}

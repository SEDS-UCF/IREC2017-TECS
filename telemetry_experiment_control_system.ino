/**
 * incμssμs - Telemetry and Experiment Control System (TECS)
 * Main Flight Program - Revised 04/01/2017
 * © 2017 SEDS-UCF
 *
 * This work is licensed under the terms of the MIT license.
 * For a copy, see <https://opensource.org/licenses/MIT>.
 */

#include <SFE_BMP180.h>
#include <Wire.h>
#include <SD.h>

#include "MPU9250.h"

#define SERIAL_DEBUG /// Enables debugging to the serial console. ENSURE THIS IS COMMENTED OUT BEFORE FLIGHT. Seriously. See my rant in loop() for more info.

const float ACCEL_BIAS_X = 0.036;
const float ACCEL_BIAS_Y = -0.006;
const float ACCEL_BIAS_Z = 0.071;
const float GYRO_BIAS_X = 1.15;
const float GYRO_BIAS_Y = 0.52;
const float GYRO_BIAS_Z = -0.18;

const float LIFTOFF_VERT_G_THRESHOLD = 1.5;
const float EXPERIMENT_VERT_G_THRESHOLD = 0.3;

const uint8_t FILTER_SIZE = 0;

const uint8_t FILE_FLUSH_THRESHOLD = 30;

const uint8_t RELAY_ONE_PIN = 5;
const uint8_t RELAY_TWO_PIN = 6;
const uint8_t RPI_SIGNAL_PIN = 7;
const uint8_t BUZZER_PIN = 9;
const uint8_t CARD_DETECT_PIN = 8;
const uint8_t CHIP_SELECT_PIN = 10;

const float M_TO_FT = 3.28084;

const uint16_t ERR_BEEP_TIMEOUT		= 5000;
const uint16_t WARN_BEEP_TIMEOUT	= 2000;

/// ERRORS:	Conditions in which the program can not continue. Most of them are cases in which a sensor was not found by the Arduino.
/// 		In an error condition, the program halts entirely. Even if allowed to run, it would collect no data and not perform the
///			tasks as required, and would in fact cause internal errors and undefined behavior as the Arduino attempted to pull data
///			from unreadable registers. A restart may fix the error, but if it occurs again, a launch scrub is necessary. Failure to
///			scrub the launch and address the problem will almost certainly result in payload mission failure.
const uint8_t ERR_SD_NO_CARD				= 1;
const uint8_t ERR_SD_INIT_FAIL				= 2;
const uint8_t ERR_BMP180_INIT_FAIL			= 3;
const uint8_t ERR_MAIN_MPU9250_INIT_FAIL	= 4;
const uint8_t ERR_BACK_MPU9250_INIT_FAIL	= 5;
const uint8_t ERR_LAST						= 5;

///	WARNINGS:	Conditions in which mission continuation would be ill-advised, but possible. Depending on the error, there is still
///				a chance of mission success, however the collected data may be flawed or incomplete. Examples include failed sensor
///				reads or a failed self-check at startup. If necessary, the launch may continue, but odds of payload mission success
///				may be greatly diminished. If a computer restart does not resolve the issue, a launch scrub is highly recommended.
const uint8_t WARN_BMP180_TEMP_START_FAIL	= ERR_LAST + 1;
const uint8_t WARN_BMP180_TEMP_GET_FAIL		= ERR_LAST + 2;
const uint8_t WARN_BMP180_PRES_START_FAIL	= ERR_LAST + 3;
const uint8_t WARN_BMP180_PRES_GET_FAIL		= ERR_LAST + 4;

const char str_space = ' ';

struct BaroData {
	float P; // barometer pressure
	float T; // barometer temperature
};

struct FilteredDataset {
	MPU9250Dataset mpu_main;
	MPU9250Dataset mpu_backup;
	BaroData baro;
	float alt; // altitude
};

MPU9250 imu9250_main;
MPU9250 imu9250_backup;
SFE_BMP180 pressure;

double baseline; // baseline pressure
uint32_t last_time;
uint32_t start_time;

uint8_t cycle_count = 0;
uint32_t total_cycles = 0;

File data_file;
char filename[12];

FilteredDataset filter_array[FILTER_SIZE];
uint8_t filter_index = 0;

bool flying = false;

void warning(char warn) {
	switch(warn) {
	case WARN_BMP180_TEMP_START_FAIL:
		#ifdef SERIAL_DEBUG
		Serial.println(F("WARNING: WARN_BMP180_TEMP_START_FAIL"));
		#endif // SERIAL_DEBUG
		break;
	case WARN_BMP180_TEMP_GET_FAIL:
		#ifdef SERIAL_DEBUG
		Serial.println(F("WARNING: WARN_BMP180_TEMP_GET_FAIL"));
		#endif // SERIAL_DEBUG
		break;
	case WARN_BMP180_PRES_START_FAIL:
		#ifdef SERIAL_DEBUG
		Serial.println(F("WARNING: WARN_BMP180_PRES_START_FAIL"));
		#endif // SERIAL_DEBUG
		break;
	case WARN_BMP180_PRES_GET_FAIL:
		#ifdef SERIAL_DEBUG
		Serial.println(F("WARNING: WARN_BMP180_PRES_GET_FAIL"));
		#endif // SERIAL_DEBUG
		break;
	default:
		break;
	}

	if (data_file) {
		data_file.print(F("# ")); data_file.print((float)(millis() - start_time) / 1000.f, 3); data_file.print(F(" WARN: ")); data_file.println(warn); data_file.flush();
	}

	if(!flying) {
		uint32_t loop_start = millis();
		while((millis() - loop_start) < WARN_BEEP_TIMEOUT) {
			digitalWrite(BUZZER_PIN, HIGH);
			delay(50);
			digitalWrite(BUZZER_PIN, LOW);
			delay(25);
		}

		delay(2000);

		for(uint8_t i = 0; i < warn; i++) {
			digitalWrite(BUZZER_PIN, HIGH);
			delay(500);
			digitalWrite(BUZZER_PIN, LOW);
			delay(250);
		}
	}
}

void error(char err) {
	switch(err) {
	case ERR_SD_NO_CARD:
		#ifdef SERIAL_DEBUG
		Serial.println(F("ERROR: ERR_SD_NO_CARD"));
		#endif // SERIAL_DEBUG
		break;
	case ERR_SD_INIT_FAIL:
		#ifdef SERIAL_DEBUG
		Serial.println(F("ERROR: ERR_SD_INIT_FAIL"));
		#endif // SERIAL_DEBUG
		break;
	case ERR_BMP180_INIT_FAIL:
		#ifdef SERIAL_DEBUG
		Serial.println(F("ERROR: ERR_BMP180_INIT_FAIL"));
		#endif // SERIAL_DEBUG
		break;
	case ERR_MAIN_MPU9250_INIT_FAIL:
		#ifdef SERIAL_DEBUG
		Serial.println(F("ERROR: ERR_MAIN_MPU9250_INIT_FAIL"));
		#endif // SERIAL_DEBUG
		break;
	case ERR_BACK_MPU9250_INIT_FAIL:
		#ifdef SERIAL_DEBUG
		Serial.println(F("ERROR: ERR_BACK_MPU9250_INIT_FAIL"));
		#endif // SERIAL_DEBUG
		break;
	default:
		break;
	}

	if (data_file) {
		data_file.print(F("# ")); data_file.print((float)(millis() - start_time) / 1000.f, 3); data_file.print(F(" ERR: ")); data_file.println(err); data_file.flush();
	}

	Serial.println(F("Program can NOT continue! Holding..."));
	while(true) {
		uint32_t loop_start = millis();
		delay(10);
		while((millis() - loop_start) < ERR_BEEP_TIMEOUT) {
			digitalWrite(BUZZER_PIN, HIGH);
			delay(50);
			digitalWrite(BUZZER_PIN, LOW);
			delay(25);
		}

		delay(2000);

		for(uint8_t i = 0; i < err; i++) {
			digitalWrite(BUZZER_PIN, HIGH);
			delay(500);
			digitalWrite(BUZZER_PIN, LOW);
			delay(250);
		}

		delay(1000);
	}
}

void deployExperiment() {
	// pull pins LOW to activate the relays
	digitalWrite(RELAY_ONE_PIN, LOW);
	digitalWrite(RELAY_TWO_PIN, LOW);
}

void liftoff() {
	flying = true;
	digitalWrite(RPI_SIGNAL_PIN, HIGH);
	Serial.println("##### RPi SIGNAL #####");
}

void setup() {
	Wire.begin();
	TWBR = 12; // enable 400 kb/s I2C "fast" mode

	#ifdef SERIAL_DEBUG
	Serial.begin(57600);
	#endif // SERIAL_DEBUG

	// setup relay pins
	pinMode(RELAY_ONE_PIN, OUTPUT);
	pinMode(RELAY_TWO_PIN, OUTPUT);

	// relays are ACTIVE LOW, so drive them HIGH here to keep them off
	digitalWrite(RELAY_ONE_PIN, HIGH);
	digitalWrite(RELAY_TWO_PIN, HIGH);

	pinMode(BUZZER_PIN, OUTPUT);

	pinMode(CHIP_SELECT_PIN, OUTPUT);
	pinMode(CARD_DETECT_PIN, INPUT_PULLUP);

	if(!digitalRead(CARD_DETECT_PIN))
		error(ERR_SD_NO_CARD);
	
	if(!SD.begin(CHIP_SELECT_PIN))
		error(ERR_SD_INIT_FAIL);

	File root = SD.open("/");
	uint16_t num_files = 1;
	while (File test = root.openNextFile()) {
		test.close();
		num_files++;
	}

	sprintf(filename, "log%04d.txt", num_files);
	data_file = SD.open(filename, FILE_WRITE);
	#ifdef SERIAL_DEBUG
	Serial.print(num_files);
	Serial.print(F(" files on card. Filename is \""));
	Serial.print(filename);
	Serial.print("\"");
	#endif // SERIAL_DEBUG

	if(!pressure.begin())
		error(ERR_BMP180_INIT_FAIL);

	baseline = getPressure().P;
	if (data_file) {
		data_file.print("# "); data_file.print((float)(millis() - start_time) / 1000.f, 3); data_file.print(" baseline mb: "); data_file.println(baseline); data_file.flush();
	}

	#ifdef SERIAL_DEBUG
	Serial.print(F("baseline pressure: ")); Serial.print(baseline); Serial.println(F(" mb"));
	#endif // SERIAL_DEBUG

	// Start by performing self test and reporting values
	float selfTest[6];
	imu9250_main.self_test(selfTest);

	if (data_file) {
		data_file.print(F("# ")); data_file.print((float)(millis() - start_time) / 1000.f, 3); data_file.print(F(" self test: "));
		data_file.print(selfTest[0],1); data_file.print(str_space);
		data_file.print(selfTest[1],1); data_file.print(str_space);
		data_file.print(selfTest[2],1); data_file.print(str_space);
		data_file.print(selfTest[3],1); data_file.print(str_space);
		data_file.print(selfTest[4],1); data_file.print(str_space);
		data_file.println(selfTest[5],1);
		data_file.flush();
	}

	#ifdef SERIAL_DEBUG
	Serial.print(F("main x-axis self test: acceleration trim within ")); Serial.print(selfTest[0],1); Serial.println(F("% of factory value"));
	Serial.print(F("main y-axis self test: acceleration trim within ")); Serial.print(selfTest[1],1); Serial.println(F("% of factory value"));
	Serial.print(F("main z-axis self test: acceleration trim within ")); Serial.print(selfTest[2],1); Serial.println(F("% of factory value"));
	Serial.print(F("main x-axis self test: gyration trim within ")); Serial.print(selfTest[3],1); Serial.println(F("% of factory value"));
	Serial.print(F("main y-axis self test: gyration trim within ")); Serial.print(selfTest[4],1); Serial.println(F("% of factory value"));
	Serial.print(F("main z-axis self test: gyration trim within ")); Serial.print(selfTest[5],1); Serial.println(F("% of factory value"));
	#endif // SERIAL_DEBUG

	imu9250_backup.self_test(selfTest);

	#ifdef SERIAL_DEBUG
	Serial.print(F("backup x-axis self test: acceleration trim within ")); Serial.print(selfTest[0],1); Serial.println(F("% of factory value"));
	Serial.print(F("backup y-axis self test: acceleration trim within ")); Serial.print(selfTest[1],1); Serial.println(F("% of factory value"));
	Serial.print(F("backup z-axis self test: acceleration trim within ")); Serial.print(selfTest[2],1); Serial.println(F("% of factory value"));
	Serial.print(F("backup x-axis self test: gyration trim within ")); Serial.print(selfTest[3],1); Serial.println(F("% of factory value"));
	Serial.print(F("backup y-axis self test: gyration trim within ")); Serial.print(selfTest[4],1); Serial.println(F("% of factory value"));
	Serial.print(F("backup z-axis self test: gyration trim within ")); Serial.print(selfTest[5],1); Serial.println(F("% of factory value"));
	#endif // SERIAL_DEBUG

	/// Early results of the testing showed promising results, with the exact same results from the bias_get program at various
	/// temperatures over the course of several hours. I've gone ahead and moved us away from the "calibrate every startup" method.
	/// The biases in place now are for my desk, which is neither perfectly flat nor perfectly normal to Earth's gravity.
	/// We should work to acquire new biases in as best an environment as possible. Doesn't need to be perfect, but it'd be nice to be close.
	float accel_bias[3] = {ACCEL_BIAS_X, ACCEL_BIAS_Y, ACCEL_BIAS_Z};
	float gyro_bias[3] = {GYRO_BIAS_X, GYRO_BIAS_Y, GYRO_BIAS_Z};
//	float accel_bias[3], gyro_bias[3];
//	imu9250.calibrate_still_bias(accel_bias, gyro_bias);

	/// TODO: These are the biases from the example, I've still got to write and run a mag calibration sketch.
	/// Additionally, magnetic calibration is much more finicky than accelerometers or gyros...
	/// It requires rotation about all axes, and is highly dependent on your current environment.
	/// This means that if we want reliable mag info, we'll need to calibrate at the range, before we load the ebay.
	float mag_bias[3] = {470.f, 120.f, 125.f};

	// Acctually setting the biases.
	imu9250_main.set_bias(accel_bias, gyro_bias, mag_bias);
	imu9250_backup.set_bias(accel_bias, gyro_bias, mag_bias);

	if(!imu9250_main.init(AFS_16G, GFS_1000DPS, MFS_16BITS, MMODE_100HZ, false))
		error(ERR_MAIN_MPU9250_INIT_FAIL);

	if(!imu9250_backup.init(AFS_2G, GFS_2000DPS, MFS_16BITS, MMODE_100HZ, true))
		error(ERR_BACK_MPU9250_INIT_FAIL);

	pinMode(RPI_SIGNAL_PIN, INPUT);
	while(!digitalRead(RPI_SIGNAL_PIN)) {}
	delay(3000);
	pinMode(RPI_SIGNAL_PIN, OUTPUT);
	digitalWrite(RPI_SIGNAL_PIN, LOW);

	for(uint8_t i = 0; i < 3; i++) {
		digitalWrite(BUZZER_PIN, HIGH);
		delay(100);
		digitalWrite(BUZZER_PIN, LOW);
		delay(50);
		digitalWrite(BUZZER_PIN, HIGH);
		delay(100);
		digitalWrite(BUZZER_PIN, LOW);
		delay(50);
		digitalWrite(BUZZER_PIN, HIGH);
		delay(500);
		digitalWrite(BUZZER_PIN, LOW);
		delay(250);
	}

	start_time = millis();
	last_time = start_time;
}

// BMP180 starttemp is ~5ms and startpressure(0) is ~5ms, we can lose accuracy but inscrease sample speed by using MPU9250 chip internal temp.
// Using MPU internal temp gives us ~25 ms delay or 40 samples/s, using BMP180 temp give us ~30 ms delay, or 33 samples/s
// Adding in SD card writing and flushing gives us a ~41ms cycle time with BMP180, about 24 Hz. IMO not fast enough. Working on it...
/// Ignore the above measurements, they were conducted with full data reporting. I forgot that serial printing is slow as hell.
/// Total measured cycle time using BMP180 temperature with all but cycle time serial printing disabled: ~29ms, about 34 Hz. Good enough.
void loop() {
	uint32_t now = millis();

	bool main_ready = false;
	bool backup_ready = false;

	while(!main_ready || !backup_ready) {
		if(imu9250_main.ready())
			main_ready = true;
		if(imu9250_backup.ready())
			backup_ready = true;
	}
	
	total_cycles++;
	
	MPU9250Dataset data_main;// = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
	imu9250_main.update(data_main); // cycle time impact: ~10ms (includes wait for MPU ready)

	MPU9250Dataset data_backup;// = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
	imu9250_backup.update(data_backup); // cycle time impact: ~10ms (includes wait for MPU ready)

	// cycle time impact: ~13ms
	BaroData bd = getPressure();
	double a = pressure.altitude(bd.P, baseline);
/*
	filter_index++;
	if(filter_index >= FILTER_SIZE) filter_index = 0;
	filter_array[filter_index] = FilteredDataset{data_main, data_backup, bd, a};

	// cycle time impact: < ~5ms
	FilteredDataset avg_data = {{0.f,0.f,0.f,0.f,0.f,0.f,0.f}, {0.f,0.f,0.f,0.f,0.f,0.f,0.f}, {0.f,0.f}, 0.f}; // zeroing out the struct members
	for(uint8_t i = 0; i < FILTER_SIZE; i++) {
		avg_data.mpu_main.Ax	+= filter_array[i].mpu_main.Ax;
		avg_data.mpu_main.Ay	+= filter_array[i].mpu_main.Ay;
		avg_data.mpu_main.Az	+= filter_array[i].mpu_main.Az;
		avg_data.mpu_main.Gx	+= filter_array[i].mpu_main.Gx;
		avg_data.mpu_main.Gy	+= filter_array[i].mpu_main.Gy;
		avg_data.mpu_main.Gz	+= filter_array[i].mpu_main.Gz;
		avg_data.mpu_main.T		+= filter_array[i].mpu_main.T;
		avg_data.mpu_backup.Ax	+= filter_array[i].mpu_backup.Ax;
		avg_data.mpu_backup.Ay	+= filter_array[i].mpu_backup.Ay;
		avg_data.mpu_backup.Az	+= filter_array[i].mpu_backup.Az;
		avg_data.mpu_backup.Gx	+= filter_array[i].mpu_backup.Gx;
		avg_data.mpu_backup.Gy	+= filter_array[i].mpu_backup.Gy;
		avg_data.mpu_backup.Gz	+= filter_array[i].mpu_backup.Gz;
		avg_data.mpu_backup.T	+= filter_array[i].mpu_backup.T;
		avg_data.baro.P			+= filter_array[i].baro.P;
		avg_data.baro.T			+= filter_array[i].baro.T;
		avg_data.alt			+= filter_array[i].alt;
	}

	avg_data.mpu_main.Ax	/= (float)FILTER_SIZE;
	avg_data.mpu_main.Ay	/= (float)FILTER_SIZE;
	avg_data.mpu_main.Az	/= (float)FILTER_SIZE;
	avg_data.mpu_main.Gx	/= (float)FILTER_SIZE;
	avg_data.mpu_main.Gy	/= (float)FILTER_SIZE;
	avg_data.mpu_main.Gz	/= (float)FILTER_SIZE;
	avg_data.mpu_main.T		/= (float)FILTER_SIZE;
	avg_data.mpu_backup.Ax	/= (float)FILTER_SIZE;
	avg_data.mpu_backup.Ay	/= (float)FILTER_SIZE;
	avg_data.mpu_backup.Az	/= (float)FILTER_SIZE;
	avg_data.mpu_backup.Gx	/= (float)FILTER_SIZE;
	avg_data.mpu_backup.Gy	/= (float)FILTER_SIZE;
	avg_data.mpu_backup.Gz	/= (float)FILTER_SIZE;
	avg_data.mpu_backup.T	/= (float)FILTER_SIZE;
	avg_data.baro.P			/= (float)FILTER_SIZE;
	avg_data.baro.T			/= (float)FILTER_SIZE;
	avg_data.alt			/= (float)FILTER_SIZE;
	*/

	#ifdef SERIAL_DEBUG /// TODO: THIS BLOCK TAKES FOREVER. COMPARATIVELY, AT LEAST. MAKE SURE SERIAL_DEBUG IS DISABLED FOR FLIGHT.
	//Serial.print((now - start_time) / 1000.f, 2); Serial.print(F(" s   "));
	Serial.print(F("\n---  main  ---> "));
	if(data_main.Ax >= 0) Serial.print(str_space); Serial.print(data_main.Ax, 2); Serial.print(F(" gX   "));
	if(data_main.Ay >= 0) Serial.print(str_space); Serial.print(data_main.Ay, 2); Serial.print(F(" gY   "));
	if(data_main.Az >= 0) Serial.print(str_space); Serial.print(data_main.Az, 2); Serial.print(F(" gZ   "));
	if(data_main.Gx >= 0) Serial.print(str_space); Serial.print(data_main.Gx, 1); Serial.print(F(" d/sX   "));
	if(data_main.Gy >= 0) Serial.print(str_space); Serial.print(data_main.Gy, 1); Serial.print(F(" d/sY   "));
	if(data_main.Gz >= 0) Serial.print(str_space); Serial.print(data_main.Gz, 1); Serial.print(F(" d/sZ   "));
	if(data_main.T >= 0) Serial.print(str_space); Serial.print(data_main.T, 1); Serial.print(F(" C   "));
	if(a >= 0) Serial.print(str_space); Serial.print(a, 1); Serial.print(F(" ft   "));
	Serial.print(now - last_time); Serial.println(" ms");

	if(imu9250_backup.ready()) {
	Serial.print(F("--- backup ---> "));
	if(data_backup.Ax >= 0) Serial.print(str_space); Serial.print(data_backup.Ax, 2); Serial.print(F(" gX   "));
	if(data_backup.Ay >= 0) Serial.print(str_space); Serial.print(data_backup.Ay, 2); Serial.print(F(" gY   "));
	if(data_backup.Az >= 0) Serial.print(str_space); Serial.print(data_backup.Az, 2); Serial.print(F(" gZ   "));
	if(data_backup.Gx >= 0) Serial.print(str_space); Serial.print(data_backup.Gx, 1); Serial.print(F(" d/sX   "));
	if(data_backup.Gy >= 0) Serial.print(str_space); Serial.print(data_backup.Gy, 1); Serial.print(F(" d/sY   "));
	if(data_backup.Gz >= 0) Serial.print(str_space); Serial.print(data_backup.Gz, 1); Serial.print(F(" d/sZ   "));
	if(data_backup.T >= 0) Serial.print(str_space); Serial.print(data_backup.T, 1); Serial.println(F(" C   "));
	}
/*	
	Serial.print(F("--- avg ---> "));
	if(avg_data.mpu_main.Ax >= 0) Serial.print(str_space); Serial.print(avg_data.mpu_main.Ax, 2); Serial.print(F(" gX   "));
	if(avg_data.mpu_main.Ay >= 0) Serial.print(str_space); Serial.print(avg_data.mpu_main.Ay, 2); Serial.print(F(" gY   "));
	if(avg_data.mpu_main.Az >= 0) Serial.print(str_space); Serial.print(avg_data.mpu_main.Az, 2); Serial.print(F(" gZ   "));
	if(avg_data.mpu_main.Gx >= 0) Serial.print(str_space); Serial.print(avg_data.mpu_main.Gx, 1); Serial.print(F(" d/sX   "));
	if(avg_data.mpu_main.Gy >= 0) Serial.print(str_space); Serial.print(avg_data.mpu_main.Gy, 1); Serial.print(F(" d/sY   "));
	if(avg_data.mpu_main.Gz >= 0) Serial.print(str_space); Serial.print(avg_data.mpu_main.Gz, 1); Serial.print(F(" d/sZ   "));
	if(avg_data.mpu_main.T >= 0) Serial.print(str_space); Serial.print(avg_data.mpu_main.T, 1); Serial.print(F(" C   "));
	if(avg_data.mpu_backup.Ax >= 0) Serial.print(str_space); Serial.print(avg_data.mpu_backup.Ax, 2); Serial.print(F(" gX   "));
	if(avg_data.mpu_backup.Ay >= 0) Serial.print(str_space); Serial.print(avg_data.mpu_backup.Ay, 2); Serial.print(F(" gY   "));
	if(avg_data.mpu_backup.Az >= 0) Serial.print(str_space); Serial.print(avg_data.mpu_backup.Az, 2); Serial.print(F(" gZ   "));
	if(avg_data.mpu_backup.Gx >= 0) Serial.print(str_space); Serial.print(avg_data.mpu_backup.Gx, 1); Serial.print(F(" d/sX   "));
	if(avg_data.mpu_backup.Gy >= 0) Serial.print(str_space); Serial.print(avg_data.mpu_backup.Gy, 1); Serial.print(F(" d/sY   "));
	if(avg_data.mpu_backup.Gz >= 0) Serial.print(str_space); Serial.print(avg_data.mpu_backup.Gz, 1); Serial.print(F(" d/sZ   "));
	if(avg_data.mpu_backup.T >= 0) Serial.print(str_space); Serial.print(avg_data.mpu_backup.T, 1); Serial.print(F(" C   "));
	if(avg_data.alt >= 0) Serial.print(str_space); Serial.print(avg_data.alt, 1); Serial.println(F(" ft   "));*/
	#endif /// SERIOUSLY. This block adds ~30ms to our cycle time, costing us a whole 10 Hz. Turn it off for flight.

	// TODO: this is the code that deploys the experiment. Currently, it's based only off the real time acceleration of the
	// vertical axis: Z-axis on ground, perhaps X- or Y-axis when mounted in the rocket. There is also a check to ensure at
	// least one second has elapsed since startup. The real time acceleration values could be changed to the average values,
	// or some other method of filtering to protect against a single unexpected reading within the deployment range setting
	// off the experiment. We should also include sanity checks of altitude and time elapsed... We know that 0g won't occur
	// at 500 ft. off the ground, so we should be checking that we are above some safe minimums.
	if(now - start_time > 1000) {
		if(data_main.Az > EXPERIMENT_VERT_G_THRESHOLD) {
			deployExperiment();
		}

		if(data_main.Az > LIFTOFF_VERT_G_THRESHOLD) {
			liftoff();
		}
	}

	// cycle time impact: ~9ms
	if (data_file) {
		data_file.print(total_cycles);
		data_file.print(str_space);
		data_file.print((float)(now - start_time) / 1000.f, 3);
		data_file.print(str_space);
		data_file.print(data_main.Ax, 2);
		data_file.print(str_space);
		data_file.print(data_main.Ay, 2);
		data_file.print(str_space);
		data_file.print(data_main.Az, 2);
		data_file.print(str_space);
		data_file.print(data_main.Gx, 1);
		data_file.print(str_space);
		data_file.print(data_main.Gy, 1);
		data_file.print(str_space);
		data_file.print(data_main.Gz, 1);
		data_file.print(str_space);
		data_file.print(data_main.T, 1);
		data_file.print(str_space);
		data_file.print(data_backup.Ax, 2);
		data_file.print(str_space);
		data_file.print(data_backup.Ay, 2);
		data_file.print(str_space);
		data_file.print(data_backup.Az, 2);
		data_file.print(str_space);
		data_file.print(data_backup.Gx, 1);
		data_file.print(str_space);
		data_file.print(data_backup.Gy, 1);
		data_file.print(str_space);
		data_file.print(data_backup.Gz, 1);
		data_file.print(str_space);
		data_file.print(data_backup.T, 1);
		data_file.print(str_space);
		data_file.print(bd.P, 1);
		data_file.print(str_space);
		data_file.print(bd.T, 1);
		data_file.print(str_space);
		data_file.print(a, 1);
		data_file.print(str_space);
/*		data_file.print(avg_data.mpu_main.Ax, 2);
		data_file.print(str_space);
		data_file.print(avg_data.mpu_main.Ay, 2);
		data_file.print(str_space);
		data_file.print(avg_data.mpu_main.Az, 2);
		data_file.print(str_space);
		data_file.print(avg_data.mpu_main.Gx, 1);
		data_file.print(str_space);
		data_file.print(avg_data.mpu_main.Gy, 1);
		data_file.print(str_space);
		data_file.print(avg_data.mpu_main.Gz, 1);
		data_file.print(str_space);
		data_file.print(avg_data.mpu_main.T, 1);
		data_file.print(str_space);
		data_file.print(avg_data.mpu_backup.Ax, 2);
		data_file.print(str_space);
		data_file.print(avg_data.mpu_backup.Ay, 2);
		data_file.print(str_space);
		data_file.print(avg_data.mpu_backup.Az, 2);
		data_file.print(str_space);
		data_file.print(avg_data.mpu_backup.Gx, 1);
		data_file.print(str_space);
		data_file.print(avg_data.mpu_backup.Gy, 1);
		data_file.print(str_space);
		data_file.print(avg_data.mpu_backup.Gz, 1);
		data_file.print(str_space);
		data_file.print(avg_data.mpu_backup.T, 1);
		data_file.print(str_space);
		data_file.print(avg_data.baro.P, 1);
		data_file.print(str_space);
		data_file.print(avg_data.baro.T, 1);
		data_file.print(str_space);
		data_file.print(avg_data.alt, 1);
		data_file.print(str_space);*/
		data_file.println(now - last_time);
	}

	last_time = now;
	cycle_count++;

	if(cycle_count > FILE_FLUSH_THRESHOLD) {
		data_file.flush();
		cycle_count = 0;
	}
}

BaroData getPressure() {
	char status;
	double T,P;

	status = pressure.startTemperature();
	if (!status)
		warning(WARN_BMP180_TEMP_START_FAIL);

	delay(status);

	status = pressure.getTemperature(T);
	if (!status)
		warning(WARN_BMP180_TEMP_GET_FAIL);

	// Start a pressure measurement:
	// The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
	// If request is successful, the number of ms to wait is returned.
	// If request is unsuccessful, 0 is returned.

	status = pressure.startPressure(0);
	if (!status)
		warning(WARN_BMP180_PRES_START_FAIL);

	// Wait for the measurement to complete:
	delay(status);

	// Retrieve the completed pressure measurement:
	// Note that the measurement is stored in the variable P.
	// Use '&P' to provide the address of P.
	// Note also that the function requires the previous temperature measurement (T).
	// (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
	// Function returns 1 if successful, 0 if failure.

	status = pressure.getPressure(P,T);
	if (status)
		return(BaroData{P, T});
	else
		warning(WARN_BMP180_PRES_GET_FAIL);
}

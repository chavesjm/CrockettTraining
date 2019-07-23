/* MPU9250_MS5637_t3 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you
 find it useful you can buy me a beer some time.

 Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor,
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and
 Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.

 This sketch is intended specifically for the MPU9250+MS5637 Add-on shield for the Teensy 3.1.
 It uses SDA/SCL on pins 17/16, respectively, and it uses the Teensy 3.1-specific Wire library i2c_t3.h.
 The MS5637 is a simple but high resolution pressure sensor, which can be used in its high resolution
 mode but with power consumption of 20 microAmp, or in a lower resolution mode with power consumption of
 only 1 microAmp. The choice will depend on the application.

 SDA and SCL should have external pull-up resistors (to 3.3V).
 4K7 resistors are on the MPU9250+MS5637 breakout board.

 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND

 Note: The MPU9250 is an I2C sensor and uses the Arduino Wire library.
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 */
#include "Arduino.h"
#include "Wire.h"
#include <SoftwareSerial.h>

//#include <i2c_t3.h>
//#include <SPI.h>

// See also MPU-9250 Register Map and Descriptions, Revision 4.0, RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in
// above document; the MPU9250 and MPU9150 are virtually identical but the latter has a different register map
//
//Magnetometer Registers
#define CT_AK8963_ADDRESS   0x0C
#define CT_AK8963_WHO_AM_I  0x00 // should return 0x48
#define CT_AK8963_INFO      0x01
#define CT_AK8963_ST1       0x02  // data ready status bit 0
#define CT_AK8963_XOUT_L	 0x03  // data
#define CT_AK8963_XOUT_H	 0x04
#define CT_AK8963_YOUT_L	 0x05
#define CT_AK8963_YOUT_H	 0x06
#define CT_AK8963_ZOUT_L	 0x07
#define CT_AK8963_ZOUT_H	 0x08
#define CT_AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define CT_AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define CT_AK8963_ASTC      0x0C  // Self test control
#define CT_AK8963_I2CDIS    0x0F  // I2C disable
#define CT_AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define CT_AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define CT_AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define CT_SELF_TEST_X_GYRO 0x00
#define CT_SELF_TEST_Y_GYRO 0x01
#define CT_SELF_TEST_Z_GYRO 0x02

#define CT_SELF_TEST_X_ACCEL 0x0D
#define CT_SELF_TEST_Y_ACCEL 0x0E
#define CT_SELF_TEST_Z_ACCEL 0x0F

#define CT_SELF_TEST_A      0x10

#define CT_XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define CT_XG_OFFSET_L      0x14
#define CT_YG_OFFSET_H      0x15
#define CT_YG_OFFSET_L      0x16
#define CT_ZG_OFFSET_H      0x17
#define CT_ZG_OFFSET_L      0x18
#define CT_SMPLRT_DIV       0x19
#define CT_CONFIG           0x1A
#define CT_GYRO_CONFIG      0x1B
#define CT_ACCEL_CONFIG     0x1C
#define CT_ACCEL_CONFIG2    0x1D
#define CT_LP_ACCEL_ODR     0x1E
#define CT_WOM_THR          0x1F

#define CT_MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define CT_ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define CT_ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define CT_FIFO_EN          0x23
#define CT_I2C_MST_CTRL     0x24
#define CT_I2C_SLV0_ADDR    0x25
#define CT_I2C_SLV0_REG     0x26
#define CT_I2C_SLV0_CTRL    0x27
#define CT_I2C_SLV1_ADDR    0x28
#define CT_I2C_SLV1_REG     0x29
#define CT_I2C_SLV1_CTRL    0x2A
#define CT_I2C_SLV2_ADDR    0x2B
#define CT_I2C_SLV2_REG     0x2C
#define CT_I2C_SLV2_CTRL    0x2D
#define CT_I2C_SLV3_ADDR    0x2E
#define CT_I2C_SLV3_REG     0x2F
#define CT_I2C_SLV3_CTRL    0x30
#define CT_I2C_SLV4_ADDR    0x31
#define CT_I2C_SLV4_REG     0x32
#define CT_I2C_SLV4_DO      0x33
#define CT_I2C_SLV4_CTRL    0x34
#define CT_I2C_SLV4_DI      0x35
#define CT_I2C_MST_STATUS   0x36
#define CT_INT_PIN_CFG      0x37
#define CT_INT_ENABLE       0x38
#define CT_DMP_INT_STATUS   0x39  // Check DMP interrupt
#define CT_INT_STATUS       0x3A
#define CT_ACCEL_XOUT_H     0x3B
#define CT_ACCEL_XOUT_L     0x3C
#define CT_ACCEL_YOUT_H     0x3D
#define CT_ACCEL_YOUT_L     0x3E
#define CT_ACCEL_ZOUT_H     0x3F
#define CT_ACCEL_ZOUT_L     0x40
#define CT_TEMP_OUT_H       0x41
#define CT_TEMP_OUT_L       0x42
#define CT_GYRO_XOUT_H      0x43
#define CT_GYRO_XOUT_L      0x44
#define CT_GYRO_YOUT_H      0x45
#define CT_GYRO_YOUT_L      0x46
#define CT_GYRO_ZOUT_H      0x47
#define CT_GYRO_ZOUT_L      0x48
#define CT_EXT_SENS_DATA_00 0x49
#define CT_EXT_SENS_DATA_01 0x4A
#define CT_EXT_SENS_DATA_02 0x4B
#define CT_EXT_SENS_DATA_03 0x4C
#define CT_EXT_SENS_DATA_04 0x4D
#define CT_EXT_SENS_DATA_05 0x4E
#define CT_EXT_SENS_DATA_06 0x4F
#define CT_EXT_SENS_DATA_07 0x50
#define CT_EXT_SENS_DATA_08 0x51
#define CT_EXT_SENS_DATA_09 0x52
#define CT_EXT_SENS_DATA_10 0x53
#define CT_EXT_SENS_DATA_11 0x54
#define CT_EXT_SENS_DATA_12 0x55
#define CT_EXT_SENS_DATA_13 0x56
#define CT_EXT_SENS_DATA_14 0x57
#define CT_EXT_SENS_DATA_15 0x58
#define CT_EXT_SENS_DATA_16 0x59
#define CT_EXT_SENS_DATA_17 0x5A
#define CT_EXT_SENS_DATA_18 0x5B
#define CT_EXT_SENS_DATA_19 0x5C
#define CT_EXT_SENS_DATA_20 0x5D
#define CT_EXT_SENS_DATA_21 0x5E
#define CT_EXT_SENS_DATA_22 0x5F
#define CT_EXT_SENS_DATA_23 0x60
#define CT_MOT_DETECT_STATUS 0x61
#define CT_I2C_SLV0_DO      0x63
#define CT_I2C_SLV1_DO      0x64
#define CT_I2C_SLV2_DO      0x65
#define CT_I2C_SLV3_DO      0x66
#define CT_I2C_MST_DELAY_CTRL 0x67
#define CT_SIGNAL_PATH_RESET  0x68
#define CT_MOT_DETECT_CTRL  0x69
#define CT_USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define CT_PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define CT_PWR_MGMT_2       0x6C
#define CT_DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define CT_DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define CT_DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define CT_DMP_REG_1        0x70
#define CT_DMP_REG_2        0x71
#define CT_FIFO_COUNTH      0x72
#define CT_FIFO_COUNTL      0x73
#define CT_FIFO_R_W         0x74
#define CT_WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define CT_XA_OFFSET_H      0x77
#define CT_XA_OFFSET_L      0x78
#define CT_YA_OFFSET_H      0x7A
#define CT_YA_OFFSET_L      0x7B
#define CT_ZA_OFFSET_H      0x7D
#define CT_ZA_OFFSET_L      0x7E

#define CT_MPU9250_ADDRESS 0x68  // Device address when ADO = 0

#define CT_SerialDebug true  // set to true to get Serial output for debugging

// Set initial input parameters
enum CT_Ascale {
	CT_AFS_2G = 0,
	CT_AFS_4G,
	CT_AFS_8G,
	CT_AFS_16G
};

enum CT_Gscale {
	CT_GFS_250DPS = 0,
	CT_GFS_500DPS,
	CT_GFS_1000DPS,
	CT_GFS_2000DPS
};

enum CT_Mscale {
	CT_MFS_14BITS = 0, // 0.6 mG per LSB
	CT_MFS_16BITS      // 0.15 mG per LSB
};

// Specify sensor full scale
//uint8_t OSR = ADC_8192;     // set pressure amd temperature oversample rate
uint8_t CT_Gscale = CT_GFS_250DPS;
uint8_t CT_Ascale = CT_AFS_2G;
uint8_t CT_Mscale = CT_MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t CT_Mmode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float CT_aRes, CT_gRes, CT_mRes;      // scale resolutions per LSB for the sensors

// Pin definitions
const int CT_potencioMeter = A0;
const int8_t CT_pushButton = 11;
const int8_t CT_ledLeftTurn = 13;
const int8_t CT_ledRightTurn = 12;
const int8_t CT_ledRightDoubleTurn = 9;
const int8_t CT_ledLeftDoubleTurn = 8;
const int8_t CT_laser = 3;
const int8_t CT_speaker = 10;

int16_t CT_MPU9250Data[7]; // used to read all 14 bytes at once from the MPU9250 accel/gyro
int16_t CT_accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t CT_gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t CT_magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float CT_magCalibration[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float CT_gyroBias[3] = {0, 0, 0}, CT_accelBias[3] = {0, 0, 0}, CT_magBias[3] = {0, 0, 0}, CT_magScale[3]  = {0, 0, 0};      // Bias corrections for gyro and accelerometer
int16_t CT_tempCount;            // temperature raw count output
float   CT_temperature;          // Stores the MPU9250 gyro internal chip temperature in degrees Celsius
float CT_SelfTest[6];            // holds results of gyro and accelerometer self test
float CT_value=4.0;
int CT_difficulty = 0;
float CT_rangeOK = 0.0;
bool CT_newMagData, CT_newValueBeta = false;

const byte CT_numChars = 32;
char CT_receivedChars[CT_numChars];

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float CT_GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float CT_GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float CT_beta = sqrt(3.0f / 4.0f) * CT_GyroMeasError;   // compute beta
float CT_zeta = sqrt(3.0f / 4.0f) * CT_GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define CT_Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define CT_Ki 0.0f

uint32_t CT_angleUpdate = 0, CT_count = 0;  // used to control display output rate
float CT_pitch, CT_yaw, CT_roll;
float CT_pitch_initial, CT_yaw_initial, CT_roll_initial;
float CT_pitch_previous, CT_yaw_previous, CT_roll_previous;
float CT_a11, CT_a12, CT_a21, CT_a22, CT_a31, CT_a32, CT_a33;            // rotation matrix coefficients for Euler angles and gravity components
float CT_deltat = 0.0f;           // integration interval for both filter schemes
uint32_t CT_lastUpdate = 0; // used to calculate integration interval
uint32_t CT_now = 0;                         // used to calculate integration interval
uint32_t CT_lastTime=0;

float CT_ax, CT_ay, CT_az, CT_gx, CT_gy, CT_gz, CT_mx, CT_my, CT_mz; // variables to hold latest sensor data values
float CT_lin_ax, CT_lin_ay, CT_lin_az;             // linear acceleration (acceleration with gravity component subtracted)
float CT_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float CT_eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method
//#define ArduinoUNO true
#ifdef ArduinoUNO
//Primera calibracion
//int CT_maxX = 305, CT_minX = -212, CT_maxY = 930, CT_minY = 423, CT_maxZ = 1207, CT_minZ = 651;
//segunda calibracion
int CT_maxX = 374, CT_minX = -119, CT_maxY = 1027, CT_minY = 467, CT_maxZ = 1070, CT_minZ = 573;
#else
int CT_maxX = 186, CT_minX = -286, CT_maxY = 928, CT_minY = 404, CT_maxZ = 280, CT_minZ = -239;
#endif

String CT_readString;

//Push Button
bool CT_pushButtonPushed = false;
bool CT_started = false;

//#define BluethootCom true

#ifdef BluethootCom
SoftwareSerial BluethootSerial(6, 5); // RX | TX
#endif


void setup()
{
	Wire.begin();
	Wire.setClock(400000);

	delay(4000);

#ifdef BluethootCom
	BluethootSerial.begin(38400); //Velocidad a la que esta configurado el modulo Bluetooth
#else
	Serial.begin(115200);
#endif

	// Set up the interrupt pin, its set as active high, push-pull
	pinMode(CT_ledRightTurn, OUTPUT);
	pinMode(CT_ledLeftTurn, OUTPUT);
	pinMode(CT_ledRightDoubleTurn, OUTPUT);
	pinMode(CT_ledLeftDoubleTurn, OUTPUT);
	pinMode(CT_speaker, OUTPUT);
	pinMode(CT_laser, OUTPUT);

	digitalWrite(CT_ledRightTurn, HIGH);
	digitalWrite(CT_ledLeftTurn, HIGH);
	digitalWrite(CT_ledRightDoubleTurn, HIGH);
	digitalWrite(CT_ledLeftDoubleTurn, HIGH);
	digitalWrite(CT_speaker,LOW);
	digitalWrite(CT_laser,LOW);

	delay(1000);

	byte c = readByte(CT_MPU9250_ADDRESS, CT_WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250

	delay(1000);

	if (c == 0x71 || c == 0x73) // WHO_AM_I should always be 0x68
	{
		MPU9250SelfTest(CT_SelfTest); // Start by performing self test and reporting values

		delay(1000);

		// get sensor resolutions, only need to do this once
		getAres();
		getGres();
		getMres();

		accelgyrocalMPU9250(CT_gyroBias, CT_accelBias); // Calibrate gyro and accelerometers, load biases in bias registers

		delay(1000);

		initMPU9250();

		// Read the WHO_AM_I register of the magnetometer, this is a good test of communication
		readByte(CT_AK8963_ADDRESS, CT_AK8963_WHO_AM_I);  // Read WHO_AM_I register for AK8963


		delay(1000);

		// Get magnetometer calibration from AK8963 ROM
		initAK8963(CT_magCalibration);

		magcalMPU9250Done(CT_magBias, CT_magScale);

		delay(1000);

	}
	else
	{
#ifdef BluethootCom
		BluethootSerial.print("Could not connect to MPU9250: 0x");
		BluethootSerial.println(c, HEX);
#else
		Serial.print("Could not connect to MPU9250: 0x");
		Serial.println(c, HEX);
#endif
		while(1) ; // Loop forever if communication doesn't happen
	}

	digitalWrite(CT_ledRightTurn, LOW);
	digitalWrite(CT_ledLeftTurn, LOW);
	digitalWrite(CT_ledRightDoubleTurn, LOW);
	digitalWrite(CT_ledLeftDoubleTurn, LOW);
}


void loop()
{

	if(CT_pushButtonPushed && !digitalRead(CT_pushButton))
	{
		CT_pushButtonPushed = false;
		CT_yaw_initial = CT_yaw;
		CT_pitch_initial = CT_pitch;
		CT_roll_initial = CT_roll;
		digitalWrite(CT_laser,LOW);
		CT_started = true;
	}
	else if(digitalRead(CT_pushButton))
	{
		CT_pushButtonPushed = true;
		digitalWrite(CT_laser,HIGH);
		CT_started = false;
	}
	else
	{
		CT_pushButtonPushed = false;
		digitalWrite(CT_laser,LOW);
	}

	if(readByte(CT_MPU9250_ADDRESS, CT_INT_STATUS) & 0x01)
	{
		readMPU9250Data(CT_MPU9250Data); // INT cleared on any read

		// Now we'll calculate the accleration value into actual g's
		CT_ax = (float)CT_MPU9250Data[0]*CT_aRes - CT_accelBias[0];  // get actual g value, this depends on scale being set
		CT_ay = (float)CT_MPU9250Data[1]*CT_aRes - CT_accelBias[1];
		CT_az = (float)CT_MPU9250Data[2]*CT_aRes - CT_accelBias[2];

		// Calculate the gyro value into actual degrees per second
		CT_gx = (float)CT_MPU9250Data[4]*CT_gRes;  // get actual gyro value, this depends on scale being set
		CT_gy = (float)CT_MPU9250Data[5]*CT_gRes;
		CT_gz = (float)CT_MPU9250Data[6]*CT_gRes;

		readMagData(CT_magCount);  // Read the x/y/z adc values

		// Calculate the magnetometer values in milliGauss
		// Include factory calibration per data sheet and user environmental corrections
		if(CT_newMagData == true)
		{
			CT_newMagData = false; // reset newMagData flag

			CT_mx = (float)CT_magCount[0]*CT_mRes*CT_magCalibration[0] - CT_magBias[0];  // get actual magnetometer value, this depends on scale being set
			CT_my = (float)CT_magCount[1]*CT_mRes*CT_magCalibration[1] - CT_magBias[1];
			CT_mz = (float)CT_magCount[2]*CT_mRes*CT_magCalibration[2] - CT_magBias[2];

			CT_mx *= CT_magScale[0];
			CT_my *= CT_magScale[1];
			CT_mz *= CT_magScale[2];

		}
	}

	CT_now = micros();
	CT_deltat = ((CT_now - CT_lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
	CT_lastUpdate = CT_now;

	// Sensors x (y)-axis of the accelerometer/gyro is aligned with the y (x)-axis of the magnetometer;
	// the magnetometer z-axis (+ down) is misaligned with z-axis (+ up) of accelerometer and gyro!
	// We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
	// For the MPU9250+MS5637 Mini breakout the +x accel/gyro is North, then -y accel/gyro is East. So if we want te quaternions properly aligned
	// we need to feed into the Madgwick function Ax, -Ay, -Az, Gx, -Gy, -Gz, My, -Mx, and Mz. But because gravity is by convention
	// positive down, we need to invert the accel data, so we pass -Ax, Ay, Az, Gx, -Gy, -Gz, My, -Mx, and Mz into the Madgwick
	// function to get North along the accel +x-axis, East along the accel -y-axis, and Down along the accel -z-axis.
	// This orientation choice can be modified to allow any convenient (non-NED) orientation convention.
	// Pass gyro rate as rad/s
	MadgwickQuaternionUpdate(-CT_ax, CT_ay, CT_az, CT_gx*PI/180.0f, -CT_gy*PI/180.0f, -CT_gz*PI/180.0f,  CT_my,  -CT_mx, CT_mz);
	//MadgwickQuaternionUpdate(-ax, ay, az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f,  my,  -mx, mz);
	//MahonyQuaternionUpdate(-ax, ay, az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f,  my,  -mx, mz);

	CT_angleUpdate = millis() - CT_count;

	if (CT_angleUpdate > 50)
	{
		//Metodo Modificado

//		// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
//		// In this coordinate system, the positive z-axis is down toward Earth.
//		// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
//		// Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
//		// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
//		// These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
//		// Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
//		// applied in the correct order which for this configuration is yaw, pitch, and then roll.
//		// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
//		//Software AHRS:
//		//   yaw   = atan2f(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
//		//   pitch = -asinf(2.0f * (q[1] * q[3] - q[0] * q[2]));
//		//   roll  = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
//		//   pitch *= 180.0f / PI;
//		//   yaw   *= 180.0f / PI;
//		//   yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
//		//   if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
//		//   roll  *= 180.0f / PI;
//		a11 =   2.0f * q[0] * q[0] -1 + 2.0f	* q[1] * q[1];
//		a21 =   2.0f * (q[1] * q[2] - q[0] * q[3]);
//		a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
//		a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
//		//a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
//		a31 =   2.0f * (q[1] * q[3] + q[0] * q[2]);
//		//a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
//		a32 =   2.0f * (q[2] * q[3] - q[0] * q[1]);
//		//a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
//		a33 =   2.0f * q[0] * q[0] -1 +2.0f * q[3] * q[3];
//		//pitch = -asinf(a32);
//		//roll  = atan2f(a31, a33);
//		//yaw   = atan2f(a12, a22);
//		//pitch *= 180.0f / PI;
//		//yaw   *= 180.0f / PI;
//		//yaw   += 1.41f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
//		//if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
//		//roll  *= 180.0f / PI;
//
//		phi = atan2(a32, a33);
//		theta = -atan(a31/sqrtf(1-a31*a31));
//		psi = atan2(a21,a11);
//
//		phi = phi * 180./ PI;
//		theta = theta * 180./ PI;
//		psi = psi * 180./ PI;

		//////Metodo original
		CT_a12 =   2.0f * (CT_q[1] * CT_q[2] + CT_q[0] * CT_q[3]);
		CT_a22 =   CT_q[0] * CT_q[0] + CT_q[1] * CT_q[1] - CT_q[2] * CT_q[2] - CT_q[3] * CT_q[3];
		CT_a31 =   2.0f * (CT_q[0] * CT_q[1] + CT_q[2] * CT_q[3]);
		CT_a32 =   2.0f * (CT_q[1] * CT_q[3] - CT_q[0] * CT_q[2]);
		CT_a33 =   CT_q[0] * CT_q[0] - CT_q[1] * CT_q[1] - CT_q[2] * CT_q[2] + CT_q[3] * CT_q[3];

		CT_pitch_previous = CT_pitch;
		CT_roll_previous = CT_roll;
		CT_yaw_previous = CT_yaw;

		CT_lastTime = (micros() - CT_lastTime);

		CT_pitch = -asinf(CT_a32);
		CT_roll  = atan2f(CT_a31, CT_a33);
		CT_yaw   = atan2f(CT_a12, CT_a22);

		CT_yaw   *= 180.0f / PI;
		//yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
		CT_yaw   -= 1.39f;
		if(CT_yaw < 0) CT_yaw   += 360.0f; // Ensure yaw stays between 0 and 360
		CT_roll  *= 180.0f / PI;


		CT_lin_ax = CT_ax + CT_a31;
		CT_lin_ay = CT_ay + CT_a32;
		CT_lin_az = CT_az - CT_a33;

		CT_difficulty = analogRead(CT_potencioMeter);
		CT_rangeOK = map(CT_difficulty,0,1023,0,300) / 100.0;


		if(CT_started && ((CT_yaw - CT_yaw_initial) > CT_rangeOK))
		{
			digitalWrite(CT_ledLeftTurn,LOW);
			digitalWrite(CT_ledRightTurn,HIGH);
			tone(CT_speaker, 1200);
		}
		else if(CT_started && ((CT_yaw- CT_yaw_initial) < (CT_rangeOK * -1.0)))
		{
			digitalWrite(CT_ledLeftTurn,HIGH);
			digitalWrite(CT_ledRightTurn,LOW);
			tone(CT_speaker, 750);
		}
		else
		{
			digitalWrite(CT_ledLeftTurn,LOW);
			digitalWrite(CT_ledRightTurn,LOW);
			digitalWrite(CT_speaker,LOW);
			noTone(CT_speaker);
		}

		if(CT_SerialDebug)
		{

#ifdef BluethootCom
			BluethootSerial.print(1/CT_deltat,2);
			BluethootSerial.print(",");
			BluethootSerial.print(CT_value,2);
			BluethootSerial.print(",");
			BluethootSerial.print(CT_yaw, 2);
			BluethootSerial.print(",");
			BluethootSerial.print(CT_pitch, 2);
			BluethootSerial.print(",");
			BluethootSerial.print(CT_roll, 2);
			BluethootSerial.print(",");
			BluethootSerial.print(CT_yaw_initial, 2);
			BluethootSerial.print(",");
			BluethootSerial.print(CT_pitch_initial, 2);
			BluethootSerial.print(",");
			BluethootSerial.print(CT_roll_initial, 2);
			BluethootSerial.print(",");
			BluethootSerial.print(CT_yaw-CT_yaw_initial, 2);
			BluethootSerial.print(",");
			BluethootSerial.print(CT_pitch-CT_pitch_initial, 2);
			BluethootSerial.print(",");
			BluethootSerial.print(CT_roll-CT_roll_initial, 2);
			BluethootSerial.print(",");
			BluethootSerial.print(CT_rangeOK, 2);
			BluethootSerial.print(",");
			BluethootSerial.print((CT_yaw-CT_yaw_previous)/(CT_lastTime/1000000.0f),4);
			BluethootSerial.print(",");
			BluethootSerial.print((CT_pitch-CT_pitch_previous)/(CT_lastTime/1000000.0f),4);
			BluethootSerial.print(",");
			BluethootSerial.print((CT_roll-CT_roll_previous)/(CT_lastTime/1000000.0f),4);
			BluethootSerial.print(",");
			BluethootSerial.print(CT_ax,2);
			BluethootSerial.print(",");
			BluethootSerial.print(CT_ay,2);
			BluethootSerial.print(",");
			BluethootSerial.print(CT_az,2);
			BluethootSerial.print(",");
			BluethootSerial.print(CT_gx,2);
			BluethootSerial.print(",");
			BluethootSerial.print(CT_gy,2);
			BluethootSerial.print(",");
			BluethootSerial.print(CT_gz,2);
			BluethootSerial.print(",");
			BluethootSerial.print(CT_mx,2);
			BluethootSerial.print(",");
			BluethootSerial.print(CT_my,2);
			BluethootSerial.print(",");
			BluethootSerial.print(CT_mz,2);
			BluethootSerial.print(",");
			BluethootSerial.print(CT_q[0],10);
			BluethootSerial.print(",");
			BluethootSerial.print(CT_q[1],10);
			BluethootSerial.print(",");
			BluethootSerial.print(CT_q[2],10);
			BluethootSerial.print(",");
			BluethootSerial.println(CT_q[3],10);
#else
			Serial.print(1/CT_deltat,2);
			Serial.print(",");
			Serial.print(CT_value,2);
			Serial.print(",");
			Serial.print(CT_yaw, 2);
			Serial.print(",");
			Serial.print(CT_pitch, 2);
			Serial.print(",");
			Serial.print(CT_roll, 2);
			Serial.print(",");
			Serial.print(CT_yaw_initial, 2);
			Serial.print(",");
			Serial.print(CT_pitch_initial, 2);
			Serial.print(",");
			Serial.print(CT_roll_initial, 2);
			Serial.print(",");
			Serial.print(CT_yaw-CT_yaw_initial, 2);
			Serial.print(",");
			Serial.print(CT_pitch-CT_pitch_initial, 2);
			Serial.print(",");
			Serial.print(CT_roll-CT_roll_initial, 2);
			Serial.print(",");
			Serial.print(CT_rangeOK, 2);
			Serial.print(",");
			Serial.print((CT_yaw-CT_yaw_previous)/(CT_lastTime/1000000.0f),4);
			Serial.print(",");
			Serial.print((CT_pitch-CT_pitch_previous)/(CT_lastTime/1000000.0f),4);
			Serial.print(",");
			Serial.print((CT_roll-CT_roll_previous)/(CT_lastTime/1000000.0f),4);
			Serial.print(",");
			Serial.print(CT_ax,2);
			Serial.print(",");
			Serial.print(CT_ay,2);
			Serial.print(",");
			Serial.print(CT_az,2);
			Serial.print(",");
			Serial.print(CT_gx,2);
			Serial.print(",");
			Serial.print(CT_gy,2);
			Serial.print(",");
			Serial.print(CT_gz,2);
			Serial.print(",");
			Serial.print(CT_mx,2);
			Serial.print(",");
			Serial.print(CT_my,2);
			Serial.print(",");
			Serial.print(CT_mz,2);
			Serial.print(",");
			Serial.print(CT_q[0],2);
			Serial.print(",");
			Serial.print(CT_q[1],2);
			Serial.print(",");
			Serial.print(CT_q[2],2);
			Serial.print(",");
			Serial.println(CT_q[3],2);
#endif
		}

		// With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and
		// >200 Hz using the Mahony scheme even though the display refreshes at only 2 Hz.
		// The filter update rate is determined mostly by the mathematical steps in the respective algorithms,
		// the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
		// an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
		// filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively.
		// This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
		// This filter update rate should be fast enough to maintain accurate platform orientation for
		// stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
		// produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
		// The 3.3 V 8 MHz Pro Mini is doing pretty well!

		CT_count = millis();
	}
}

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

void getMres() {
  switch (CT_Mscale)
  {
 	// Possible magnetometer scales (and their register bit settings) are:
	// 14 bit resolution (0) and 16 bit resolution (1)
    case CT_MFS_14BITS:
          CT_mRes = 10.*4912./8190.; // Proper scale to return milliGauss
          break;
    case CT_MFS_16BITS:
          CT_mRes = 10.*4912./32760.0; // Proper scale to return milliGauss
          break;
  }
}

void getGres() {
  switch (CT_Gscale)
  {
 	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case CT_GFS_250DPS:
          CT_gRes = 250.0/32768.0;
          break;
    case CT_GFS_500DPS:
          CT_gRes = 500.0/32768.0;
          break;
    case CT_GFS_1000DPS:
          CT_gRes = 1000.0/32768.0;
          break;
    case CT_GFS_2000DPS:
          CT_gRes = 2000.0/32768.0;
          break;
  }
}

void getAres() {
  switch (CT_Ascale)
  {
 	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case CT_AFS_2G:
          CT_aRes = 2.0/32768.0;
          break;
    case CT_AFS_4G:
          CT_aRes = 4.0/32768.0;
          break;
    case CT_AFS_8G:
          CT_aRes = 8.0/32768.0;
          break;
    case CT_AFS_16G:
          CT_aRes = 16.0/32768.0;
          break;
  }
}

void readMPU9250Data(int16_t * destination)
{
  uint8_t rawData[14];  // x/y/z accel register data stored here
  readBytes(CT_MPU9250_ADDRESS, CT_ACCEL_XOUT_H, 14, &rawData[0]);  // Read the 14 raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
  destination[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;
  destination[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;
  destination[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;
  destination[6] = ((int16_t)rawData[12] << 8) | rawData[13] ;
}

void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(CT_MPU9250_ADDRESS, CT_ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}


void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(CT_MPU9250_ADDRESS, CT_GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

void readMagData(int16_t * destination)
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  CT_newMagData = (readByte(CT_AK8963_ADDRESS, CT_AK8963_ST1) & 0x01);
  if(CT_newMagData == true) { // wait for magnetometer data ready bit to be set
  readBytes(CT_AK8963_ADDRESS, CT_AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
  uint8_t c = rawData[6]; // End data read by reading ST2 register
    if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
    destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
    destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
   }
  }
}

int16_t readTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(CT_MPU9250_ADDRESS, CT_TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
  return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}

void initAK8963(float * destination)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  writeByte(CT_AK8963_ADDRESS, CT_AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  writeByte(CT_AK8963_ADDRESS, CT_AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);
  readBytes(CT_AK8963_ADDRESS, CT_AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128)/256. + 1.;
  destination[2] =  (float)(rawData[2] - 128)/256. + 1.;
  writeByte(CT_AK8963_ADDRESS, CT_AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(CT_AK8963_ADDRESS, CT_AK8963_CNTL, CT_Mscale << 4 | CT_Mmode); // Set magnetometer data resolution and sample ODR
  delay(10);
}


void initMPU9250()
{
 // wake up device
  writeByte(CT_MPU9250_ADDRESS, CT_PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  delay(100); // Wait for all registers to reset

 // get stable time source
  writeByte(CT_MPU9250_ADDRESS, CT_PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200);

 // Configure Gyro and Thermometer
 // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
 // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 // be higher than 1 / 0.0059 = 170 Hz
 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
 // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(CT_MPU9250_ADDRESS, CT_CONFIG, 0x03);

 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(CT_MPU9250_ADDRESS, CT_SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
                                    // determined inset in CONFIG above

 // Set gyroscope full scale range
 // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(CT_MPU9250_ADDRESS, CT_GYRO_CONFIG); // get current GYRO_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x03; // Clear Fchoice bits [1:0]
  c = c & ~0x18; // Clear GFS bits [4:3]
  c = c | CT_Gscale << 3; // Set full scale range for the gyro
 // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  writeByte(CT_MPU9250_ADDRESS, CT_GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

 // Set accelerometer full-scale range configuration
  c = readByte(CT_MPU9250_ADDRESS, CT_ACCEL_CONFIG); // get current ACCEL_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | CT_Ascale << 3; // Set full scale range for the accelerometer
  writeByte(CT_MPU9250_ADDRESS, CT_ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(CT_MPU9250_ADDRESS, CT_ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeByte(CT_MPU9250_ADDRESS, CT_ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
//   writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
   writeByte(CT_MPU9250_ADDRESS, CT_INT_PIN_CFG, 0x12);  // INT is 50 microsecond pulse and any read to clear
   writeByte(CT_MPU9250_ADDRESS, CT_INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
   delay(100);
}


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void accelgyrocalMPU9250(float * dest1, float * dest2)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

 // reset device
  writeByte(CT_MPU9250_ADDRESS, CT_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);

 // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
 // else use the internal oscillator, bits 2:0 = 001
  writeByte(CT_MPU9250_ADDRESS, CT_PWR_MGMT_1, 0x01);
  writeByte(CT_MPU9250_ADDRESS, CT_PWR_MGMT_2, 0x00);
  delay(200);

// Configure device for bias calculation
  writeByte(CT_MPU9250_ADDRESS, CT_INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(CT_MPU9250_ADDRESS, CT_FIFO_EN, 0x00);      // Disable FIFO
  writeByte(CT_MPU9250_ADDRESS, CT_PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(CT_MPU9250_ADDRESS, CT_I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(CT_MPU9250_ADDRESS, CT_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(CT_MPU9250_ADDRESS, CT_USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);

// Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(CT_MPU9250_ADDRESS, CT_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(CT_MPU9250_ADDRESS, CT_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(CT_MPU9250_ADDRESS, CT_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(CT_MPU9250_ADDRESS, CT_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

// Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(CT_MPU9250_ADDRESS, CT_USER_CTRL, 0x40);   // Enable FIFO
  writeByte(CT_MPU9250_ADDRESS, CT_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
  writeByte(CT_MPU9250_ADDRESS, CT_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(CT_MPU9250_ADDRESS, CT_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(CT_MPU9250_ADDRESS, CT_FIFO_R_W, 12, &data[0]); // read data for averaging
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
  writeByte(CT_MPU9250_ADDRESS, CT_XG_OFFSET_H, data[0]);
  writeByte(CT_MPU9250_ADDRESS, CT_XG_OFFSET_L, data[1]);
  writeByte(CT_MPU9250_ADDRESS, CT_YG_OFFSET_H, data[2]);
  writeByte(CT_MPU9250_ADDRESS, CT_YG_OFFSET_L, data[3]);
  writeByte(CT_MPU9250_ADDRESS, CT_ZG_OFFSET_H, data[4]);
  writeByte(CT_MPU9250_ADDRESS, CT_ZG_OFFSET_L, data[5]);

// Output scaled gyro biases for display in the main program
  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(CT_MPU9250_ADDRESS, CT_XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(CT_MPU9250_ADDRESS, CT_YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(CT_MPU9250_ADDRESS, CT_ZA_OFFSET_H, 2, &data[0]);
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
/*  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
*/
// Output scaled accelerometer biases for display in the main program
   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity;
   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

void magcalMPU9250(float * dest1, float * dest2)
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  delay(4000);

    // shoot for ~fifteen seconds of mag data
    if(CT_Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
    if(CT_Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
   for(ii = 0; ii < sample_count; ii++) {
    readMagData(mag_temp);  // Read the mag data
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    if(CT_Mmode == 0x02) delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
    if(CT_Mmode == 0x06) delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
    }

    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

    dest1[0] = (float) mag_bias[0]*CT_mRes*CT_magCalibration[0];  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*CT_mRes*CT_magCalibration[1];
    dest1[2] = (float) mag_bias[2]*CT_mRes*CT_magCalibration[2];

    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);

   Serial.println("Mag Calibration done!");
}


void magcalMPU9250Done(float * dest1, float * dest2)
{
//  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767};// mag_temp[3] = {0, 0, 0};

  //Serial.println("Mag Calibration: Wave device in a figure eight until done!");
//  delay(4000);

//    // shoot for ~fifteen seconds of mag data
//    if(Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
//    if(Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
//   for(ii = 0; ii < sample_count; ii++) {
//    readMagData(mag_temp);  // Read the mag data
//    for (int jj = 0; jj < 3; jj++) {
//      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
//      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
//      Serial.println(mag_temp[jj],10);
//    }
//    if(Mmode == 0x02) delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
//    if(Mmode == 0x06) delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
//    }

   mag_max[0] = CT_maxX;
   mag_max[1] = CT_maxY;
   mag_max[2] = CT_maxZ;

   mag_min[0] = CT_minX;
   mag_min[1] = CT_minY;
   mag_min[2] = CT_minZ;

//    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
//    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
//    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

    dest1[0] = (float) mag_bias[0]*CT_mRes*CT_magCalibration[0];  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*CT_mRes*CT_magCalibration[1];
    dest1[2] = (float) mag_bias[2]*CT_mRes*CT_magCalibration[2];

    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);

   //Serial.println("Mag Calibration done!");
}



// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
   uint8_t selfTest[6];
   int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
   float factoryTrim[6];
   uint8_t FS = 0;

  writeByte(CT_MPU9250_ADDRESS, CT_SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  writeByte(CT_MPU9250_ADDRESS, CT_CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(CT_MPU9250_ADDRESS, CT_GYRO_CONFIG, FS<<3);  // Set full scale range for the gyro to 250 dps
  writeByte(CT_MPU9250_ADDRESS, CT_ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(CT_MPU9250_ADDRESS, CT_ACCEL_CONFIG, FS<<3); // Set full scale range for the accelerometer to 2 g

  for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer

  readBytes(CT_MPU9250_ADDRESS, CT_ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
  aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    readBytes(CT_MPU9250_ADDRESS, CT_GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
  gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
  aAvg[ii] /= 200;
  gAvg[ii] /= 200;
  }

// Configure the accelerometer for self-test
   writeByte(CT_MPU9250_ADDRESS, CT_ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
   writeByte(CT_MPU9250_ADDRESS, CT_GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   delay(25);  // Delay a while to let the device stabilize

  for( int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer

  readBytes(CT_MPU9250_ADDRESS, CT_ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    readBytes(CT_MPU9250_ADDRESS, CT_GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
  aSTAvg[ii] /= 200;
  gSTAvg[ii] /= 200;
  }

 // Configure the gyro and accelerometer for normal operation
   writeByte(CT_MPU9250_ADDRESS, CT_ACCEL_CONFIG, 0x00);
   writeByte(CT_MPU9250_ADDRESS, CT_GYRO_CONFIG,  0x00);
   delay(25);  // Delay a while to let the device stabilize

   // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
   selfTest[0] = readByte(CT_MPU9250_ADDRESS, CT_SELF_TEST_X_ACCEL); // X-axis accel self-test results
   selfTest[1] = readByte(CT_MPU9250_ADDRESS, CT_SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
   selfTest[2] = readByte(CT_MPU9250_ADDRESS, CT_SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
   selfTest[3] = readByte(CT_MPU9250_ADDRESS, CT_SELF_TEST_X_GYRO);  // X-axis gyro self-test results
   selfTest[4] = readByte(CT_MPU9250_ADDRESS, CT_SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
   selfTest[5] = readByte(CT_MPU9250_ADDRESS, CT_SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

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
     destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.;   // Report percent differences
     destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.; // Report percent differences
   }

}

// I2C scan function

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
      Serial.print("Unknow error at address 0x");
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


// I2C read/write functions for the MPU9250 and AK8963 sensors

        void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

        uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	Wire.endTransmission(false);        // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.requestFrom(address, 1);  // Read one byte from slave register address
	Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

        void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	Wire.write(subAddress);            // Put slave register address in Tx buffer
	Wire.endTransmission(false);  // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
//        Wire.requestFrom(address, count);  // Read bytes from slave register address
        Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address
	while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}

void recvWithStartEndMarkers() {
	static boolean recvInProgress = false;
	static byte ndx = 0;
	char startMarker = '<';
	char endMarker = '>';
	char rc;

#ifdef BluethootCom
	while (BluethootSerial.available() > 0 && CT_newValueBeta == false) {
		rc = BluethootSerial.read();
#else
	while (Serial.available() > 0 && CT_newValueBeta == false) {
			rc = Serial.read();
#endif

		if (recvInProgress == true) {
			if (rc != endMarker) {
				CT_receivedChars[ndx] = rc;
				ndx++;
				if (ndx >= CT_numChars) {
					ndx = CT_numChars - 1;
				}
			}
			else {
				CT_receivedChars[ndx] = '\0'; // terminate the string
				recvInProgress = false;
				ndx = 0;
				CT_newValueBeta = true;
			}
		}

		else if (rc == startMarker) {
			recvInProgress = true;
		}
	}
}

/************************************************************
MPU9250_DMP_Quaternion
 Quaternion example for MPU-9250 DMP Arduino Library
Jim Lindblom @ SparkFun Electronics
original creation date: November 23, 2016
https://github.com/sparkfun/SparkFun_MPU9250_DMP_Arduino_Library

The MPU-9250's digital motion processor (DMP) can calculate
four unit quaternions, which can be used to represent the
rotation of an object.

This exmaple demonstrates how to configure the DMP to
calculate quaternions, and prints them out to the serial
monitor. It also calculates pitch, roll, and yaw from those
values.

Development environment specifics:
Arduino IDE 1.6.12
SparkFun 9DoF Razor IMU M0

Supported Platforms:
- ATSAMD21 (Arduino Zero, SparkFun SAMD21 Breakouts)
 *************************************************************/
#include "Arduino.h"
#include "SparkFunMPU9250-DMP.h"
#include "BluetoothSerial.h"


enum TrainerStatus{
	IDLE,
	SELECTING,
	SELECTED,
	PLAYING
};

MPU9250_DMP imu;

//#define BluetoothConf
#ifdef BluetoothConf
BluetoothSerial SerialOutput;
#else
#define SerialOutput Serial
#endif

//#define DIFFICULTY_LEVEL 3

unsigned long m_last_update = 0;
float m_yaw = 0;
float m_pitch = 0;
float m_roll = 0;
float m_initial_yaw = 0;
float m_initial_pitch = 0;
float m_initial_roll = 0;
float m_lower_error_yaw = 0;
float m_upper_error_yaw = 0;
uint16_t m_potenciometer_raw_value = 0;
float m_difficult_level = 0;

TrainerStatus m_status = IDLE;

/***************************
BUZZER: GPIO16.
LASER: GPIO23
POTENCIOMETRO: GPIO35
BATERIA: GPIO34.
I2C SDA: GPIO21.
I2C SCL: GPIO22.
LED LEFT: GPIO18.
LED RIGHT:GPIO32.
PULSADOR: GPIO17.
*******************************/

#define MAX_ERROR 3

#define BUTTON_PIN 17
#define BUZZER_PIN 16
#define LASER_PIN 23
#define POTENCIOMETER_PIN 35
#define BATTERY_PIN 34
#define RIGHT_LED_PIN 32
#define LEFT_LED_PIN 18

void setup()
{
	esp_log_level_set("*", ESP_LOG_NONE);

	int freq = getCpuFrequencyMhz();

	SerialOutput.begin(115200);
	SerialOutput.println(freq,DEC);

#ifdef BluetoothConf
	SerialOutput.begin("CrocketTrainer");
#endif


	m_status = IDLE;

	pinMode(BUTTON_PIN, INPUT);
	pinMode(RIGHT_LED_PIN,OUTPUT);
	pinMode(LEFT_LED_PIN,OUTPUT);
	pinMode(BUZZER_PIN, OUTPUT);

	// Call imu.begin() to verify communication and initialize
	if (imu.begin() != INV_SUCCESS)
	{
		while (1)
		{
			SerialOutput.println("Unable to communicate with MPU-9250");
			SerialOutput.println("Check connections, and try again.");
			SerialOutput.println();
			delay(5000);
		}
	}else{
		SerialOutput.println("IMU initialized!!");
	}

	/*imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);   // Enable all sensors
    imu.setGyroFSR(2000);           // Set Gyro dps, options are +/- 250, 500, 1000, or 2000 dps
    imu.setAccelFSR(16);            // Set Accel g, options are +/- 2, 4, 8, or 16 g
    imu.setSampleRate(100);        // Set sample rate of Accel/Gyro, range between 4Hz-1kHz
    imu.setLPF(5);                  // Set LPF corner frequency of Accel/Gyro, acceptable cvalues are 188, 98, 42, 20, 10, 5 (Hz)
    imu.setCompassSampleRate(100);  // Set Mag rate, range between: 1-100Hz
    int res = imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT |   // 6-axis (accel/gyro) quaternion calculation
                 DMP_FEATURE_GYRO_CAL   ,   // Gyroscope calibration (0's out after 8 seconds of no motion)
                 200);*/

	int res = imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
			DMP_FEATURE_GYRO_CAL, // Use gyro calibration
			50); // Set DMP FIFO rate to 10 Hz
	// DMP_FEATURE_LP_QUAT can also be used. It uses the
	// accelerometer in low-power mode to estimate quat's.
	// DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive

	if(!res){
		SerialOutput.println("DMP initialized!!");
	}
	else{
		SerialOutput.print("DMP not initialized with code: ");
		SerialOutput.println(res,HEX);
	}

	res = imu.resetFifo();

	if(!res){
		SerialOutput.println("FIFO reseted!!");
	}
	else{
		SerialOutput.print("FIFO not reseted with code: ");
		SerialOutput.println(res,HEX);
	}

	tone(BUZZER_PIN,1000,3000);
	digitalWrite(LEFT_LED_PIN,HIGH);
	digitalWrite(RIGHT_LED_PIN,HIGH);
	delay(3000);
	digitalWrite(LEFT_LED_PIN,LOW);
	digitalWrite(RIGHT_LED_PIN,LOW);
}

void loop()
{
	if(!digitalRead(BUTTON_PIN)){
		m_status = SELECTING;
		m_initial_pitch = 0;
		m_initial_yaw = 0;
		m_initial_roll = 0;
		m_difficult_level = 0;
		m_potenciometer_raw_value = 0;

		noTone(BUZZER_PIN);
		digitalWrite(LEFT_LED_PIN,LOW);
		digitalWrite(RIGHT_LED_PIN,LOW);

	}else{
		if(m_status == SELECTING){
			m_status = SELECTED;
		}
	}

	// Check for new data in the FIFO
	if ( imu.fifoAvailable() )
	{
		// Use dmpUpdateFifo to update the ax, gx, mx, etc. values
		if ( imu.dmpUpdateFifo() == INV_SUCCESS)
		{
			// computeEulerAngles can be used -- after updating the
			// quaternion values -- to estimate roll, pitch, and yaw
			imu.computeEulerAngles();
			printIMUData();
		}
	}
}

void printIMUData(void)
{
	// After calling dmpUpdateFifo() the ax, gx, mx, etc. values
	// are all updated.
	// Quaternion values are, by default, stored in Q30 long
	// format. calcQuat turns them into a float between -1 and 1
	float q0 = imu.calcQuat(imu.qw);
	float q1 = imu.calcQuat(imu.qx);
	float q2 = imu.calcQuat(imu.qy);
	float q3 = imu.calcQuat(imu.qz);

	m_yaw = imu.yaw;
	m_pitch = imu.pitch;
	m_roll = imu.roll;

	if(m_status == SELECTED && m_initial_pitch == 0 && m_initial_roll == 0 && m_initial_yaw == 0){
		m_status = PLAYING;
		m_initial_pitch = m_pitch;
		m_initial_roll = m_roll;
		m_initial_yaw = m_yaw;
	}

	if(m_status == PLAYING){
		uint16_t value = analogRead(POTENCIOMETER_PIN);
		if(value != m_potenciometer_raw_value){
			m_potenciometer_raw_value = value;

			m_difficult_level = (m_potenciometer_raw_value * MAX_ERROR) / 4098.0;
			m_upper_error_yaw = (m_yaw + m_difficult_level) > 360 ? ((m_yaw + m_difficult_level) - 360):(m_yaw + m_difficult_level);
			m_lower_error_yaw = (m_yaw - m_difficult_level) < 0 ? (360 - (m_yaw - m_difficult_level)):(m_yaw - m_difficult_level);

		}
	}

	if(m_status == PLAYING){
		if(m_yaw > m_upper_error_yaw){
			tone(BUZZER_PIN,2000,0);
			digitalWrite(LEFT_LED_PIN,HIGH);
			digitalWrite(RIGHT_LED_PIN,LOW);
		}else if(m_yaw < m_lower_error_yaw){
			tone(BUZZER_PIN,1000,0);
			digitalWrite(LEFT_LED_PIN,LOW);
			digitalWrite(RIGHT_LED_PIN,HIGH);
		}else{
			noTone(BUZZER_PIN);
			digitalWrite(RIGHT_LED_PIN,LOW);
			digitalWrite(LEFT_LED_PIN,LOW);
		}
	}

	unsigned long current_update = millis();
	float freq = (1.0/(current_update - m_last_update))*1000;
	m_last_update = current_update;

	SerialOutput.print(String(current_update));
	SerialOutput.print(';');
	SerialOutput.print(String(freq));
	SerialOutput.print(';');
	SerialOutput.print(String(0));
	SerialOutput.print(';');
	SerialOutput.print(String(imu.yaw));
	SerialOutput.print(';');
	SerialOutput.print(String(imu.pitch));
	SerialOutput.print(';');
	SerialOutput.print(String(imu.roll));
	SerialOutput.print(';');
	SerialOutput.print(String(m_initial_yaw));
	SerialOutput.print(';');
	SerialOutput.print(String(m_initial_pitch));
	SerialOutput.print(';');
	SerialOutput.print(String(m_initial_roll));
	SerialOutput.print(';');
	SerialOutput.print(String(m_yaw - m_initial_yaw));
	SerialOutput.print(';');
	SerialOutput.print(String(m_pitch - m_initial_pitch));
	SerialOutput.print(';');
	SerialOutput.print(String(m_roll - m_initial_roll));
	SerialOutput.print(';');
	SerialOutput.println(String(m_difficult_level));

}

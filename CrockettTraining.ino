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
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "math.h"


enum TrainerStatus{
	IDLE,
	SELECTING,
	SELECTED,
	PLAYING
};

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
sensors_event_t m_orientationData;

#define BluetoothConf
#ifdef BluetoothConf
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialOutput;

#else

#define SerialOutput Serial
#endif

//#define DIFFICULTY_LEVEL 3

unsigned long m_last_update = 0;
float m_yaw = 0;
float m_pitch = 0;
float m_roll = 0;
float m_yaw2 = 0;
float m_pitch2 = 0;
float m_roll2 = 0;
float m_initial_yaw = 0;
float m_initial_pitch = 0;
float m_initial_roll = 0;
float m_yaw_error = 0;
float m_pitch_error = 0;
float m_roll_error = 0;
uint16_t m_potenciometer_raw_value = 0;
float m_difficult_level = 0;

uint16_t m_printer_counter = 0;

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

uint16_t BNO055_SAMPLERATE_DELAY_MS = 20; //how often to read data from the board

void setup()
{
	esp_log_level_set("*", ESP_LOG_NONE);

	int freq = getCpuFrequencyMhz();

	SerialOutput.begin(115200);
	SerialOutput.println(freq,DEC);
	Serial.begin(115200);

#ifdef BluetoothConf
	SerialOutput.begin("CrocketTrainer");
#endif


	m_status = IDLE;

	pinMode(BUTTON_PIN, INPUT);
	pinMode(RIGHT_LED_PIN,OUTPUT);
	pinMode(LEFT_LED_PIN,OUTPUT);
	pinMode(BUZZER_PIN, OUTPUT);

	// Call imu.begin() to verify communication and initialize
	if (!bno.begin())
	{
		while (1)
		{
			SerialOutput.println("Unable to communicate with BNO055");
			SerialOutput.println("Check connections, and try again.");
			SerialOutput.println();
			delay(5000);
		}
	}else{
		SerialOutput.println("IMU initialized!!");
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
	int incomingByte = 0;
	if (SerialOutput.available() > 0) {
		// read the incoming byte:
		incomingByte = Serial.read();
		Serial.println("Read = ");
		Serial.println(incomingByte);
		Serial.println(char(incomingByte));


		if(char(incomingByte) == 'R' || incomingByte == -1){
			m_status = SELECTING;
			m_initial_pitch = 0;
			m_initial_yaw = 0;
			m_initial_roll = 0;
			m_difficult_level = 0;
			m_potenciometer_raw_value = 0;
			m_yaw_error = 0;
			m_pitch_error = 0;
			m_roll_error = 0;

			noTone(BUZZER_PIN);
			digitalWrite(LEFT_LED_PIN,LOW);
			digitalWrite(RIGHT_LED_PIN,LOW);

		}
	}

	if(!digitalRead(BUTTON_PIN)){
		m_status = SELECTING;
		m_initial_pitch = 0;
		m_initial_yaw = 0;
		m_initial_roll = 0;
		m_difficult_level = 0;
		m_potenciometer_raw_value = 0;
		m_yaw_error = 0;
		m_pitch_error = 0;
		m_roll_error = 0;

		noTone(BUZZER_PIN);
		digitalWrite(LEFT_LED_PIN,LOW);
		digitalWrite(RIGHT_LED_PIN,LOW);

	}else{
		if(m_status == SELECTING){
			m_status = SELECTED;
		}
	}

	unsigned long tStart = micros();

	while ((micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000))
	{
		//poll until the next sample is ready
	}

	// Get Data
	if ( bno.getEvent(&m_orientationData, Adafruit_BNO055::VECTOR_EULER))
	{
		calculateIMUPosition();

		m_printer_counter++;

		if (m_printer_counter == 4){
			printIMUData();
			m_printer_counter = 0;
		}


	}
}

void calculateIMUPosition(void)
{
	m_yaw = m_orientationData.orientation.x;
	m_pitch = m_orientationData.orientation.y;
	m_roll = m_orientationData.orientation.z;

	if(m_status == SELECTED && m_initial_pitch == 0 && m_initial_roll == 0 && m_initial_yaw == 0){
		m_status = PLAYING;
		tone(BUZZER_PIN, 1000, 200);
		m_initial_pitch = m_pitch;
		m_initial_roll = m_roll;
		m_initial_yaw = m_yaw;
		m_yaw_error = 0;
		m_pitch_error = 0;
		m_roll_error = 0;
	}

	//m_potenciometer_raw_value = analogRead(POTENCIOMETER_PIN);
	//m_difficult_level = (m_potenciometer_raw_value * MAX_ERROR) / 4098.0;
	m_difficult_level = 3.0;

	if(m_status == PLAYING){

		m_yaw_error = getAngleError(m_yaw, m_initial_yaw);

		if(m_yaw_error > m_difficult_level){
			tone(BUZZER_PIN,2000,0);
			digitalWrite(LEFT_LED_PIN,HIGH);
			digitalWrite(RIGHT_LED_PIN,LOW);
		}else if(m_yaw_error < -m_difficult_level){
			tone(BUZZER_PIN,1000,0);
			digitalWrite(LEFT_LED_PIN,LOW);
			digitalWrite(RIGHT_LED_PIN,HIGH);
		}else{
			noTone(BUZZER_PIN);
			digitalWrite(RIGHT_LED_PIN,LOW);
			digitalWrite(LEFT_LED_PIN,LOW);
		}
	}
}

void printIMUData(void)
{
	unsigned long current_update = millis();
	float freq = (1.0/(current_update - m_last_update))*1000;
	m_last_update = current_update;

	SerialOutput.print(String(current_update));
	SerialOutput.print(';');
	SerialOutput.print(String(freq));
	SerialOutput.print(';');
	SerialOutput.print(String(0));
	SerialOutput.print(';');
	SerialOutput.print(String(m_yaw));
	SerialOutput.print(';');
	SerialOutput.print(String(m_pitch));
	SerialOutput.print(';');
	SerialOutput.print(String(m_roll));
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

float customMod(double a, double b){
	return a - floor(a/b) * b;
}

float getAngleError(float currentAngle, float initialAngle){
	return (customMod((currentAngle - initialAngle) + 180.0, 360.0) - 180.0);
}

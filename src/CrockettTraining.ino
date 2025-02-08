
#include <Arduino.h>
#include "BluetoothSerial.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "math.h"


#define VERSION "ECroST 10_11_2024"

enum TrainerStatus{
	IDLE,
	SELECTING,
	COUNTDOWN,
	SELECTED,
	PLAYING
};

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
sensors_event_t m_orientationData;

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial BluetoothOutput;

bool m_debug_mode = false;
bool m_laser_always_powered_on = false;
unsigned long m_serial_last_update = 0;
unsigned long m_bluetooth_last_update = 0;
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

uint8_t m_sys_calibration = 0;
uint8_t m_gyro_calibration = 0;
uint8_t m_accel_calibration = 0;
uint8_t m_mag_calibration = 0;

uint64_t m_countdown_timer = 0;
uint64_t m_countdown_signal_timer = 0;
bool m_countdown_signaled = false;

bool m_always_laser_signaled = false;

TrainerStatus m_status = IDLE;

/***************************
BUZZER: GPIO16.
LASER: GPIO23
//POTENCIOMETRO: GPIO35
POTENCIOMETRO: GPIO33
//BATERIA: GPIO34.
BATERIA: GPIO35.
I2C SDA: GPIO21.
I2C SCL: GPIO22.
LED LEFT: GPIO18.
LED RIGHT:GPIO32.
PULSADOR: GPIO17.
*******************************/

#define MAX_ERROR 5

#define BUTTON_PIN 27//1.4 ESP32 Version => 17
#define BUZZER_PIN 25//1.4 ESP32 Version => 16
#define LASER_PIN 23
//#define POTENCIOMETER_PIN 35
#define POTENCIOMETER_PIN 33
#define BATTERY_PIN 35
#define RIGHT_LED_PIN 32
#define LEFT_LED_PIN 18

#define SSID_NAME "ECroST"
#define BAUDRATE_SERIALPORT_OUTPUT 115200

uint16_t BNO055_SAMPLERATE_DELAY_MS = 20; //how often to read data from the board

void setup()
{
	esp_log_level_set("*", ESP_LOG_NONE);

	int freq = getCpuFrequencyMhz();

	m_status = IDLE;

	pinMode(BUTTON_PIN, INPUT);
	pinMode(RIGHT_LED_PIN,OUTPUT);
	pinMode(LEFT_LED_PIN,OUTPUT);
	pinMode(BUZZER_PIN, OUTPUT);
	pinMode(LASER_PIN, OUTPUT);
	pinMode(POTENCIOMETER_PIN,INPUT);
  
	//To enter in Debug Mode the Button must be pushed when the device
	//is Powered ON
	bool debug_mode = !digitalRead(BUTTON_PIN);
	Serial.begin(BAUDRATE_SERIALPORT_OUTPUT);

	// Call imu.begin() to verify communication and initialize
	if (!bno.begin(OPERATION_MODE_IMUPLUS))
	{
		while (1)
		{
			Serial.println("Unable to communicate with BNO055");
			Serial.println("Check connections, and try again.");
			Serial.println();
			delay(5000);
		}
	}
  
	
  	tone(BUZZER_PIN,1000,1000);
  	digitalWrite(LEFT_LED_PIN,HIGH);
  	digitalWrite(RIGHT_LED_PIN,HIGH);
	delay(1000);
	digitalWrite(LEFT_LED_PIN,LOW);
	digitalWrite(RIGHT_LED_PIN,LOW);
  
    //When the check beep is played and the Button continues pushed
	//The debug mode will be enabled
    if(debug_mode && !digitalRead(BUTTON_PIN)){
		
		delay(1000);
    	tone(BUZZER_PIN,1000,200);
    	delay(200);
		tone(BUZZER_PIN,1000,200);
    	delay(2000);

		m_debug_mode = true;
      	Serial.begin(BAUDRATE_SERIALPORT_OUTPUT);
      	BluetoothOutput.begin(SSID_NAME);   
	}
}

void loop()
{
	int incomingByte = 0;

	if(!digitalRead(BUTTON_PIN)){
		
		if(m_countdown_timer == 0 || m_status == COUNTDOWN){
			m_countdown_timer = millis();
		}

		if(!m_countdown_signaled && (millis() - m_countdown_timer) >= 3000){
			tone(BUZZER_PIN,1000,100);
		    m_countdown_signaled = true;
			
		}

		if(!m_always_laser_signaled && (millis() - m_countdown_timer) >= 8000){
			tone(BUZZER_PIN,1000,200);
			m_always_laser_signaled = true;
		}

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
		digitalWrite(LASER_PIN,HIGH);

	}else{
		if(m_status == SELECTING){

			if((millis() - m_countdown_timer) >= 3000){
				
				if((millis() - m_countdown_timer) >= 8000){
					m_laser_always_powered_on = true;
				}
				else{
					m_laser_always_powered_on = false;
				}
				
				m_status = COUNTDOWN;
				m_countdown_timer = millis();
				m_countdown_signal_timer = 0;
				m_countdown_signaled = false;
				m_always_laser_signaled = false;

				
			}else{
				m_status = SELECTED;
				m_countdown_timer = 0;
				m_laser_always_powered_on = false;
			}
		}else if(m_status == COUNTDOWN){

			if(m_countdown_signal_timer == 0){
				m_countdown_signal_timer = millis();
				tone(BUZZER_PIN,1000,100);
			}else if((millis() - m_countdown_timer) >= 10000){
				m_countdown_signal_timer = 0;
				m_countdown_signaled = false;
				m_always_laser_signaled = false;
				m_countdown_timer = 0;
				tone(BUZZER_PIN,1000,500);
				m_status = SELECTED;			
			}else if((millis() - m_countdown_signal_timer) >= 1000){
				m_countdown_signal_timer = millis();
				tone(BUZZER_PIN,1000,100);
			}
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
		bno.getCalibration(&m_sys_calibration, 
		&m_gyro_calibration, 
		&m_accel_calibration, 
		&m_mag_calibration);

		calculateIMUPosition();

    if(m_debug_mode){
		  printBluetoothIMUData();
		  printSerialIMUData();
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

		if(!m_laser_always_powered_on){
			digitalWrite(LASER_PIN, LOW);
		}
		
		tone(BUZZER_PIN, 1000, 200);
		m_initial_pitch = m_pitch;
		m_initial_roll = m_roll;
		m_initial_yaw = m_yaw;
		m_yaw_error = 0;
		m_pitch_error = 0;
		m_roll_error = 0;
	}

	m_potenciometer_raw_value = analogRead(POTENCIOMETER_PIN);
	m_difficult_level = (m_potenciometer_raw_value * MAX_ERROR) / 4095.0;

	if(m_status == PLAYING){

		m_yaw_error = getAngleError(m_yaw, m_initial_yaw);

		if(m_yaw_error > m_difficult_level){
			digitalWrite(LEFT_LED_PIN,HIGH);
      digitalWrite(RIGHT_LED_PIN,LOW);
			//tone(BUZZER_PIN,2000,0);
      tone(BUZZER_PIN,4000,0);
		}else if(m_yaw_error < -m_difficult_level){
			digitalWrite(LEFT_LED_PIN,LOW);
			digitalWrite(RIGHT_LED_PIN,HIGH);
      //tone(BUZZER_PIN,1000,0);
      tone(BUZZER_PIN,2000,0);
		}else{
			noTone(BUZZER_PIN);
			digitalWrite(RIGHT_LED_PIN,LOW);
			digitalWrite(LEFT_LED_PIN,LOW);
		}
	}
}

void printSerialIMUData(void)
{
	unsigned long current_update = millis();
	float freq = (1.0/(current_update - m_serial_last_update))*1000;
	m_serial_last_update = current_update;

  Serial.print(String(VERSION));
  Serial.print(';');
	Serial.print(String(current_update));
	Serial.print(';');
	Serial.print(String(freq));
	Serial.print(';');
	Serial.print(String(0));
	Serial.print(';');
	Serial.print(String(m_yaw));
	Serial.print(';');
	Serial.print(String(m_pitch));
	Serial.print(';');
	Serial.print(String(m_roll));
	Serial.print(';');
	Serial.print(String(m_initial_yaw));
	Serial.print(';');
	Serial.print(String(m_initial_pitch));
	Serial.print(';');
	Serial.print(String(m_initial_roll));
	Serial.print(';');
	Serial.print(String(m_yaw_error));
	Serial.print(';');
	Serial.print(String(m_pitch - m_initial_pitch));
	Serial.print(';');
	Serial.print(String(m_roll - m_initial_roll));
	Serial.print(';');
	Serial.print(String(m_difficult_level));
	Serial.print(';');
	Serial.print(String(m_sys_calibration));
	Serial.print(';');
	Serial.print(String(m_gyro_calibration));
	Serial.print(';');
	Serial.print(String(m_accel_calibration));
	Serial.print(';');
	Serial.println(String(m_mag_calibration));

}

void printBluetoothIMUData(void)
{
	unsigned long current_update = millis();
	float freq = (1.0/(current_update - m_bluetooth_last_update))*1000;
	m_bluetooth_last_update = current_update;

  BluetoothOutput.print(String(VERSION));
  BluetoothOutput.print(';');
	BluetoothOutput.print(String(current_update));
	BluetoothOutput.print(';');
	BluetoothOutput.print(String(freq));
	BluetoothOutput.print(';');
	BluetoothOutput.print(String(0));
	BluetoothOutput.print(';');
	BluetoothOutput.print(String(m_yaw));
	BluetoothOutput.print(';');
	BluetoothOutput.print(String(m_pitch));
	BluetoothOutput.print(';');
	BluetoothOutput.print(String(m_roll));
	BluetoothOutput.print(';');
	BluetoothOutput.print(String(m_initial_yaw));
	BluetoothOutput.print(';');
	BluetoothOutput.print(String(m_initial_pitch));
	BluetoothOutput.print(';');
	BluetoothOutput.print(String(m_initial_roll));
	BluetoothOutput.print(';');
	BluetoothOutput.print(String(m_yaw_error));
	BluetoothOutput.print(';');
	BluetoothOutput.print(String(m_pitch - m_initial_pitch));
	BluetoothOutput.print(';');
	BluetoothOutput.print(String(m_roll - m_initial_roll));
	BluetoothOutput.print(';');
	BluetoothOutput.print(String(m_difficult_level));
	BluetoothOutput.print(';');
	BluetoothOutput.print(String(m_sys_calibration));
	BluetoothOutput.print(';');
	BluetoothOutput.print(String(m_gyro_calibration));
	BluetoothOutput.print(';');
	BluetoothOutput.print(String(m_accel_calibration));
	BluetoothOutput.print(';');
	BluetoothOutput.println(String(m_mag_calibration));
}

float customMod(double a, double b){
	return a - floor(a/b) * b;
}

float getAngleError(float currentAngle, float initialAngle){
	return (customMod((currentAngle - initialAngle) + 180.0, 360.0) - 180.0);
}

#ifndef CARBOARD_HPP
#define CARBOARD_HPP

#include <array>
#include <cinttypes>

#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <Servo.h>

// External declarations
extern SoftwareSerial debugSerial;

// MSP decoding states
enum class MSPState {
	IDLE,
	HEADER_START,      // $ received
	HEADER_M,          // M received
	HEADER_ARROW,      // > received
	HEADER_SIZE,       // Size received
	HEADER_CMD,        // Command received
	DATA,              // Receiving data
	CHECKSUM           // Checksum received
};

class CarBoard {
	private:
		Servo _throttleServo;
		Servo _steeringServo;

		uint16_t _batt_adc = 0;
		unsigned long _batt_adc_time = 0;

		std::array<int16_t, 3> _imu_xl;
		std::array<int16_t, 3> _imu_g;
		unsigned long _imu_sample_time = 0;

		uint8_t _ir_value;
		unsigned long _ir_time = 0;

		uint16_t _steering_left = 2000;
		uint16_t _steering_right = 1000;
		uint16_t _throttle_start_fw = 0;
		uint16_t _throttle_start_bw = 0;
		
		// MSP protocol handling
		MSPState _mspState = MSPState::IDLE;
		uint8_t _mspDataSize = 0;
		uint8_t _mspCommand = 0;
		uint8_t _mspChecksum = 0;
		uint8_t _mspReceived = 0;
		uint8_t _mspData[64];  // Buffer for MSP data
		
		// Serial buffer
		static constexpr const size_t SERIAL_BUFFER_SIZE = 256;
		uint8_t _serialBuffer[SERIAL_BUFFER_SIZE];
		size_t _bufferHead = 0;
		size_t _bufferTail = 0;
		
		// Add a byte to the circular buffer
		void addToBuffer(uint8_t byte);
		
		// Get a byte from the circular buffer, returns -1 if buffer is empty
		int getFromBuffer();
		
		// Process a byte according to MSP protocol
		void processMSPByte(uint8_t c);
		
		// Interpret received MSP message based on command
		void interpretMSPMessage();

	public:
		void init();
		void loop();

		std::array<uint8_t, 6> mac() const;

		template<typename T>
		T eeprom_load() const {
			T ret;
			EEPROM.get(0, ret);
			return ret;
		}
		template<typename T>
		void eeprom_save(const T & data) {
			EEPROM.put(0, data);
			EEPROM.commit();
		}

		Stream & debug_serial() const;

		void setThrottle(int16_t i_speed);
		void setSteering(int16_t i_angle);
		void setHeadlights(uint16_t i_pwr);
		void setColor(uint8_t r, uint8_t g, uint8_t b);

		void setSteeringTrim(int16_t v);
		void setThrottleStart(uint16_t fw, uint16_t bw);

		uint16_t batteryLevel_ADC() const;
		int16_t batterySOC() const;

		std::array<int16_t, 3> imuAccelerometerData() const { return _imu_xl; }
		std::array<int16_t, 3> imuGyroscopeData() const { return _imu_g; }

		uint8_t ir_value() const { return _ir_value; }
		
		// Send an MSP request to the VTX
		void sendMSPRequest(uint8_t command, const uint8_t* data = nullptr, size_t dataSize = 0);
		
		// Send an MSP response to a received command
		void sendMSPResponse(uint8_t command);
		
		// Helper methods for common MSP commands
		void mspRequestApiVersion();
		void mspRequestBoardInfo();
		void mspRequestName();
		void mspSetName(const char* name, size_t nameLength);
};

#endif

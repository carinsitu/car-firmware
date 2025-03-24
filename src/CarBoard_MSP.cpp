#include <CarBoard.hpp>

// Send MSP response to a command
void CarBoard::sendMSPResponse(uint8_t command) {
	// Prepare response data based on command
	uint8_t responseData[32]; // Buffer for response data
	uint8_t dataSize = 0;	  // Size of response data

	// Handle different commands
	if (command == 101) { // MSP_STATUS
		debugSerial.println("MSP: Sending STATUS response");

		// MSP_STATUS response format:
		// uint16_t cycleTime    - Current main loop cycle time in microseconds
		// uint16_t i2cErrors    - Count of I2C bus errors
		// uint16_t sensors      - Sensor status flags (0x01=acc, 0x02=baro, 0x04=mag, 0x08=gps, 0x10=sonar)
		// uint32_t flightModes  - Current flight mode flags
		// uint8_t configProfile - Currently active profile index
		// uint16_t systemLoad   - System load (0-100%)
		// uint16_t gyroTemp     - Gyro temperature (for compatible boards)

		// Filling in data:
		uint16_t cycleTime = 2000;	 // 2ms loop time (example)
		uint16_t i2cErrors = 0;		 // No I2C errors
		uint16_t sensors = 0x01;	 // We have accelerometer
		uint32_t flightModes = 0;	 // No flight modes active
		uint8_t configProfile = 0;	 // Profile 0
		uint16_t systemLoad = 30;	 // 30% system load (example)
		uint16_t gyroTemp = 25 * 10; // 25°C (stored as 10x value)

		// Pack data into response buffer
		responseData[dataSize++] = cycleTime & 0xFF;
		responseData[dataSize++] = (cycleTime >> 8) & 0xFF;

		responseData[dataSize++] = i2cErrors & 0xFF;
		responseData[dataSize++] = (i2cErrors >> 8) & 0xFF;

		responseData[dataSize++] = sensors & 0xFF;
		responseData[dataSize++] = (sensors >> 8) & 0xFF;

		responseData[dataSize++] = flightModes & 0xFF;
		responseData[dataSize++] = (flightModes >> 8) & 0xFF;
		responseData[dataSize++] = (flightModes >> 16) & 0xFF;
		responseData[dataSize++] = (flightModes >> 24) & 0xFF;

		responseData[dataSize++] = configProfile;

		responseData[dataSize++] = systemLoad & 0xFF;
		responseData[dataSize++] = (systemLoad >> 8) & 0xFF;

		responseData[dataSize++] = gyroTemp & 0xFF;
		responseData[dataSize++] = (gyroTemp >> 8) & 0xFF;
	}
	// Add more command handlers as needed (else if blocks)

	// Send the response if we have data
	if (dataSize > 0) {
		// Format: $M>[data_length][command][data][checksum]
		debugSerial.printf("MSP: Sending response to command %d with %d bytes of data\n", command, dataSize);

		Serial.write('$');
		Serial.write('M');
		Serial.write('>'); // '>' for responses (not '<')

		uint8_t checksum = 0;

		// Data length
		Serial.write(dataSize);
		checksum ^= dataSize;
		debugSerial.printf("MSP TX: $M> len=%d ", dataSize);

		// Command
		Serial.write(command);
		checksum ^= command;
		debugSerial.printf("cmd=%d ", command);

		// Data
		if (dataSize > 0) {
			debugSerial.printf("data=[");
			for (size_t i = 0; i < dataSize; i++) {
				Serial.write(responseData[i]);
				checksum ^= responseData[i];
				debugSerial.printf("%02X ", responseData[i]);
			}
			debugSerial.printf("] ");
		}

		// Checksum
		Serial.write(checksum);
		debugSerial.printf("crc=%02X\n", checksum);
	}
}
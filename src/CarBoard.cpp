#include <CarBoard.hpp>
#include <ImuLSM6DS3.hpp>
#include <Max17261.hpp>

#include <ESP8266WiFi.h>
#include <NeoPixelBus.h>
#include <IRrecv.h>

// Pin definitions
constexpr const int PIN_DEBUG_RX = 3;
constexpr const int PIN_DEBUG_TX = 1;
constexpr const int PIN_THROTTLE = 14;
constexpr const int PIN_STEERING = 0;
constexpr const int PIN_HEADLIGHTS = 16;
constexpr const int PIN_IR = 12;

// Global objects
SoftwareSerial debugSerial(PIN_DEBUG_RX, PIN_DEBUG_TX);
NeoPixelBus<NeoGrbFeature, NeoEsp8266Uart1800KbpsMethod> strip(/* length */ 1);
ImuLSM6DS3 imu(Wire);
IRrecv ir_recv(PIN_IR);
decode_results ir_result;
Max17261 battery;

// Add a byte to the circular buffer
void CarBoard::addToBuffer(uint8_t byte) {
	size_t nextHead = (_bufferHead + 1) % SERIAL_BUFFER_SIZE;
	if (nextHead != _bufferTail) {
		_serialBuffer[_bufferHead] = byte;
		_bufferHead = nextHead;
	}
	// If buffer is full, byte is lost
}

// Get a byte from the circular buffer, returns -1 if buffer is empty
int CarBoard::getFromBuffer() {
	if (_bufferHead == _bufferTail) {
		return -1; // Buffer empty
	}
	uint8_t byte = _serialBuffer[_bufferTail];
	_bufferTail = (_bufferTail + 1) % SERIAL_BUFFER_SIZE;
	return byte;
}

// Process a byte according to MSP protocol
void CarBoard::processMSPByte(uint8_t c) {
	switch (_mspState) {
		case MSPState::IDLE:
			if (c == '$') {
				_mspState = MSPState::HEADER_START;
				debugSerial.println("\nMSP: Start");
			} else {
				debugSerial.printf("x%02X ", c); // Print non-MSP bytes as hex
			}
			break;

		case MSPState::HEADER_START:
			if (c == 'M') {
				_mspState = MSPState::HEADER_M;
				debugSerial.println("MSP: M received");
			} else {
				_mspState = MSPState::IDLE;
				debugSerial.printf("MSP: Invalid char after $: %02X\n", c);
			}
			break;

		case MSPState::HEADER_M:
			if (c == '<') {
				_mspState = MSPState::HEADER_ARROW;
				debugSerial.println("MSP: < received");
			} else {
				_mspState = MSPState::IDLE;
				debugSerial.printf("MSP: Invalid char after M: %02X\n", c);
			}
			break;

		case MSPState::HEADER_ARROW:
			_mspDataSize = c;
			_mspChecksum = c; // Init checksum
			_mspReceived = 0;
			_mspState = MSPState::HEADER_SIZE;
			debugSerial.printf("MSP: Size: %d\n", _mspDataSize);
			break;

		case MSPState::HEADER_SIZE:
			_mspCommand = c;
			_mspChecksum ^= c;
			_mspState = (_mspDataSize > 0) ? MSPState::DATA : MSPState::CHECKSUM;
			debugSerial.printf("MSP: Command: %d\n", _mspCommand);
			if (_mspDataSize == 0) {
				debugSerial.println("MSP: No data, waiting for checksum");
			}
			break;

		case MSPState::HEADER_CMD: // Add missing case
			// This state is not used, but added to silence warning
			_mspState = MSPState::IDLE;
			debugSerial.println("MSP: Invalid state HEADER_CMD");
			break;

		case MSPState::DATA:
			_mspChecksum ^= c;
			if (_mspReceived < sizeof(_mspData)) {
				_mspData[_mspReceived] = c;
				debugSerial.printf("MSP: Data[%d]: %02X\n", _mspReceived, c);
				_mspReceived++;
			}
			if (_mspReceived >= _mspDataSize) {
				_mspState = MSPState::CHECKSUM;
				debugSerial.println("MSP: Data complete, waiting for checksum");
			}
			break;

		case MSPState::CHECKSUM:
			if (_mspChecksum == c) {
				// Valid MSP message received
				debugSerial.printf("\nMSP: Valid message cmd %d, size %d: ", _mspCommand, _mspDataSize);
				for (uint8_t i = 0; i < _mspReceived; i++) {
					debugSerial.printf("%02X ", _mspData[i]);
				}
				debugSerial.println();

				// Interpret specific MSP messages
				interpretMSPMessage();
			} else {
				debugSerial.printf("\nMSP: Checksum error: got %02X, calculated %02X\n", c, _mspChecksum);
			}
			_mspState = MSPState::IDLE;
			break;
	}
}

// Interpret received MSP message based on command
void CarBoard::interpretMSPMessage() {
	debugSerial.printf("MSP: Interpreting command %d\n", _mspCommand);

	switch (_mspCommand) {
		case 1: // MSP_API_VERSION
			debugSerial.println("MSP: Command is API_VERSION");
			if (_mspReceived >= 3) {
				debugSerial.printf("MSP: API Version: %d.%d.%d\n",
								   _mspData[0], _mspData[1], _mspData[2]);
			} else {
				debugSerial.printf("MSP: Invalid API_VERSION data length: %d (expected 3+)\n", _mspReceived);
			}
			break;

		case 2: // MSP_FC_VARIANT
			debugSerial.println("MSP: Command is FC_VARIANT");
			if (_mspReceived >= 4) {
				char fcVariant[5];
				memcpy(fcVariant, _mspData, 4);
				fcVariant[4] = '\0';
				debugSerial.printf("MSP: FC Variant: %s\n", fcVariant);
			} else {
				debugSerial.printf("MSP: Invalid FC_VARIANT data length: %d (expected 4+)\n", _mspReceived);
			}
			break;

		case 3: // MSP_FC_VERSION
			debugSerial.println("MSP: Command is FC_VERSION");
			if (_mspReceived >= 3) {
				debugSerial.printf("MSP: FC Version: %d.%d.%d\n",
								   _mspData[0], _mspData[1], _mspData[2]);
			} else {
				debugSerial.printf("MSP: Invalid FC_VERSION data length: %d (expected 3+)\n", _mspReceived);
			}
			break;

		case 4: // MSP_BOARD_INFO
			debugSerial.println("MSP: Command is BOARD_INFO");
			if (_mspReceived >= 6) {
				char boardName[5];
				memcpy(boardName, _mspData, 4);
				boardName[4] = '\0';
				debugSerial.printf("MSP: Board: %s, version: %d\n",
								   boardName, _mspData[4]);
			} else {
				debugSerial.printf("MSP: Invalid BOARD_INFO data length: %d (expected 6+)\n", _mspReceived);
			}
			break;

		case 10: // MSP_NAME
			debugSerial.println("MSP: Command is NAME");
			if (_mspReceived > 0) {
				char name[65];
				size_t len = (_mspReceived < 64) ? _mspReceived : 64;
				memcpy(name, _mspData, len);
				name[len] = '\0';
				debugSerial.printf("MSP: Device name: %s\n", name);
			} else {
				debugSerial.println("MSP: NAME data is empty");
			}
			break;

		case 101: // MSP_STATUS
			debugSerial.println("MSP: Command is STATUS");
			// Prepare and send a response with status information
			sendMSPResponse(_mspCommand);
			break;

			// Add more command interpretations as needed

		default:
			debugSerial.printf("MSP: Unknown command %d\n", _mspCommand);
			break;
	}
}

void CarBoard::init() {
	_throttleServo.attach(PIN_THROTTLE);
	setThrottle(0);

	_steeringServo.attach(PIN_STEERING);
	setSteering(0);

	pinMode(PIN_HEADLIGHTS, OUTPUT);
	setHeadlights(0);

	EEPROM.begin(256);

	Serial.begin(115200);
	Serial.swap();		// Change uart mux
	U0C0 |= BIT(UCTXI); // Invert TX signal

	debugSerial.begin(76800);
	debugSerial.printf("\n\nCarInSitu %s\n", FIRMWARE_VERSION);

	strip.Begin();
	setColor(0, 0, 0);

	_batt_adc = analogRead(0);

	Wire.begin();
	imu.init();
	ir_recv.enableIRIn();

	// Init battery

	// 800 (400mAh on 10mΩ)
	// 900 (450mAh on 10mΩ)
	// 1200 (600mAh on 10mΩ)
	const uint16_t designCapacity = 900;
	// FIXME: Tune this value according to our application
	const uint16_t iChgTerm = 0x0640; // (250mA on 10mΩ)
	// VE: Empty Voltage Target, during load
	// VR: Recovery voltage
	const uint16_t vEmpty = 0xA761; // VE/VR: 0xAA61 → 3.4V/3.88V (0xA561 → 3.3V/3.88V (default))
	// In typical cases, if charge voltage > 4,275 then 0x8400 else 0x8000
	// FIXME: Tune this value according to our charge voltage
	const uint16_t modelCFG = 0x8000;
	battery.begin(
		designCapacity,
		iChgTerm,
		vEmpty,
		modelCFG);

	// Send MSP_API_VERSION request to get VTX protocol version
	mspRequestApiVersion();

	debugSerial.println("VTX MSP request sent");
}

void CarBoard::loop() {
	unsigned long now = millis();
	if (now - _batt_adc_time > 200) {
		_batt_adc_time = now;
		_batt_adc = analogRead(0);
	}

	// Read incoming data from Serial and store in buffer
	while (Serial.available()) {
		addToBuffer(Serial.read());
	}

	// Process buffered data through MSP parser
	while (debugSerial.availableForWrite()) {
		int byte = getFromBuffer();
		if (byte == -1) {
			break; // Buffer empty
		}
		processMSPByte(byte);
	}

	if (now - _imu_sample_time >= 10) {
		_imu_sample_time = now;
		auto imu2car_coord = [](auto vec) {
			std::swap(vec[0], vec[1]);
			vec[0] *= -1;
			return vec;
		};
		if (imu.accelerometerDataReady()) _imu_xl = imu2car_coord(imu.readAccelerometer());
		if (imu.gyroscopeDataReady()) _imu_g = imu2car_coord(imu.readGyroscope());
	}
	if (ir_recv.decode(&ir_result)) {
		if (ir_result.decode_type == RC6 && ir_result.bits == 12) {
			uint16_t value = ir_result.value;
			if ((value & 0xf00) == 0xa00) {
				_ir_value = value;
				_ir_time = now;
			}
		}
		ir_recv.resume();
	}
	if (now - _ir_time >= 200) _ir_value = 0;

	battery.process();
}

std::array<uint8_t, 6> CarBoard::mac() const {
	std::array<uint8_t, 6> ret;
	WiFi.macAddress(ret.data());
	return ret;
}

Stream& CarBoard::debug_serial() const {
	return debugSerial;
}

void CarBoard::setSteering(int16_t i_angle) {
	const int16_t value = map(i_angle, -32768, 32767, _steering_left, _steering_right);
	_steeringServo.writeMicroseconds(value);
}

void CarBoard::setThrottle(int16_t i_speed) {
	static int16_t i_current_speed = 0;
	const int16_t value = (i_speed > 0) ? map(i_speed, 0, 32767, 1500 - _throttle_start_fw, 1000) : (i_speed < 0) ? map(i_speed, -32768, 0, 2000, 1500 + _throttle_start_bw)
																												  : 1500;

	// Workaround: Brushed motors can have difficulties to start to rotate
	// To workaround this inertial effect, we overshoot during a short time, then set desired value
	// FIXME: Implement this in a non-blocking way
	if ((i_current_speed == 0) && (i_speed != 0)) {
		const int16_t i_value = i_speed > 0 ? 1000 : 2000;
		_throttleServo.writeMicroseconds(i_value);
		delay(20);
	}
	i_current_speed = i_speed;

	_throttleServo.writeMicroseconds(value);
}

void CarBoard::setHeadlights(uint16_t i_pwr) {
	const uint8_t value = map(i_pwr, 0, 65535, 0, 255);
	analogWrite(PIN_HEADLIGHTS, value);
}

void CarBoard::setColor(uint8_t r, uint8_t g, uint8_t b) {
	strip.SetPixelColor(0, RgbColor(r, g, b));
	strip.Show();
}

void CarBoard::setSteeringTrim(int16_t v) {
	const int16_t trim = std::max(static_cast<int16_t>(-450), std::min(v, static_cast<int16_t>(450)));
	_steering_left = 2000 + (trim < 0 ? trim : 0);
	_steering_right = 1000 + (trim > 0 ? trim : 0);
}

void CarBoard::setThrottleStart(uint16_t fw, uint16_t bw) {
	_throttle_start_fw = std::min(fw, static_cast<uint16_t>(450));
	_throttle_start_bw = std::min(bw, static_cast<uint16_t>(450));
}

uint16_t CarBoard::batteryLevel_ADC() const {
	return _batt_adc;
}

int16_t CarBoard::batterySOC() const {
	const int16_t soc = battery.readStateOfCharge();
	if (soc < 0) {
		return soc;
	}
	if (soc < 256 * 20) {
		return 0;
	}
	// We keep 20% of the battery capacity as a safety margin as we notice that the battery SoC drops
	// below 20% before the car is powered off.
	return ((float)soc - (256 * 20)) * (100.0 / 80.0);
}

void CarBoard::sendMSPRequest(uint8_t command, const uint8_t* data, size_t dataSize) {
	// MSP protocol: $M<[data_length][command][data][checksum]
	debugSerial.printf("\nMSP: Sending command %d with %d bytes of data\n", command, dataSize);

	Serial.write('$');
	Serial.write('M');
	Serial.write('<');

	uint8_t checksum = 0;

	// Data length
	Serial.write(dataSize);
	checksum ^= dataSize;
	debugSerial.printf("MSP TX: $M< len=%d ", dataSize);

	// Command
	Serial.write(command);
	checksum ^= command;
	debugSerial.printf("cmd=%d ", command);

	// Data
	if (dataSize > 0) {
		debugSerial.printf("data=[");
		for (size_t i = 0; i < dataSize; i++) {
			Serial.write(data[i]);
			checksum ^= data[i];
			debugSerial.printf("%02X ", data[i]);
		}
		debugSerial.printf("] ");
	}

	// Checksum
	Serial.write(checksum);
	debugSerial.printf("crc=%02X\n", checksum);
}

// Common MSP commands:
// 1  - MSP_API_VERSION - Get API version
// 2  - MSP_FC_VARIANT  - Get flight controller variant
// 3  - MSP_FC_VERSION  - Get flight controller version
// 4  - MSP_BOARD_INFO  - Get board info
// 5  - MSP_BUILD_INFO  - Get build info
// 10 - MSP_NAME        - Get device name
// 11 - MSP_SET_NAME    - Set device name

// Helper methods for common MSP commands
void CarBoard::mspRequestApiVersion() {
	sendMSPRequest(1); // MSP_API_VERSION
}

void CarBoard::mspRequestBoardInfo() {
	sendMSPRequest(4); // MSP_BOARD_INFO
}

void CarBoard::mspRequestName() {
	sendMSPRequest(10); // MSP_NAME
}

void CarBoard::mspSetName(const char* name, size_t nameLength) {
	sendMSPRequest(11, reinterpret_cast<const uint8_t*>(name), nameLength); // MSP_SET_NAME
}

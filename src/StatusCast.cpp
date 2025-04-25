#include <StatusCast.hpp>

#include <ESP8266WiFi.h>

void StatusCast::init() {
}

void StatusCast::requestStatus(status_t status) {
	_on_demand_queue.push(status);
}

void StatusCast::loop() {
	unsigned long now = millis();
	if (now - _last_msg < 25) return;
	_last_msg = now;

	if (!WiFi.isConnected()) return;

	std::array<uint8_t, 256> msg;
	int len = 0;

	msg[len++] = 'C';
	msg[len++] = 'I';
	msg[len++] = 'S';

	const constexpr std::array<status_t, 4> exportsA = {STATUS_RSSI, STATUS_IR, STATUS_SIMULATION, STATUS_HEADLIGHTS};
	const constexpr std::array<status_t, 4> exportsB = {STATUS_COLOR, STATUS_BATTERY, STATUS_IMU, STATUS_PILOT};
	const auto& exports = _last_msg_id ? exportsA : exportsB;
	_last_msg_id ^= 1;

	// Process regular status updates
	for (auto s : exports) {
		if (s != STATUS_NO_REPORT) {
			len = addStatus(s, msg.data(), len, msg.size());
		}
	}

	// Process any on-demand status requests
	while (!_on_demand_queue.empty() && len < (msg.size() - 10)) { // Leave some buffer space
		status_t status = _on_demand_queue.front();
		_on_demand_queue.pop();

		len = addStatus(status, msg.data(), len, msg.size());
	}

	// Send the message if we have data
	if (len > 3) { // CIS header is 3 bytes
		const IPAddress multicast{239, 255, 0, 1};
		_udp.beginPacketMulticast(multicast, 4211, WiFi.localIP());
		_udp.write(msg.data(), len);
		_udp.endPacket();
	}
}

// Helper method to add status to the message buffer
int StatusCast::addStatus(status_t status, uint8_t* msg, int len, int max_len) {
	int original_len = len;

	switch (status) {
		case STATUS_NO_REPORT:
			break; // Do nothing
		case STATUS_RSSI: {
			if (len + 5 > max_len) return original_len; // Not enough space

			msg[len++] = STATUS_RSSI;
			const int32_t rssi = WiFi.RSSI();
			msg[len++] = (rssi >> 0) & 0xFF;
			msg[len++] = (rssi >> 8) & 0xFF;
			msg[len++] = (rssi >> 16) & 0xFF;
			msg[len++] = (rssi >> 24) & 0xFF;
			break;
		}
		case STATUS_IR: {
			if (len + 2 > max_len) return original_len; // Not enough space

			msg[len++] = STATUS_IR;
			const uint8_t ir = _car.ir_value();
			msg[len++] = ir;
			break;
		}
		case STATUS_SIMULATION: {
			if (len + 7 > max_len) return original_len; // Not enough space

			msg[len++] = STATUS_SIMULATION;
			auto& sim = _car.simulation();
			int16_t x = sim.x();
			msg[len++] = (x >> 0) & 0xFF;
			msg[len++] = (x >> 8) & 0xFF;
			int16_t y = sim.y();
			msg[len++] = (y >> 0) & 0xFF;
			msg[len++] = (y >> 8) & 0xFF;
			int16_t theta = sim.theta();
			msg[len++] = (theta >> 0) & 0xFF;
			msg[len++] = (theta >> 8) & 0xFF;
			break;
		}
		case STATUS_HEADLIGHTS: {
			if (len + 3 > max_len) return original_len; // Not enough space

			msg[len++] = STATUS_HEADLIGHTS;
			uint16_t hl = _car.headlightsPower();
			msg[len++] = (hl >> 0) & 0xFF;
			msg[len++] = (hl >> 8) & 0xFF;
			break;
		}
		case STATUS_COLOR: {
			if (len + 7 > max_len) return original_len; // Not enough space

			msg[len++] = STATUS_COLOR;
			auto color = _car.color();
			msg[len++] = color[0];
			msg[len++] = color[1];
			msg[len++] = color[2];
			auto rear_color = _car.rear_color();
			msg[len++] = rear_color[0];
			msg[len++] = rear_color[1];
			msg[len++] = rear_color[2];
			break;
		}
		case STATUS_BATTERY: {
			if (len + 5 > max_len) return original_len; // Not enough space

			msg[len++] = STATUS_BATTERY;
			auto batt_adc = _car.batteryLevel_ADC();
			msg[len++] = (batt_adc >> 0) & 0xFF;
			msg[len++] = (batt_adc >> 8) & 0xFF;
			auto batt_soc = _car.batterySOC();
			msg[len++] = (batt_soc >> 0) & 0xFF;
			msg[len++] = (batt_soc >> 8) & 0xFF;
			break;
		}
		case STATUS_IMU: {
			if (len + 13 > max_len) return original_len; // Not enough space

			msg[len++] = STATUS_IMU;
			auto imu_xl = _car.imu_accelerometerData();
			msg[len++] = (imu_xl[0] >> 0) & 0xFF;
			msg[len++] = (imu_xl[0] >> 8) & 0xFF;
			msg[len++] = (imu_xl[1] >> 0) & 0xFF;
			msg[len++] = (imu_xl[1] >> 8) & 0xFF;
			msg[len++] = (imu_xl[2] >> 0) & 0xFF;
			msg[len++] = (imu_xl[2] >> 8) & 0xFF;
			auto imu_g = _car.imu_gyroscopeData();
			msg[len++] = (imu_g[0] >> 0) & 0xFF;
			msg[len++] = (imu_g[0] >> 8) & 0xFF;
			msg[len++] = (imu_g[1] >> 0) & 0xFF;
			msg[len++] = (imu_g[1] >> 8) & 0xFF;
			msg[len++] = (imu_g[2] >> 0) & 0xFF;
			msg[len++] = (imu_g[2] >> 8) & 0xFF;
			break;
		}
		case STATUS_PILOT: {
			if (len + 6 > max_len) return original_len; // Not enough space

			msg[len++] = STATUS_PILOT;
			auto throtlle = _car.speed();
			msg[len++] = (throtlle >> 0) & 0xFF;
			msg[len++] = (throtlle >> 8) & 0xFF;
			auto steering = _car.angle();
			msg[len++] = (steering >> 0) & 0xFF;
			msg[len++] = (steering >> 8) & 0xFF;
			auto started = _car.pilot_started();
			msg[len++] = started;
			break;
		}
		case STATUS_VERSION: {
			// Copy the null-terminated firmware version string
			const char* version = FIRMWARE_VERSION;
			int version_len = strlen(version);

			// Check if we have enough space for version + null terminator
			if (len + version_len + 2 > max_len) return original_len; // Not enough space

			msg[len++] = STATUS_VERSION;

			// Copy the version string with null terminator
			memcpy(&msg[len], version, version_len + 1);
			len += version_len + 1;

			_car.log().printf("DEBUG: Sent firmware version: %s\n", version);
			break;
		}
	}

	return len;
}

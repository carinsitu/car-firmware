#include <Max17261.hpp>

#include <Wire.h>

#define MAX17261_STATUS_BOOT	 4
#define MAX17261_STATUS_POR		 2
#define MAX17261_STATUS_MODELCFG 1
#define MAX17261_STATUS_RUN		 0

Max17261::Max17261() {
	_status = MAX17261_STATUS_BOOT;
	_debugStream = nullptr;
}

bool Max17261::begin(
	uint16_t designCapacity,
	uint16_t iChgTerm,
	uint16_t vEmpty,
	uint16_t modelCFG,
	Stream* debugStream) {
	_designCapacity = designCapacity;
	_iChgTerm = iChgTerm;
	_vEmpty = vEmpty;
	_modelCFG = modelCFG;
	_debugStream = debugStream;

	// Initialize I2C if needed
	Wire.begin();

	// Check if device is responding
	Wire.beginTransmission(MAX1726X_I2C_ADDR);
	bool devicePresent = (Wire.endTransmission() == 0);

	if (_debugStream) {
		uint16_t devName = read(MAX1726X_DEVNAME_REG);
		_debugStream->printf("MAX17261: Device Name: 0x%04X, Device Present: %d\n", devName, devicePresent);

		// Check STATUS register
		uint16_t status = read(MAX1726X_STATUS_REG);
		_debugStream->printf("MAX17261: STATUS: 0x%04X\n", status);

		// Check configuration
		uint16_t config = read(MAX1726X_CONFIG_REG);
		_debugStream->printf("MAX17261: CONFIG: 0x%04X\n", config);
	}

	if (!devicePresent) {
		if (_debugStream) {
			_debugStream->println("MAX17261: Device not responding!");
		}
		_status = MAX17261_STATUS_BOOT;
		return false;
	}

	// Force POR bit setup - helps ensure proper initialization
	uint16_t status_reg = read(MAX1726X_STATUS_REG);
	write(MAX1726X_STATUS_REG, status_reg | 0x0002); // Set POR bit

	_status = MAX17261_STATUS_RUN;
	delay(100); // Short delay
	return true;
}

void Max17261::process() {
	static uint32_t lastProcessTime = 0;
	uint32_t now = millis();

	// Don't try to process too frequently
	if (now - lastProcessTime < 500) {
		return;
	}
	lastProcessTime = now;
	if (_debugStream) {
		switch (_status) {
			case MAX17261_STATUS_BOOT:
				_debugStream->println("MAX17261: Boot");
				break;
			case MAX17261_STATUS_POR:
				_debugStream->println("MAX17261: Power-on Reset");
				break;
			case MAX17261_STATUS_MODELCFG:
				_debugStream->println("MAX17261: Model Config");
				break;
			case MAX17261_STATUS_RUN:
				_debugStream->println("MAX17261: Running");
				break;
		}
	}
	switch (_status) {
		case MAX17261_STATUS_BOOT:
			// Nothing to do, user should run begin()
			break;
		case MAX17261_STATUS_RUN:
			// Step 0
			if (statusPOR()) {
				_status = MAX17261_STATUS_POR;
			}
			break;
		case MAX17261_STATUS_POR:
			// Step 1
			if (startupOperationsCompleted()) {
				// Step 2
				_hibcfg = read(MAX1726X_HIBCFG_REG); // Store original HibCFG value
				write(MAX1726X_COMMAND_REG, 0x0090); // Exit Hibernate Mode step 1
				write(MAX1726X_HIBCFG_REG, 0x0000);	 // Exit Hibernate Mode step 2
				write(MAX1726X_COMMAND_REG, 0x0000); // Exit Hibernate Mode step 3

				// Step 2.1
				write(MAX1726X_DESIGNCAP_REG, _designCapacity); // Design capacity
				write(MAX1726X_ICHGTERM_REG, _iChgTerm);		// Charge Termination Current (cf. End-of-charge detection)
				write(MAX1726X_VEMPTY_REG, _vEmpty);			// Empty and recovery voltages

				write(MAX1726X_MODELCFG_REG, _modelCFG | 0x8000); // Set model configuration with refresh bit

				_status = MAX17261_STATUS_MODELCFG;
			}
			break;
		case MAX17261_STATUS_MODELCFG:
			if (modelCfgRefreshed()) {
				// Step 2.1 (final stage)
				write(MAX1726X_HIBCFG_REG, _hibcfg); // Restore Original HibCFG value

				// Step 3
				uint16_t status_reg = read(MAX1726X_STATUS_REG);		  // Read Status
				writeAndVerify(MAX1726X_STATUS_REG, status_reg & 0xFFFD); // Write and Verify Status with POR bit Cleared

				write(MAX1726X_STATUS_REG, 0x0000); // Clear POR bit and threshold alerts

				_status = MAX17261_STATUS_RUN;
			}
			break;
	}
}

uint16_t Max17261::read(const uint8_t reg) {
	Wire.beginTransmission(MAX1726X_I2C_ADDR);
	Wire.write(reg);
	Wire.endTransmission(false);

	Wire.requestFrom(MAX1726X_I2C_ADDR, 2);
	uint16_t value = Wire.read();
	value |= (uint16_t)Wire.read() << 8;
	return value;
}

bool Max17261::write(const uint8_t reg, const uint16_t value) {
	Wire.beginTransmission(MAX1726X_I2C_ADDR);
	Wire.write(reg);
	Wire.write(value & 0xFF);
	Wire.write((value >> 8) & 0xFF);
	return Wire.endTransmission();
}

bool Max17261::writeAndVerify(const uint8_t reg, const uint16_t value) {
	int attempt = 0;
	uint16_t valueRead;
	do {
		write(reg, value);
		delay(1); // NOTE: ATM, this is acceptable even in a non-blocking context
		valueRead = read(reg);
	} while ((value != valueRead) && (attempt++ < 3));

	return (attempt < 3);
}

bool Max17261::statusPOR() {
	if (_debugStream) {
		_debugStream->printf("STATUS_REG: 0x%04X\n", read(MAX1726X_STATUS_REG));
	}
	return (read(MAX1726X_STATUS_REG) & 0x0002);
}

bool Max17261::startupOperationsCompleted() {
	uint16_t fstat = read(MAX1726X_FSTAT_REG);
	bool dnr = fstat & 0x0001;

	if (_debugStream) {
		_debugStream->printf("FSTAT: 0x%04X, DNR bit: %d\n", fstat, dnr);

		// Additional debug info to help diagnose the issue
		uint16_t status = read(MAX1726X_STATUS_REG);
		_debugStream->printf("STATUS: 0x%04X\n", status);

		// Check specific model configuration
		uint16_t modelCfg = read(MAX1726X_MODELCFG_REG);
		_debugStream->printf("MODELCFG: 0x%04X\n", modelCfg);

		// Show TIMER register status
		uint16_t timer = read(MAX1726X_TIMER_REG);
		_debugStream->printf("TIMER: 0x%04X\n", timer);

		// Show TIMERH register for extended timer info
		uint16_t timerh = read(MAX1726X_TIMERH_REG);
		_debugStream->printf("TIMERH: 0x%04X\n", timerh);
	}
	return (!dnr);
}

bool Max17261::modelCfgRefreshed() {
	return (!(read(MAX1726X_MODELCFG_REG) & 0x8000));
}

int16_t Max17261::readRemainingCapacity() {
	if (_status != MAX17261_STATUS_RUN) {
		return -_status;
	}
	return ((int16_t)(read(MAX1726X_REPCAP_REG) >> 1)); // WARN: Depends on RSense
}

int16_t Max17261::readStateOfCharge() {
	if (_status != MAX17261_STATUS_RUN) {
		return -_status;
	}
	return ((int16_t)read(MAX1726X_REPSOC_REG));
}

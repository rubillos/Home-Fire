#include <Arduino.h>

#include "HomeSpan.h" 
#include "Wire.h"
#include <Adafruit_NeoPixel.h>

#include <WiFi.h>
#include "driver/rmt.h"
#include "esp_log.h"

//////////////////////////////////////////////

constexpr const char* versionString = "v0.1";

constexpr const char* accessoryName = "Fireplace";

constexpr const char* displayName = "Fireplace-Controller";
constexpr const char* modelName = "Fireplace-Controller-ESP32";

constexpr uint8_t indicatorDataPin = PIN_NEOPIXEL;
constexpr uint8_t indicatorPowerPin = NEOPIXEL_POWER;

constexpr uint8_t radioDataPin = MISO;

//////////////////////////////////////////////

// constexpr uint32_t igniteTimeMS = 15 * 1000;
// constexpr uint32_t extinguishTimeMS = 15 * 1000;
// constexpr uint32_t fullRangeValveTimeMS = 12 * 1000;

constexpr uint32_t igniteTimeMS = 5 * 1000;
constexpr uint32_t extinguishTimeMS = 5 * 1000;
constexpr uint32_t fullRangeValveTimeMS = 5 * 1000;

constexpr uint32_t radioPeriodUS = 265;
constexpr uint32_t packetGapTimeMS = 20;

constexpr uint8_t addressLength = 17;
constexpr uint32_t remoteAddress = 0B01011100000100001;

constexpr uint8_t cmdLength = 6;
typedef enum {
	cmdPilot = 0B110011,
	cmdUp = 0B111011,
	cmdDown = 0B000000
} CmdCode;

//////////////////////////////////////////////

bool wifiConnected = false;
bool hadWifiConnection = false;

#if defined(DEBUG)
// #define SerPrintf(...) Serial.printf(__VA_ARGS__)
#define SerPrintf(...) Serial.printf("%d: ", millis()); Serial.printf(__VA_ARGS__)
#define SerBegin(...) Serial.begin(__VA_ARGS__)
#else
#define SerPrintf(...)
#define SerBegin(...)
#endif

//////////////////////////////////////////////

uint64_t millis64() {
	volatile static uint32_t low32 = 0, high32 = 0;
	uint32_t new_low32 = millis();

	if (new_low32 < low32)
		high32++;

	low32 = new_low32;

	return (uint64_t) high32 << 32 | low32;
}

//////////////////////////////////////////////

typedef enum {
	stateOff = 0,
	stateIgniting,
	stateLitIdle,
	stateIncreasing,
	stateDecreasing,
	stateExtinguishing
} hardwareState;

constexpr float homeKitFullScale = 100.0;

constexpr float valveMinValue = 10.0;
constexpr float valveMaxValue = 100.0;
constexpr float valveStepValue = 10.0;

constexpr uint16_t messageLength = addressLength + cmdLength;
constexpr uint32_t addrCmdTimeMS = ((messageLength*3*radioPeriodUS)+999) / 1000;
constexpr uint32_t packetDurationMS = addrCmdTimeMS + packetGapTimeMS;

uint64_t lastSendCommandTime = 0;
uint64_t dontSendBeforeTime = 0;

//////////////////////////////////////////////

constexpr uint8_t ledStatusLevel = 0x10;
constexpr uint32_t whiteColorFlag = 1 << 24;

#define MAKE_RGB(r, g, b) (r<<16 | g<<8 | b)
#define MAKE_RGB_STATUS(r, g, b) ((r*ledStatusLevel)<<16 | (g*ledStatusLevel)<<8 | (b*ledStatusLevel))

constexpr uint32_t blackColor = MAKE_RGB_STATUS(0, 0, 0);
constexpr uint32_t flashColor = MAKE_RGB_STATUS(1, 0, 1);
constexpr uint32_t startColor = MAKE_RGB_STATUS(0, 1, 0);
constexpr uint32_t connectingColor = MAKE_RGB_STATUS(0, 0, 1);
constexpr uint32_t offColor = MAKE_RGB_STATUS(0, 1, 1);
constexpr uint32_t sendColor = MAKE_RGB_STATUS(1, 1, 1);

uint32_t makeFlameColor(float level) {
	constexpr uint8_t ledBright = 30;
	uint8_t rL = ledBright*6/10;
	uint8_t gL = ledBright*6/10;
	uint8_t rH = ledBright;
	uint8_t gH = 0;
	uint8_t r = rL + (rH - rL) * level;
	uint8_t g = gL + (gH - gL) * level;

	return MAKE_RGB(r, g, 0);
}

//////////////////////////////////////////////

Adafruit_NeoPixel indicator(1, indicatorDataPin);
uint32_t currentIndicatorColor = 0xFFFFFFFF;

void setIndicator(uint32_t color) {
	if (color != currentIndicatorColor) {
		currentIndicatorColor = color;

		if (color & whiteColorFlag) {
			color &= 0xFF;
			indicator.setPixelColor(0, color, color, color);
		}
		else {
			indicator.setPixelColor(0, color>>16, (color >> 8) & 0xFF, color & 0xFF);
		}
		indicator.show();
	}
}

void setIndicator8(uint8_t color8) {
	setIndicator(whiteColorFlag | color8);
}

void flashIndicator(uint32_t color, uint16_t count, uint16_t period) {
	for (auto i=0; i<count; i++) {
		setIndicator(color);
		delay(period/4);
		setIndicator(0);
		delay(period*3/4);
	}
}

void updateIndicator(hardwareState state, float valvePosition) {
	uint32_t color;

	if (!wifiConnected && ((millis()%600) < 200)) {
		color = connectingColor;
	}
	else if ((millis64()<lastSendCommandTime+200) && ((millis()%200) < 50)) {
		color = sendColor;
	}
	else if (state == stateOff) {
		color = offColor;
	}
	else if ((state == stateIgniting || state == stateExtinguishing) && ((millis()%200) < 30)) {
		color = blackColor;
	}
	else {
		color = makeFlameColor((valvePosition-valveMinValue)/(valveMaxValue-valveMinValue));
	}

	setIndicator(color);
}

//////////////////////////////////////////////

rmt_item32_t pilotMessage[messageLength];
rmt_item32_t upMessage[messageLength];
rmt_item32_t downMessage[messageLength];

void encodeBit(rmt_item32_t* item, bool bit) {
	item->duration0 = radioPeriodUS * (bit ? 2 : 1);
	item->level0 = true;
	item->duration1 = radioPeriodUS * (bit ? 1 : 2);
	item->level1 = false;
}

rmt_item32_t* encodeBits(rmt_item32_t* item, uint32_t bits, uint8_t length) {
	if (length) {
		uint32_t mask = 1 << (length-1);

		for (auto i=0; i<length; i++) {
			encodeBit(item++, (bits & mask) != 0);
			mask >>= 1;
		}
	}
	return item;
}

void encodeCommand(rmt_item32_t* item, uint32_t cmd, uint32_t address = remoteAddress) {
	item = encodeBits(item, address, addressLength);
	item = encodeBits(item, cmd, cmdLength);
}

// Adafruit's NeoPixel driver steps on channel 0, use channel 1
constexpr rmt_channel_t rmtChannel = (rmt_channel_t)1;
extern bool rmt_reserved_channels[];

void initRadio() {
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX((gpio_num_t)radioDataPin, rmtChannel);

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(rmtChannel, 0, 0));

	// mark our channel used in the Adafruit driver.
	rmt_reserved_channels[rmtChannel] = true;

	encodeCommand(pilotMessage, cmdPilot);
	encodeCommand(upMessage, cmdUp);
	encodeCommand(downMessage, cmdDown);
}

void sendMessage(rmt_item32_t* message) {
	while (millis64() < dontSendBeforeTime);

	lastSendCommandTime = millis64();
	dontSendBeforeTime = lastSendCommandTime + packetDurationMS;

    ESP_ERROR_CHECK(rmt_write_items(rmtChannel, message, messageLength, false));
}

void sendCommand(CmdCode cmd) {
	switch (cmd) {
		case cmdPilot:	sendMessage(pilotMessage);		break;
		case cmdDown:	sendMessage(downMessage);		break;
		case cmdUp:		sendMessage(upMessage);			break;
	}
}

//////////////////////////////////////////////

struct Fireplace : Service::Fan {
	SpanCharacteristic *_onOff = NULL;
	SpanCharacteristic *_level = NULL;

	hardwareState _state = stateOff;
	float _currentValvePosition = 0;

	uint64_t _operationStartTime;
	uint64_t _operationEndTime;
	float _valveStartPosition = 0;
	float _valveEndPosition = 0;
	uint16_t _startStopCount = 0;

	float _overrideLevel = -1;

	Fireplace() : Service::Fan() {
		_onOff = new Characteristic::Active(false);
		_level = new Characteristic::RotationSpeed(valveMaxValue);
		_level->setRange(valveMinValue, valveMaxValue, valveStepValue);
	}

	boolean update() {
		uint64_t time = millis64();
		bool onOffUpdated = _onOff->updated();
		bool onOffValue = _onOff->getNewVal()>0;
		bool levelUpdated = _level->updated();
		float levelValue = _level->getNewVal<float>();
		bool ignite = false;
		bool extinguish = false;
		bool changeLevel = false;

		if (onOffUpdated && onOffValue && _state==stateOff) {
			ignite = true;
		}
		else if (onOffUpdated && !onOffValue && _state!=stateOff && _state!=stateExtinguishing) {
			extinguish = true;
		}
		else if (levelUpdated && _state!=stateIgniting && _state!=stateExtinguishing) {
			if (_state == stateLitIdle || _state == stateIncreasing || _state == stateDecreasing) {
				changeLevel = true;
			}
		}

		if (levelUpdated && !changeLevel && !ignite && !extinguish) {
			_overrideLevel = _level->getVal<float>();
		}

		if (ignite) {
			SerPrintf("Ignite...\n");
			_currentValvePosition = valveMinValue;
			_state = stateIgniting;
			_startStopCount = 10;
			_valveStartPosition = _currentValvePosition;
			_valveEndPosition = valveMaxValue;
			_operationStartTime = time;
			_operationEndTime = time + igniteTimeMS;
		}
		else if (extinguish) {
			SerPrintf("Extinguish...\n");
			_state = stateExtinguishing;
			_startStopCount = 5;
			_valveStartPosition = _currentValvePosition;
			_valveEndPosition = 10;
			_operationStartTime = time;
			_operationEndTime = time + extinguishTimeMS;
		}
		else if (changeLevel) {
			_valveStartPosition = _currentValvePosition;
			_valveEndPosition = levelValue;
			_operationStartTime = time;
			_operationEndTime = time + (abs(_valveEndPosition-_valveStartPosition) / (valveMaxValue-valveMinValue) * fullRangeValveTimeMS);
			_state = (_valveEndPosition > _valveStartPosition) ? stateIncreasing : stateDecreasing;
			SerPrintf("Changing level from %0.1f%% to %0.1f%% over %dmS.\n", _valveStartPosition, _valveEndPosition, (int)(_operationEndTime-_operationStartTime));
		}

		return true;
	}

	void loop() {
		uint64_t time = millis64();

		if (_state == stateIncreasing || _state == stateDecreasing || _state == stateIgniting || _state == stateExtinguishing) {
			_currentValvePosition = _valveStartPosition + (time - _operationStartTime) * (_valveEndPosition-_valveStartPosition) / (_operationEndTime - _operationStartTime);
			_currentValvePosition = max(valveMinValue, min(valveMaxValue, _currentValvePosition));
			// SerPrintf("Valve: %0.1f%%\n", _currentValvePosition);
		}

		if (_state == stateIgniting) {
			_overrideLevel = 100;
		}

		if ((_state == stateIncreasing || _state == stateDecreasing) && (time >= _operationEndTime)) {
			_state = stateLitIdle;
			_currentValvePosition = _valveEndPosition;
			SerPrintf("Change done.\n");
		}
		else if (_state == stateIgniting && time >= _operationEndTime) {
			_state = stateLitIdle;
			_currentValvePosition = valveMaxValue;
			SerPrintf("Ignited.\n");
		}
		else if (_state == stateExtinguishing && time >= _operationEndTime) {
			_state = stateOff;
			SerPrintf("Extinguished.\n");
		}

		bool onOffShouldBe = _state!=stateOff && _state!=stateExtinguishing;
		if (_onOff->getVal() != onOffShouldBe) {
			SerPrintf("Force onoOff to: %d\n", onOffShouldBe);
			_onOff->setVal(onOffShouldBe);
		}

		if (_overrideLevel != -1) {
			if (_overrideLevel != _level->getVal<float>()) {
				SerPrintf("Force level to: %0.1f%%\n", _overrideLevel);
				_level->setVal(_overrideLevel);
			}
			_overrideLevel = -1;
		}

		if (time >= dontSendBeforeTime) {
			if (_state == stateIncreasing) {
				sendCommand(cmdUp);
				// SerPrintf("Send: Up\n");
			}
			else if (_state == stateDecreasing) {
				sendCommand(cmdDown);
				// SerPrintf("Send: Down\n");
			}
			else if (_state == stateIgniting && _startStopCount) {
				sendCommand(cmdPilot);
				_startStopCount -= 1;
				SerPrintf("Send: Pilot\n");
			}
			else if (_state == stateExtinguishing && _startStopCount) {
				sendCommand(cmdPilot);
				_startStopCount -= 1;
				SerPrintf("Send: Pilot\n");
			}
		}

		updateIndicator(_state, _currentValvePosition);
	}
};

//////////////////////////////////////////////

Fireplace* fireplace;

void createDevices() {
	SPAN_ACCESSORY();   // create Bridge

	SPAN_ACCESSORY(accessoryName);
		fireplace = new Fireplace();
}

//////////////////////////////////////////////

void statusChanged(HS_STATUS status) {
	if (status == HS_WIFI_CONNECTING) {
		wifiConnected = false;
		if (hadWifiConnection) {
			SerPrintf("Lost WIFI Connection...\n");
		}
	}
}

void wifiReady() {
	wifiConnected = true;
	hadWifiConnection = true;
	SerPrintf("WIFI: Ready.\n");
}

//////////////////////////////////////////////

void setup() {
	pinMode(indicatorPowerPin, OUTPUT);
	digitalWrite(indicatorPowerPin, HIGH);

	indicator.begin();

	#ifdef DEBUG
	flashIndicator(flashColor, 20, 200);
	setIndicator(startColor);
	#endif

	SerBegin(115200);
	SerPrintf("Home-Fire Startup\n");

	SerPrintf("Init HomeSpan...\n");
	homeSpan.setSketchVersion(versionString);
	homeSpan.setWifiCallback(wifiReady);
	homeSpan.setStatusCallback(statusChanged);
	homeSpan.begin(Category::Bridges, displayName, DEFAULT_HOST_NAME, modelName);

	SerPrintf("Create devices...\n");
	createDevices();

	SerPrintf("Setup Radio...\n");
	initRadio();

	SerPrintf("Init complete.\n");

	SerPrintf("Wait for WiFi...\n");
	setIndicator(connectingColor);
}

void loop() {
	homeSpan.poll();

	// HomeSpan does not call the wifi callback after the first connection
	// We do that manually here
	if (hadWifiConnection && !wifiConnected) {
		static uint64_t nextWifiCheck = 0;
		uint64_t time = millis64();

		if (time > nextWifiCheck) {
			if (WiFi.status()==WL_CONNECTED) {
				wifiReady();
			}

			nextWifiCheck = time + 500;
		}
	}
}


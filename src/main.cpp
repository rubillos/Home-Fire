#include <Arduino.h>

#include "HomeSpan.h" 
#include "Wire.h"
#include <Adafruit_NeoPixel.h>

#include <WiFi.h>
// #include <driver/rmt.h>
#include "esp32-hal.h"

//////////////////////////////////////////////

constexpr const char* versionString = "v0.1";

constexpr const char* displayName = "Fireplace-Controller";
constexpr const char* modelName = "Fireplace-Controller-ESP32";

constexpr uint8_t indicatorDataPin = PIN_NEOPIXEL;
constexpr uint8_t indicatorPowerPin = NEOPIXEL_POWER;

constexpr uint8_t radioDataPin = MISO;

//////////////////////////////////////////////

bool wifiConnected = false;

constexpr uint8_t ledStatusLevel = 0x10;

#define MAKE_RGB(r, g, b) (r<<16 | g<<8 | b)
constexpr uint32_t flashColor = MAKE_RGB(ledStatusLevel, 0, ledStatusLevel);
constexpr uint32_t startColor = MAKE_RGB(0, ledStatusLevel, 0);
constexpr uint32_t connectingColor = MAKE_RGB(0, 0, ledStatusLevel);
constexpr uint32_t offColor = MAKE_RGB(0, ledStatusLevel, ledStatusLevel);
constexpr uint32_t sendColor = MAKE_RGB(ledStatusLevel, ledStatusLevel, ledStatusLevel);

uint32_t currentIndicatorColor = 0xFFFFFFFF;

constexpr uint32_t whiteColorFlag = 1 << 24;

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
constexpr uint32_t cmdPilot = 0B110011;
constexpr uint32_t cmdUp = 0B111011;
constexpr uint32_t cmdDown = 0B000000;

uint64_t lastSendCommandTime = 0;
uint64_t dontSendBeforeTime = 0;

//////////////////////////////////////////////

Adafruit_NeoPixel indicator(1, indicatorDataPin);

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
	else {
		color = makeFlameColor(valvePosition);
	}

	setIndicator(color);
}

//////////////////////////////////////////////

constexpr uint16_t messageLength = addressLength + cmdLength;
constexpr uint32_t messageTimeUS = messageLength * 3 * radioPeriodUS;
constexpr uint32_t messageHoldMS = ((messageTimeUS+999)/1000) + packetGapTimeMS;

rmt_data_t pilotMessage[messageLength];
rmt_data_t upMessage[messageLength];
rmt_data_t downMessage[messageLength];

void encodeBit(rmt_data_t* item, bool bit) {
	item->duration0 = radioPeriodUS * (bit ? 2 : 1);
	item->level0 = true;
	item->duration1 = radioPeriodUS * (bit ? 1 : 2);
	item->level1 = false;
}

void encodeDelay(rmt_data_t* item, uint16_t delay) {
	item->duration0 = delay;
	item->level0 = false;
	item->duration1 = 0;
	item->level1 = false;
}

rmt_data_t* encodeBits(rmt_data_t* item, uint32_t bits, uint8_t length) {
	if (length) {
		uint32_t mask = 1 << (length-1);

		for (auto i=0; i<length; i++) {
			encodeBit(item++, (bits & mask) != 0);
			mask >>= 1;
		}
	}
	return item;
}

void encodeCommand(rmt_data_t* item, uint32_t cmd, uint32_t address = remoteAddress) {
	item = encodeBits(item, address, addressLength);
	item = encodeBits(item, cmd, cmdLength);
}

rmt_obj_t* rmt_send = NULL;

void initRMT() {
    if ((rmt_send = rmtInit(radioDataPin, RMT_TX_MODE, RMT_MEM_64)) == NULL)
    {
        Serial.println("init rmt sender failed\n");
    }

    float realTick = rmtSetTick(rmt_send, 1000);
    SerPrintf("real tick set to: %fns\n", realTick);

	encodeCommand(pilotMessage, cmdPilot);
	encodeCommand(upMessage, cmdUp);
	encodeCommand(downMessage, cmdDown);
}

void sendMessage(rmt_data_t* message) {
	while (millis64() < dontSendBeforeTime);

	lastSendCommandTime = millis64();
	rmtWrite(rmt_send, message, messageLength);
	dontSendBeforeTime = lastSendCommandTime + messageHoldMS;
}

void sendBit(bool bit) {
	digitalWrite(radioDataPin, HIGH);
	delayMicroseconds(radioPeriodUS * (bit ? 2 : 1));
	digitalWrite(radioDataPin, LOW);
	delayMicroseconds(radioPeriodUS * (bit ? 1 : 2));
}

void sendBits(uint32_t bits, uint8_t length) {
	if (length) {
		uint32_t mask = 1 << (length-1);

		for (auto i=0; i<length; i++) {
			sendBit((bits & mask) != 0);
			mask >>= 1;
		}
	}
}

void sendCommand(uint32_t cmd, uint32_t address = remoteAddress) {
	while (millis64() < dontSendBeforeTime);

	lastSendCommandTime = millis64();

	sendBits(address, addressLength);
	sendBits(cmd, cmdLength);
	
	dontSendBeforeTime = millis64() + packetGapTimeMS;
}

//////////////////////////////////////////////

void sendPilot() {
	sendCommand(cmdPilot);
	// sendMessage(pilotMessage);
}

void sendUp() {
	sendCommand(cmdUp);
	// sendMessage(upMessage);
}

void sendDown() {
	sendCommand(cmdDown);
	// sendMessage(downMessage);
}

void initRadio() {
	pinMode(radioDataPin, OUTPUT);
	digitalWrite(radioDataPin, LOW);
	// initRMT();
}

//////////////////////////////////////////////

struct FireplaceFan : Service::Fan {
	SpanCharacteristic *_onOff = NULL;
	SpanCharacteristic *_level = NULL;

	hardwareState _state = stateOff;

	uint64_t _operationStartTime;
	uint64_t _operationEndTime;

	float _startValvePosition = 0;
	float _endValvePosition = 0;

	uint16_t _startStopCount = 0;

	float _currentValvePosition = 0;

	bool _setOnOff = false;
	bool _newOnOffValue = false;

	bool _setLevel = false;
	float _newLevelValue = 0;

	FireplaceFan() : Service::Fan() {
		_onOff = new Characteristic::Active(false);
		_level = new Characteristic::RotationSpeed(valveMaxValue);
		_level->setRange(valveMinValue, valveMaxValue, valveStepValue);
	}

	boolean update() {
		uint64_t time = millis64();
		bool onOffUpdate = _onOff->updated();
		bool onOffValue = _onOff->getNewVal()>0;
		bool levelUpdate = _level->updated();
		float levelValue = _level->getNewVal<float>();
		bool ignite = false;
		bool extinguish = false;
		bool changeLevel = false;

		if (onOffUpdate && onOffValue && _state==stateOff) {
			if (_state == stateExtinguishing) {
				_setOnOff = true;
				_newOnOffValue = false;
			}
			else {
				ignite = true;
			}
		}
		else if (onOffUpdate && !onOffValue && _state!=stateOff) {
			if (_state == stateIgniting) {
				_setOnOff = false;
				_newOnOffValue = true;
				if (levelValue != valveMaxValue) {
					_setLevel = true;
					_newLevelValue = valveMaxValue;
				}
			}
			else {
				extinguish = true;
			}
		}
		else if (levelUpdate) {
			if (_state == stateLitIdle || _state == stateIncreasing || _state == stateDecreasing) {
				changeLevel = true;
			}
		}

		if (levelUpdate && !changeLevel) {
			_setLevel = true;
			_newLevelValue = _level->getVal<float>();
		}

		if (ignite) {  // ignite
			SerPrintf("Ignite...\n");
			if (levelValue != valveMaxValue) {
				_setLevel = true;
				_newLevelValue = valveMaxValue;
			}
			_currentValvePosition = valveMinValue;
			_state = stateIgniting;
			_startStopCount = 10;
			_startValvePosition = _currentValvePosition;
			_endValvePosition = valveMaxValue;
			_operationStartTime = time;
			_operationEndTime = time + igniteTimeMS;
		}
		else if (extinguish) {  // extinguish
			SerPrintf("Extinguish...\n");
			_state = stateExtinguishing;
			_startStopCount = 5;
			_startValvePosition = _currentValvePosition;
			_endValvePosition = 10;
			_operationStartTime = time;
			_operationEndTime = time + extinguishTimeMS;
		}
		else if (changeLevel) {
			_startValvePosition = _currentValvePosition;
			_endValvePosition = levelValue;
			_operationStartTime = time;
			_operationEndTime = time + (abs(_endValvePosition-_startValvePosition) / (valveMaxValue-valveMinValue) * fullRangeValveTimeMS);
			_state = (_endValvePosition > _startValvePosition) ? stateIncreasing : stateDecreasing;
			SerPrintf("Changing level from %0.1f%% to %0.1f%% over %dmS.\n", _startValvePosition, _endValvePosition, (int)(_operationEndTime-_operationStartTime));
		}

		return true;
	}

	void loop() {
		uint64_t time = millis64();

		if (_state == stateIncreasing || _state == stateDecreasing || _state == stateIgniting || _state == stateExtinguishing) {
			_currentValvePosition = _startValvePosition + (time - _operationStartTime) * (_endValvePosition-_startValvePosition) / (_operationEndTime - _operationStartTime);
			_currentValvePosition = max(valveMinValue, min(valveMaxValue, _currentValvePosition));
			// SerPrintf("Valve: %0.1f%%\n", _currentValvePosition);
		}

		if (_state == stateIncreasing || _state == stateDecreasing) {
			if (time >= _operationEndTime) {
				_currentValvePosition = _endValvePosition;
				_state = stateLitIdle;
				SerPrintf("Change done.\n");
			}
		}
		else if (_state == stateIgniting && time >= _operationEndTime) {
			_state = stateLitIdle;
			_currentValvePosition = valveMaxValue;
			if (_onOff->getVal() != 1) {
				_onOff->setVal(1);
				_setOnOff = false;
			}
			SerPrintf("Ignited.\n");
		}
		else if (_state == stateExtinguishing && time >= _operationEndTime) {
			_state = stateOff;
			if (_onOff->getVal() != 0) {
				_onOff->setVal(0);
				_setOnOff = false;
			}
			SerPrintf("Extinguished.\n");
		}

		if (_setOnOff) {
			SerPrintf("Reset onoOff to: %d\n", _newOnOffValue);
			_onOff->setVal(_newOnOffValue);
			_setOnOff = false;
		}
		if (_setLevel) {
			SerPrintf("Reset level to: %0.1f%%\n", _newLevelValue);
			_level->setVal(_newLevelValue);
			_setLevel = false;
		}

		if (time >= dontSendBeforeTime) {
			if (_state == stateIncreasing) {
				sendUp();
				// SerPrintf("Up\n");
		 }
			else if (_state == stateDecreasing) {
				sendDown();
				// SerPrintf("Down\n");
			}
			else if (_state == stateIgniting && _startStopCount) {
				sendPilot();
				_startStopCount -= 1;
				SerPrintf("Pilot\n");
			}
			else if (_state == stateExtinguishing && _startStopCount) {
				sendPilot();
				_startStopCount -= 1;
				SerPrintf("Pilot\n");
			}
		}

		updateIndicator(_state, _currentValvePosition / valveMaxValue);
	}
};

//////////////////////////////////////////////

FireplaceFan* fireplace;

void createDevices() {
	SPAN_ACCESSORY();   // create Bridge

	SPAN_ACCESSORY("Fireplace");
		fireplace = new FireplaceFan();
}

//////////////////////////////////////////////

void addCommands() {
	// new SpanUserCommand('s',"show CPU stats", cmdShowCPUStats);
}

//////////////////////////////////////////////

void statusChanged(HS_STATUS status) {
	if (status == HS_WIFI_CONNECTING) {
		wifiConnected = false;
		SerPrintf("Lost WIFI Connection...\n");
	}
}

void wifiReady() {
	wifiConnected = true;
	SerPrintf("WIFI: Ready.\n");
}

//////////////////////////////////////////////

void setup() {
	pinMode(indicatorPowerPin, OUTPUT);
	digitalWrite(indicatorPowerPin, HIGH);

	indicator.begin();
	flashIndicator(flashColor, 20, 200);
	setIndicator(startColor);

	SerBegin(115200);
	SerPrintf("Tim-Fire Startup\n");

	SerPrintf("Init HomeSpan\n");
	homeSpan.setSketchVersion(versionString);
	homeSpan.setWifiCallback(wifiReady);
	homeSpan.setStatusCallback(statusChanged);
	homeSpan.begin(Category::Bridges, displayName, DEFAULT_HOST_NAME, modelName);

	SerPrintf("Create devices\n");
	createDevices();
	addCommands();

	SerPrintf("Setup Radio\n");
	initRadio();

	SerPrintf("Wait for WiFi...\n");
	setIndicator(connectingColor);

	SerPrintf("Init complete.\n");
}

void loop() {
	homeSpan.poll();

	if (!wifiConnected) {
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


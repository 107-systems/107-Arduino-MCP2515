<a href="https://uavcan.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/uavcan.svg" width="20%"></a>
`107-Arduino-MCP2515`
=====================
[![Arduino Library Badge](https://www.ardu-badge.com/badge/107-Arduino-MCP2515.svg?)](https://www.ardu-badge.com/107-Arduino-MCP2515)
[![Compile Examples](https://github.com/107-systems/107-Arduino-MCP2515/workflows/Compile%20Examples/badge.svg)](https://github.com/107-systems/107-Arduino-MCP2515/actions?workflow=Compile+Examples)
[![Check Arduino](https://github.com/107-systems/107-Arduino-MCP2515/actions/workflows/check-arduino.yml/badge.svg)](https://github.com/107-systems/107-Arduino-MCP2515/actions/workflows/check-arduino.yml)
[![Check keywords.txt](https://github.com/107-systems/107-Arduino-MCP2515/actions/workflows/check-keywords-txt.yml/badge.svg)](https://github.com/107-systems/107-Arduino-MCP2515/actions/workflows/check-keywords-txt.yml)
[![General Formatting Checks](https://github.com/107-systems/107-Arduino-MCP2515/workflows/General%20Formatting%20Checks/badge.svg)](https://github.com/107-systems/107-Arduino-MCP2515/actions?workflow=General+Formatting+Checks)
[![Spell Check](https://github.com/107-systems/107-Arduino-MCP2515/workflows/Spell%20Check/badge.svg)](https://github.com/107-systems/107-Arduino-MCP2515/actions?workflow=Spell+Check)

<p align="center">
  <a href="https://github.com/107-systems/107-Arduino-DroneCore"><img src="https://github.com/107-systems/.github/raw/main/logo/viper.jpg" width="40%"></a>
</p>

Arduino library for controlling the MCP2515 in order to receive/transmit CAN frames. This library is prepared to interface easily with [libcanard](https://github.com/UAVCAN/libcanard) for using [UAVCAN](https://uavcan.org/) on Arduino via [107-Arduino-UAVCAN](https://github.com/107-systems/107-Arduino-UAVCAN).

This library works for
* [ArduinoCore-samd](https://github.com/arduino/ArduinoCore-samd): [`Arduino Zero`](https://store.arduino.cc/arduino-zero), [`MKR 1000`](https://store.arduino.cc/arduino-mkr1000-wifi), [`MKR WiFi 1010`](https://store.arduino.cc/arduino-mkr-wifi-1010), [`Nano 33 IoT`](https://store.arduino.cc/arduino-nano-33-iot), [`MKR GSM 1400`](https://store.arduino.cc/arduino-mkr-gsm-1400-1415), [`MKR NB 1500`](https://store.arduino.cc/arduino-mkr-nb-1500-1413), [`MKR WAN 1300/1310`](https://store.arduino.cc/mkr-wan-1310) :heavy_check_mark:
* [ArduinoCore-mbed](https://github.com/arduino/ArduinoCore-mbed): [`Portenta H7`](https://store.arduino.cc/portenta-h7), [`Nano 33 BLE`](https://store.arduino.cc/arduino-nano-33-ble), [`Nano RP2040 Connect`](https://store.arduino.cc/nano-rp2040-connect), [`Edge Control`](https://store.arduino.cc/edge-control) :heavy_check_mark:
* [arduino-esp32](https://github.com/espressif/arduino-esp32): `ESP32 Dev Module`, `ESP32 Wrover Module`, ... :heavy_check_mark:
* [arduino-pico](https://github.com/earlephilhower/arduino-pico): [`Raspberry Pi Pico`](https://www.raspberrypi.org/products/raspberry-pi-pico), `Adafruit Feather RP2040`, ... :heavy_check_mark:

## Example
```C++
#include <SPI.h>
#include <ArduinoMCP2515.h>
/* ... */
static int const MKRCAN_MCP2515_CS_PIN  = 3;
static int const MKRCAN_MCP2515_INT_PIN = 7;
/* ... */
void onReceiveBufferFull(uint32_t const timestamp_us, uint32_t const id, uint8_t const * data, uint8_t const len)
{
  Serial.println(id, HEX);
}
void onTransmitBufferEmpty(ArduinoMCP2515 * this_ptr)
{
  /* You can use this callback to refill the transmit buffer via this_ptr->transmit(...) */
}
/* ... */
ArduinoMCP2515 mcp2515([](){ digitalWrite(MKRCAN_MCP2515_CS_PIN, LOW); },
                       [](){ digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH); },
                       [](uint8_t const d) -> uint8_t { return SPI.transfer(d); },
                       micros,
                       onReceiveBufferFull,
                       onTransmitBufferEmpty);
/* ... */
void setup()
{
  Serial.begin(9600);
  while(!Serial) { }

  SPI.begin();
  pinMode(MKRCAN_MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);

  pinMode(MKRCAN_MCP2515_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MKRCAN_MCP2515_INT_PIN), [](){ mcp2515.onExternalEventHandler(); }, FALLING);

  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_250kBPS_16MHZ);
  mcp2515.setNormalMode();
}

void loop()
{
  uint8_t const data[8] = {0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF};
  mcp2515.transmit(1 /* id */, data, 8 /* len */);
  delay(100);
}
```

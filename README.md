`107-Arduino-MCP2515`
=====================
[![Compile Examples](https://github.com/107-systems/107-Arduino-MCP2515/workflows/Compile%20Examples/badge.svg)](https://github.com/107-systems/107-Arduino-MCP2515/actions?workflow=Compile+Examples)
[![Extra Library Checks](https://github.com/107-systems/107-Arduino-MCP2515/workflows/Extra%20Library%20Checks/badge.svg)](https://github.com/107-systems/107-Arduino-MCP2515/actions?workflow=Extra+Library+Checks)
[![General Formatting Checks](https://github.com/107-systems/107-Arduino-MCP2515/workflows/General%20Formatting%20Checks/badge.svg)](https://github.com/107-systems/107-Arduino-MCP2515/actions?workflow=General+Formatting+Checks)
[![Spell Check](https://github.com/107-systems/107-Arduino-MCP2515/workflows/Spell%20Check/badge.svg)](https://github.com/107-systems/107-Arduino-MCP2515/actions?workflow=Spell+Check)

<p align="center">
  <a href="https://github.com/107-systems/107-Arduino-Viper"><img src="extras/logo/viper-logo.jpg" width="40%"></a>
</p>

Arduino library for controlling the MCP2515 in order to receive/transmit CAN frames.

## Example
```C++
#include <SPI.h>
#include <ArduinoMCP2515.h>
/* ... */
static int const MKRCAN_MCP2515_CS_PIN  = 3;
static int const MKRCAN_MCP2515_INT_PIN = 7;
/* ... */
void    spi_select            ()                                                           { digitalWrite(MKRCAN_MCP2515_CS_PIN, LOW); }
void    spi_deselect          ()                                                           { digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH); }
uint8_t spi_transfer          (uint8_t const data)                                         { return SPI.transfer(data); }
void    onReceiveBufferFull   (uint32_t const id, uint8_t const * data, uint8_t const len) { Serial.println(id, HEX); }
void    onTransmitBufferEmpty (ArduinoMCP2515 * this_ptr)                                  { /* You can use this callback to refill the transmit buffer via this_ptr->transmit(...) */ }
void    onMCP2515ExternalEvent()                                                           { mcp2515.onExternalEventHandler(); }
/* ... */
ArduinoMCP2515 mcp2515(spi_select, spi_deselect, spi_transfer, micros, onReceiveBufferFull, onTransmitBufferEmpty);
/* ... */
void setup() {
  Serial.begin(9600);
  while(!Serial) { }

  SPI.begin();
  pinMode(MKRCAN_MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);

  pinMode(MKRCAN_MCP2515_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MKRCAN_MCP2515_INT_PIN), onMCP2515ExternalEvent, FALLING);

  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_250kBPS);
  mcp2515.setNormalMode();
}

void loop() {
  uint8_t const data[8] = {0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF};
  mcp2515.transmit(1 /* id */, data, 8 /* len */);
  delay(100);
}
```

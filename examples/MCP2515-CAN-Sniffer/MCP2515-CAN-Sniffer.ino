/**
 * @brief   This example enables the listen-only mode and prints all received CAN frames to the serial interface.
 * @author  Alexander Entinger, MSc / LXRobotics GmbH
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <SPI.h>

#include <ArduinoMCP2515.h>

#undef max
#undef min
#include <algorithm>

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static int const MKRCAN_MCP2515_CS_PIN  = 3;
static int const MKRCAN_MCP2515_INT_PIN = 7;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void onReceiveBufferFull(uint32_t const, uint32_t const, uint8_t const *, uint8_t const);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

ArduinoMCP2515 mcp2515([](){ digitalWrite(MKRCAN_MCP2515_CS_PIN, LOW); },
                       [](){ digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH); },
                       [](uint8_t const d) -> uint8_t { return SPI.transfer(d); },
                       micros,
                       onReceiveBufferFull,
                       nullptr);

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(9600);
  while(!Serial) { }

  /* Setup SPI access */
  SPI.begin();
  pinMode(MKRCAN_MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);

  /* Attach interrupt handler to register MCP2515 signaled by taking INT low */
  pinMode(MKRCAN_MCP2515_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MKRCAN_MCP2515_INT_PIN), [](){ mcp2515.onExternalEventHandler(); }, FALLING);

  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_250kBPS_16MHZ);
  mcp2515.setListenOnlyMode();
}

void loop()
{

}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void onReceiveBufferFull(uint32_t const timestamp_us, uint32_t const id, uint8_t const * data, uint8_t const len)
{
  Serial.print("[ ");
  Serial.print(timestamp_us);
  Serial.print("] ");

  Serial.print("ID");
  if(id & MCP2515::CAN_EFF_BITMASK) Serial.print("(EXT)");
  if(id & MCP2515::CAN_RTR_BITMASK) Serial.print("(RTR)");
  Serial.print(" ");
  Serial.print(id, HEX);

  Serial.print(" DATA[");
  Serial.print(len);
  Serial.print("] ");
  std::for_each(data,
                data+len,
                [](uint8_t const elem) {
                  Serial.print(elem, HEX);
                  Serial.print(" ");
                });
  Serial.println();
}

/**
 * @brief   This example enables the loopback mode to test the transmission and reception of CAN frames via MCP2515 without any physical bus connection.
 * @author  Alexander Entinger, MSc / LXRobotics GmbH
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <SPI.h>
#undef max
#undef min

#include <ArduinoMCP2515.h>

#include <algorithm>

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void mcp2515_spi_select();
void mcp2515_spi_deselect();
uint8_t spi_transfer(uint8_t const data);
void mcp2515_onReceiveBufferFull(uint32_t const id, uint8_t const * data, uint8_t const len);
void mcp2515_OnTransmitBufferEmpty(ArduinoMCP2515 * this_ptr);

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

static std::array<uint32_t, 10> const TEST_ID_VECTOR{0,1,2,3,4,5,6,7,8,9};
static uint8_t                  const TEST_DATA[]   = {0xDE, 0xAD, 0xBE, 0xEF};
static uint8_t                  const TEST_DATA_LEN = sizeof(TEST_DATA)/sizeof(uint8_t);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

ArduinoMCP2515 mcp2515(mcp2515_spi_select,
                       mcp2515_spi_deselect,
                       spi_transfer,
                       mcp2515_onReceiveBufferFull,
                       mcp2515_OnTransmitBufferEmpty);

/**************************************************************************************
 * CALLBACK FUNCTIONS
 **************************************************************************************/

void onMCP2515ExternalEvent()
{
  mcp2515.onExternalEventHandler();
}

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
  attachInterrupt(digitalPinToInterrupt(MKRCAN_MCP2515_INT_PIN), onMCP2515ExternalEvent, FALLING);

  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_250kBPS);
  mcp2515.setLoopbackMode();

  std::for_each(TEST_ID_VECTOR.cbegin(),
                TEST_ID_VECTOR.cend(),
                [](uint32_t const id)
                {
                  if(!mcp2515.transmit(id, TEST_DATA, TEST_DATA_LEN)) {
                    Serial.println("ERROR TX");
                  }
                  delay(10);
                });
}

void loop()
{

}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void mcp2515_spi_select()
{
  digitalWrite(MKRCAN_MCP2515_CS_PIN, LOW);
}

void mcp2515_spi_deselect()
{
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);
}

uint8_t spi_transfer(uint8_t const data)
{
  return SPI.transfer(data);
}

void mcp2515_onReceiveBufferFull(uint32_t const id, uint8_t const * data, uint8_t const len)
{
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

void mcp2515_OnTransmitBufferEmpty(ArduinoMCP2515 * this_ptr)
{
  /* Serial.println("Transmit Buffer empty"); */
}

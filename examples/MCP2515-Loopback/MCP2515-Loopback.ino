/**
 * @brief   This example enables the loopback mode to test the transmission and reception of CAN frames via MCP2515 without any physical bus connection.
 * @author  Alexander Entinger, MSc / LXRobotics GmbH
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <ArduinoMCP2515.h>

#include <algorithm>

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void onCanFrameReceive(uint32_t const id, uint8_t const * data, uint8_t const len);

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

static std::vector<uint32_t> const TEST_ID_VECTOR{0,1,2,3,4,5,6,7,8,9,10};
static uint8_t               const TEST_DATA[]   = {0xDE, 0xAD, 0xBE, 0xEF};
static uint8_t               const TEST_DATA_LEN = sizeof(TEST_DATA)/sizeof(uint8_t);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

ArduinoMCP2515 MCP2515(MKRCAN_MCP2515_CS_PIN, MKRCAN_MCP2515_INT_PIN, onCanFrameReceive);

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  MCP2515.begin();
  MCP2515.setBitRate(CanBitRate::BR_250kBPS);
  MCP2515.setLoopbackMode();
  
  std::for_each(TEST_ID_VECTOR.cbegin(),
                TEST_ID_VECTOR.cend(),
                [](uint32_t const id)
                {
                  if(!MCP2515.transmit(id, TEST_DATA, TEST_DATA_LEN)) {
                    Serial.println("MCP2515.transmit() failed - transmit buffer full");
                  }
                  delay(500);
                });
}

void loop()
{
  
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void onCanFrameReceive(uint32_t const id, uint8_t const * data, uint8_t const len)
{
  Serial.print("ID ");
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

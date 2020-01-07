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

void onCanFrameReceive(uint32_t const id, std::vector<uint8_t> const & data);

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

static std::vector<uint32_t> const TEST_ID_VECTOR{0,1,2,3,4,5,6,7,8,9,10};
static std::vector<uint8_t>  const TEST_DATA     {0xDE, 0xAD, 0xBE, 0xEF};

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

ArduinoMCP2515 MCP2515(MKRCAN_MCP2515_CS_PIN, MKRCAN_MCP2515_INT_PIN, onCanFrameReceive);

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  std::for_each(TEST_ID_VECTOR.cbegin(),
                TEST_ID_VECTOR.cend(),
                [](uint32_t const id)
                {
                  if(!MCP2515.transmit(id, TEST_DATA)) {
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

void onCanFrameReceive(uint32_t const id, std::vector<uint8_t> const & data)
{
  Serial.println(toStr(id, data).c_str());
}

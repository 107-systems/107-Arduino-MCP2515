/**
 * @brief   This example enables the listen-only mode and prints all received CAN frames to the serial interface.
 * @author  Alexander Entinger, MSc / LXRobotics GmbH
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <SPI.h>

#include <107-Arduino-MCP2515.h>

#undef max
#undef min
#include <algorithm>

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static int         const MKRCAN_MCP2515_CS_PIN  = 10;
static int         const MKRCAN_MCP2515_INT_PIN = 3;
static SPISettings const MCP2515x_SPI_SETTING{1000000, MSBFIRST, SPI_MODE0};

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void onReceiveBufferFull(uint32_t const, uint32_t const, uint8_t const *, uint8_t const);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

ArduinoMCP2515 mcp2515([]() { digitalWrite(MKRCAN_MCP2515_CS_PIN, LOW); },
                       []() { digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH); },
                       [](uint8_t const d) { return SPI.transfer(d); },
                       micros,
                       onReceiveBufferFull,
                       nullptr);

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  //Serial.begin(9600);
  //while(!Serial) { }

  /* Setup SPI access */
  SPI.begin();
  SPI.beginTransaction(MCP2515x_SPI_SETTING);
  pinMode(MKRCAN_MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);

  /* Attach interrupt handler to register MCP2515 signaled by taking INT low */
  pinMode(MKRCAN_MCP2515_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MKRCAN_MCP2515_INT_PIN), [](){ mcp2515.onExternalEventHandler(); }, LOW);

  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_250kBPS_16MHZ); // CAN bit rate and MCP2515 clock speed

  /* Enable filtering of CAN messages. The MCP2515 has two message receive buffers.
   * For each message receive buffer a filter mask can be defined. In addition, for
   * each message receive buffer one or more filters can be defined. For RXMB0 you
   * can have a total of 2 filters, for RXMB1 you can have a total of 4 filters.
   */
  /* RXB0 filter configuration: All messages with an ID 0x01FFFFxx shall pass. */
  uint32_t const RXMB0_MASK = 0x000000FF;
  size_t const RXMB0_FILTER_SIZE = 2;
  uint32_t const RXMB0_FILTER[RXMB0_FILTER_SIZE] = {
    0x01FFFF0F,
    0x01FFFFF0
  };
  /* Note: To satisfy the demand for filtering all messages with ID 0x01FFFFxx a
   * single filter with a value of 0x01FFFFFF would suffice, but then we could not
   * really demo the usage of this library.
   */
  mcp2515.enableFilter(MCP2515::RxB::RxB0, RXMB0_MASK, RXMB0_FILTER, RXMB0_FILTER_SIZE);
  /* RXB1 filter configuration: CAN messages with the following IDs shall pass:
   * - 0x00000001
   * - 0x00000002
   * - 0x00000004
   * - 0x00000008
   */
  uint32_t const RXMB1_MASK = 0x0000000F;
  size_t const RXMB1_FILTER_SIZE = 4;
  uint32_t const RXMB1_FILTER[RXMB1_FILTER_SIZE] = {
    0x00000001,
    0x00000002,
    0x00000004,
    0x00000008
  };
  mcp2515.enableFilter(MCP2515::RxB::RxB1, RXMB1_MASK, RXMB1_FILTER, RXMB1_FILTER_SIZE);

  /* Leave configuration mode and start listening. */
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
  /*
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
  */
}

/**
 * @brief   This example enables the listen-only mode and prints all received CAN frames to the serial interface.
 * @author  Alexander Entinger, MSc / LXRobotics GmbH
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <SPI.h>

#include <107-Arduino-MCP2515.h>

#include "api/HardwareCAN.h"

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static int         const MKRCAN_MCP2515_CS_PIN  = 3;
static int         const MKRCAN_MCP2515_INT_PIN = 7;
static SPISettings const MCP2515x_SPI_SETTING{10*1000*1000UL, MSBFIRST, SPI_MODE0};

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static uint32_t const CAN_ID = 0x20;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void onReceiveBufferFull(uint32_t const, uint32_t const, uint8_t const *, uint8_t const);

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class MCP2515_HardwareCAN : public arduino::HardwareCAN
{
private:
  ArduinoMCP2515 & _mcp2515;
  CanMsgRingbuffer _can_rx_buf;

public:
  MCP2515_HardwareCAN(ArduinoMCP2515 & mcp2515)
  : _mcp2515{mcp2515}
  { }
  virtual ~MCP2515_HardwareCAN() { }


  virtual bool begin(CanBitRate const can_bitrate) override
  {
    _mcp2515.begin();
    _mcp2515.setBitRate(MCP2515::CanBitRate::BR_250kBPS_16MHZ);
    _mcp2515.setNormalMode();
    return true;
  }
  virtual void end() override { }

  virtual int write(CanMsg const &msg) override
  {
    return _mcp2515.transmit(msg.id, msg.data, msg.data_length);
  }

  virtual size_t available() override
  {
    return _can_rx_buf.available();
  }

  virtual CanMsg read() override
  {
    return _can_rx_buf.dequeue();
  }

  void onCanMsgReceived(uint32_t const id, uint8_t const * data, uint8_t const len)
  {
    /* Extract the received CAN message. */
    CanMsg const msg
      (
        id,
        len,
        data
      );
    /* Store the received CAN message in the receive buffer. */
    _can_rx_buf.enqueue(msg);
  }
};

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

ArduinoMCP2515 mcp2515([]() { digitalWrite(MKRCAN_MCP2515_CS_PIN, LOW); },
                       []() { digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH); },
                       [](uint8_t const d) { return SPI.transfer(d); },
                       micros,
                       onReceiveBufferFull,
                       nullptr);

MCP2515_HardwareCAN CANx(mcp2515);

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(115200);
  while(!Serial) { }

  /* Setup SPI access */
  SPI.begin();
  SPI.beginTransaction(MCP2515x_SPI_SETTING);
  pinMode(MKRCAN_MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);

  /* Attach interrupt handler to register MCP2515 signaled by taking INT low */
  pinMode(MKRCAN_MCP2515_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MKRCAN_MCP2515_INT_PIN), [](){ mcp2515.onExternalEventHandler(); }, LOW);

  if (!CANx.begin(CanBitRate::BR_250k))
  {
    Serial.println("CAN.begin(...) failed.");
    for (;;) {}
  }
}

static uint32_t msg_cnt = 0;

void loop()
{
  /* Assemble a CAN message with the format of
   * 0xCA 0xFE 0x00 0x00 [4 byte message counter]
   */
  uint8_t const msg_data[] = {0xCA,0xFE,0,0,0,0,0,0};
  memcpy((void *)(msg_data + 4), &msg_cnt, sizeof(msg_cnt));
  CanMsg msg(CAN_ID, sizeof(msg_data), msg_data);

  /* Transmit the CAN message, capture and display an
   * error core in case of failure.
   */
  if (int const rc = CANx.write(msg); rc <= 0)
  {
    Serial.print  ("CAN.write(...) failed with error code ");
    Serial.println(rc);
    for (;;) { }
  }

  /* Increase the message counter. */
  msg_cnt++;

  /* Only send one message per second. */
  delay(1000);
}


/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void onReceiveBufferFull(uint32_t const timestamp_us, uint32_t const id, uint8_t const * data, uint8_t const len)
{
  CANx.onCanMsgReceived(id, data, len);
}

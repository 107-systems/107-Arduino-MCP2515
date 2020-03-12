/**
 * @brief   Arduino library for controlling the MCP2515 in order to receive/transmit CAN frames.
 * @author  Alexander Entinger, MSc / LXRobotics GmbH
 * @license LGPL 3.0
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <ArduinoMCP2515.h>

#include <algorithm>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace MCP2515;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static CanBitRateConfig const BIT_RATE_CONFIG_ARRAY[] =
{
  BitRate_125kBPS_16MHz,
  BitRate_250kBPS_16MHz,
  BitRate_500kBPS_16MHz,
  BitRate_1000kBPS_16MHz
};

/**************************************************************************************
 * INLINE FUNCTIONS
 **************************************************************************************/

inline bool isBitSet(uint8_t const reg_val, uint8_t const bit_pos)
{
  return ((reg_val & (1<<bit_pos)) == (1<<bit_pos));
}

inline bool isBitClr(uint8_t const reg_val, uint8_t const bit_pos)
{
  return !isBitSet(reg_val, bit_pos);
}

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

ArduinoMCP2515::ArduinoMCP2515(MCP2515::SpiSelectFunc select, MCP2515::SpiDeselectFunc deselect, MCP2515::SpiTransferFunc transfer, OnCanFrameReceiveFunc on_can_frame_rx)
: _io{select, deselect, transfer}
, _ctrl{_io}
, _on_can_frame_rx{on_can_frame_rx}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void ArduinoMCP2515::begin()
{
  _io.reset();
  configureMCP2515();
}

void ArduinoMCP2515::setBitRate(CanBitRate const bit_rate)
{
  _ctrl.setBitRateConfig(BIT_RATE_CONFIG_ARRAY[static_cast<size_t>(bit_rate)]);
}

bool ArduinoMCP2515::transmit(uint32_t const id, uint8_t const * data, uint8_t const len)
{
  uint8_t const status = _ctrl.status();

  if (isBitClr(status, bp(STATUS::TX0REQ)))
  {
    _ctrl.transmit(TxB::TxB0, id, data, len);
    return true;
  }
  else if (isBitClr(status, bp(STATUS::TX1REQ)))
  {
    _ctrl.transmit(TxB::TxB1, id, data, len);
    return true;
  }
  else if (isBitClr(status, bp(STATUS::TX2REQ)))
  {
    _ctrl.transmit(TxB::TxB2, id, data, len);
    return true;
  }
}

void ArduinoMCP2515::onExternalEventHandler()
{
  uint8_t const status = _ctrl.status();

  if(isBitSet(status, bp(STATUS::RX0IF)))
  {
    _ctrl.receive(RxB::RxB0, _on_can_frame_rx);
    _ctrl.clearIntFlag(CANINTF::RX0IF);
  }

  if(isBitSet(status, bp(STATUS::RX1IF)))
  {
    _ctrl.receive(RxB::RxB1, _on_can_frame_rx);
    _ctrl.clearIntFlag(CANINTF::RX1IF);
  }
}

/**************************************************************************************
 * PRIVATE FUNCTION DEFINITION
 **************************************************************************************/

void ArduinoMCP2515::configureMCP2515()
{
  /* Enable interrupts:
   *   Receive Buffer 0 Full
   *   Receive Buffer 1 Full
   */
  setBit(_io, Register::CANINTE, bp(CANINTE::RX0IE));
  setBit(_io, Register::CANINTE, bp(CANINTE::RX1IE));
  /* Turn masks/filters off */
  setBit(_io, Register::RXB0CTRL, bp(RXB0CTRL::RXM1));
  setBit(_io, Register::RXB0CTRL, bp(RXB0CTRL::RXM0));
  setBit(_io, Register::RXB1CTRL, bp(RXB1CTRL::RXM1));
  setBit(_io, Register::RXB1CTRL, bp(RXB1CTRL::RXM0));
  /* Enable roll-over to RXB1 if RXB0 is full */
  setBit(_io, Register::RXB0CTRL, bp(RXB0CTRL::BUKT));
}

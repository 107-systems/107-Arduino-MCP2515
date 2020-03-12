/**
 * @brief   Arduino library for controlling the MCP2515 in order to receive/transmit CAN frames.
 * @author  Alexander Entinger, MSc / LXRobotics GmbH
 * @license LGPL 3.0
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "MCP2515_Control.h"

#include <Arduino.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace MCP2515
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

MCP2515_Control::MCP2515_Control(MCP2515_Io & io)
: _io{io}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

bool MCP2515_Control::setMode(Mode const mode)
{
  uint8_t const mode_val = static_cast<uint8_t>(mode);

  _io.modifyRegister(Register::CANCTRL, CANCTRL_REQOP_MASK, mode_val);

  for(unsigned long const start = millis(); (millis() - start) < 10; )
  {
    uint8_t const canstat_op_mode = (_io.readRegister(Register::CANSTAT) & CANSTAT_OP_MASK);
    if(canstat_op_mode == mode_val) {
      return true;
    }
  }

  return false;
}

void MCP2515_Control::setBitRateConfig(CanBitRateConfig const bit_rate_config)
{
  _io.writeRegister(Register::CNF1, bit_rate_config.CNF1);
  _io.writeRegister(Register::CNF2, bit_rate_config.CNF2);
  _io.writeRegister(Register::CNF3, bit_rate_config.CNF3);
}

void MCP2515_Control::clearIntFlag(CANINTF const int_flag)
{
  clrBit(_io, Register::CANINTF, bp(int_flag));
}

void MCP2515_Control::receive(RxB const rxb, OnCanFrameReceiveFunc on_can_frame_rx)
{
  RxTxBuffer rx_buffer;

  /* Read content of receive buffer */
  _io.readRxBuffer(rxb, rx_buffer.buf);

  /* Assemble ID from registers */
  uint32_t id = (static_cast<uint32_t>(rx_buffer.reg.sidh) << 3) + (static_cast<uint32_t>(rx_buffer.reg.sidl) >> 5);

  /* Read amount of bytes received */
  uint8_t const len = rx_buffer.reg.dlc & 0x0F;

  /* Call registered callback with received data */
  on_can_frame_rx(id, rx_buffer.reg.data, len);
}


/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* MCP2515 */

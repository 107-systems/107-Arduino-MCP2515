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

void MCP2515_Control::transmit(TxB const txb, uint32_t const id, uint8_t const * data, uint8_t const len)
{
  RxTxBuffer tx_buffer;

  bool const is_ext = (id & CAN_EFF_BITMASK) == CAN_EFF_BITMASK;
  bool const is_rtr = (id & CAN_RTR_BITMASK) == CAN_RTR_BITMASK;

  /* Load address registers */
  /*  ID[28:27] = EID[17:16]
   *  ID[26:19] = EID[15: 8]
   *  ID[18:11] = EID[ 7: 0]
   *  ID[10: 3] = SID[10: 3]
   *  ID[ 3: 0] = SID[ 3: 0]
   */
  tx_buffer.reg.sidl = static_cast<uint8_t>((id & 0x00000007) << 5);
  tx_buffer.reg.sidh = static_cast<uint8_t>((id & 0x000007F8) >> 3);
  if(is_ext)
  {
    tx_buffer.reg.sidl |= static_cast<uint8_t>((id & 0x18000000) >> 27);
    tx_buffer.reg.sidl |= (1 << static_cast<uint8_t>(TXBnSIDL::EXIDE));
    tx_buffer.reg.eid0  = static_cast<uint8_t>((id & 0x0007F800) >> 11);
    tx_buffer.reg.eid8  = static_cast<uint8_t>((id & 0x07F80000) >> 19);
  }
  else
  {
    tx_buffer.reg.eid0  = 0;
    tx_buffer.reg.eid8  = 0;
  }

  /* Load data length register */
  tx_buffer.reg.dlc = is_rtr ? (len | (1 << static_cast<uint8_t>(TXBnDLC::RTR))) : len;

  /* Load data buffer */
  memcpy(tx_buffer.reg.data, data, len);

  /* Write to transmit buffer */
  _io.loadTxBuffer(txb, tx_buffer.buf);

  /* Request transmission */
  _io.requestTx(txb);
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

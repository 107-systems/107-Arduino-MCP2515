/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-MCP2515/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "MCP2515_Control.h"

#include <Arduino.h>

#undef min
#undef max
#include <algorithm>

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

void MCP2515_Control::transmit(TxB const txb, uint32_t const id, uint8_t const * data, uint8_t const len)
{
  RxTxBuffer tx_buffer;

  bool const is_ext = (id & CAN_EFF_BITMASK) == CAN_EFF_BITMASK;
  bool const is_rtr = (id & CAN_RTR_BITMASK) == CAN_RTR_BITMASK;

  /* Load address registers */
  if (is_ext) {
    tx_buffer.reg.eid0  = static_cast<uint8_t>((id & 0x000000FF) >> 0);
    tx_buffer.reg.eid8  = static_cast<uint8_t>((id & 0x0000FF00) >> 8);
    tx_buffer.reg.sidl  = static_cast<uint8_t>((id & 0x00030000) >> 16);
    tx_buffer.reg.sidl += static_cast<uint8_t>((id & 0x001C0000) >> 16) << 3;
    tx_buffer.reg.sidl |= bm(TXBnSIDL::EXIDE);
    tx_buffer.reg.sidh  = static_cast<uint8_t>((id & 0x1FE00000) >> 21);
  }
  else
  {
    tx_buffer.reg.sidl = static_cast<uint8_t>((id & 0x00000007) << 5);
    tx_buffer.reg.sidh = static_cast<uint8_t>((id & 0x000007F8) >> 3);
    tx_buffer.reg.eid0 = 0;
    tx_buffer.reg.eid8 = 0;
  }

  /* Load data length register */
  tx_buffer.reg.dlc = is_rtr ? (len | bm(TXBnDLC::RTR)) : len;

  /* Load data buffer */
  memcpy(tx_buffer.reg.data, data, std::min<uint8_t>(len, 8));

  /* Write to transmit buffer */
  _io.loadTxBuffer(txb, tx_buffer.buf);

  /* Request transmission */
  _io.requestTx(txb);
}

void MCP2515_Control::receive(RxB const rxb, uint32_t & id, uint8_t * data, uint8_t & len)
{
  RxTxBuffer rx_buffer;

  /* Read content of receive buffer */
  _io.readRxBuffer(rxb, rx_buffer.buf);

  /* Assemble ID from registers */
  id = (static_cast<uint32_t>(rx_buffer.reg.sidh) << 3) + (static_cast<uint32_t>(rx_buffer.reg.sidl) >> 5);

  if((rx_buffer.reg.sidl & bm(RXBnSIDL::IDE)) == bm(RXBnSIDL::IDE))
  {
    id = (id << 2) + (rx_buffer.reg.sidl & 0x03);
    id = (id << 8) + rx_buffer.reg.eid8;
    id = (id << 8) + rx_buffer.reg.eid0;
    id |= CAN_EFF_BITMASK;
  }

  Register const ctrl_reg_addr = (rxb == RxB::RxB0) ? Register::RXB0CTRL : Register::RXB1CTRL;
  uint8_t const ctrl_reg_val = _io.readRegister(ctrl_reg_addr);
  if((ctrl_reg_val & bm(RXB0CTRL::RXRTR)) == bm(RXB0CTRL::RXRTR))
  {
    id |= CAN_RTR_BITMASK;
  }

  /* Read amount of bytes received */
  len = rx_buffer.reg.dlc & 0x0F;

  /* Call registered callback with received data */
  memcpy(data, rx_buffer.reg.data, std::min<uint8_t>(len, 8));
}

uint8_t MCP2515_Control::error()
{
  return _io.readRegister(Register::EFLG);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* MCP2515 */

/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-MCP2515/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "MCP2515_Config.h"

#include <Arduino.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace MCP2515
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

MCP2515_Config::MCP2515_Config(MCP2515_Io & io)
: _io{io}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

bool MCP2515_Config::setMode(Mode const mode)
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

void MCP2515_Config::setBitRateConfig(CanBitRateConfig const bit_rate_config)
{
  _io.writeRegister(Register::CNF1, bit_rate_config.CNF1);
  _io.writeRegister(Register::CNF2, bit_rate_config.CNF2);
  _io.writeRegister(Register::CNF3, bit_rate_config.CNF3);
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void MCP2515_Config::setFilterId(Register const rxf_n_sidh, uint32_t const id)
{
  RxFilterId filter_id;

  bool const is_ext = (id & CAN_EFF_BITMASK) == CAN_EFF_BITMASK;

  filter_id.reg.eid0  = static_cast<uint8_t>((id & 0x000000FF) >> 0);
  filter_id.reg.eid8  = static_cast<uint8_t>((id & 0x0000FF00) >> 8);
  filter_id.reg.sidl  = static_cast<uint8_t>((id & 0x00030000) >> 16);
  filter_id.reg.sidl += static_cast<uint8_t>((id & 0x001C0000) >> 16) << 3;
  filter_id.reg.sidl |= is_ext ? bm(RXFnSIDL::EXIDE) : 0;
  filter_id.reg.sidh  = static_cast<uint8_t>((id & 0x1FE00000) >> 21);

  _io.writeRegister(rxf_n_sidh, filter_id.buf, sizeof(filter_id.buf));
}

void MCP2515_Config::setFilterMask(Register const rxm_n_sidh, uint32_t const mask)
{
  RxFilterMask filter_mask;

  filter_mask.reg.eid0  = static_cast<uint8_t>((mask & 0x000000FF) >> 0);
  filter_mask.reg.eid8  = static_cast<uint8_t>((mask & 0x0000FF00) >> 8);
  filter_mask.reg.sidl  = static_cast<uint8_t>((mask & 0x00030000) >> 16);
  filter_mask.reg.sidl += static_cast<uint8_t>((mask & 0x001C0000) >> 16) << 3;
  filter_mask.reg.sidh  = static_cast<uint8_t>((mask & 0x1FE00000) >> 21);

  _io.writeRegister(rxm_n_sidh, filter_mask.buf, sizeof(filter_mask.buf));
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* MCP2515 */

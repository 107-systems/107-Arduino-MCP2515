/**
 * @brief   Arduino library for controlling the MCP2515 in order to receive/transmit CAN frames.
 * @license LGPL 3.0
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
 * NAMESPACE
 **************************************************************************************/

} /* MCP2515 */

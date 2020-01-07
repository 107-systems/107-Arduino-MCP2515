/**
 * @brief   Arduino library for controlling the MCP2515 in order to receive/transmit CAN frames.
 * @author  Alexander Entinger, MSc / LXRobotics GmbH
 * @license LGPL 3.0
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "MCP2515/MCP2515_Io.h"

#include <SPI.h>

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

MCP2515_Io::MCP2515_Io(int const cs_pin)
: _cs_pin{cs_pin}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

uint8_t MCP2515_Io::readRegister(Register const reg)
{
  uint8_t const instruction = static_cast<uint8_t>(Instruction::READ);
  uint8_t const reg_addr    = static_cast<uint8_t>(reg);

  select();
                       SPI.transfer(instruction);
                       SPI.transfer(reg_addr);
  uint8_t const data = SPI.transfer(0);
  deselect();
}

void MCP2515_Io::writeRegister(Register const reg, uint8_t const data)
{
  uint8_t const instruction = static_cast<uint8_t>(Instruction::WRITE);
  uint8_t const reg_addr    = static_cast<uint8_t>(reg);

  select();
  SPI.transfer(instruction);
  SPI.transfer(reg_addr);
  SPI.transfer(data);
  deselect();
}

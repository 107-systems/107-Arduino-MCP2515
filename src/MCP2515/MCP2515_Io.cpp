/**
 * @brief   Arduino library for controlling the MCP2515 in order to receive/transmit CAN frames.
 * @author  Alexander Entinger, MSc / LXRobotics GmbH
 * @license LGPL 3.0
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "MCP2515_Io.h"

#include <assert.h>

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

void MCP2515_Io::begin()
{
  init_cs ();
  init_spi();
}

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

void MCP2515_Io::modifyRegister(Register const reg, uint8_t const mask, uint8_t const data)
{
  uint8_t const instruction = static_cast<uint8_t>(Instruction::BITMOD);
  uint8_t const reg_addr    = static_cast<uint8_t>(reg);

  select();
  SPI.transfer(instruction);
  SPI.transfer(reg_addr);
  SPI.transfer(mask);
  SPI.transfer(data);
  deselect();
}

void MCP2515_Io::setBit(Register const reg, uint8_t const bit_pos)
{
  assert(bit_pos < 8);
  uint8_t reg_val = readRegister(reg);
  reg_val |= (1<<bit_pos);
  writeRegister(reg, reg_val);
}

void MCP2515_Io::clrBit(Register const reg, uint8_t const bit_pos)
{
  assert(bit_pos < 8);
  uint8_t reg_val = readRegister(reg);
  reg_val &= ~(1<<bit_pos);
  writeRegister(reg, reg_val);
}

void MCP2515_Io::reset()
{
  uint8_t const instruction = static_cast<uint8_t>(Instruction::RESET);

  select();
  SPI.transfer(instruction);
  deselect();

  delay(10);
}

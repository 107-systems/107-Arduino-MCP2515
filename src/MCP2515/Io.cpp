/**
 * @brief   Arduino library for controlling the MCP2515 in order to receive/transmit CAN frames.
 * @author  Alexander Entinger, MSc / LXRobotics GmbH
 * @license LGPL 3.0
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "Io.h"

#include <assert.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace MCP2515
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Io::Io(int const cs_pin)
: _cs_pin{cs_pin}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void Io::begin()
{
  init_cs ();
  init_spi();
}

uint8_t Io::readRegister(Register const reg)
{
  uint8_t const instruction = static_cast<uint8_t>(Instruction::READ);
  uint8_t const reg_addr    = static_cast<uint8_t>(reg);

  select();
                       SPI.transfer(instruction);
                       SPI.transfer(reg_addr);
  uint8_t const data = SPI.transfer(0);
  deselect();

  return data;
}

void Io::readRegister(Register const reg, uint8_t * data, uint8_t const len)
{
  uint8_t const instruction = static_cast<uint8_t>(Instruction::READ);
  uint8_t const reg_addr    = static_cast<uint8_t>(reg);

  select();
  SPI.transfer(instruction);
  SPI.transfer(reg_addr);
  for(uint8_t b = 0; b < len; b++)
  {
    data[b] = SPI.transfer(0);
  }
  deselect();
}

void Io::writeRegister(Register const reg, uint8_t const data)
{
  uint8_t const instruction = static_cast<uint8_t>(Instruction::WRITE);
  uint8_t const reg_addr    = static_cast<uint8_t>(reg);

  select();
  SPI.transfer(instruction);
  SPI.transfer(reg_addr);
  SPI.transfer(data);
  deselect();
}

void Io::writeRegister(Register const reg, uint8_t const * data, uint8_t const len)
{
  uint8_t const instruction = static_cast<uint8_t>(Instruction::WRITE);
  uint8_t const reg_addr    = static_cast<uint8_t>(reg);

  select();
  SPI.transfer(instruction);
  SPI.transfer(reg_addr);
  for(uint8_t b = 0; b < len; b++)
  {
    SPI.transfer(data[b]);
  }
  deselect();
}

void Io::modifyRegister(Register const reg, uint8_t const mask, uint8_t const data)
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

void Io::reset()
{
  uint8_t const instruction = static_cast<uint8_t>(Instruction::RESET);

  select();
  SPI.transfer(instruction);
  deselect();

  delay(10);
}

uint8_t Io::status()
{
  uint8_t const instruction = static_cast<uint8_t>(Instruction::READ_STATUS);

  select();
                         SPI.transfer(instruction);
  uint8_t const status = SPI.transfer(0);
  deselect();

  return status;
}

/**************************************************************************************
 * FREE FUNCTION DEFINITION
 **************************************************************************************/

void setBit(Io & io, Register const reg, uint8_t const bit_pos)
{
  assert(bit_pos < 8);
  uint8_t const bit_mask = (1<<bit_pos);
  io.modifyRegister(reg, bit_mask, bit_mask);
}

void clrBit(Io & io, Register const reg, uint8_t const bit_pos)
{
  assert(bit_pos < 8);
  uint8_t const bit_mask = (1<<bit_pos);
  io.modifyRegister(reg, bit_mask, 0);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* MCP2515 */

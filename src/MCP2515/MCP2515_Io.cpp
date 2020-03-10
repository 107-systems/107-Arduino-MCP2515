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
 * NAMESPACE
 **************************************************************************************/

namespace MCP2515
{

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static Instruction const TABLE_LOAD_TX_BUFFER[] =
{
  Instruction::LOAD_TX0,
  Instruction::LOAD_TX1,
  Instruction::LOAD_TX2
};

static Instruction const TABLE_REQUEST_TO_SEND[] =
{
  Instruction::RTS_TX0,
  Instruction::RTS_TX1,
  Instruction::RTS_TX2
};

static Instruction const TABLE_READ_RX_BUFFER[] =
{
  Instruction::READ_RX0,
  Instruction::READ_RX1
};

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

void MCP2515_Io::reset()
{
  uint8_t const instruction = static_cast<uint8_t>(Instruction::RESET);

  select();
  SPI.transfer(instruction);
  deselect();

  delay(10);
}

uint8_t MCP2515_Io::status()
{
  uint8_t const instruction = static_cast<uint8_t>(Instruction::READ_STATUS);

  select();
                         SPI.transfer(instruction);
  uint8_t const status = SPI.transfer(0);
  deselect();

  return status;
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

  return data;
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

void MCP2515_Io::loadTxBuffer(TxB const txb, uint8_t const * tx_buf_data)
{
  uint8_t const instruction = static_cast<uint8_t>(TABLE_LOAD_TX_BUFFER[static_cast<uint8_t>(txb)]);

  select();
  SPI.transfer(instruction);
  for(uint8_t b = 0; b < TX_BUF_SIZE; b++)
  {
    SPI.transfer(tx_buf_data[b]);
  }
  deselect();
}

void MCP2515_Io::requestTx(TxB const txb)
{
  uint8_t const instruction = static_cast<uint8_t>(TABLE_REQUEST_TO_SEND[static_cast<uint8_t>(txb)]);

  select();
  SPI.transfer(instruction);
  deselect();
}

void MCP2515_Io::readRxBuffer(RxB const rxb, uint8_t * rx_buf_data)
{
  uint8_t const instruction = static_cast<uint8_t>(TABLE_READ_RX_BUFFER[static_cast<uint8_t>(rxb)]);

  select();
  SPI.transfer(instruction);
  for(uint8_t b = 0; b < RX_BUF_SIZE; b++)
  {
    rx_buf_data[b] = SPI.transfer(0);
  }
  deselect();
}

/**************************************************************************************
 * FREE FUNCTION DEFINITION
 **************************************************************************************/

void setBit(MCP2515_Io & io, Register const reg, uint8_t const bit_pos)
{
  assert(bit_pos < 8);
  uint8_t const bit_mask = (1<<bit_pos);
  io.modifyRegister(reg, bit_mask, bit_mask);
}

void clrBit(MCP2515_Io & io, Register const reg, uint8_t const bit_pos)
{
  assert(bit_pos < 8);
  uint8_t const bit_mask = (1<<bit_pos);
  io.modifyRegister(reg, bit_mask, 0);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* MCP2515 */

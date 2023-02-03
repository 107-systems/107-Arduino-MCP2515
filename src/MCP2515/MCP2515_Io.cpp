/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-MCP2515/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "MCP2515_Io.h"

#include <Arduino.h>

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

MCP2515_Io::MCP2515_Io(SpiSelectFunc select, SpiDeselectFunc deselect, SpiTransferFunc transfer)
: _select{select}
, _deselect{deselect}
, _transfer{transfer}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void MCP2515_Io::reset()
{
  uint8_t const instruction = static_cast<uint8_t>(Instruction::RESET);

  _select();
  _transfer(instruction);
  _deselect();

  delay(10);
}

uint8_t MCP2515_Io::status()
{
  uint8_t const instruction = static_cast<uint8_t>(Instruction::READ_STATUS);

  _select();
                         _transfer(instruction);
  uint8_t const status = _transfer(0);
  _deselect();

  return status;
}

uint8_t MCP2515_Io::readRegister(Register const reg)
{
  uint8_t const instruction = static_cast<uint8_t>(Instruction::READ);
  uint8_t const reg_addr    = static_cast<uint8_t>(reg);

  _select();
                       _transfer(instruction);
                       _transfer(reg_addr);
  uint8_t const data = _transfer(0);
  _deselect();

  return data;
}

void MCP2515_Io::writeRegister(Register const reg, uint8_t const data)
{
  uint8_t const instruction = static_cast<uint8_t>(Instruction::WRITE);
  uint8_t const reg_addr    = static_cast<uint8_t>(reg);

  _select();
  _transfer(instruction);
  _transfer(reg_addr);
  _transfer(data);
  _deselect();
}

void MCP2515_Io::writeRegister(Register const reg, uint8_t const * data, size_t const num_bytes)
{
  uint8_t const instruction = static_cast<uint8_t>(Instruction::WRITE);
  uint8_t const reg_addr    = static_cast<uint8_t>(reg);

  _select();
  _transfer(instruction);
  _transfer(reg_addr);
  for (size_t b = 0; b < num_bytes; b++)
    _transfer(data[b]);
  _deselect();
}

void MCP2515_Io::modifyRegister(Register const reg, uint8_t const mask, uint8_t const data)
{
  uint8_t const instruction = static_cast<uint8_t>(Instruction::BITMOD);
  uint8_t const reg_addr    = static_cast<uint8_t>(reg);

  _select();
  _transfer(instruction);
  _transfer(reg_addr);
  _transfer(mask);
  _transfer(data);
  _deselect();
}

void MCP2515_Io::setBit(Register const reg, uint8_t const bit_pos)
{
  assert(bit_pos < 8);
  uint8_t const bit_mask = (1<<bit_pos);
  modifyRegister(reg, bit_mask, bit_mask);
}

void MCP2515_Io::clrBit(Register const reg, uint8_t const bit_pos)
{
  assert(bit_pos < 8);
  uint8_t const bit_mask = (1<<bit_pos);
  modifyRegister(reg, bit_mask, 0);
}

void MCP2515_Io::loadTxBuffer(TxB const txb, uint8_t const * tx_buf_data)
{
  uint8_t const instruction = static_cast<uint8_t>(TABLE_LOAD_TX_BUFFER[static_cast<uint8_t>(txb)]);

  _select();
  _transfer(instruction);
  for(uint8_t b = 0; b < TX_BUF_SIZE; b++)
  {
    _transfer(tx_buf_data[b]);
  }
  _deselect();
}

void MCP2515_Io::requestTx(TxB const txb)
{
  uint8_t const instruction = static_cast<uint8_t>(TABLE_REQUEST_TO_SEND[static_cast<uint8_t>(txb)]);

  _select();
  _transfer(instruction);
  _deselect();
}

void MCP2515_Io::readRxBuffer(RxB const rxb, uint8_t * rx_buf_data)
{
  uint8_t const instruction = static_cast<uint8_t>(TABLE_READ_RX_BUFFER[static_cast<uint8_t>(rxb)]);

  _select();
  _transfer(instruction);
  for(uint8_t b = 0; b < RX_BUF_SIZE; b++)
  {
    rx_buf_data[b] = _transfer(0);
  }
  _deselect();
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* MCP2515 */

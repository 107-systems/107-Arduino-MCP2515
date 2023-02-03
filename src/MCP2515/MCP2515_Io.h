/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-MCP2515/graphs/contributors.
 */

#ifndef MCP2515_MCP2515_IO_H_
#define MCP2515_MCP2515_IO_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <stdint.h>

#undef max
#undef min
#include <functional>

#include "MCP2515_Const.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace MCP2515
{

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

typedef std::function<void()>                 SpiSelectFunc;
typedef std::function<void()>                 SpiDeselectFunc;
typedef std::function<uint8_t(uint8_t const)> SpiTransferFunc;

enum class TxB : uint8_t
{
  TxB0 = 0,
  TxB1 = 1,
  TxB2 = 2
};

enum class RxB : uint8_t
{
  RxB0 = 0,
  RxB1 = 1
};

union RxTxBuffer
{
  struct
  {
    uint8_t sidh;
    uint8_t sidl;
    uint8_t eid8;
    uint8_t eid0;
    uint8_t dlc;
    uint8_t data[8];
  } reg;
  uint8_t buf[5+8];
};

union RxFilterId
{
  struct
  {
    uint8_t sidh;
    uint8_t sidl;
    uint8_t eid8;
    uint8_t eid0;
  } reg;
  uint8_t buf[4];
};

typedef RxFilterId RxFilterMask;

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class MCP2515_Io
{

public:

  MCP2515_Io(SpiSelectFunc select, SpiDeselectFunc deselect, SpiTransferFunc transfer);


  static uint8_t constexpr TX_BUF_SIZE = 5 + 8;
  static uint8_t constexpr RX_BUF_SIZE = TX_BUF_SIZE;


  void    reset();
  uint8_t status();
  uint8_t readRegister  (Register const reg);
  void    writeRegister (Register const reg, uint8_t const data);
  void    writeRegister (Register const reg, uint8_t const * data, size_t const num_bytes);
  void    modifyRegister(Register const reg, uint8_t const mask, uint8_t const data);
  void    setBit        (Register const reg, uint8_t const bit_pos);
  void    clrBit        (Register const reg, uint8_t const bit_pos);
  void    loadTxBuffer  (TxB const txb, uint8_t const * tx_buf_data); /* tx_buf_data = {SIDH, SIDL, EID8, EID0, DLC, DATA[0-8 Byte] } */
  void    requestTx     (TxB const txb);
  void    readRxBuffer  (RxB const rxb, uint8_t * rx_buf_data);       /* rx_buf_data = {SIDH, SIDL, EID8, EID0, DLC, DATA[0-8 Byte] } */


private:

  SpiSelectFunc _select;
  SpiDeselectFunc _deselect;
  SpiTransferFunc _transfer;

};

static_assert(sizeof(RxTxBuffer) == MCP2515_Io::TX_BUF_SIZE, "Union RxTxBuffer exceeds expected size of MCP2515_Io::TX/RX_BUF_SIZE bytes");

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* MCP2515 */

#endif /* MCP2515_MCP2515_IO_H_ */

/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-MCP2515/graphs/contributors.
 */

#ifndef MCP2515_MCP2515_CONTROL_H_
#define MCP2515_MCP2515_CONTROL_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "MCP2515_Io.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace MCP2515
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

class MCP2515_Control
{

public:

  MCP2515_Control(MCP2515_Io & io);


  void transmit(TxB const txb, uint32_t const   id, uint8_t const * data, uint8_t const   len);
  void receive (RxB const rxb, uint32_t       & id, uint8_t       * data, uint8_t       & len);

  uint8_t error();

  inline void    reset       ()                       { _io.reset(); }
  inline uint8_t status      ()                       { return _io.status(); }
  inline void    clearIntFlag(CANINTF const int_flag) { _io.clrBit(Register::CANINTF, bp(int_flag)); }
  inline void    clearErrFlag(EFLG const err_flag)    { _io.clrBit(Register::EFLG, bp(err_flag)); }

private:

  MCP2515_Io & _io;

};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* MCP2515 */

#endif /* MCP2515_MCP2515_CONTROL_H_ */

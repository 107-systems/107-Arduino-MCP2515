/**
 * @brief   Arduino library for controlling the MCP2515 in order to receive/transmit CAN frames.
 * @author  Alexander Entinger, MSc / LXRobotics GmbH
 * @license LGPL 3.0
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
 * TYPEDEF
 **************************************************************************************/

typedef std::function<void(uint32_t const, uint8_t const *, uint8_t const)> OnCanFrameReceiveFunc;

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

class MCP2515_Control
{

public:

  MCP2515_Control(MCP2515_Io & io);


  void           transmit    (TxB const txb, uint32_t const id, uint8_t const * data, uint8_t const len);
  void           receive     (RxB const rxb, OnCanFrameReceiveFunc on_can_frame_rx);

  inline void    reset       ()                       { _io.reset(); }
  inline uint8_t status      ()                       { return _io.status(); }
  inline void    clearIntFlag(CANINTF const int_flag) {  clrBit(_io, Register::CANINTF, bp(int_flag)); }


private:

  MCP2515_Io & _io;

};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* MCP2515 */

#endif /* MCP2515_MCP2515_CONTROL_H_ */

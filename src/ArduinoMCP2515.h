/**
 * @brief   Arduino library for controlling the MCP2515 in order to receive/transmit CAN frames.
 * @author  Alexander Entinger, MSc / LXRobotics GmbH
 * @license LGPL 3.0
 */

#ifndef ARDUINO_MCP2515_H_
#define ARDUINO_MCP2515_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include "MCP2515/MCP2515_Io.h"

#undef min
#undef max
#include <vector>
#include <string>
#include <functional>

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static int const MKRCAN_MCP2515_CS_PIN = 3;

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

typedef std::function<void(uint32_t const, std::vector<uint8_t> const &)> OnCanFrameReceiveFunc;

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class ArduinoMCP2515
{

public:

  ArduinoMCP2515(int const cs_pin,
                 OnCanFrameReceiveFunc on_can_frame_rx);


  bool transmit(uint32_t const id, std::vector<uint8_t> const & data);
  

private:

  MCP2515_Io            _io;
  OnCanFrameReceiveFunc _on_can_frame_rx;

};

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

std::string toStr(uint32_t const id, std::vector<uint8_t> const & data);

#endif /* ARDUINO_MCP2515_H_ */

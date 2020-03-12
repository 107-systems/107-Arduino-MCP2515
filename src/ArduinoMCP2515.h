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
#include "MCP2515/MCP2515_Control.h"

#undef min
#undef max
#include <vector>
#include <string>
#include <functional>

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static int const MKRCAN_MCP2515_CS_PIN  = 3;
static int const MKRCAN_MCP2515_INT_PIN = 7;

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

enum class CanBitRate : size_t
{
  BR_125kBPS  = 0,
  BR_250kBPS  = 1,
  BR_500kBPS  = 2,
  BR_1000kBPS = 3
};

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class ArduinoMCP2515
{

public:

  ArduinoMCP2515(MCP2515::SpiSelectFunc select, MCP2515::SpiDeselectFunc deselect, MCP2515::SpiTransferFunc transfer, MCP2515::OnCanFrameReceiveFunc on_can_frame_rx);


  void begin();

  void setBitRate(CanBitRate const bit_rate);

  inline bool setNormalMode    () { return _ctrl.setMode(MCP2515::Mode::Normal);     }
  inline bool setSleepMode     () { return _ctrl.setMode(MCP2515::Mode::Sleep);      }
  inline bool setLoopbackMode  () { return _ctrl.setMode(MCP2515::Mode::Loopback);   }
  inline bool setListenOnlyMode() { return _ctrl.setMode(MCP2515::Mode::ListenOnly); }
  inline bool setConfigMode    () { return _ctrl.setMode(MCP2515::Mode::Config);     }

  bool transmit(uint32_t const id, uint8_t const * data, uint8_t const len);

  void onExternalEventHandler();


private:

  MCP2515::MCP2515_Io _io;
  MCP2515::MCP2515_Control _ctrl;
  MCP2515::OnCanFrameReceiveFunc _on_can_frame_rx;

  void configureMCP2515();

};

#endif /* ARDUINO_MCP2515_H_ */

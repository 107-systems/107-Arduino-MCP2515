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
#include "MCP2515/MCP2515_Event.h"

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

typedef std::function<void(uint32_t const, uint8_t const *, uint8_t const)> OnCanFrameReceiveFunc;

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class ArduinoMCP2515
{

public:

  ArduinoMCP2515(int const cs_pin,
                 int const int_pin,
                 OnCanFrameReceiveFunc on_can_frame_rx);


  void begin();

  inline bool setNormalMode    () { return setMode(Mode::Normal);     }
  inline bool setSleepMode     () { return setMode(Mode::Sleep);      }
  inline bool setLoopbackMode  () { return setMode(Mode::Loopback);   }
  inline bool setListenOnlyMode() { return setMode(Mode::ListenOnly); }
  inline bool setConfigMode    () { return setMode(Mode::Config);     }

  bool transmit(uint32_t const id, uint8_t const * data, uint8_t const len);


private:

  MCP2515_Io            _io;
  MCP2515_Event         _event;
  OnCanFrameReceiveFunc _on_can_frame_rx;

  static uint8_t constexpr CANCTRL_REQOP_MASK = 0xE0;
  static uint8_t constexpr CANSTAT_OP_MASK    = CANCTRL_REQOP_MASK;

  enum class Mode : uint8_t
  {
    Normal     = 0x00,
    Sleep      = 0x20,
    Loopback   = 0x40,
    ListenOnly = 0x60,
    Config     = 0x80
  };

  bool setMode(Mode const mode);

};

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

std::string toStr(uint32_t const id, uint8_t const * data, uint8_t const len);

#endif /* ARDUINO_MCP2515_H_ */

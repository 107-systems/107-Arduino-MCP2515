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
#include "MCP2515/MCP2515_Types.h"

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

  ArduinoMCP2515(int const cs_pin,
                 int const int_pin,
                 OnCanFrameReceiveFunc on_can_frame_rx);


  void begin();

  void setBitRate(CanBitRate const bit_rate);

  inline bool setNormalMode    () { return setMode(MCP2515_Mode::Normal);     }
  inline bool setSleepMode     () { return setMode(MCP2515_Mode::Sleep);      }
  inline bool setLoopbackMode  () { return setMode(MCP2515_Mode::Loopback);   }
  inline bool setListenOnlyMode() { return setMode(MCP2515_Mode::ListenOnly); }
  inline bool setConfigMode    () { return setMode(MCP2515_Mode::Config);     }

  bool transmit(uint32_t const id, uint8_t const * data, uint8_t const len);


private:

  MCP2515_Io            _io;
  MCP2515_Event         _event;
  OnCanFrameReceiveFunc _on_can_frame_rx;

  bool setMode(MCP2515_Mode const mode);
  void setBitRateConfig(MCP2515_CanBitRateConfig const bit_rate_config);

};

#endif /* ARDUINO_MCP2515_H_ */

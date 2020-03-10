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

static uint32_t constexpr CAN_EFF_BITMASK  = 0x80000000;
static uint32_t constexpr CAN_RTR_BITMASK  = 0x40000000;
static uint32_t constexpr CAN_ERR_BITMASK  = 0x20000000;

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

  ArduinoMCP2515(int const cs_pin, OnCanFrameReceiveFunc on_can_frame_rx);


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
  OnCanFrameReceiveFunc _on_can_frame_rx;

  void configureMCP2515();

  void transmit(MCP2515::TxB const txb, uint32_t const id, uint8_t const * data, uint8_t const len);
  void receive (MCP2515::RxB const rxb);

};

#endif /* ARDUINO_MCP2515_H_ */

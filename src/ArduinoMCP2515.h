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

#include "MCP2515/Io.h"

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

  ArduinoMCP2515(int const cs_pin,
                 int const int_pin,
                 OnCanFrameReceiveFunc on_can_frame_rx);


  void begin();

  void setBitRate(CanBitRate const bit_rate);

  inline bool setNormalMode    () { return setMode(Mode::Normal);     }
  inline bool setSleepMode     () { return setMode(Mode::Sleep);      }
  inline bool setLoopbackMode  () { return setMode(Mode::Loopback);   }
  inline bool setListenOnlyMode() { return setMode(Mode::ListenOnly); }
  inline bool setConfigMode    () { return setMode(Mode::Config);     }

  bool transmit(uint32_t const id, uint8_t const * data, uint8_t const len);

  static void onExternalEvent();


private:

  MCP2515::Io            _io;
  int const              _int_pin;
  OnCanFrameReceiveFunc  _on_can_frame_rx;

  void configureEventCallback();
  void configureMCP2515();

  enum class Mode : uint8_t
  {
    Normal     = 0x00,
    Sleep      = 0x20,
    Loopback   = 0x40,
    ListenOnly = 0x60,
    Config     = 0x80
  };

  bool setMode(Mode const mode);

  void transmit(MCP2515::Register const tx_buf_sidh, MCP2515::Register const tx_buf_ctrl, uint32_t const id, uint8_t const * data, uint8_t const len);
  void receive (MCP2515::Register const rx_buf_ctrl);

  void onExternalEventHandler();

};

#endif /* ARDUINO_MCP2515_H_ */

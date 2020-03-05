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
#include "MCP2515/Types.h"

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

  inline bool setNormalMode    () { return setMode(MCP2515::Mode::Normal);     }
  inline bool setSleepMode     () { return setMode(MCP2515::Mode::Sleep);      }
  inline bool setLoopbackMode  () { return setMode(MCP2515::Mode::Loopback);   }
  inline bool setListenOnlyMode() { return setMode(MCP2515::Mode::ListenOnly); }
  inline bool setConfigMode    () { return setMode(MCP2515::Mode::Config);     }

  bool transmit(uint32_t const id, uint8_t const * data, uint8_t const len);
  bool receive (uint32_t * id, uint8_t * data, uint8_t * len);

  static void onExternalEvent();


private:

  MCP2515::Io            _io;
  int const              _int_pin;
  OnCanFrameReceiveFunc  _on_can_frame_rx;

  void setupEventCallback();

  bool setMode(MCP2515::Mode const mode);
  void setBitRateConfig(MCP2515::CanBitRateConfig const bit_rate_config);

  bool transmit(MCP2515::TxBuffer const tx_buf, uint32_t const id, uint8_t const * data, uint8_t const len);
  bool receive (MCP2515::RxBuffer const rx_buf, uint32_t * id, uint8_t * data, uint8_t * len);

};

#endif /* ARDUINO_MCP2515_H_ */

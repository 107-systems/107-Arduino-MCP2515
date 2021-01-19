/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-MCP2515/graphs/contributors.
 */

#ifndef ARDUINO_MCP2515_H_
#define ARDUINO_MCP2515_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include "MCP2515/MCP2515_Io.h"
#include "MCP2515/MCP2515_Config.h"
#include "MCP2515/MCP2515_Control.h"

#undef min
#undef max
#include <vector>
#include <string>
#include <functional>

#if defined __has_include
#  if __has_include (<libcanard/canard.h>)
#    include <libcanard/canard.h>
#    define LIBCANARD 1
#  elif __has_include (<canard.h>)
#    include <canard.h>
#    define LIBCANARD 1
#  endif
#else
#  define LIBCANARD 0
#endif

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

enum class CanBitRate : size_t
{
  BR_125kBPS_16MHZ  = 0,
  BR_250kBPS_16MHZ  = 1,
  BR_500kBPS_16MHZ  = 2,
  BR_1000kBPS_16MHZ = 3,
  BR_125kBPS_8MHZ  = 4,
  BR_250kBPS_8MHZ  = 5,
  BR_500kBPS_8MHZ  = 6,
  BR_1000kBPS_8MHZ = 7
};

typedef std::function<unsigned long()> MicroSecondFunc;
#if LIBCANARD
typedef std::function<void(CanardFrame const & frame)> OnReceiveBufferFullFunc;
#else
typedef std::function<void(uint32_t const, uint32_t const, uint8_t const *, uint8_t const)> OnReceiveBufferFullFunc;
#endif
class ArduinoMCP2515;
typedef std::function<void(ArduinoMCP2515 *)> OnTransmitBufferEmptyFunc;

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class ArduinoMCP2515
{

public:

  ArduinoMCP2515(MCP2515::SpiSelectFunc select,
                 MCP2515::SpiDeselectFunc deselect,
                 MCP2515::SpiTransferFunc transfer,
                 MicroSecondFunc micros,
                 OnReceiveBufferFullFunc on_rx_buf_full,
                 OnTransmitBufferEmptyFunc on_tx_buf_empty);


  void begin();

  void setBitRate(CanBitRate const bit_rate);

  inline bool setNormalMode    () { return _cfg.setMode(MCP2515::Mode::Normal);     }
  inline bool setSleepMode     () { return _cfg.setMode(MCP2515::Mode::Sleep);      }
  inline bool setLoopbackMode  () { return _cfg.setMode(MCP2515::Mode::Loopback);   }
  inline bool setListenOnlyMode() { return _cfg.setMode(MCP2515::Mode::ListenOnly); }
  inline bool setConfigMode    () { return _cfg.setMode(MCP2515::Mode::Config);     }

#if LIBCANARD
  bool transmit(CanardFrame const & frame);
#else
  bool transmit(uint32_t const id, uint8_t const * data, uint8_t const len);
#endif

  void onExternalEventHandler();


private:

  MCP2515::MCP2515_Io _io;
  MCP2515::MCP2515_Config _cfg;
  MCP2515::MCP2515_Control _ctrl;
  MicroSecondFunc _micros;
  OnReceiveBufferFullFunc _on_rx_buf_full;
  OnTransmitBufferEmptyFunc _on_tx_buf_empty;

  bool transmitCANFrame        (uint32_t const id, uint8_t const * data, uint8_t const len);
  void onReceiveBuffer_0_Full  ();
  void onReceiveBuffer_1_Full  ();
  void onTransmitBuffer_0_Empty();
  void onTransmitBuffer_1_Empty();
  void onTransmitBuffer_2_Empty();
  void onReceiveBuffer_n_Full  (unsigned long const timestamp_us, uint32_t const id, uint8_t const * data, uint8_t const len) const;
};

#endif /* ARDUINO_MCP2515_H_ */

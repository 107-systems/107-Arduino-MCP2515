/**
 * @brief   Arduino library for controlling the MCP2515 in order to receive/transmit CAN frames.
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
#include "MCP2515/MCP2515_Config.h"
#include "MCP2515/MCP2515_Control.h"

#undef min
#undef max
#include <vector>
#include <string>
#include <functional>

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

class ArduinoMCP2515;
typedef std::function<void(uint32_t const, uint8_t const *, uint8_t const)> OnReceiveBufferFullFunc;
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
                 OnReceiveBufferFullFunc on_rx_buf_full,
                 OnTransmitBufferEmptyFunc on_tx_buf_empty);


  void begin();

  void setBitRate(CanBitRate const bit_rate);

  inline bool setNormalMode    () { return _cfg.setMode(MCP2515::Mode::Normal);     }
  inline bool setSleepMode     () { return _cfg.setMode(MCP2515::Mode::Sleep);      }
  inline bool setLoopbackMode  () { return _cfg.setMode(MCP2515::Mode::Loopback);   }
  inline bool setListenOnlyMode() { return _cfg.setMode(MCP2515::Mode::ListenOnly); }
  inline bool setConfigMode    () { return _cfg.setMode(MCP2515::Mode::Config);     }

  bool transmit(uint32_t const id, uint8_t const * data, uint8_t const len);

  void onExternalEventHandler();


private:

  MCP2515::MCP2515_Io _io;
  MCP2515::MCP2515_Config _cfg;
  MCP2515::MCP2515_Control _ctrl;
  OnReceiveBufferFullFunc _on_rx_buf_full;
  OnTransmitBufferEmptyFunc _on_tx_buf_empty;

  void onReceiveBuffer_0_Full  ();
  void onReceiveBuffer_1_Full  ();
  void onTransmitBuffer_0_Empty();
  void onTransmitBuffer_1_Empty();
  void onTransmitBuffer_2_Empty();

};

#endif /* ARDUINO_MCP2515_H_ */

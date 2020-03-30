/**
 * @brief   Arduino library for controlling the MCP2515 in order to receive/transmit CAN frames.
 * @license LGPL 3.0
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <ArduinoMCP2515.h>

#include <algorithm>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace MCP2515;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static CanBitRateConfig const BIT_RATE_CONFIG_ARRAY[] =
{
  BitRate_125kBPS_16MHz,
  BitRate_250kBPS_16MHz,
  BitRate_500kBPS_16MHz,
  BitRate_1000kBPS_16MHz
};

/**************************************************************************************
 * INLINE FUNCTIONS
 **************************************************************************************/

inline bool isBitSet(uint8_t const reg_val, uint8_t const bit_pos)
{
  return ((reg_val & (1<<bit_pos)) == (1<<bit_pos));
}

inline bool isBitClr(uint8_t const reg_val, uint8_t const bit_pos)
{
  return !isBitSet(reg_val, bit_pos);
}

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

ArduinoMCP2515::ArduinoMCP2515(SpiSelectFunc select,
                               SpiDeselectFunc deselect,
                               SpiTransferFunc transfer,
                               OnReceiveBufferFullFunc on_rx_buf_full,
                               OnTransmitBufferEmptyFunc on_tx_buf_empty)
: _io{select, deselect, transfer}
, _cfg{_io}
, _ctrl{_io}
, _on_rx_buf_full{on_rx_buf_full}
, _on_tx_buf_empty{on_tx_buf_empty}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void ArduinoMCP2515::begin()
{
  _ctrl.reset();
  _cfg.disableFilter_RxB0();
  _cfg.disableFilter_RxB1();
  _cfg.enableRollover_RxB0();
  _cfg.enableIntFlag(CANINTE::RX0IE);
  _cfg.enableIntFlag(CANINTE::RX1IE);
  _cfg.enableIntFlag(CANINTE::TX0IE);
  _cfg.enableIntFlag(CANINTE::TX1IE);
  _cfg.enableIntFlag(CANINTE::TX2IE);
}

void ArduinoMCP2515::setBitRate(CanBitRate const bit_rate)
{
  _cfg.setBitRateConfig(BIT_RATE_CONFIG_ARRAY[static_cast<size_t>(bit_rate)]);
}

bool ArduinoMCP2515::transmit(uint32_t const id, uint8_t const * data, uint8_t const len)
{
  uint8_t const status = _ctrl.status();

  if (isBitClr(status, bp(STATUS::TX0REQ)))
  {
    _ctrl.transmit(TxB::TxB0, id, data, len);
    return true;
  }
  else if (isBitClr(status, bp(STATUS::TX1REQ)))
  {
    _ctrl.transmit(TxB::TxB1, id, data, len);
    return true;
  }
  else if (isBitClr(status, bp(STATUS::TX2REQ)))
  {
    _ctrl.transmit(TxB::TxB2, id, data, len);
    return true;
  }
}

void ArduinoMCP2515::onExternalEventHandler()
{
  uint8_t const status = _ctrl.status();

  if(isBitSet(status, bp(STATUS::RX0IF))) onReceiveBuffer_0_Full();
  if(isBitSet(status, bp(STATUS::RX1IF))) onReceiveBuffer_1_Full();
  if(isBitSet(status, bp(STATUS::TX0IF))) onTransmitBuffer_0_Empty();
  if(isBitSet(status, bp(STATUS::TX1IF))) onTransmitBuffer_1_Empty();
  if(isBitSet(status, bp(STATUS::TX2IF))) onTransmitBuffer_2_Empty();
}

void ArduinoMCP2515::onReceiveBuffer_0_Full()
{
  uint32_t id = 0;
  uint8_t data[8] = {0}, len = 0;

  _ctrl.receive(RxB::RxB0, id, data, len);
  _on_rx_buf_full(id, data, len);
  _ctrl.clearIntFlag(CANINTF::RX0IF);
}

void ArduinoMCP2515::onReceiveBuffer_1_Full()
{
  uint32_t id = 0;
  uint8_t data[8] = {0}, len = 0;

  _ctrl.receive(RxB::RxB1, id, data, len);
  _on_rx_buf_full(id, data, len);
  _ctrl.clearIntFlag(CANINTF::RX1IF);
}

void ArduinoMCP2515::onTransmitBuffer_0_Empty()
{
  _on_tx_buf_empty(this);
  _ctrl.clearIntFlag(CANINTF::TX0IF);
}

void ArduinoMCP2515::onTransmitBuffer_1_Empty()
{
  _on_tx_buf_empty(this);
  _ctrl.clearIntFlag(CANINTF::TX1IF);
}

void ArduinoMCP2515::onTransmitBuffer_2_Empty()
{
  _on_tx_buf_empty(this);
  _ctrl.clearIntFlag(CANINTF::TX2IF);
}

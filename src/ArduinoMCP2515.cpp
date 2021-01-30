/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-MCP2515/graphs/contributors.
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
  BitRate_1000kBPS_16MHz,
  BitRate_125kBPS_8MHz,
  BitRate_250kBPS_8MHz,
  BitRate_500kBPS_8MHz,
  BitRate_1000kBPS_8MHz
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
                               MicroSecondFunc micros,
                               OnReceiveBufferFullFunc on_rx_buf_full,
                               OnTransmitBufferEmptyFunc on_tx_buf_empty)
: _io{select, deselect, transfer}
, _cfg{_io}
, _ctrl{_io}
, _micros{micros}
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

#if LIBCANARD
bool ArduinoMCP2515::transmit(CanardFrame const & frame)
{
  return transmitCANFrame(CAN_EFF_BITMASK | frame.extended_can_id,
                          reinterpret_cast<uint8_t const *>(frame.payload),
                          static_cast<uint8_t const>(frame.payload_size));
}
#else
bool ArduinoMCP2515::transmit(uint32_t const id, uint8_t const * data, uint8_t const len)
{
  return transmitCANFrame(id, data, len);
}
#endif

void ArduinoMCP2515::onExternalEventHandler()
{
  uint8_t const status = _ctrl.status();

  if(isBitSet(status, bp(STATUS::RX0IF))) onReceiveBuffer_0_Full();
  if(isBitSet(status, bp(STATUS::RX1IF))) onReceiveBuffer_1_Full();
  if(isBitSet(status, bp(STATUS::TX0IF))) onTransmitBuffer_0_Empty();
  if(isBitSet(status, bp(STATUS::TX1IF))) onTransmitBuffer_1_Empty();
  if(isBitSet(status, bp(STATUS::TX2IF))) onTransmitBuffer_2_Empty();
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

bool ArduinoMCP2515::transmitCANFrame(uint32_t const id, uint8_t const * data, uint8_t const len)
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

  return false;
}

void ArduinoMCP2515::onReceiveBuffer_0_Full()
{
  uint32_t id = 0;
  uint8_t data[8] = {0}, len = 0;
  unsigned long const rx_timestamp_us = _micros();

  _ctrl.receive(RxB::RxB0, id, data, len);
  onReceiveBuffer_n_Full(rx_timestamp_us, id, data, len);
  _ctrl.clearIntFlag(CANINTF::RX0IF);
}

void ArduinoMCP2515::onReceiveBuffer_1_Full()
{
  uint32_t id = 0;
  uint8_t data[8] = {0}, len = 0;
  unsigned long const rx_timestamp_us = _micros();

  _ctrl.receive(RxB::RxB1, id, data, len);
  onReceiveBuffer_n_Full(rx_timestamp_us, id, data, len);
  _ctrl.clearIntFlag(CANINTF::RX1IF);
}

void ArduinoMCP2515::onTransmitBuffer_0_Empty()
{
  if (_on_tx_buf_empty)
    _on_tx_buf_empty(this);
  _ctrl.clearIntFlag(CANINTF::TX0IF);
}

void ArduinoMCP2515::onTransmitBuffer_1_Empty()
{
  if (_on_tx_buf_empty)
    _on_tx_buf_empty(this);
  _ctrl.clearIntFlag(CANINTF::TX1IF);
}

void ArduinoMCP2515::onTransmitBuffer_2_Empty()
{
  if (_on_tx_buf_empty)
    _on_tx_buf_empty(this);
  _ctrl.clearIntFlag(CANINTF::TX2IF);
}

void ArduinoMCP2515::onReceiveBuffer_n_Full(unsigned long const timestamp_us, uint32_t const id, uint8_t const * data, uint8_t const len) const
{
  if (_on_rx_buf_full)
  {
#if LIBCANARD
    CanardFrame const frame
    {
      timestamp_us,                        /* timestamp_usec  */
      id & CAN_ADR_BITMASK,                /* extended_can_id limited to 29 bit */
      len,                                 /* payload_size    */
      reinterpret_cast<const void *>(data) /* payload         */
    };
    _on_rx_buf_full(frame);
#else
    _on_rx_buf_full(timestamp_us, id, data, len);
#endif
  }
}

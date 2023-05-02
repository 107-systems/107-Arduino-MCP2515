/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-MCP2515/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <107-Arduino-MCP2515.h>

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
  BitRate_10kBPS_16MHz,
  BitRate_20kBPS_16MHz,
  BitRate_50kBPS_16MHz,
  BitRate_100kBPS_16MHz,
  BitRate_125kBPS_16MHz,
  BitRate_250kBPS_16MHz,
  BitRate_500kBPS_16MHz,
  BitRate_800kBPS_16MHz,
  BitRate_1000kBPS_16MHz,

  BitRate_10kBPS_8MHz,
  BitRate_20kBPS_8MHz,
  BitRate_50kBPS_8MHz,
  BitRate_100kBPS_8MHz,
  BitRate_125kBPS_8MHz,
  BitRate_250kBPS_8MHz,
  BitRate_500kBPS_8MHz,
  BitRate_800kBPS_8MHz,
  BitRate_1000kBPS_8MHz,

  BitRate_10kBPS_10MHz,
  BitRate_20kBPS_10MHz,
  BitRate_50kBPS_10MHz,
  BitRate_1000kBPS_10MHz,
  BitRate_125kBPS_10MHz,
  BitRate_250kBPS_10MHz,
  BitRate_500kBPS_10MHz,
  BitRate_1000kBPS_10MHz,

  BitRate_10kBPS_12MHz,
  BitRate_20kBPS_12MHz,
  BitRate_50kBPS_12MHz,
  BitRate_100kBPS_12MHz,
  BitRate_125kBPS_12MHz,
  BitRate_250kBPS_12MHz,
  BitRate_500kBPS_12MHz,
  BitRate_1000kBPS_12MHz
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
                               OnTransmitBufferEmptyFunc on_tx_buf_empty,
                               OnCanErrorFunc on_error,
                               OnCanWarningFunc on_warning)
: _io{select, deselect, transfer}
, _cfg{_io}
, _ctrl{_io}
, _micros{micros}
, _on_rx_buf_full{on_rx_buf_full}
, _on_tx_buf_empty{on_tx_buf_empty}
, _on_error{on_error}
, _on_warning{on_warning}
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

  if (_on_tx_buf_empty)
  {
    _cfg.enableIntFlag(CANINTE::TX0IE);
    _cfg.enableIntFlag(CANINTE::TX1IE);
    _cfg.enableIntFlag(CANINTE::TX2IE);
  }

  if (_on_rx_buf_full)
  {
    _cfg.enableIntFlag(CANINTE::RX0IE);
    _cfg.enableIntFlag(CANINTE::RX1IE);
  }

  /* Conditionally enable error interrupt
   * only if we have error callbacks
   * registered.
   */
  if (_on_error || _on_warning)
    _cfg.enableIntFlag(CANINTE::ERRIE);
}

void ArduinoMCP2515::setBitRate(CanBitRate const bit_rate)
{
  _cfg.setBitRateConfig(BIT_RATE_CONFIG_ARRAY[static_cast<size_t>(bit_rate)]);
}

void ArduinoMCP2515::enableFilter(RxB const rxb, uint32_t const mask, uint32_t const * filter, size_t const filter_size)
{
  if (rxb == RxB::RxB0)
  {
    if (filter_size >= 1) _cfg.setFilterId_RxF0(filter[0]);
    if (filter_size >= 2) _cfg.setFilterId_RxF1(filter[1]);
    _cfg.setFilterMask_RxB0(mask);
    _cfg.enableFilter_RxB0();
  }
  else
  {
    if (filter_size >= 1) _cfg.setFilterId_RxF2(filter[0]);
    if (filter_size >= 2) _cfg.setFilterId_RxF3(filter[1]);
    if (filter_size >= 3) _cfg.setFilterId_RxF5(filter[2]);
    if (filter_size >= 4) _cfg.setFilterId_RxF3(filter[3]);
    _cfg.setFilterMask_RxB1(mask);
    _cfg.enableFilter_RxB1();
  }
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
  /* Obtain current status and call the appropriate callback
   * handlers to facilitate the necessary actions.
   */
  uint8_t const status = _ctrl.status();

  if(isBitSet(status, bp(STATUS::RX0IF))) onReceiveBuffer_0_Full();
  if(isBitSet(status, bp(STATUS::RX1IF))) onReceiveBuffer_1_Full();
  if(isBitSet(status, bp(STATUS::TX0IF))) onTransmitBuffer_0_Empty();
  if(isBitSet(status, bp(STATUS::TX1IF))) onTransmitBuffer_1_Empty();
  if(isBitSet(status, bp(STATUS::TX2IF))) onTransmitBuffer_2_Empty();


  /* Only perform error checks if a callback has been
   * registered. Otherwise, it's an unnecessary SPI
   * transaction which consumes valuable processing
   * time.
   */
  if (!_on_error && !_on_warning)
    return;

  /* Check if an error flag is set and - should an error flag
   * be set - deal with it.
   */
  uint8_t const error_flag = _ctrl.error();

  bool const is_error = (error_flag > EFLG_ERR_MASK) > 0;
  if (is_error && _on_error)
  {
    _on_error(static_cast<EFLG>(error_flag & EFLG_ERR_MASK));

    /* RX0OVR and RX1OVR need to be cleared manually,
     * otherwise the error will persist and we will
     * not be able to ever again obtain received
     * CAN frames.
     */
    if (isBitSet(error_flag, bp(EFLG::RX0OVR)))
      _ctrl.clearErrFlag(EFLG::RX0OVR);

    if (isBitSet(error_flag, bp(EFLG::RX1OVR)))
      _ctrl.clearErrFlag(EFLG::RX1OVR);
  }

  bool const is_warning = (error_flag > EFLG_WAR_MASK) > 0;
  if (is_warning && _on_warning)
    _on_warning(static_cast<EFLG>(error_flag & EFLG_WAR_MASK));

  /* Finally clear the error interrupt flag so that we are not
   * continuously caught in the ERROR handling loop.
   */
  _ctrl.clearIntFlag(CANINTF::ERRIF);
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

bool ArduinoMCP2515::transmitCANFrame(uint32_t const id, uint8_t const * data, uint8_t const len)
{
  if (isBitClr(_io.readRegister(Register::TXB0CTRL), bp(TXBnCTRL::TXREQ)))
  {
    _ctrl.transmit(TxB::TxB0, id, data, len);
    return true;
  }

#if LIBCANARD
  /* Only use a single transmit buffer in order to prevent unintentional
   * priority inversion while transmitting OpenCyphal/CAN frames.
   */
  return false;
#endif

  if (isBitClr(_io.readRegister(Register::TXB1CTRL), bp(TXBnCTRL::TXREQ)))
  {
    _ctrl.transmit(TxB::TxB1, id, data, len);
    return true;
  }

  if (isBitClr(_io.readRegister(Register::TXB2CTRL), bp(TXBnCTRL::TXREQ)))
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
  _ctrl.clearIntFlag(CANINTF::RX0IF);
  onReceiveBuffer_n_Full(rx_timestamp_us, id, data, len);
}

void ArduinoMCP2515::onReceiveBuffer_1_Full()
{
  uint32_t id = 0;
  uint8_t data[8] = {0}, len = 0;
  unsigned long const rx_timestamp_us = _micros();

  _ctrl.receive(RxB::RxB1, id, data, len);
  _ctrl.clearIntFlag(CANINTF::RX1IF);
  onReceiveBuffer_n_Full(rx_timestamp_us, id, data, len);
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
#if (CANARD_VERSION_MAJOR == 1)
      timestamp_us,                        /* timestamp_usec  */
#endif
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

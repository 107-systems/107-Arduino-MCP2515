/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-MCP2515/graphs/contributors.
 */

#ifndef MCP2515_MCP2515_CONST_H_
#define MCP2515_MCP2515_CONST_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <stdint.h>
#include <stdlib.h>

#undef max
#undef min
#include <array>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace MCP2515
{

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

enum class Register : uint8_t
{
  RXF0SIDH = 0x00,
  RXF0SIDL = 0x01,
  RXF0EID8 = 0x02,
  RXF0EID0 = 0x03,
  RXF1SIDH = 0x04,
  RXF1SIDL = 0x05,
  RXF1EID8 = 0x06,
  RXF1EID0 = 0x07,
  RXF2SIDH = 0x08,
  RXF2SIDL = 0x09,
  RXF2EID8 = 0x0A,
  RXF2EID0 = 0x0B,
  CANSTAT  = 0x0E,
  CANCTRL  = 0x0F,
  RXF3SIDH = 0x10,
  RXF3SIDL = 0x11,
  RXF3EID8 = 0x12,
  RXF3EID0 = 0x13,
  RXF4SIDH = 0x14,
  RXF4SIDL = 0x15,
  RXF4EID8 = 0x16,
  RXF4EID0 = 0x17,
  RXF5SIDH = 0x18,
  RXF5SIDL = 0x19,
  RXF5EID8 = 0x1A,
  RXF5EID0 = 0x1B,
  TEC      = 0x1C,
  REC      = 0x1D,
  RXM0SIDH = 0x20,
  RXM0SIDL = 0x21,
  RXM0EID8 = 0x22,
  RXM0EID0 = 0x23,
  RXM1SIDH = 0x24,
  RXM1SIDL = 0x25,
  RXM1EID8 = 0x26,
  RXM1EID0 = 0x27,
  CNF3     = 0x28,
  CNF2     = 0x29,
  CNF1     = 0x2A,
  CANINTE  = 0x2B,
  CANINTF  = 0x2C,
  EFLG     = 0x2D,
  TXB0CTRL = 0x30,
  TXB0SIDH = 0x31,
  TXB0SIDL = 0x32,
  TXB0EID8 = 0x33,
  TXB0EID0 = 0x34,
  TXB0DLC  = 0x35,
  TXB0DATA = 0x36,
  TXB1CTRL = 0x40,
  TXB1SIDH = 0x41,
  TXB1SIDL = 0x42,
  TXB1EID8 = 0x43,
  TXB1EID0 = 0x44,
  TXB1DLC  = 0x45,
  TXB1DATA = 0x46,
  TXB2CTRL = 0x50,
  TXB2SIDH = 0x51,
  TXB2SIDL = 0x52,
  TXB2EID8 = 0x53,
  TXB2EID0 = 0x54,
  TXB2DLC  = 0x55,
  TXB2DATA = 0x56,
  RXB0CTRL = 0x60,
  RXB0SIDH = 0x61,
  RXB0SIDL = 0x62,
  RXB0EID8 = 0x63,
  RXB0EID0 = 0x64,
  RXB0DLC  = 0x65,
  RXB0DATA = 0x66,
  RXB1CTRL = 0x70,
  RXB1SIDH = 0x71,
  RXB1SIDL = 0x72,
  RXB1EID8 = 0x73,
  RXB1EID0 = 0x74,
  RXB1DLC  = 0x75,
  RXB1DATA = 0x76
};

enum class Instruction : uint8_t
{
  WRITE       = 0x02,
  READ        = 0x03,
  BITMOD      = 0x05,
  LOAD_TX0    = 0x40,
  LOAD_TX1    = 0x42,
  LOAD_TX2    = 0x44,
  RTS_TX0     = 0x81,
  RTS_TX1     = 0x82,
  RTS_TX2     = 0x84,
  RTS_ALL     = 0x87,
  READ_RX0    = 0x90,
  READ_RX1    = 0x94,
  READ_STATUS = 0xA0,
  RX_STATUS   = 0xB0,
  RESET       = 0xC0
};

enum class TXBnDLC : uint8_t
{
  RTR = 2,
};

enum class TXBnSIDL : uint8_t
{
  EXIDE = 3,
};

enum class STATUS : uint8_t
{
  TX2IF  = 7,
  TX2REQ = 6,
  TX1IF  = 5,
  TX1REQ = 4,
  TX0IF  = 3,
  TX0REQ = 2,
  RX1IF  = 1,
  RX0IF  = 0,
};

enum class CANINTE : uint8_t
{
  ERRIE = 5,
  TX2IE = 4,
  TX1IE = 3,
  TX0IE = 2,
  RX1IE = 1,
  RX0IE = 0,
};

enum class CANINTF : uint8_t
{
  MERRF = 7,
  WAKIF = 6,
  ERRIF = 5,
  TX2IF = 4,
  TX1IF = 3,
  TX0IF = 2,
  RX1IF = 1,
  RX0IF = 0
};

enum class RXB0CTRL : uint8_t
{
  RXM1    = 6,
  RXM0    = 5,
  RXRTR   = 3,
  BUKT    = 2,
  BUKT1   = 1,
  FILHIT0 = 0
};

enum class RXB1CTRL : uint8_t
{
  RXM1    = 6,
  RXM0    = 5,
  RXRTR   = 3,
  FILHIT2 = 2,
  FILHIT1 = 1,
  FILHIT0 = 0
};

enum class RXBnSIDL : uint8_t
{
  IDE = 3,
};

enum class RXFnSIDL : uint8_t
{
  EXIDE = 3,
};

enum class CANCTRL : uint8_t
{
  REQOP2 = 7,
  REQOP1 = 6,
  REQOP0 = 5
};

enum class CANSTAT : uint8_t
{
  OPMOD2 = 7,
  OPMOD1 = 6,
  OPMOD0 = 5
};

enum class TXBnCTRL : uint8_t
{
  TXREQ = 3,
};

enum class EFLG : uint8_t
{
  RX1OVR = 7,
  RX0OVR = 6,
  TXBO   = 5,
  TXEP   = 4,
  RXEP   = 3,
  TXWAR  = 2,
  RXWAR  = 1,
  EWARN  = 0,
};

/**************************************************************************************
 * CONVERSION FUNCTIONS
 **************************************************************************************/

template <typename Enumeration>
constexpr auto bp(Enumeration const value) -> typename std::underlying_type<Enumeration>::type
{
    return static_cast<typename std::underlying_type<Enumeration>::type>(value);
}

template <typename Enumeration>
constexpr auto bm(Enumeration const value) -> typename std::underlying_type<Enumeration>::type
{
    return (1 << bp(value));
}

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

const char * toStr(EFLG const err_flag);

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static uint8_t constexpr CANCTRL_REQOP_MASK = bm(CANCTRL::REQOP2) | bm(CANCTRL::REQOP1) | bm(CANCTRL::REQOP0);
static uint8_t constexpr CANSTAT_OP_MASK    = bm(CANSTAT::OPMOD2) | bm(CANSTAT::OPMOD1) | bm(CANSTAT::OPMOD0);
static uint8_t constexpr RXB0CTRL_RXM_MASK  = bm(RXB0CTRL::RXM1)  | bm(RXB0CTRL::RXM0);
static uint8_t constexpr RXB1CTRL_RXM_MASK  = bm(RXB1CTRL::RXM1)  | bm(RXB1CTRL::RXM0);

static uint8_t constexpr EFLG_ERR_MASK      = bm(EFLG::RX1OVR) | bm(EFLG::RX0OVR) | bm(EFLG::TXBO) | bm(EFLG::TXEP) | bm(EFLG::RXEP);
static uint8_t constexpr EFLG_WAR_MASK      = bm(EFLG::TXWAR) | bm(EFLG::RXWAR) | bm(EFLG::EWARN);

static uint32_t constexpr CAN_EFF_BITMASK   = 0x80000000;
static uint32_t constexpr CAN_RTR_BITMASK   = 0x40000000;
static uint32_t constexpr CAN_ERR_BITMASK   = 0x20000000;
static uint32_t constexpr CAN_ADR_BITMASK   = ~(CAN_EFF_BITMASK | CAN_RTR_BITMASK | CAN_ERR_BITMASK);

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* MCP2515 */

#endif /* MCP2515_MCP2515_CONST_H_ */

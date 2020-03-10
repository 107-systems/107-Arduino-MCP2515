/**
 * @brief   Arduino library for controlling the MCP2515 in order to receive/transmit CAN frames.
 * @author  Alexander Entinger, MSc / LXRobotics GmbH
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
 * TYPEDEF
 **************************************************************************************/

typedef struct
{
  uint8_t CNF1;
  uint8_t CNF2;
  uint8_t CNF3;
} CanBitRateConfig;

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

static CanBitRateConfig constexpr BitRate_125kBPS_16MHz  = {0x03, 0xF0, 0x86};
static CanBitRateConfig constexpr BitRate_250kBPS_16MHz  = {0x41, 0xF1, 0x85};
static CanBitRateConfig constexpr BitRate_500kBPS_16MHz  = {0x00, 0xF0, 0x86};
static CanBitRateConfig constexpr BitRate_1000kBPS_16MHz = {0x00, 0xD0, 0x82};

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

ArduinoMCP2515::ArduinoMCP2515(int const cs_pin, OnCanFrameReceiveFunc on_can_frame_rx)
: _io{cs_pin}
, _ctrl{_io}
, _on_can_frame_rx{on_can_frame_rx}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void ArduinoMCP2515::begin()
{
  _io.begin();
  _io.reset();
  configureMCP2515();
}

void ArduinoMCP2515::setBitRate(CanBitRate const bit_rate)
{
  _io.writeRegister(Register::CNF1, BIT_RATE_CONFIG_ARRAY[static_cast<size_t>(bit_rate)].CNF1);
  _io.writeRegister(Register::CNF2, BIT_RATE_CONFIG_ARRAY[static_cast<size_t>(bit_rate)].CNF2);
  _io.writeRegister(Register::CNF3, BIT_RATE_CONFIG_ARRAY[static_cast<size_t>(bit_rate)].CNF3);
}

bool ArduinoMCP2515::transmit(uint32_t const id, uint8_t const * data, uint8_t const len)
{
  uint8_t const status  = _io.status();

  if (isBitClr(status, bp(STATUS::TX0REQ)))
  {
    transmit(TxB::TxB0, id, data, len);
    return true;
  }
  else if (isBitClr(status, bp(STATUS::TX1REQ)))
  {
    transmit(TxB::TxB1, id, data, len);
    return true;
  }
  else if (isBitClr(status, bp(STATUS::TX2REQ)))
  {
    transmit(TxB::TxB2, id, data, len);
    return true;
  }
}

void ArduinoMCP2515::onExternalEventHandler()
{
  uint8_t const status  = _io.status();

  if(isBitSet(status, static_cast<uint8_t>(STATUS::RX0IF)))
  {
    receive(RxB::RxB0);
    clrBit(_io, Register::CANINTF, bp(CANINTF::RX0IF));
  }

  if(isBitSet(status, static_cast<uint8_t>(STATUS::RX1IF)))
  {
    receive(RxB::RxB1);
    clrBit(_io, Register::CANINTF, bp(CANINTF::RX1IF));
  }
}

/**************************************************************************************
 * PRIVATE FUNCTION DEFINITION
 **************************************************************************************/

void ArduinoMCP2515::configureMCP2515()
{
  /* Enable interrupts:
   *   Receive Buffer 0 Full
   *   Receive Buffer 1 Full
   */
  setBit(_io, Register::CANINTE, bp(CANINTE::RX0IE));
  setBit(_io, Register::CANINTE, bp(CANINTE::RX1IE));
  /* Turn masks/filters off */
  setBit(_io, Register::RXB0CTRL, bp(RXB0CTRL::RXM1));
  setBit(_io, Register::RXB0CTRL, bp(RXB0CTRL::RXM0));
  setBit(_io, Register::RXB1CTRL, bp(RXB1CTRL::RXM1));
  setBit(_io, Register::RXB1CTRL, bp(RXB1CTRL::RXM0));
  /* Enable roll-over to RXB1 if RXB0 is full */
  //setBit(_io, Register::RXB0CTRL, bp(RXB0CTRL::BUKT));
}

void ArduinoMCP2515::transmit(TxB const txb, uint32_t const id, uint8_t const * data, uint8_t const len)
{
  RxTxBuffer tx_buffer;

  bool const is_ext = (id & CAN_EFF_BITMASK) == CAN_EFF_BITMASK;
  bool const is_rtr = (id & CAN_RTR_BITMASK) == CAN_RTR_BITMASK;

  /* Load address registers */
  /*  ID[28:27] = EID[17:16]
   *  ID[26:19] = EID[15: 8]
   *  ID[18:11] = EID[ 7: 0]
   *  ID[10: 3] = SID[10: 3]
   *  ID[ 3: 0] = SID[ 3: 0]
   */
  tx_buffer.reg.sidl = static_cast<uint8_t>((id & 0x00000007) << 5);
  tx_buffer.reg.sidh = static_cast<uint8_t>((id & 0x000007F8) >> 3);
  if(is_ext)
  {
    tx_buffer.reg.sidl |= static_cast<uint8_t>((id & 0x18000000) >> 27);
    tx_buffer.reg.sidl |= (1 << static_cast<uint8_t>(TXBnSIDL::EXIDE));
    tx_buffer.reg.eid0  = static_cast<uint8_t>((id & 0x0007F800) >> 11);
    tx_buffer.reg.eid8  = static_cast<uint8_t>((id & 0x07F80000) >> 19);
  }
  else
  {
    tx_buffer.reg.eid0  = 0;
    tx_buffer.reg.eid8  = 0;
  }

  /* Load data length register */
  tx_buffer.reg.dlc = is_rtr ? (len | (1 << static_cast<uint8_t>(TXBnDLC::RTR))) : len;

  /* Load data buffer */
  memcpy(tx_buffer.reg.data, data, len);

  /* Write to transmit buffer */
  _io.loadTxBuffer(txb, tx_buffer.buf);

  /* Request transmission */
  _io.requestTx(txb);
}

void ArduinoMCP2515::receive(RxB const rxb)
{
  RxTxBuffer rx_buffer;

  /* Read content of receive buffer */
  _io.readRxBuffer(rxb, rx_buffer.buf);

  /* Assemble ID from registers */
  uint32_t id = (static_cast<uint32_t>(rx_buffer.reg.sidh) << 3) + (static_cast<uint32_t>(rx_buffer.reg.sidl) >> 5);

  /* Read amount of bytes received */
  uint8_t const len = rx_buffer.reg.dlc & 0x0F;

  /* Call registered callback with received data */
  _on_can_frame_rx(id, rx_buffer.reg.data, len);
}

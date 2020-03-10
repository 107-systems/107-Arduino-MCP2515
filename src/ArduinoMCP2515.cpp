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
 * STATIC MEMBER INITIALISATION
 **************************************************************************************/

static ArduinoMCP2515 * this_ptr = nullptr;

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

ArduinoMCP2515::ArduinoMCP2515(int const cs_pin,
                               int const int_pin,
                               OnCanFrameReceiveFunc on_can_frame_rx)
: _io{cs_pin}
, _ctrl{_io}
, _int_pin{int_pin}
, _on_can_frame_rx{on_can_frame_rx}
{
  this_ptr = this;
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void ArduinoMCP2515::begin()
{
  _io.begin();
  _io.reset();
  configureEventCallback();
  configureMCP2515();
}

void ArduinoMCP2515::setBitRate(CanBitRate const bit_rate)
{
  _io.writeRegister(MCP2515::Register::CNF1, BIT_RATE_CONFIG_ARRAY[static_cast<size_t>(bit_rate)].CNF1);
  _io.writeRegister(MCP2515::Register::CNF2, BIT_RATE_CONFIG_ARRAY[static_cast<size_t>(bit_rate)].CNF2);
  _io.writeRegister(MCP2515::Register::CNF3, BIT_RATE_CONFIG_ARRAY[static_cast<size_t>(bit_rate)].CNF3);
}

bool ArduinoMCP2515::transmit(uint32_t const id, uint8_t const * data, uint8_t const len)
{
  bool msg_tx_success = false;

  std::for_each(MCP2515::TX_BUFFERS.cbegin(),
                MCP2515::TX_BUFFERS.cend(),
                [&](MCP2515::TxBuffer const tx_buf)
                {
                  uint8_t const ctrl_val = _io.readRegister(tx_buf.CTRL);
                  if(isBitClr(ctrl_val, static_cast<uint8_t>(MCP2515::TXBnCTRL::TXREQ))) {
                    transmit(tx_buf.SIDH, tx_buf.CTRL, id, data, len);
                    msg_tx_success = true;
                    return;
                  }
                });

  return msg_tx_success;
}

void ArduinoMCP2515::onExternalEvent()
{
  this_ptr->onExternalEventHandler();
}

/**************************************************************************************
 * PRIVATE FUNCTION DEFINITION
 **************************************************************************************/

void ArduinoMCP2515::configureEventCallback()
{
  pinMode(_int_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(_int_pin), ArduinoMCP2515::onExternalEvent, FALLING);
}

void ArduinoMCP2515::configureMCP2515()
{
  /* Enable interrupts:
   *   Receive Buffer 0 Full
   *   Receive Buffer 1 Full
   */
  _io.setBit(MCP2515::Register::CANINTE, static_cast<uint8_t>(MCP2515::CANINTE::RX0IE));
  _io.setBit(MCP2515::Register::CANINTE, static_cast<uint8_t>(MCP2515::CANINTE::RX1IE));
  /* Turn masks/filters off */
  _io.setBit(MCP2515::Register::RXB0CTRL, static_cast<uint8_t>(MCP2515::RXB0CTRL::RXM1));
  _io.setBit(MCP2515::Register::RXB0CTRL, static_cast<uint8_t>(MCP2515::RXB0CTRL::RXM0));
  _io.setBit(MCP2515::Register::RXB1CTRL, static_cast<uint8_t>(MCP2515::RXB1CTRL::RXM1));
  _io.setBit(MCP2515::Register::RXB1CTRL, static_cast<uint8_t>(MCP2515::RXB1CTRL::RXM0));
  /* Enable roll-over to RXB1 if RXB0 is full */
  //_io.setBit(MCP2515::Register::RXB0CTRL, static_cast<uint8_t>(MCP2515::RXB0CTRL::BUKT));
}

void ArduinoMCP2515::transmit(MCP2515::Register const tx_buf_sidh, MCP2515::Register const tx_buf_ctrl, uint32_t const id, uint8_t const * data, uint8_t const len)
{
  union TxBuffer
  {
    struct
    {
      uint8_t sidh;
      uint8_t sidl;
      uint8_t eid8;
      uint8_t eid0;
      uint8_t dlc;
      uint8_t data[8];
    } reg;
    uint8_t buf[5+8];
  };
  static_assert(sizeof(TxBuffer) == 13, "Union TxBuffer exceeds expected size of 13 bytes");

  TxBuffer tx_buffer;

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
    tx_buffer.reg.sidl |= (1 << static_cast<uint8_t>(MCP2515::TXBnSIDL::EXIDE));
    tx_buffer.reg.eid0  = static_cast<uint8_t>((id & 0x0007F800) >> 11);
    tx_buffer.reg.eid8  = static_cast<uint8_t>((id & 0x07F80000) >> 19);
  }
  else
  {
    tx_buffer.reg.eid0  = 0;
    tx_buffer.reg.eid8  = 0;
  }

  /* Load data length register */
  tx_buffer.reg.dlc = is_rtr ? (len | (1 << static_cast<uint8_t>(MCP2515::TXBnDLC::RTR))) : len;

  /* Load data buffer */
  memcpy(tx_buffer.reg.data, data, len);

  /* Write to transmit buffer */
  _io.writeRegister(tx_buf_sidh, tx_buffer.buf, sizeof(tx_buffer));

  /* Request transmission */
  _io.setBit(tx_buf_ctrl, static_cast<uint8_t>(MCP2515::TXBnCTRL::TXREQ));
}

void ArduinoMCP2515::receive(MCP2515::Register const rx_buf_ctrl)
{
  union RxBuffer
  {
    struct
    {
      uint8_t ctrl;
      uint8_t sidh;
      uint8_t sidl;
      uint8_t eid8;
      uint8_t eid0;
      uint8_t dlc;
      uint8_t data[8];
    } reg;
    uint8_t buf[6+8];
  };
  static_assert(sizeof(RxBuffer) == 14, "Union RxBuffer exceeds expected size of 14 bytes");

  /* Read content of receive buffer */
  RxBuffer rx_buffer;
  _io.readRegister(rx_buf_ctrl, rx_buffer.buf, sizeof(rx_buffer));

  /* Assemble ID from registers */
  uint32_t id = (static_cast<uint32_t>(rx_buffer.reg.sidh) << 3) + (static_cast<uint32_t>(rx_buffer.reg.sidl) >> 5);

  /* Read amount of bytes received */
  uint8_t const len = rx_buffer.reg.dlc & 0x0F;

  /* Call registered callback with received data */
  _on_can_frame_rx(id, rx_buffer.reg.data, len);
}

void ArduinoMCP2515::onExternalEventHandler()
{
  uint8_t const status  = _io.status();

  if(isBitSet(status, static_cast<uint8_t>(MCP2515::STATUS::RX0IF)))
  {
    receive(MCP2515::Register::RXB0CTRL);
    _io.clrBit(MCP2515::Register::CANINTF, static_cast<uint8_t>(MCP2515::CANINTF::RX0IF));
  }

  if(isBitSet(status, static_cast<uint8_t>(MCP2515::STATUS::RX1IF)))
  {
    receive(MCP2515::Register::RXB1CTRL);
    _io.clrBit(MCP2515::Register::CANINTF, static_cast<uint8_t>(MCP2515::CANINTF::RX1IF));
  }
/*
  uint8_t const canintf = _io.readRegister(MCP2515::Register::CANINTF);
  uint8_t const eflg    = _io.readRegister(MCP2515::Register::EFLG);

  Serial.print("CANINTF = ");
  Serial.println(canintf, HEX);
  Serial.print("EFLG    = ");
  Serial.println(eflg, HEX);
*/
}

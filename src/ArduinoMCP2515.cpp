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
 * GLOBAL CONSTANTS
 **************************************************************************************/

static MCP2515_CanBitRateConfig constexpr BitRate_125kBPS_16MHz  = {0x03, 0xF0, 0x86};
static MCP2515_CanBitRateConfig constexpr BitRate_250kBPS_16MHz  = {0x41, 0xF1, 0x85};
static MCP2515_CanBitRateConfig constexpr BitRate_500kBPS_16MHz  = {0x00, 0xF0, 0x86};
static MCP2515_CanBitRateConfig constexpr BitRate_1000kBPS_16MHz = {0x00, 0xD0, 0x82};

static MCP2515_CanBitRateConfig const BIT_RATE_CONFIG_ARRAY[] =
{
  BitRate_125kBPS_16MHz,
  BitRate_250kBPS_16MHz,
  BitRate_500kBPS_16MHz,
  BitRate_1000kBPS_16MHz
};

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

ArduinoMCP2515::ArduinoMCP2515(int const cs_pin,
                               int const int_pin,
                               OnCanFrameReceiveFunc on_can_frame_rx)
: _io{cs_pin}
, _event{int_pin}
, _on_can_frame_rx{on_can_frame_rx}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void ArduinoMCP2515::begin()
{
  _io.begin();
  _event.begin();

  _io.reset();
}

void ArduinoMCP2515::setBitRate(CanBitRate const bit_rate)
{
  setBitRateConfig(BIT_RATE_CONFIG_ARRAY[static_cast<size_t>(bit_rate)]);
}

bool ArduinoMCP2515::transmit(uint32_t const id, uint8_t const * data, uint8_t const len)
{
  std::for_each(MCP2515_TX_BUFFERS.cbegin(),
                MCP2515_TX_BUFFERS.cend(),
                [=](TxBuffer const tx_buf)
                {
                  uint8_t const ctrl_val = _io.readRegister(tx_buf.CTRL);
                  if(isBitClr(ctrl_val, static_cast<uint8_t>(TXBnCTRL::TXREQ))) {
                    return transmit(tx_buf, id, data, len);
                  }
                });

  return false;
}

/**************************************************************************************
 * PRIVATE FUNCTION DEFINITION
 **************************************************************************************/

bool ArduinoMCP2515::setMode(MCP2515_Mode const mode)
{
  uint8_t const mode_val = static_cast<uint8_t>(mode);

  _io.modifyRegister(Register::CANCTRL, MCP2515_CANCTRL_REQOP_MASK, mode_val);

  for(unsigned long const start = millis(); (millis() - start) < 10; )
  {
    uint8_t const canstat_op_mode = (_io.readRegister(Register::CANSTAT) & MCP2515_CANSTAT_OP_MASK);
    if(canstat_op_mode == mode_val) {
      return true;
    }
  }

  return false;
}

void ArduinoMCP2515::setBitRateConfig(MCP2515_CanBitRateConfig const bit_rate_config)
{
  _io.writeRegister(Register::CNF1, bit_rate_config.CNF1);
  _io.writeRegister(Register::CNF2, bit_rate_config.CNF2);
  _io.writeRegister(Register::CNF3, bit_rate_config.CNF3);
}

bool ArduinoMCP2515::transmit(TxBuffer const tx_buf, uint32_t const id, uint8_t const * data, uint8_t const len)
{
  bool const is_ext = (id & CAN_EFF_BITMASK) == CAN_EFF_BITMASK;
  bool const is_rtr = (id & CAN_RTR_BITMASK) == CAN_RTR_BITMASK;

  /* Load address registers */
  /*  ID[28:27] = EID[17:16]
   *  ID[26:19] = EID[15: 8]
   *  ID[18:11] = EID[ 7: 0]
   *  ID[10: 3] = SID[10: 3]
   *  ID[ 3: 0] = SID[ 3: 0]
   */
  uint8_t sidl = static_cast<uint8_t>((id & 0x00000007) << 5);
  _io.writeRegister(tx_buf.SIDH, static_cast<uint8_t>((id & 0x000007F8) >> 3));
  if(is_ext)
  {
    sidl |= static_cast<uint8_t>((id & 0x18000000) >> 27);
    sidl |= static_cast<uint8_t>(TXBnSIDL::EXIDE);
    _io.writeRegister(tx_buf.EID0, static_cast<uint8_t>((id & 0x0007F800) >> 11));
    _io.writeRegister(tx_buf.EID8, static_cast<uint8_t>((id & 0x07F80000) >> 19));
  }
  else
  {
    _io.writeRegister(tx_buf.EID0, 0);
    _io.writeRegister(tx_buf.EID8, 0);
  }
  _io.writeRegister(tx_buf.SIDL, sidl);

  /* Load data length register */
  uint8_t const dlc = is_rtr ? (len | static_cast<uint8_t>(TXBnDLC::RTR)) : len;
  _io.writeRegister(tx_buf.DLC, dlc);

  /* Load data buffer */
  _io.writeRegister(tx_buf.DATA, data, len);

  /* Request transmission */
  _io.setBit(tx_buf.CTRL, static_cast<uint8_t>(TXBnCTRL::TXREQ));

  return true;
}

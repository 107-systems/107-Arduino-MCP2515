/**
 * @brief   Arduino library for controlling the MCP2515 in order to receive/transmit CAN frames.
 * @author  Alexander Entinger, MSc / LXRobotics GmbH
 * @license LGPL 3.0
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <ArduinoMCP2515.h>

#include <sstream>
#include <iomanip>
#include <algorithm>

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
}

void ArduinoMCP2515::setBitRate(CanBitRate const bit_rate)
{
  setBitRateConfig(BIT_RATE_CONFIG_ARRAY[static_cast<size_t>(bit_rate)]);
}

bool ArduinoMCP2515::transmit(uint32_t const id, uint8_t const * data, uint8_t const len)
{
  return false; /* TODO */
}

/**************************************************************************************
 * PRIVATE FUNCTION DEFINITION
 **************************************************************************************/

bool ArduinoMCP2515::setMode(Mode const mode)
{
  uint8_t const mode_val = static_cast<uint8_t>(mode);

  _io.modifyRegister(Register::CANCTRL, CANCTRL_REQOP_MASK, mode_val);

  for(unsigned long const start = millis(); (millis() - start) < 10; )
  {
    uint8_t const canstat_op_mode = (_io.readRegister(Register::CANSTAT) & CANSTAT_OP_MASK);
    if(canstat_op_mode == mode_val) {
      return true;
    }
  }

  return false;
}

void ArduinoMCP2515::setBitRateConfig(CanBitRateConfig const bit_rate_config)
{
  _io.writeRegister(Register::CNF1, bit_rate_config.CNF1);
  _io.writeRegister(Register::CNF2, bit_rate_config.CNF2);
  _io.writeRegister(Register::CNF3, bit_rate_config.CNF3);
}

/**************************************************************************************
 * PUBLIC FUNCTION DEFINITION
 **************************************************************************************/

std::string toStr(uint32_t const id, uint8_t const * data, uint8_t const len)
{
  std::stringstream ss;
  ss << std::hex
     << "ID: " << std::setw(4) << std::setfill(' ') << id << " DATA: ";
  std::for_each(data,
                data + len,
                [&ss](uint8_t const data)
                {
                  ss << std::setw(2) << std::setfill('0') << data << " ";
                });
  return ss.str();
}

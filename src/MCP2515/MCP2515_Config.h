/**
 * @brief   Arduino library for controlling the MCP2515 in order to receive/transmit CAN frames.
 * @license LGPL 3.0
 */

#ifndef MCP2515_MCP2515_CONFIG_H_
#define MCP2515_MCP2515_CONFIG_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "MCP2515_Io.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace MCP2515
{

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

enum class Mode : uint8_t
{
  Normal     = 0x00,
  Sleep      = 0x20,
  Loopback   = 0x40,
  ListenOnly = 0x60,
  Config     = 0x80
};

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

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

class MCP2515_Config
{

public:

  MCP2515_Config(MCP2515_Io & io);


         bool setMode            (Mode const mode);
         void setBitRateConfig   (CanBitRateConfig const bit_rate_config);

  inline void enableIntFlag      (CANINTE const int_flag) { _io.setBit(Register::CANINTE, bp(int_flag)); }
  inline void disableFilter_RxB0 ()                       { _io.modifyRegister(Register::RXB0CTRL, RXB0CTRL_RXM_MASK, RXB0CTRL_RXM_MASK); }
  inline void disableFilter_RxB1 ()                       { _io.modifyRegister(Register::RXB1CTRL, RXB1CTRL_RXM_MASK, RXB1CTRL_RXM_MASK); }
  inline void enableRollover_RxB0()                       { _io.setBit(Register::RXB0CTRL, bp(RXB0CTRL::BUKT)); }


private:

  MCP2515_Io & _io;

};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* MCP2515 */

#endif /* MCP2515_MCP2515_CONFIG_H_ */

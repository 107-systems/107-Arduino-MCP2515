/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-MCP2515/graphs/contributors.
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

static CanBitRateConfig constexpr BitRate_250kBPS_8MHz  = {0x00, 0xb1, 0x05};
static CanBitRateConfig constexpr BitRate_500kBPS_8MHz  = {0x00, 0x90, 0x02};
static CanBitRateConfig constexpr BitRate_125kBPS_8MHz  = {0x01, 0xb1, 0x05};
static CanBitRateConfig constexpr BitRate_1000kBPS_8MHz = {0x00, 0x80, 0x00};

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

class MCP2515_Config
{
public:
  MCP2515_Config(MCP2515_Io & io);


         bool setMode            (Mode const mode);
         void setBitRateConfig   (CanBitRateConfig const bit_rate_config);

  inline void setFilterId_RxF0  (uint32_t const id)   { setFilterId(Register::RXF0SIDL, id); }
  inline void setFilterId_RxF1  (uint32_t const id)   { setFilterId(Register::RXF1SIDL, id); }
  inline void setFilterId_RxF2  (uint32_t const id)   { setFilterId(Register::RXF2SIDL, id); }
  inline void setFilterId_RxF3  (uint32_t const id)   { setFilterId(Register::RXF3SIDL, id); }
  inline void setFilterId_RxF4  (uint32_t const id)   { setFilterId(Register::RXF4SIDL, id); }
  inline void setFilterId_RxF5  (uint32_t const id)   { setFilterId(Register::RXF5SIDL, id); }
  inline void setFilterMask_RxB0(uint32_t const mask) { setFilterMask(Register::RXM0SIDH, mask); }
  inline void setFilterMask_RxB1(uint32_t const mask) { setFilterMask(Register::RXM1SIDH, mask); }

  inline void enableIntFlag      (CANINTE const int_flag) { _io.setBit(Register::CANINTE, bp(int_flag)); }
  inline void disableFilter_RxB0 ()                       { _io.modifyRegister(Register::RXB0CTRL, RXB0CTRL_RXM_MASK, RXB0CTRL_RXM_MASK); }
  inline void disableFilter_RxB1 ()                       { _io.modifyRegister(Register::RXB1CTRL, RXB1CTRL_RXM_MASK, RXB1CTRL_RXM_MASK); }
  inline void enableRollover_RxB0()                       { _io.setBit(Register::RXB0CTRL, bp(RXB0CTRL::BUKT)); }
  inline void enableFilter_RxB0  ()                       { _io.modifyRegister(Register::RXB0CTRL, RXB0CTRL_RXM_MASK, 0); }
  inline void enableFilter_RxB1  ()                       { _io.modifyRegister(Register::RXB1CTRL, RXB1CTRL_RXM_MASK, 0); }


private:
  MCP2515_Io & _io;

  void setFilterId  (Register const rxf_n_sidh, uint32_t const id);
  void setFilterMask(Register const rxm_n_sidh, uint32_t const mask);
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* MCP2515 */

#endif /* MCP2515_MCP2515_CONFIG_H_ */

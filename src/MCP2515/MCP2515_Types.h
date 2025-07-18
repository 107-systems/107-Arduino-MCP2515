/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-MCP2515/graphs/contributors.
 */

#ifndef ARDUINO_MCP2515_TYPES_H_
#define ARDUINO_MCP2515_TYPES_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <functional>
#include <cstdint>

#include "MCP2515/MCP2515_Const.h"

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

typedef std::function<unsigned long()> MicroSecondFunc;
typedef std::function<unsigned long()> MilliSecondFunc;
class ArduinoMCP2515;
typedef std::function<void(ArduinoMCP2515 *)> OnTransmitBufferEmptyFunc;
typedef std::function<void(MCP2515::EFLG const)> OnCanErrorFunc;
typedef std::function<void(MCP2515::EFLG const)> OnCanWarningFunc;

enum class CanBitRate : size_t
{
  BR_10kBPS_16MHZ = 0,
  BR_20kBPS_16MHZ,
  BR_50kBPS_16MHZ,
  BR_100kBPS_16MHZ,
  BR_125kBPS_16MHZ,
  BR_250kBPS_16MHZ,
  BR_500kBPS_16MHZ,
  BR_800kBPS_16MHZ,
  BR_1000kBPS_16MHZ,

  BR_10kBPS_8MHZ,
  BR_20kBPS_8MHZ,
  BR_50kBPS_8MHZ,
  BR_100kBPS_8MHZ,
  BR_125kBPS_8MHZ,
  BR_250kBPS_8MHZ,
  BR_500kBPS_8MHZ,
  BR_800kBPS_8MHZ,
  BR_1000kBPS_8MHZ,

  BR_10kBPS_10MHZ,
  BR_20kBPS_10MHZ,
  BR_50kBPS_10MHZ,
  BR_100kBPS_10MHZ,
  BR_125kBPS_10MHZ,
  BR_250kBPS_10MHZ,
  BR_500kBPS_10MHZ,
  BR_1000kBPS_10MHZ,

  BR_10kBPS_12MHZ,
  BR_20kBPS_12MHZ,
  BR_50kBPS_12MHZ,
  BR_100kBPS_12MHZ,
  BR_125kBPS_12MHZ,
  BR_250kBPS_12MHZ,
  BR_500kBPS_12MHZ,
  BR_1000kBPS_12MHZ
};

#endif /* ARDUINO_MCP2515_TYPES_H_ */

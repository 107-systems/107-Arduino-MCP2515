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
#if LIBCANARD
typedef std::function<void(CanardFrame const & frame)> OnReceiveBufferFullFunc;
#else
typedef std::function<void(uint32_t const, uint32_t const, uint8_t const *, uint8_t const)> OnReceiveBufferFullFunc;
#endif
class ArduinoMCP2515;
typedef std::function<void(ArduinoMCP2515 *)> OnTransmitBufferEmptyFunc;
typedef std::function<void(MCP2515::EFLG const)> OnCanErrorFunc;
typedef std::function<void(MCP2515::EFLG const)> OnCanWarningFunc;

#endif /* ARDUINO_MCP2515_TYPES_H_ */

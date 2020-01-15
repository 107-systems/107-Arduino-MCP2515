/**
 * @brief   Arduino library for controlling the MCP2515 in order to receive/transmit CAN frames.
 * @author  Alexander Entinger, MSc / LXRobotics GmbH
 * @license LGPL 3.0
 */

#ifndef MCP2515_MCP2515_TYPES_H_
#define MCP2515_MCP2515_TYPES_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <stdint.h>

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

enum class MCP2515_Mode : uint8_t
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
} MCP2515_CanBitRateConfig;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static uint8_t constexpr MCP2515_CANCTRL_REQOP_MASK = 0xE0;
static uint8_t constexpr MCP2515_CANSTAT_OP_MASK    = 0xE0;

#endif /* MCP2515_MCP2515_TYPES_H_ */

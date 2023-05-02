/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-MCP2515/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "MCP2515_Const.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace MCP2515
{

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

const char * toStr(EFLG const err_flag)
{
  switch(err_flag)
  {
    case EFLG::RX1OVR: return "RX1OVR"; break;
    case EFLG::RX0OVR: return "RX0OVR"; break;
    case EFLG::TXBO  : return "TXBO";   break;
    case EFLG::TXEP  : return "TXEP";   break;
    case EFLG::RXEP  : return "RXEP";   break;
    case EFLG::TXWAR : return "TXWAR";  break;
    case EFLG::RXWAR : return "RXWAR";  break;
    case EFLG::EWARN : return "EWARN";  break;
    default: __builtin_unreachable(); return ""; break;
  }
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* MCP2515 */

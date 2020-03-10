/**
 * @brief   Arduino library for controlling the MCP2515 in order to receive/transmit CAN frames.
 * @author  Alexander Entinger, MSc / LXRobotics GmbH
 * @license LGPL 3.0
 */

#ifndef MCP2515_MCP2515_IO_H_
#define MCP2515_MCP2515_IO_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <stdint.h>

#include <Arduino.h>
#include <SPI.h>

#include "MCP2515_Const.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace MCP2515
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class MCP2515_Io
{

public:

  MCP2515_Io(int const cs_pin);


  void    begin();

  uint8_t readRegister  (Register const reg);
  void    readRegister  (Register const reg, uint8_t * data, uint8_t const len);
  void    writeRegister (Register const reg, uint8_t const data);
  void    writeRegister (Register const reg, uint8_t const * data, uint8_t const len);
  void    modifyRegister(Register const reg, uint8_t const mask, uint8_t const data);

  void    reset();
  uint8_t status();

private:

  int const _cs_pin;

  inline void init_cs () { pinMode(_cs_pin, OUTPUT); deselect(); }
  inline void init_spi() { SPI.begin(); }

  inline void select  () { digitalWrite(_cs_pin, LOW);  }
  inline void deselect() { digitalWrite(_cs_pin, HIGH); }

};

/**************************************************************************************
 * FREE FUNCTION DECLARATION
 **************************************************************************************/

void setBit(MCP2515_Io & io, Register const reg, uint8_t const bit_pos);
void clrBit(MCP2515_Io & io, Register const reg, uint8_t const bit_pos);

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* MCP2515 */

#endif /* MCP2515_MCP2515_IO_H_ */

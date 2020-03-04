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
#include <stdlib.h>

#undef max
#undef min
#include <array>

#include <Arduino.h>
#include <SPI.h>

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

enum class Register : uint8_t
{
  RXF0SIDH = 0x00,
  RXF0SIDL = 0x01,
  RXF0EID8 = 0x02,
  RXF0EID0 = 0x03,
  RXF1SIDH = 0x04,
  RXF1SIDL = 0x05,
  RXF1EID8 = 0x06,
  RXF1EID0 = 0x07,
  RXF2SIDH = 0x08,
  RXF2SIDL = 0x09,
  RXF2EID8 = 0x0A,
  RXF2EID0 = 0x0B,
  CANSTAT  = 0x0E,
  CANCTRL  = 0x0F,
  RXF3SIDH = 0x10,
  RXF3SIDL = 0x11,
  RXF3EID8 = 0x12,
  RXF3EID0 = 0x13,
  RXF4SIDH = 0x14,
  RXF4SIDL = 0x15,
  RXF4EID8 = 0x16,
  RXF4EID0 = 0x17,
  RXF5SIDH = 0x18,
  RXF5SIDL = 0x19,
  RXF5EID8 = 0x1A,
  RXF5EID0 = 0x1B,
  TEC      = 0x1C,
  REC      = 0x1D,
  RXM0SIDH = 0x20,
  RXM0SIDL = 0x21,
  RXM0EID8 = 0x22,
  RXM0EID0 = 0x23,
  RXM1SIDH = 0x24,
  RXM1SIDL = 0x25,
  RXM1EID8 = 0x26,
  RXM1EID0 = 0x27,
  CNF3     = 0x28,
  CNF2     = 0x29,
  CNF1     = 0x2A,
  CANINTE  = 0x2B,
  CANINTF  = 0x2C,
  EFLG     = 0x2D,
  TXB0CTRL = 0x30,
  TXB0SIDH = 0x31,
  TXB0SIDL = 0x32,
  TXB0EID8 = 0x33,
  TXB0EID0 = 0x34,
  TXB0DLC  = 0x35,
  TXB0DATA = 0x36,
  TXB1CTRL = 0x40,
  TXB1SIDH = 0x41,
  TXB1SIDL = 0x42,
  TXB1EID8 = 0x43,
  TXB1EID0 = 0x44,
  TXB1DLC  = 0x45,
  TXB1DATA = 0x46,
  TXB2CTRL = 0x50,
  TXB2SIDH = 0x51,
  TXB2SIDL = 0x52,
  TXB2EID8 = 0x53,
  TXB2EID0 = 0x54,
  TXB2DLC  = 0x55,
  TXB2DATA = 0x56,
  RXB0CTRL = 0x60,
  RXB0SIDH = 0x61,
  RXB0SIDL = 0x62,
  RXB0EID8 = 0x63,
  RXB0EID0 = 0x64,
  RXB0DLC  = 0x65,
  RXB0DATA = 0x66,
  RXB1CTRL = 0x70,
  RXB1SIDH = 0x71,
  RXB1SIDL = 0x72,
  RXB1EID8 = 0x73,
  RXB1EID0 = 0x74,
  RXB1DLC  = 0x75,
  RXB1DATA = 0x76
};

enum class Instruction : uint8_t
{
  WRITE       = 0x02,
  READ        = 0x03,
  BITMOD      = 0x05,
  LOAD_TX0    = 0x40,
  LOAD_TX1    = 0x42,
  LOAD_TX2    = 0x44,
  RTS_TX0     = 0x81,
  RTS_TX1     = 0x82,
  RTS_TX2     = 0x84,
  RTS_ALL     = 0x87,
  READ_RX0    = 0x90,
  READ_RX1    = 0x94,
  READ_STATUS = 0xA0,
  RX_STATUS   = 0xB0,
  RESET       = 0xC0
};

typedef struct TxBuffer
{
  Register CTRL, SIDH, SIDL, EID8, EID0, DLC, DATA;
};

enum class TXBnCTRL : uint8_t
{
  TXREQ = 0x08,
};

enum class TXBnDLC : uint8_t
{
  RTR = 0x04,
};

enum class TXBnSIDL : uint8_t
{
  EXIDE = 0x08,
};

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

size_t constexpr MCP2515_NUM_TX_BUFFERS = 3;

TxBuffer constexpr MCP2515_TX_BUFFER_0 = {Register::TXB0CTRL, Register::TXB0SIDH, Register::TXB0SIDL, Register::TXB0EID8, Register::TXB0EID0, Register::TXB0DLC, Register::TXB0DATA};
TxBuffer constexpr MCP2515_TX_BUFFER_1 = {Register::TXB1CTRL, Register::TXB1SIDH, Register::TXB1SIDL, Register::TXB1EID8, Register::TXB1EID0, Register::TXB1DLC, Register::TXB1DATA};
TxBuffer constexpr MCP2515_TX_BUFFER_2 = {Register::TXB2CTRL, Register::TXB2SIDH, Register::TXB2SIDL, Register::TXB2EID8, Register::TXB2EID0, Register::TXB2DLC, Register::TXB2DATA};

std::array<TxBuffer, MCP2515_NUM_TX_BUFFERS> constexpr MCP2515_TX_BUFFERS = {MCP2515_TX_BUFFER_0, MCP2515_TX_BUFFER_1, MCP2515_TX_BUFFER_2};

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class MCP2515_Io
{

public:

  MCP2515_Io(int const cs_pin);


  void    begin();

  uint8_t readRegister  (Register const reg);
  void    writeRegister (Register const reg, uint8_t const data);
  void    writeRegister (Register const reg, uint8_t const * data, uint8_t const len);
  void    modifyRegister(Register const reg, uint8_t const mask, uint8_t const data);
  void    setBit        (Register const reg, uint8_t const bit_pos);
  void    clrBit        (Register const reg, uint8_t const bit_pos);

  void    reset();

private:

  int const _cs_pin;

  inline void init_cs () { pinMode(_cs_pin, OUTPUT); deselect(); }
  inline void init_spi() { SPI.begin(); }

  inline void select  () { digitalWrite(_cs_pin, LOW);  }
  inline void deselect() { digitalWrite(_cs_pin, HIGH); }

};

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

inline bool isBitSet(uint8_t const reg_val, uint8_t const bit_mask)
{
  return ((reg_val & bit_mask) == bit_mask);
}

inline bool isBitClr(uint8_t const reg_val, uint8_t const bit_mask)
{
  return !isBitSet(reg_val, bit_mask);
}

#endif /* MCP2515_MCP2515_IO_H_ */

/**
 * @brief   Arduino library for controlling the MCP2515 in order to receive/transmit CAN frames.
 * @author  Alexander Entinger, MSc / LXRobotics GmbH
 * @license LGPL 3.0
 */

#ifndef MCP2515_MCP2515_EVENT_H_
#define MCP2515_MCP2515_EVENT_H_

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class MCP2515_Event
{

public:

  MCP2515_Event(int const int_pin);

         void begin();
  static void onExternalEvent();
  

private:

  int const _int_pin;

};

#endif /* MCP2515_MCP2515_EVENT_H_ */

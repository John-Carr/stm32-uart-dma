/*
 * GPS.hpp
 *
 *  Created on: Apr 21, 2022
 *      Author: John Carr
 */

#ifndef SOLARGATORSBSP_DRIVERS_INC_GPS_HPP_
#define SOLARGATORSBSP_DRIVERS_INC_GPS_HPP_

#include "main.h"

namespace SolarGators {
namespace Drivers {

class Gps {
public:
  GPS(USART_TypeDef* uart_instance);
  virtual ~GPS();
  void handleData(uint8_t data);

private:
  static constexpr uint32_t MAX_SENTENCE_LENGTH 100;
  char ping[MAX_SENTENCE_LENGTH];
  char pong[MAX_SENTENCE_LENGTH];
  char* rx_ptr_;
  char* data_ptr;
};

} /* namespace Drivers */
} /* namespace SolarGators */

#endif /* SOLARGATORSBSP_DRIVERS_INC_GPS_HPP_ */

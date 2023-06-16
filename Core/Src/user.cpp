/*
 * user.cpp
 *
 *  Created on: Jun 16, 2023
 *      Author: John Carr
 */

#include "main.h"
#include "user.hpp"
#include "GPS.hpp"

using namespace SolarGators;

extern "C" void CPP_UserSetup(void);
extern "C" void UartCallback(void);

Drivers::Gps gps(&huart2);
void CPP_UserSetup(void)
{
  gps.startReception();
}

void UartCallback()
{
  gps.rxCpltCallback();
}

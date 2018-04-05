#pragma once

#include <AP_SerialManager/AP_SerialManager.h>


class AP_uZedSerial
{
public:
  AP_uZedSerial();
  // static bool detect(AP_SerialManager &serial_manager);
  bool get_flag(int16_t &agc_flag);
  
private:
  AP_HAL::UARTDriver *uart = nullptr;
  int16_t nbytes;
  // bool send_telem();
};

#pragma once

#include <AP_SerialManager/AP_SerialManager.h>


class AP_uZedSerial
{
public:
  AP_uZedSerial(AP_SerialManager &serial_manager);
  static bool detect(AP_SerialManager &serial_manager);
  bool get_flag(int16_t &agc_flag);
  
private:
  AP_HAL::UARTDriver *uart = nullptr;
  // bool send_telem();
};

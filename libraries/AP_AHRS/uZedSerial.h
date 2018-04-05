#pragma once

#include <AP_SerialManager/AP_SerialManager.h>


class AP_uZedSerial
{
public:
  AP_uZedSerial();
  // static bool detect(AP_SerialManager &serial_manager);
  bool get_flag(Vector3i &agc);
  
private:
	bool first_call;
  	AP_HAL::UARTDriver *uart = nullptr;
  	int16_t nbytes;
  	char c;
  	char tmp_c;
  // bool send_telem();
};

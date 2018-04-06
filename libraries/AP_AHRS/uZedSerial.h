#pragma once

#include <AP_SerialManager/AP_SerialManager.h>


class AP_uZedSerial
{
public:
  AP_uZedSerial();
  // static bool detect(AP_SerialManager &serial_manager);
  static AP_uZedSerial *get_instance();

  bool get_flag(Vector3i &agc);

  static bool detect();
  
private:
	static AP_uZedSerial *_instance;
	bool first_call;
  	AP_HAL::UARTDriver *uart = nullptr;
  	int16_t nbytes;
  	// char c1;
    // char c2;
  	char tmp_c;
    // uint16_t agc_feedback;

  union InertialInts {
    uint16_t i;
    char c[2] ;
  } c2i16;

  // bool send_telem();
};

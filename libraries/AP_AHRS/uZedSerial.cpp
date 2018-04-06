#include <AP_HAL/AP_HAL.h>
#include "uZedSerial.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_AHRS/AP_AHRS.h>
#include <ctype.h>

// External ref to AP HAL to gain access to serial ports
extern const AP_HAL::HAL& hal;


AP_uZedSerial *AP_uZedSerial::_instance = nullptr;

// Constructor: not entirely sure what should go in here at the moment. I'm
// thinking somthing to do with AP_AHRS since thats where where our switch is
AP_uZedSerial::AP_uZedSerial() :
first_call(true)
{
  hal.console->printf("Creating new MicroZed obj\n");
  AP_SerialManager &serial_manager = AP::serialmanager();
  uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_uZed,0);
  if (uart != nullptr) {
    uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_uZed,0));
    hal.console->printf("Started sensor on uartD\n");
  }
}

AP_uZedSerial *AP_uZedSerial::get_instance()
{
  if (!_instance) {
    _instance = new AP_uZedSerial();
  }
  return _instance;
}

bool AP_uZedSerial::detect()
{
  AP_SerialManager &serial_manager = AP::serialmanager();
  return serial_manager.find_serial(AP_SerialManager::SerialProtocol_uZed,0) != nullptr;
}

bool AP_uZedSerial::get_flag(Vector3i &agc)
{
	if (uart == nullptr) {
    	return false;
  	}
  	if (first_call) {
  		nbytes = uart->available();
  		while (nbytes-- > 0) {
  			tmp_c = uart->read();
  		}
  		first_call = false;
  	}

  	nbytes = uart->available();
 	if (nbytes == 0) {
   		return false;
 	} else {
   		c = uart->read();
   		agc[0] = agc[1];
   		agc[1] = (int16_t)c;
      agc[2] = 0;
   		hal.console->printf("agc_feedback: %d\n", agc[1]);
 	}
 	return true;
}

// bool AP_uZedSerial::send_telem()
// {
//   if (uart == nullptr) {
//     return false;
//   }
//   // Construct telem data packet
//   // uart->write();
// }

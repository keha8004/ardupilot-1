#include <AP_HAL/AP_HAL.h>
#include "uZedSerial.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_AHRS/AP_AHRS.h>
#include <ctype.h>

// External ref to AP HAL to gain access to serial ports
extern const AP_HAL::HAL& hal;


// Constructor: not entirely sure what should go in here at the moment. I'm
// thinking somthing to do with AP_AHRS since thats where where our switch is
AP_uZedSerial::AP_uZedSerial(AP_SerialManager &serial_manager)
{
  uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_uZed,0);
  if (uart != nullptr) {
    uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_uZed,0));
  }
}

bool AP_uZedSerial::detect(AP_SerialManager &serial_manager)
{
  return serial_manager.find_serial(AP_SerialManager::SerialProtocol_uZed,0) != nullptr;
}

bool AP_uZedSerial::get_flag(int16_t &agc_flag)
{
  if (uart == nullptr) {
    return false;
  }
 int16_t nbytes = uart->available();
 if (nbytes == 0) {
   return false;
 } else {
   char c = uart->read();
   agc_flag = (int16_t)c;
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

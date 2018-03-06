/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
// spec sheet: https://www.siliconsensing.com/media/30801/dmu11-00-0100-132-rev-3.pdf

  /* DMU11 CUSTOM MESSAGE CONTENT:
      Word    Data Item     Type      Byte (index) Numbers
      0       Header        16 bit    0-1
      1       Msg count     16 bit    2-3
      2-3     xRate         float     4-7
      4-5     xAcc          float     8-11
      6-7     yRate         float     12-15
      8-9     yAcc          float     16-19
      10-11   zRate         float     20-23
      12-13   zAcc          float     24-27
      14-15   IMU Temp      float     28-31
      16      starup BIT    16 bit    32-33
      17      sys Op. BIT   16 bit    34-35
      18      error ind.    16 bit    36-37
      19      Checksum      16 bit    38-39
  */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
//#include <float.h>
#include <cmath>
#include "AP_InertialSensor_DMU11.h"

// Declare external reference to HAL to gain access to namespace objects
extern const AP_HAL::HAL& hal;

// DMU11 Constructor, sets up backend, creates DMU11 object, and finds UART port
AP_InertialSensor_DMU11::AP_InertialSensor_DMU11(AP_InertialSensor &imu) :
    AP_InertialSensor_Backend(imu),
    HEADER1(0x55),
    HEADER2(0xAA),
    MESSAGE_SIZE(40),
    DEG2RAD(0.01745329251),
    initialize_message(true),    // Initialized to true, indicating that the message buffer needs to be initialized
                                    // for the correct message formatting. This will be set to false once the format is initialized
                                    // and only set true again if theres an error in the message format
    first_call(true),
    update_status(false)
{
    AP_SerialManager &serial_manager = AP::serialmanager();

    //hal.console->printf("Creating new DMU11 obj\n");
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_DMU11, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_DMU11, 0));
        //hal.console->printf("Started sensor on uartE\n");
    }
}

// Probe for detected uartE sensor
AP_InertialSensor_Backend *AP_InertialSensor_DMU11::probe(AP_InertialSensor &imu)
{
    AP_SerialManager &serial_manager = AP::serialmanager();
  // Return nullptr if no sensor is connected on uartE
  if (serial_manager.find_serial(AP_SerialManager::SerialProtocol_DMU11, 0) == nullptr){
    //hal.console->printf("No detected sensor on uartE\n");
    return nullptr;
  }
  // Otherwise declare pointer to new object
  //AP_InertialSensor_DMU11 *sensor = new AP_InertialSensor_DMU11(imu,serial_manager);
  AP_InertialSensor_DMU11 *sensor = new AP_InertialSensor_DMU11(imu);
  //hal.console->printf("Detected sensor on uartE\n");
  return sensor;
}


// "Start" the sensor, register the DMU11 as one new gyro and one new accel
void AP_InertialSensor_DMU11::start(void)
{
  // Register 1 accel and 1 gyro, sampling raw at 1000Hz, with DMU11 protocol
  _gyro_instance = _imu.register_gyro(200,0);
  hal.console->printf("Registered DMU11 gyro [%u]\n", _gyro_instance);
  _accel_instance = _imu.register_accel(200,0);
  hal.console->printf("Registered DMU11 accel [%u]\n", _accel_instance);
}


// Copy filtered data to frontend
void AP_InertialSensor_DMU11::accumulate(void)
{
    if (uart == nullptr) {
        AP_BoardConfig::sensor_config_error("Error: UART port not configured");
    }
    //AP_BoardConfig::sensor_config_error("error");

    /*
      If this is the first read, the message buffer needs to be initialized
      such that the header line 0x55AA occupies the first two indices of the buffer
    */

    if (first_call) {
      nbytes = uart->available();
      uint64_t first = AP_HAL::micros64();
      while (nbytes-- > 0) {
        c = uart->read();
      }
      uint64_t last = AP_HAL::micros64();
      if ( (last-first) < 5000 ) {
        uint64_t wait = 5000 - (last-first);
        hal.scheduler->delay_microseconds(wait);
        first_call = false;
      }
    }

    //if (initialize_message) {
      // Check number of available bytes
      nbytes = uart->available();
      if (nbytes < 40) {
        hal.console->printf("Not enough data available on DMU11.\n");
        update_status = false;
        return;
      }

      c = uart->read();
      nbytes--;

      if (c != HEADER1) {
          // hal.console->printf("Buffer reset on HEADER1\n");
          // notify_gyro_fifo_reset(_gyro_instance);
          // notify_accel_fifo_reset(_accel_instance);
          initialize_message = true;
          find_header();
          if (initialize_message == true) {
            hal.console->printf("Unable to reset buffer on HEADER1\n");
            update_status = false;
            return;
          }
        }
        else {
          message[0] = c;
          c = uart->read();
          nbytes--;
          if (c != HEADER2) {
            hal.console->printf("Buffer reset on HEADER2\n");
            notify_gyro_fifo_reset(_gyro_instance);
            notify_accel_fifo_reset(_accel_instance);
            initialize_message = true;
            find_header();
            if (initialize_message == true) {
              hal.console->printf("Unable to reset buffer on HEADER2\n");
              update_status = false;
              return;
            }
          }
          else {
            message[1] = c;
            msg_len = 2;
          }
        }

//// Now the message buffer has been initialized and can be filled normally
    // Check number of available bytes
    nbytes = uart->available();
    while (nbytes-- > 0) {
      message[msg_len++] = uart->read();

      if (msg_len == MESSAGE_SIZE) {
        /*
          If the message size has been maxed out (40 bytes) it is time to parse through the contents.
          There should be no need to break out of the loop after we're done parsing, as the buffer has already
          been initialized to put 0x55AA at the beginning, and the next byte read in after the buffer is
          full should be the start of the header file 0x55 (ideally).
          In the event that something has gone wrong, we will break out of the reading loop, mark an error, set
          initialize_message.
          ---
          msg_len is only reset once the data is extracted within parse_data() when the message size reaches its max.
          This way, subsequent calls to accumulate will automatically resume filling the message buffer where it left off,
          as well as allow us to finish reading in the entirety of the uart buffer after the message has filled up and
          been processed.
        */
        parse_data();

      }
    } // while (nbytes-- > 0)
    return;

}

void AP_InertialSensor_DMU11::find_header(void)
{
      while (nbytes-- > 0) {

        // read byte from buffer
        c = uart->read();

        // check for header line 0x55AA
        if (c != HEADER1) {
            continue;
        }
        //hal.console->printf("char: %c\n", c);
        tmp_c = c;
        // We found 0x55, now need to do another read to verify that
        // the next byte is 0xAA
        c = uart->read();
        nbytes--; // manually decrement for extra read statement
        if (c != HEADER2) {
            // Second byte isnt the expected second part of the header line (0xAA)
            // so its just another piece of data
            continue;  // Back to top of loop to try again
          }
          // We got here which means the header line has been found.
          // Now we can start filling the message buffer
          // First two indices are filled manually with the header that has already read
          message[0] = tmp_c;
          message[1] = c;
          initialize_message = false;
          msg_len = 2;
          break;  // Break out of while loop
      } // while(nbytes-->0)
      if (initialize_message == true) {
        hal.console->printf("Failed to initialize DMU message\n");
      }
      return;
}



void AP_InertialSensor_DMU11::parse_data(void)
{
  /* DMU11 MESSAGE CONTENT:
      Word    Data Item     Type      Byte (index) Numbers
      0       Header        16 bit    0-1
      1       Msg count     16 bit    2-3
      2-3     xRate         float     4-7
      4-5     xAcc          float     8-11
      6-7     yRate         float     12-15
      8-9     yAcc          float     16-19
      10-11   zRate         float     20-23
      12-13   zAcc          float     24-27
      14-15   IMU Temp      float     28-31
      16      starup BIT    16 bit    32-33
      17      sys Op. BIT   16 bit    34-35
      18      error ind.    16 bit    36-37
      19      Checksum      16 bit    38-39
  */


  // Debug
  parse_count++;
  // hal.console->printf("Calls to parse_data(): %d\n", parse_count);
  // Use union to extract meaningful data and convert to the approriate types
  // uint64_t now = AP_HAL::micros64();
  // hal.console->printf("Sample micros: %lu\n\n", now);


  // Unit Conversion Multipliers
  const float ACCEL_SCALE = -GRAVITY_MSS;
  const float GYRO_SCALE = -DEG2RAD;

  c2i16.c[0] = message[39];
  c2i16.c[1] = message[38];
  checksum = c2i16.i;

  if ( !VerifyChecksum() ) {
    update_status = false;
    initialize_message = true;
    hal.console->printf("Checksums not equal: return from parse_data()\n");
    return;
  }


  // u_float.c = {message[7],message[6],message[5],message[4]};
  // xRate = u_float.f;
  u_float.c[0] = message[7];
  u_float.c[1] = message[6];
  u_float.c[2] = message[5];
  u_float.c[3] = message[4];
  xRate = u_float.f;
  hal.console->printf("xRate: %f  ",xRate);
  xRate *= GYRO_SCALE;


  // u_float.c = {message[11],message[10],message[9],message[8]};
  // xAcc = u_float.f;
  u_float.c[0] = message[11];
  u_float.c[1] = message[10];
  u_float.c[2] = message[9];
  u_float.c[3] = message[8];
  xAcc = u_float.f;
  hal.console->printf("xAcc: %f\n  ",xAcc);
  xAcc *= ACCEL_SCALE;

  // u_float.c = {message[15],message[14],message[13],message[12]};
  // yRate = u_float.f;
  u_float.c[0] = message[15];
  u_float.c[1] = message[14];
  u_float.c[2] = message[13];
  u_float.c[3] = message[12];
  yRate = u_float.f;
  hal.console->printf("yRate: %f  ",yRate);
  yRate *= GYRO_SCALE;

  // u_float.c = {message[19],message[18],message[17],message[16]};
  // yAcc = u_float.f;
  u_float.c[0] = message[19];
  u_float.c[1] = message[18];
  u_float.c[2] = message[17];
  u_float.c[3] = message[16];
  yAcc = u_float.f;
  hal.console->printf("yAcc: %f\n  ",yAcc);
  yAcc *= ACCEL_SCALE;

  // u_float.c = {message[23],message[22],message[21],message[20]};
  // zRate = u_float.f;
  u_float.c[0] = message[23];
  u_float.c[1] = message[22];
  u_float.c[2] = message[21];
  u_float.c[3] = message[20];
  zRate = u_float.f;
  hal.console->printf("zRate: %f  ",zRate);
  zRate *= GYRO_SCALE;

  // u_float.c = {message[27],message[26],message[25],message[24]};
  // zAcc = u_float.f;
  u_float.c[0] = message[27];
  u_float.c[1] = message[26];
  u_float.c[2] = message[25];
  u_float.c[3] = message[24];
  zAcc = u_float.f;
  hal.console->printf("zAcc: %f\n  ",zAcc);
  zAcc *= ACCEL_SCALE;

  // Save to imu data types
  Vector3f gyro = Vector3f(xRate,yRate,zRate);
  // gyro *= -DEG2RAD;

  Vector3f accel = Vector3f(xAcc,yAcc,zAcc);
  // accel *= -GRAVITY_MSS;

  //hal.console->printf("Acc: %f %f %f\n",accel.x,accel.y,accel.z);
  //hal.console->printf("Gyro: %f %f %f\n",gyro.x,gyro.y,gyro.z);


  // Notify of new measurements
  // _rotate_and_correct_gyro(_gyro_instance,gyro);
  _notify_new_gyro_raw_sample(_gyro_instance,gyro);
   //_notify_new_gyro_raw_sample(_gyro_instance,gyro,now);

  // _rotate_and_correct_accel(_accel_instance,accel);
  _notify_new_accel_raw_sample(_accel_instance,accel);
  //_notify_new_accel_raw_sample(_accel_instance,accel,now);

  //AP_BoardConfig::sensor_config_error("error");

  msg_len = 0;

  update_status = true;
  return;

}


bool AP_InertialSensor_DMU11::VerifyChecksum(void) {
  uint16_t count = 0;
  uint32_t sum = 0;
  while ( count < 38 ) {
    c2i16.c[1] = message[count];
    c2i16.c[0] = message[count+1];
    sum += c2i16.i;
    count+=2;
  }

  // Bitwise & 0xFFFF
  sum &= 0xFFFF;
  uint16_t twos_comp = ~sum+1;

  hal.console->printf("\nChecksum: %d\n", checksum);
  hal.console->printf("VerifyChecksum: %d\n\n", twos_comp);
  if ( twos_comp == checksum ) {
    hal.console->printf("VerifyChecksum: returning true\n");
    return true;
  }
  else {
    hal.console->printf("VerifyChecksum: returning false\n");
    return false;
  }
}


bool AP_InertialSensor_DMU11::update(void)
{
  //hal.console->printf("Updating");
    update_status = false;
    accumulate();
    if ( !update_status ) {
      return true;
    } 

    update_accel(_accel_instance);
    update_gyro(_gyro_instance);

    return true;
}


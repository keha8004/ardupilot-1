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

/*
Expected Data:

Item    Word    Data Item               Value/Units
0       0       Header                  16 Bit, 0x55AA
1       1       Message Count           16 Bit, 0 to 65535 decimal
2       2-3     Axis X Rate             32 Bit Single Precision FP, (deg/s)
3       4-5     Axis X Accel            32 Bit Single Precision FP, (g)
4       6-7     Axis Y Rate             32 Bit Single Precision FP, (deg/s)
5       8-9     Axis Y Accel            32 Bit Single Precision FP, (g)
6       10-11   Axis Z Rate             32 Bit Single Precision FP, (deg/s)
7       12-13   Axis Z Accel            32 Bit Single Precision FP, (g)
8       14-15   Aux Input V             32 Bit Single Precision FP, (V)
9       16-17   Avg IMU Temp            32 Bit Single Precision FP, (degC)
10      18-19   Axis X Delta Theta      32 Bit Single Precision FP, (deg)
11      20-21   Axis X Delta Vel        32 Bit Single Precision FP, (m/s)
12      22-23   Axis Y Delta Theta      32 Bit Single Precision FP, (deg)
13      24-25   Axis Y Delta Vel        32 Bit Single Precision FP, (m/s)
14      26-27   Axis Z Delta Theta      32 Bit Single Precision FP, (deg)
15      28-29   Axis Z Delta Vel        32 Bit Single Precision FP, (m/s)
16      30      Sys Startup Flags       16 Bit, 0 to 65535 decimal
17      31      Sys Op Flags            16 Bit, 0 to 65535 decimal
18      32      Bit Flag Error Ind      16 Bit, 0 to 65535 decimal
19      33      Checksum                16 Bit 2-s complement of 16 Bit Sum of prev 0-18 items
*/


#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor_DMU11.h"

// Declare external reference to HAL to gain access to namespace objects
extern const AP_HAL::HAL& hal;

// DMU11 Constructor, sets up backend, creates DMU11 object, and finds UART port
AP_InertialSensor_DMU11::AP_InertialSensor_DMU11(AP_InertialSensor &imu) :
    AP_InertialSensor_Backend(imu)
{
    AP_SerialManager &serial_manager = AP::serialmanager();

    hal.console->printf("Creating new DMU11 obj\n");
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_DMU11, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_DMU11, 0));
        hal.console->printf("Started sensor on uartE\n");
    }
}

// Probe for detected uartE sensor
AP_InertialSensor_Backend *AP_InertialSensor_DMU11::probe(AP_InertialSensor &imu)
{
    AP_SerialManager &serial_manager = AP::serialmanager();
  // Return nullptr if no sensor is connected on uartE
  if (serial_manager.find_serial(AP_SerialManager::SerialProtocol_DMU11, 0) == nullptr){
    hal.console->printf("No detected sensor on uartE\n");
    return nullptr;
  }
  // Otherwise declare pointer to new object
  //AP_InertialSensor_DMU11 *sensor = new AP_InertialSensor_DMU11(imu,serial_manager);
  AP_InertialSensor_DMU11 *sensor = new AP_InertialSensor_DMU11(imu);
  hal.console->printf("Detected sensor on uartE\n");
  return sensor;
}


//"Start" the sensor, register the DMU11 as one new gyro and one new accel
void AP_InertialSensor_DMU11::start(void)
{
  // Register 1 accel and 1 gyro, sampling raw at 1000Hz, with DMU11 protocol
  _gyro_instance = _imu.register_gyro(1000,0);
  hal.console->printf("Registered DMU11 gyro [%u]\n", _gyro_instance);
  _accel_instance = _imu.register_accel(1000,0);
  hal.console->printf("Registered DMU11 accel [%u]\n", _accel_instance);
}

// read - return last value measured by sensor
// Vector3f stuff, refer to AP_InertialSensor.cpp lines ~1400-1600
// bool AP_InertialSensor_DMU11::get_reading(Vector3f &gyro, Vector3f &accel)

/*
bool AP_InertialSensor_DMU11::get_DMU11_data(void)
{
    if (uart == nullptr) {
        is_DMU11_data = false;
        return false;

    }
    uint16_t count = 0;
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
      // read byte from buffer
      char c = uart->read();
      // immediately print to pixhawk console to verify data
      hal.console->printf("%c",c);

      count++;
    }


    if (count == 0) {
        is_DMU11_data = false;
        return false;

    }
    // reading_cm = 100 * sum / count;
    is_DMU11_data = true;
    return true;

}
*/

/*
   Copy filtered data to frontend
*/

bool AP_InertialSensor_DMU11::update(void)
{

    //update_accel(_accel_instance);
    //update_gyro(_gyro_instance);

    if (uart == nullptr) {
        return false;
    }

    uint16_t count = 0;
    int16_t nbytes = uart->available();
    //hal.console->printf("nbytes: %d\n",nbytes);
    while (nbytes-- > 0) {
        //hal.console->printf("count: %d\n",count);
      // read byte from buffer
      char c = uart->read();
      // immediately print to pixhawk console to verify data
      hal.console->printf("DMU11 Data: %c\n",c);

      count++;
    }

    //hal.console->printf("count: %d",count);

    if (count == 0) {
        return false;

    }
    // reading_cm = 100 * sum / count;
    return true;
}


/*
bool AP_InertialSensor_DMU11::update(void)
{
  return false;
}
*/


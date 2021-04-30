//
// The MIT License (MIT)
//
// Copyright (c) 2020 EAIBOT. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
#pragma once
#include "utils.h"
#include "ydlidar_driver.h"
#include <math.h>

using namespace ydlidar;

/**
 * @ref "Dataset"
 * @par Dataset
<table>
<tr><th>LIDAR      <th> Model  <th>  Baudrate <th>  SampleRate(K) <th> Range(m)  		<th>  Frequency(HZ) <th> Intenstiy(bit) <th> SingleChannel<th> voltage(V)
<tr><th> S2-Pro    <td> 4	   <td>  115200   <td>   3            <td>  0.12~8          <td> 5~8            <td> false          <td> false    	  <td> 4.8~5.2
</table>
 */

/**
 * @par example: S2-Pro LiDAR
 * @code
  ///< Defining an CYdLidar instance.
  CYdLidar laser;
  //////////////////////string property/////////////////
  /// lidar port
  std::string port = "/dev/ydlidar";
  laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
  /// ignore array
  std::string ignore_array;
  ignore_array.clear();
  laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(),
                    ignore_array.size());

  //////////////////////int property/////////////////
  /// lidar baudrate
  int optval = 115200;
  laser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));
  /// abnormal count
  optval = 4;
  laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));

  //////////////////////bool property/////////////////
  /// fixed angle resolution
  bool b_optvalue = false;
  laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
  /// rotate 180
  laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
  /// Counterclockwise
  laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
  b_optvalue = true;
  laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));

  //////////////////////float property/////////////////
  /// unit: °
  float f_optvalue = 180.0f;
  laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
  f_optvalue = -180.0f;
  laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));

  /// unit: m
  f_optvalue = 10.f;
  laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
  f_optvalue = 0.1f;
  laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
  /// unit: Hz
  f_optvalue = 6.f
  laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));
 * @endcode
 */


class YDLIDAR_API CYdLidar {
 public:
  CYdLidar(); //!< Constructor
  virtual ~CYdLidar();  //!< Destructor: turns the laser off.

  /**
   * @brief set lidar properties
   * @param optname        option name
   * @todo string properties
   * - @ref LidarPropSerialPort
   * - @ref LidarPropIgnoreArray
   * @note set string property example
   * @code
   * CYdLidar laser;
   * std::string lidar_port = "/dev/ydlidar";
   * laser.setlidaropt(LidarPropSerialPort,lidar_port.c_str(), lidar_port.size());
   * @endcode
   * @todo int properties
   * - @ref LidarPropSerialBaudrate
   * - @ref LidarPropLidarType
   * - @ref LidarPropDeviceType
   * - @ref LidarPropSampleRate
   * @note set int property example
   * @code
   * CYdLidar laser;
   * int lidar_baudrate = 230400;
   * laser.setlidaropt(LidarPropSerialPort,&lidar_baudrate, sizeof(int));
   * @endcode
   * @todo bool properties
   * - @ref LidarPropFixedResolution
   * - @ref LidarPropReversion
   * - @ref LidarPropInverted
   * - @ref LidarPropAutoReconnect
   * - @ref LidarPropSingleChannel
   * - @ref LidarPropIntenstiy
   * @note set bool property example
   * @code
   * CYdLidar laser;
   * bool lidar_fixedresolution = true;
   * laser.setlidaropt(LidarPropSerialPort,&lidar_fixedresolution, sizeof(bool));
   * @endcode
   * @todo float properties
   * - @ref LidarPropMaxRange
   * - @ref LidarPropMinRange
   * - @ref LidarPropMaxAngle
   * - @ref LidarPropMinAngle
   * - @ref LidarPropScanFrequency
   * @note set float property example, Must be float type, not double type.
   * @code
   * CYdLidar laser;
   * float lidar_maxrange = 16.0f;
   * laser.setlidaropt(LidarPropSerialPort,&lidar_maxrange, sizeof(float));
   * @endcode
   * @param optval         option value
   * - std::string(or char*)
   * - int
   * - bool
   * - float
   * @param optlen         option length
   * - data type size
   * @return true if the Property is set successfully, otherwise false.
   * @see LidarProperty
   */
  bool setlidaropt(int optname, const void *optval, int optlen);

  /**
   * @brief get lidar property
   * @param optname         option name
   * @todo string properties
   * - @ref LidarPropSerialPort
   * - @ref LidarPropIgnoreArray
   * @note get string property example
   * @code
   * CYdLidar laser;
   * char lidar_port[30];
   * laser.getlidaropt(LidarPropSerialPort,lidar_port, sizeof(lidar_port));
   * @endcode
   * @todo int properties
   * - @ref LidarPropSerialBaudrate
   * - @ref LidarPropLidarType
   * - @ref LidarPropDeviceType
   * - @ref LidarPropSampleRate
   * @note get int property example
   * @code
   * CYdLidar laser;
   * int lidar_baudrate;
   * laser.getlidaropt(LidarPropSerialPort,&lidar_baudrate, sizeof(int));
   * @endcode
   * @todo bool properties
   * - @ref LidarPropFixedResolution
   * - @ref LidarPropReversion
   * - @ref LidarPropInverted
   * - @ref LidarPropAutoReconnect
   * - @ref LidarPropSingleChannel
   * - @ref LidarPropIntenstiy
   * @note get bool property example
   * @code
   * CYdLidar laser;
   * bool lidar_fixedresolution;
   * laser.getlidaropt(LidarPropSerialPort,&lidar_fixedresolution, sizeof(bool));
   * @endcode
   * @todo float properties
   * - @ref LidarPropMaxRange
   * - @ref LidarPropMinRange
   * - @ref LidarPropMaxAngle
   * - @ref LidarPropMinAngle
   * - @ref LidarPropScanFrequency
   * @note set float property example
   * @code
   * CYdLidar laser;
   * float lidar_maxrange;
   * laser.getlidaropt(LidarPropSerialPort,&lidar_maxrange, sizeof(float));
   * @endcode
   * @param optval          option value
   * - std::string(or char*)
   * - int
   * - bool
   * - float
   * @param optlen          option length
   * - data type size
   * @return true if the Property is get successfully, otherwise false.
   * @see LidarProperty
   */
  bool getlidaropt(int optname, void *optval, int optlen);

  /**
   * @brief Initialize the SDK and LiDAR.
   * @return true if successfully initialized, otherwise false.
   */
  bool initialize();

  /**
  * @brief Return LiDAR's version information in a numeric form.
  * @param version Pointer to a version structure for returning the version information.
  */
  void GetLidarVersion(LidarVersion &version);

  /**
   * @brief Start the device scanning routine which runs on a separate thread and enable motor.
   * @return true if successfully started, otherwise false.
   */
  bool  turnOn();  //!< See base class docs

  /**
   * @brief Get the LiDAR Scan Data. turnOn is successful before doProcessSimple scan data.
   * @param[out] outscan             LiDAR Scan Data
   * @param[out] hardwareError       hardware error status
   * @return true if successfully started, otherwise false.
   */
  bool doProcessSimple(LaserScan &scan_msg, bool &hardwareError);

  /**
   * @brief Stop the device scanning thread and disable motor.
   * @return true if successfully Stoped, otherwise false.
   */
  bool  turnOff(); //!< See base class docs

  /**
   * @brief Uninitialize the SDK and Disconnect the LiDAR.
   */
  void disconnecting(); //!< Closes the comms with the laser. Shouldn't have to be directly needed by the user


  /**
   * @brief Get the last error information of a (lidar or serial)
   * @return a human-readable description of the given error information
   * or the last error information of a (lidar or serial)
   */
  lidar_error_t getDriverError() const;

 protected:
  /** Returns true if communication has been established with the device. If it's not,
    *  try to create a comms channel.
    * \return false on error.
    */
  bool  checkCOMMs();

  /** Returns true if health status and device information has been obtained with the device. If it's not,
    * \return false on error.
    */
  bool  checkStatus();

  /**
   * @brief checkScanFrequency
   * @return
   */
  bool checkScanFrequency();

  /**
   * @brief checkZeroOffsetAngle
   * @return
   */
  bool checkZeroOffsetAngle();

  /** Returns true if the normal scan runs with the device. If it's not,
    * \return false on error.
    */
  bool checkHardware();

  /**
   * @brief checkHealth
   * @param info
   * @return
   */
  bool checkHealth(const ct_packet_t &info);

  /** returns true if the lidar data is normal, If it's not*/
  bool checkLidarAbnormal();

  /** Returns true if the device is in good health, If it's not*/
  bool getDeviceHealth(uint32_t timeout = 500);

  /** Returns true if the device information is correct, If it's not*/
  bool getDeviceInfo(uint32_t timeout = 500);

 private:
  ydlidar::YDlidarDriver *lidarPtr;
  LaserFan               laser_packages;
  uint32_t               point_interval_time;
  uint32_t               package_transfer_time;///零位包传送时间
  uint64_t               last_node_time;
  int                    fixed_size;
  int                    sample_rate;
  float                  frequency_offset;
  float                  zero_offset_angle;
  bool                   isScanning;
  bool                   isConnected;
  bool                   m_GlassNoise;
  bool                   m_SunNoise;
  LidarVersion           m_LidarVersion;      ///< LiDAR Version information

 private:
  std::string m_SerialPort;         ///< LiDAR serial port
  std::string m_IgnoreString;       ///< LiDAR ignore array string
  std::vector<float> m_IgnoreArray; ///< LiDAR ignore array

  bool m_FixedResolution;           ///< LiDAR fixed angle resolution
  bool m_Reversion;                 ///< LiDAR reversion
  bool m_Inverted;                  ///< LiDAR inverted
  bool m_AutoReconnect;             ///< LiDAR hot plug
  bool m_Intensity;                 ///< LiDAR intensity

  int m_SerialBaudrate;             ///< LiDAR serial baudrate or network port
  int m_AbnormalCheckCount;         ///< LiDAR abnormal count

  float m_MaxAngle;                 ///< LiDAR maximum angle
  float m_MinAngle;                 ///< LiDAR minimum angle
  float m_MaxRange;                 ///< LiDAR maximum range
  float m_MinRange;                 ///< LiDAR minimum range
  float m_ScanFrequency;            ///< LiDAR scanning frequency
};	// End of class


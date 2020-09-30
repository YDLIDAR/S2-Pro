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
/** @page YDlidarDriver
 * YDlidarDriver API
    <table>
        <tr><th>Library     <td>YDlidarDriver
        <tr><th>File        <td>ydlidar_driver.h
        <tr><th>Author      <td>Tony [code at ydlidar com]
        <tr><th>Source      <td>https://github.com/ydlidar/S2-Pro
        <tr><th>Version     <td>1.0.0
    </table>

* @copyright    Copyright (c) 2018-2020  EAIBOT
     Jump to the @link ::ydlidar::YDlidarDriver @endlink interface documentation.
*/
#ifndef YDLIDAR_DRIVER_H
#define YDLIDAR_DRIVER_H
#include <stdlib.h>
#include <map>
#include "serial.h"
#include "locker.h"
#include "thread.h"
#include "ydlidar_protocol.h"

#if !defined(__cplusplus)
#ifndef __cplusplus
#error "The YDLIDAR SDK requires a C++ compiler to be built"
#endif
#endif

using namespace std;
using namespace serial;


namespace ydlidar {

std::string format(const char *fmt, ...);

class YDlidarDriver {
 public:
  /**
  * A constructor.
  * A more elaborate description of the constructor.
  */
  YDlidarDriver();

  /**
  * A destructor.
  * A more elaborate description of the destructor.
  */
  virtual ~YDlidarDriver();

  /**
   * @brief Connecting Lidar \n
   * After the connection if successful, you must use ::disconnect to close
   * @param[in] port_path    serial port
   * @param[in] baudrate    serial baudrate，S2-Pro：
   *     115200
   * @return connection status
   * @retval 0     success
   * @retval < 0   failed
   * @note After the connection if successful, you must use ::disconnect to close
   * @see function ::YDlidarDriver::disconnect ()
   */
  result_t connect(const char *port_path, uint32_t baudrate);

  /*!
  * @brief Disconnect the LiDAR.
  */
  void disconnect();

  /**
  * @brief Get SDK Version \n
  * static function
  * @return Version
  */
  static std::string getSDKVersion();


  /**
   * @brief getDriverError
   * @return
   */
  lidar_error_t getDriverError();
  /**
   * @brief get Health status \n
   * @return result status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE or RESULT_TIMEOUT   failed
   */
  result_t getHealth(device_health &health, uint32_t timeout = DEFAULT_TIMEOUT);

  /**
   * @brief get Device information \n
   * @param[in] info     Device information
   * @param[in] timeout  timeout
   * @return result status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE or RESULT_TIMEOUT   failed
   */
  result_t getDeviceInfo(device_info &info, uint32_t timeout = DEFAULT_TIMEOUT);

  /**
   * @brief Get lidar scan frequency \n
   * @param[in] frequency    scanning frequency
   * @param[in] timeout      timeout
   * @return return status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   * @note Non-scan state, perform currect operation.
   */
  result_t getScanFrequency(scan_frequency_t &frequency,
                            uint32_t timeout = DEFAULT_TIMEOUT);

  /**
   * @brief Increase the scanning frequency by 1.0 HZ \n
   * @param[in] frequency    scanning frequency
   * @param[in] timeout      timeout
   * @return return status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   * @note Non-scan state, perform currect operation.
   */
  result_t setScanFrequencyAdd(scan_frequency_t &frequency,
                               uint32_t timeout = DEFAULT_TIMEOUT);

  /**
   * @brief Reduce the scanning frequency by 1.0 HZ \n
   * @param[in] frequency    scanning frequency
   * @param[in] timeout      timeout
   * @return return status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   * @note Non-scan state, perform currect operation.
   */
  result_t setScanFrequencyDis(scan_frequency_t &frequency,
                               uint32_t timeout = DEFAULT_TIMEOUT);

  /**
   * @brief Increase the scanning frequency by 0.1 HZ \n
   * @param[in] frequency    scanning frequency
   * @param[in] timeout      timeout
   * @return return status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   * @note Non-scan state, perform currect operation.
   */
  result_t setScanFrequencyAddMic(scan_frequency_t &frequency,
                                  uint32_t timeout = DEFAULT_TIMEOUT);

  /**
   * @brief Reduce the scanning frequency by 0.1 HZ \n
   * @param[in] frequency    scanning frequency
   * @param[in] timeout      timeout
   * @return return status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   * @note Non-scan state, perform currect operation.
   */
  result_t setScanFrequencyDisMic(scan_frequency_t &frequency,
                                  uint32_t timeout = DEFAULT_TIMEOUT);

  /**
   * @brief fetches zero angle tolerance values from lidar’s internal memory while lidar assembly \n
   * @param[in] angle　　　   zero offset angle
   * @param[in] timeout      timeout
   * @return return status
   * @retval RESULT_OK       success
   * @retval RESULT_TIMEOUT  Failed
   * @retval RESULT_FAILE    Angle is not calibrated
   * @note Non-scan state, perform currect operation.
   */
  result_t getZeroOffsetAngle(offset_angle_t &angle,
                              uint32_t timeout =  DEFAULT_TIMEOUT);

  /**
    * @brief lidarPortList Get Lidar Port lists
    * @return online lidars
    */
  static std::map<std::string, std::string> lidarPortList();

  /**
   * @brief Is it connected to the lidar \n
   * @return connection status
   * @retval true     connected
   * @retval false    Non-connected
   */
  bool isConnected() const;

  /**
   * @brief Is the Lidar in the scan \n
   * @return scanning status
   * @retval true     scanning
   * @retval false    non-scanning
   */
  bool isScanning() const;

  /**
   * @brief getPointTime
   * @return
   */
  uint32_t getPointIntervalTime() const;

  /**
   * @brief getPackageTime
   * @return
   */
  uint32_t getPackageTransferTime() const;

  /**
   * @brief whether to support hot plug \n
   * @param[in] enable    hot plug :
   *   true	support
   *   false no support
   */
  void setAutoReconnect(const bool &enable);

  /**
   * @brief setSingleChannel
   * @param enable
   */
  void setSingleChannel(bool enable);

  /**
   * @brief Turn on scanning \n
   * @param[in] force    Scan mode
   * @param[in] timeout  timeout
   * @return result status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   * @note Just turn it on once
   */
  result_t startScan(uint32_t timeout = DEFAULT_TIMEOUT) ;

  /*!
   * @brief stop Scanning state
   * @param timeout  timeout
   * @return status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   */
  result_t stopScan(uint32_t timeout = DEFAULT_TIMEOUT);


  /**
   * @brief turn off scanning \n
   * @return result status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   */
  result_t stop();


  /**
   * @brief Get a circle of laser data \n
   * @param[in] fan        Laser data
   * @param[in] count      one circle of laser points
   * @param[in] timeout    timeout
   * @return return status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   * @note Before starting, you must start the start the scan successfully with the ::startScan function
   */
  result_t grabScanData(LaserFan *fan, uint32_t timeout = DEFAULT_TIMEOUT) ;

  /**
   * @brief start motor \n
   * @return return status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   */
  result_t startMotor();

  /**
   * @brief stop motor \n
   * @return return status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   */
  result_t stopMotor();

  /**
   * @brief flush
   */
  void flush();

 protected:

  /**
  * @brief Data parsing thread \n
  * @note Before you create a dta parsing thread, you must use the ::startScan function to start the lidar scan successfully.
  */
  result_t createThread();

  /**
  * @brief Automatically reconnect the lidar \n
  * @param[in] force    scan model
  * @param[in] timeout  timeout
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_FAILE    failed
  * @note Lidar abnormality automatically reconnects.
  */
  result_t startAutoScan(uint32_t timeout = DEFAULT_TIMEOUT) ;

  /**
  * @brief Unpacking \n
  * @param[in] package lidar point information
  * @param[in] timeout     timeout
  */
  result_t waitPackage(LaserFan &package, uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief get unpacked data \n
  * @param[in] package laser node
  * @param[in] count      lidar points size
  * @param[in] timeout      timeout
  * @return result status
  * @retval RESULT_OK       success
  * @retval RESULT_TIMEOUT  timeout
  * @retval RESULT_FAILE    failed
  */
  result_t waitScanData(LaserFan &package, uint32_t timeout = DEFAULT_TIMEOUT);

  /**
    * @brief data parsing thread \n
    */
  int cacheScanData();

  /**
  * @brief send data to lidar \n
  * @param[in] cmd 	 command code
  * @param[in] payload      payload
  * @param[in] payloadsize      payloadsize
  * @return result status
  * @retval RESULT_OK       success
  * @retval RESULT_FAILE    failed
  */
  result_t sendCommand(uint8_t cmd, const void *payload = NULL,
                       size_t payloadsize = 0);

  /**
  * @brief Waiting for the specified size data from the lidar \n
  * @param[in] data_count 	 wait max data size
  * @param[in] timeout    	 timeout
  * @param[in] returned_size   really data size
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_TIMEOUT  wait timeout
  * @retval RESULT_FAILE    failed
  * @note when timeout = -1, it will block...
  */
  result_t waitForData(size_t data_count, uint32_t timeout = DEFAULT_TIMEOUT,
                       size_t *returned_size = NULL);

  /**
  * @brief get data from serial \n
  * @param[in] data 	 data
  * @param[in] size    date size
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_FAILE    failed
  */
  result_t getData(uint8_t *data, size_t size);

  /**
  * @brief send data to serial \n
  * @param[in] data 	 data
  * @param[in] size    data size
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_FAILE    failed
  */
  result_t sendData(const uint8_t *data, size_t size);

  /**
  * @brief disable Data scan channel \n
  */
  void disableDataGrabbing();


  /*!
  * @brief set DTR \n
  */
  void setDTR();


  /*!
  * @brief clear DTR \n
  */
  void clearDTR();

  /**
   * @brief flushSerial
   */
  void flushSerial();

  /**
   * @brief setDriverError
   * @param er
   */
  void setDriverError(const lidar_error_t &er);

 public:
  /* Variable for LIDAR compatibility */
  /// LiDAR connected state
  bool     m_isConnected;
  /// LiDAR Scanning state
  bool     m_isScanning;
  /// auto reconnect
  bool     isAutoReconnect;
  /// auto connecting state
  bool     isAutoconnting;

  enum {
    DEFAULT_TIMEOUT = 1000,    /**< Default timeout. */
    MAX_SCAN_NODES = 2048,	   /**< Default Max Scan Count. */
    DEFAULT_TIMEOUT_COUNT = 2,
  };
  enum {
    YDLIDAR_F4      = 1,/**< F4 LiDAR Model. */
    YDLIDAR_T1      = 2,/**< T1 LiDAR Model. */
    YDLIDAR_F2      = 3,/**< F2 LiDAR Model. */
    YDLIDAR_S4      = 4,/**< S4 LiDAR Model. */
    YDLIDAR_G4      = 5,/**< G4 LiDAR Model. */
    YDLIDAR_X4      = 6,/**< X4 LiDAR Model. */
    YDLIDAR_G4PRO   = 7,/**< G4PRO LiDAR Model. */
    YDLIDAR_F4PRO   = 8,/**< F4PRO LiDAR Model. */
    YDLIDAR_R2      = 9,/**< R2 LiDAR Model. */
    YDLIDAR_G10     = 10,/**< G10 LiDAR Model. */
    YDLIDAR_S4B     = 11,/**< S4B LiDAR Model. */
    YDLIDAR_S2      = 12,/**< S2 LiDAR Model. */
    YDLIDAR_G6      = 13,/**< G6 LiDAR Model. */
    YDLIDAR_G2A     = 14,/**< G2A LiDAR Model. */
    YDLIDAR_G2B     = 15,/**< G2 LiDAR Model. */
    YDLIDAR_G2C     = 16,/**< G2C LiDAR Model. */
    YDLIDAR_G4B     = 17,/**< G4B LiDAR Model. */
    YDLIDAR_G4C     = 18,/**< G4C LiDAR Model. */
    YDLIDAR_G1      = 19,/**< G1 LiDAR Model. */
    YDLIDAR_G5      = 20,/**< G5 LiDAR Model. */
    YDLIDAR_G7      = 21,/**< G7 LiDAR Model. */

    YDLIDAR_TG15    = 100,/**< TG15 LiDAR Model. */
    YDLIDAR_TG30    = 101,/**< T30 LiDAR Model. */
    YDLIDAR_TG50    = 102,/**< TG50 LiDAR Model. */

    YDLIDAR_T15     = 200,/**< T15 LiDAR Model. */
    YDLIDAR_Tail,
  };
  Event          _dataEvent;			 ///< data event
  Locker         _lock;				///< thread lock
  Locker         _serial_lock;		///< serial lock
  Locker         _error_lock;       ///< error lock
  Thread 	     _thread;				///< thread id

 private:
  serial::Serial *_serial;			///< serial
  LaserFan       m_global_fan;
  std::string    serial_port;///< lidar port
  uint32_t       baudrate_;
  bool           isSupportMotorCtrl;
  bool           single_channel;
  uint32_t       point_interval_time;
  uint32_t       transfer_delay;
  uint32_t       package_transfer_time;
  lidar_error_t  m_error_info;
  ct_packet_t    m_global_ct;

};
}

#endif // YDLIDAR_DRIVER_H

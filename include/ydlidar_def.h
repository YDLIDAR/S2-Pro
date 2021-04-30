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

#include <v8stdint.h>
#include <vector>
#include <ydlidar_cmd.h>

#define SUNNOISEINTENSITY 0x03
#define GLASSNOISEINTENSITY 0x02


typedef enum  {
  NoError,
  DeviceNotFoundError,
  PermissionError,
  OpenError,
  ParityError,
  FramingError,
  BreakConditionError,
  WriteError,
  ReadError,
  ResourceError,
  UnsupportedOperationError,
  TimeoutError,
  NotOpenError,
  HeaderError,
  FirstSampleAngleError,
  LastSampleAngleError,
  PackageNumberError,
  CheckSumError,
  SensorError,
  EncodeError,
  PWRError,
  PDError,
  LDError,
  DataError,
  TrembleError,
  LidarNotFoundError,
  UnknownError,
} lidar_error_t;

/** Lidar Properties,Lidar Can set and get parameter property index.\n
 * float properties must be float type, not double type.
*/
typedef enum {
  /* char* properties */
  LidarPropSerialPort = 0,/**< Lidar serial port or network ipaddress */
  LidarPropIgnoreArray,/**< Lidar ignore angle array */
  /* int properties */
  LidarPropSerialBaudrate = 10,/**< lidar serial baudrate or network port */
  LidarPropLidarType,/**< lidar type code */
  LidarPropDeviceType,/**< lidar connection type code */
  LidarPropSampleRate,/**< lidar sample rate */
  LidarPropAbnormalCheckCount,/**< abnormal maximum check times */
  /* float properties */
  LidarPropMaxRange = 20,/**< lidar maximum range */
  LidarPropMinRange,/**< lidar minimum range */
  LidarPropMaxAngle,/**< lidar maximum angle */
  LidarPropMinAngle,/**< lidar minimum angle */
  LidarPropScanFrequency,/**< lidar scanning frequency */
  /* bool properties */
  LidarPropFixedResolution = 30,/**< fixed angle resolution flag */
  LidarPropReversion,/**< lidar reversion flag */
  LidarPropInverted,/**< lidar inverted flag */
  LidarPropAutoReconnect,/**< lidar hot plug flag */
  LidarPropSingleChannel,/**< lidar single-channel flag */
  LidarPropIntenstiy,/**< lidar intensity flag */
  LidarPropSupportMotorDtrCtrl,/**< lidar support motor Dtr ctrl flag */
  LidarPropSupportHeartBeat,/**< lidar support heartbeat flag */
} LidarProperty;

#pragma pack(1)
/// LiDAR Intensity Nodes Package
struct node_package_header_t {
  uint8_t   packageHeaderMSB;///< package header MSB
  uint8_t   packageHeaderLSB;///< package header LSB
  uint8_t   packageSync: 1; ///< package sync flag
  uint8_t   packageCTInfo: 7; ///< package ct info
  uint8_t   nowPackageNum;///< package number
  uint16_t  packageFirstSampleAngleSync: 1;///first sample angle sync flag
  uint16_t  packageFirstSampleAngle: 15; ///< first sample angle
  uint16_t  packageLastSampleAngleSync: 1; ///< last sample angle sync flag
  uint16_t  packageLastSampleAngle: 15; ///< last sample angle
  uint16_t  checkSum;///< checksum
} __attribute__((packed));

static_assert(sizeof(node_package_header_t) == 10,
              "response scan header size mismatch.");


namespace response_health_error {
enum bits : uint8_t {
  SensorError = 1 << 0,  // sensor error
  EncodeError = 1 << 1,  // encode error
  PWRError = 1 << 2,  // ref error
  PDError = 1 << 3,  // pd error
  LDError = 1 << 4,  // ld error
  DataError = 1 << 5,  // data error
  CSError = 1 << 6,  // cs error
};
}  // namespace response_health_error

namespace response_scan_packet_sync {
enum bits : uint8_t {
  sync = 1 << 0,                 // beginning of new full scan or sample angle flag
  // Reserved for future error bits
  reserved1 = 1 << 1,
  reserved2 = 1 << 2,
  reserved3 = 1 << 3,
  reserved4 = 1 << 4,
  reserved5 = 1 << 5,
  reserved6 = 1 << 6,
  reserved7 = 1 << 7,
};
}  // namespace response_scan_packet_sync


/// package node info
struct node_package_payload_t {
  uint16_t PackageSampleSi: 2; ///< si
  uint16_t PackageSampleDistance: 14; ///< range
} __attribute__((packed));
static_assert(sizeof(node_package_payload_t) == 2,
              "response scan payload size mismatch.");

struct scan_packet_t {
  node_package_header_t header;
  node_package_payload_t payload[40];
} __attribute__((packed));

static_assert(sizeof(scan_packet_t) == 90,
              "response scan packet size mismatch.");

struct node_package_intensity_payload_t {
  uint8_t PackageSampleIntensity;/// intensity
  node_package_payload_t PackageSample; ///< range
} __attribute__((packed));
static_assert(sizeof(node_package_intensity_payload_t) == 3,
              "response scan intensity payload size mismatch.");


struct scan_intensity_packet_t {
  node_package_header_t header;
  node_package_intensity_payload_t  payload[40];
} __attribute__((packed)) ;

static_assert(sizeof(scan_intensity_packet_t) == 130,
              "response scan intensity packet size mismatch.");


struct device_info {
  uint8_t   model; ///< 雷达型号
  uint16_t  firmware_version; ///< 固件版本号
  uint8_t   hardware_version; ///< 硬件版本号
  uint8_t   serialnum[16];    ///< 系列号
} __attribute__((packed)) ;
static_assert(sizeof(device_info) == 20,
              "device info size mismatch.");

struct device_health {
  uint8_t   status; ///< 健康状体
  uint16_t  error_code; ///< 错误代码
} __attribute__((packed))  ;

struct sampling_rate_t {
  uint8_t rate;	///< 采样频率
} __attribute__((packed))  ;

struct scan_frequency_t {
  uint32_t frequency;	///< 扫描频率
} __attribute__((packed))  ;

/// LiDAR Zero Offset Angle
struct offset_angle_t {
  int32_t angle;
} __attribute__((packed))  ;

struct cmd_packet_t {
  uint8_t syncByte;
  uint8_t cmd_flag;
  uint8_t size;
  uint8_t data;
} __attribute__((packed)) ;

/// LiDAR response Header
struct lidar_ans_header_t {
  uint8_t  syncByte1;
  uint8_t  syncByte2;
  uint32_t size: 30;
  uint32_t subType: 2;
  uint8_t  type;
} __attribute__((packed));

static_assert(sizeof(lidar_ans_header_t) == 7,
              "Ans header size mismatch.");

struct ct_packet_t {
  uint8_t size;
  uint8_t index;
  uint8_t info[100];
  uint8_t crc;
  uint8_t cs;
  uint8_t valid;
} __attribute__((packed)) ;

static_assert(sizeof(ct_packet_t) == 105,
              "ct packet size mismatch.");


/** The numeric version information struct.  */
typedef struct {
  uint8_t hardware;   /**< Hardware version*/
  uint8_t soft_major;      /**< major number */
  uint8_t soft_minor;      /**< minor number */
  uint8_t soft_patch;      /**< patch number */
  uint8_t sn[16];     /**< serial number*/
} LidarVersion;

#pragma pack()


struct LaserPoint {
  float angle;
  float range;
  uint8_t interference_sign;
  uint8_t intensity;
  LaserPoint &operator = (const LaserPoint &data) {
    angle = data.angle;
    range = data.range;
    interference_sign = data.interference_sign;
    intensity = data.intensity;
    return *this;
  }
};

struct LaserFan {
  uint8_t    sync_flag;  //sync flag
  /// Array of lidar points
  ct_packet_t info;
  std::vector<LaserPoint> points;
  LaserFan &operator = (const LaserFan &data) {
    this->sync_flag = data.sync_flag;
    this->info = data.info;
    this->points = data.points;
    return *this;
  }
};


//! A struct for returning configuration from the YDLIDAR
struct LaserConfig {
  //! Start angle for the laser scan [rad].  0 is forward and angles are measured clockwise when viewing YDLIDAR from the top.
  float min_angle;
  //! Stop angle for the laser scan [rad].   0 is forward and angles are measured clockwise when viewing YDLIDAR from the top.
  float max_angle;
  /// angle resoltuion [rad]
  float angle_increment;
  //! Scan resoltuion [s]
  float time_increment;
  //! Time between scans
  float scan_time;
  //! Minimum range [m]
  float min_range;
  //! Maximum range [m]
  float max_range;

  LaserConfig &operator = (const LaserConfig &data) {
    this->min_angle = data.min_angle;
    this->max_angle = data.max_angle;
    this->angle_increment = data.angle_increment;
    this->time_increment = data.time_increment;
    this->scan_time = data.scan_time;
    this->min_range = data.min_range;
    this->max_range = data.max_range;
    return *this;
  }
};


struct LaserScan {
  //! System time when first range was measured in nanoseconds
  uint64_t stamp;
  //! Array of laser point
  std::vector<LaserPoint> points;
  //! Configuration of scan
  LaserConfig config;
  LaserScan &operator = (const LaserScan &data) {
    this->stamp = data.stamp;
    this->points = data.points;
    this->config = data.config;
    return *this;
  }
};


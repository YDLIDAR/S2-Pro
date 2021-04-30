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
#include "CYdLidar.h"
#include "common.h"
#include <map>
#include <numeric>
#include "angles.h"
#include "LogModule.h"


using namespace std;
using namespace ydlidar;
using namespace impl;


/*-------------------------------------------------------------
						Constructor
-------------------------------------------------------------*/
CYdLidar::CYdLidar(): lidarPtr(nullptr) {
  m_SerialPort        = "/dev/ydlidar";
  m_SerialBaudrate    = 115200;
  m_FixedResolution   = false;
  m_Reversion         = false;
  m_Inverted          = true;
  m_AutoReconnect     = false;
  m_MaxAngle          = 180.f;
  m_MinAngle          = -180.f;
  m_MaxRange          = 8.0;
  m_MinRange          = 0.08;
  m_AbnormalCheckCount = 3;
  isScanning          = false;
  isConnected         = false;
  m_GlassNoise        = true;
  m_SunNoise          = true;
  m_Intensity         = false;
  point_interval_time = 1e9 / 3000;
  package_transfer_time = 0;
  last_node_time      = getTime();
  fixed_size          = 500;
  sample_rate         = 3;
  m_ScanFrequency     = 6.0;
  frequency_offset    = 0.0;
  zero_offset_angle   = 0.0;
  m_IgnoreArray.clear();
  memset(&m_LidarVersion, 0, sizeof(LidarVersion));
}

/*-------------------------------------------------------------
                    ~CYdLidar
-------------------------------------------------------------*/
CYdLidar::~CYdLidar() {
  disconnecting();
}

bool CYdLidar::setlidaropt(int optname, const void *optval, int optlen) {
  if (optval == NULL) {
#if defined(_WIN32)
    SetLastError(EINVAL);
#else
    errno = EINVAL;
#endif
    return false;
  }

  if (optname >= LidarPropFixedResolution) {
    if (optlen != sizeof(bool)) {
#if defined(_WIN32)
      SetLastError(EINVAL);
#else
      errno = EINVAL;
#endif
      return false;
    }

  } else if (optname >= LidarPropMaxRange) {
    if (optlen != sizeof(float)) {
#if defined(_WIN32)
      SetLastError(EINVAL);
#else
      errno = EINVAL;
#endif
      return false;
    }
  } else if (optname >= LidarPropSerialBaudrate) {
    if (optlen != sizeof(int)) {
#if defined(_WIN32)
      SetLastError(EINVAL);
#else
      errno = EINVAL;
#endif
      return false;
    }
  } else {

  }


  bool ret = true;

  switch (optname) {
    case LidarPropSerialPort:
      m_SerialPort = (const char *)optval;
      break;

    case LidarPropIgnoreArray:
      m_IgnoreString = (const char *)optval;
      m_IgnoreArray = ydlidar::split(m_IgnoreString, ',');

      if (m_IgnoreArray.size() % 2 != 0) {
        m_IgnoreArray.clear();
        ret = false;
      }

      break;

    case LidarPropFixedResolution:
      m_FixedResolution = *(bool *)(optval);
      break;

    case LidarPropReversion:
      m_Reversion = *(bool *)(optval);
      break;

    case LidarPropInverted:
      m_Inverted = *(bool *)(optval);
      break;

    case LidarPropAutoReconnect:
      m_AutoReconnect = *(bool *)(optval);
      break;

    case LidarPropMaxRange:
      m_MaxRange = *(float *)(optval);
      break;

    case LidarPropMinRange:
      m_MinRange = *(float *)(optval);
      break;

    case LidarPropMaxAngle:
      m_MaxAngle = *(float *)(optval);
      break;

    case LidarPropMinAngle:
      m_MinAngle = *(float *)(optval);
      break;

    case LidarPropScanFrequency:
      m_ScanFrequency = *(float *)(optval);
      break;

    case LidarPropSerialBaudrate:
      m_SerialBaudrate = *(int *)(optval);
      break;

    case LidarPropAbnormalCheckCount:
      m_AbnormalCheckCount = *(int *)(optval);
      break;

  case LidarPropIntenstiy:
      m_Intensity = *(bool*)(optval);

    default :
      ret = false;
      break;
  }

  return ret;
}

bool CYdLidar::getlidaropt(int optname, void *optval, int optlen) {
  if (optval == NULL) {
#if defined(_WIN32)
    SetLastError(EINVAL);
#else
    errno = EINVAL;
#endif
    return false;
  }

  if (optname >= LidarPropFixedResolution) {
    if (optlen != sizeof(bool)) {
#if defined(_WIN32)
      SetLastError(EINVAL);
#else
      errno = EINVAL;
#endif
      return false;
    }

  } else if (optname >= LidarPropMaxRange) {
    if (optlen != sizeof(float)) {
#if defined(_WIN32)
      SetLastError(EINVAL);
#else
      errno = EINVAL;
#endif
      return false;
    }
  } else if (optname >= LidarPropSerialBaudrate) {
    if (optlen != sizeof(int)) {
#if defined(_WIN32)
      SetLastError(EINVAL);
#else
      errno = EINVAL;
#endif
      return false;
    }
  } else {

  }

  bool ret = true;

  switch (optname) {
    case LidarPropSerialPort:
      memcpy(optval, m_SerialPort.c_str(), optlen);
      break;

    case LidarPropIgnoreArray:
      memcpy(optval, m_IgnoreString.c_str(), optlen);
      break;

    case LidarPropFixedResolution:
      memcpy(optval, &m_FixedResolution, optlen);
      break;

    case LidarPropReversion:
      memcpy(optval, &m_Reversion, optlen);
      break;

    case LidarPropInverted:
      memcpy(optval, &m_Inverted, optlen);
      break;

    case LidarPropAutoReconnect:
      memcpy(optval, &m_AutoReconnect, optlen);
      break;

    case LidarPropMaxRange:
      memcpy(optval, &m_MaxRange, optlen);
      break;

    case LidarPropMinRange:
      memcpy(optval, &m_MinRange, optlen);
      break;

    case LidarPropMaxAngle:
      memcpy(optval, &m_MaxAngle, optlen);
      break;

    case LidarPropMinAngle:
      memcpy(optval, &m_MinAngle, optlen);
      break;

    case LidarPropScanFrequency:
      memcpy(optval, &m_ScanFrequency, optlen);
      break;

    case LidarPropSerialBaudrate:
      memcpy(optval, &m_SerialBaudrate, optlen);
      break;

    case LidarPropAbnormalCheckCount:
      memcpy(optval, &m_AbnormalCheckCount, optlen);
      break;

    default :
      ret = false;
      break;
  }

  return ret;

}

/*-------------------------------------------------------------
                        initialize
-------------------------------------------------------------*/
void CYdLidar::GetLidarVersion(LidarVersion &version) {
  memcpy(&version, &m_LidarVersion, sizeof(LidarVersion));
}

void CYdLidar::disconnecting() {
  if (lidarPtr) {
    lidarPtr->disconnect();
    delete lidarPtr;
    lidarPtr    = nullptr;
    isConnected = false;
  }
}

lidar_error_t CYdLidar::getDriverError() const {
  if (lidarPtr) {
    return lidarPtr->getDriverError();
  }

  return UnknownError;
}

/*-------------------------------------------------------------
						doProcessSimple
-------------------------------------------------------------*/
bool  CYdLidar::doProcessSimple(LaserScan &scan_msg, bool &hardwareError) {
  hardwareError			= false;

  // Bound?
  if (!checkHardware()) {
    scan_msg.points.clear();
    delay(200 / m_ScanFrequency);
    hardwareError = false;
    return false;
  }

  //  wait Scan data:
  uint64_t tim_scan_start = getTime();
  laser_packages.points.clear();
  result_t op_result = lidarPtr->grabScanData(&laser_packages);
  uint64_t tim_scan_end = getTime();

  // Fill in scan data:
  if (IS_OK(op_result)) {
    LaserPoint point;

    if (m_MaxAngle < m_MinAngle) {
      float temp = m_MinAngle;
      m_MinAngle = m_MaxAngle;
      m_MaxAngle = temp;
    }

    size_t count = laser_packages.points.size();
    uint64_t scan_time = point_interval_time * (count - 1);
    tim_scan_end -= package_transfer_time;
    tim_scan_end -= point_interval_time;
    tim_scan_start = tim_scan_end -  scan_time ;
    last_node_time = tim_scan_end;
    scan_msg.points.clear();
    scan_msg.config.scan_time = static_cast<float>(1.0 * scan_time / 1e9);
    scan_msg.config.min_angle = angles::from_degrees(m_MinAngle);
    scan_msg.config.max_angle = angles::from_degrees(m_MaxAngle);
    scan_msg.config.angle_increment = 2 * M_PI / (count - 1);

    if (m_FixedResolution) {
      scan_msg.config.angle_increment = 2 * M_PI / (fixed_size - 1);
    }

    scan_msg.config.time_increment = scan_msg.config.scan_time / (double)(
                                       count - 1);
    scan_msg.stamp = tim_scan_start;
    scan_msg.config.min_range = m_MinRange;
    scan_msg.config.max_range = m_MaxRange;

    if (!checkHealth(laser_packages.info)) {
      hardwareError = true;
      return false;
    }

    for (int i = 0; i < count; ++i) {
      point = laser_packages.points[i];
      point.range = point.range / 1000.f;
      point.angle = angles::from_degrees(point.angle + zero_offset_angle);

      if (m_Reversion) {
        point.angle += M_PI;
      }

      //Is it counter clockwise
      if (m_Inverted) {
        point.angle = 2 * M_PI - point.angle;
      }

      point.angle = angles::normalize_angle(point.angle);


      if (m_GlassNoise && point.interference_sign == GLASSNOISEINTENSITY) {
        point.range = 0.0;
      }

      if (m_SunNoise && point.interference_sign == SUNNOISEINTENSITY) {
        point.range  = 0.0;
      }

      if (point.range > m_MaxRange || point.range < m_MinRange) {
        point.range = 0.0;
      }

      if (point.angle >= scan_msg.config.min_angle &&
          point.angle <= scan_msg.config.max_angle) {
        if (scan_msg.points.empty()) {
          scan_msg.stamp = tim_scan_start + i * point_interval_time;
        }

        scan_msg.points.push_back(point);
      }
    }

    if (m_FixedResolution) {
      scan_msg.points.resize(fixed_size);
    }

    return true;

  } else {
    if (IS_FAIL(op_result)) {
      // Error? Retry connection
    }
  }

  return false;
}

bool CYdLidar::checkHealth(const ct_packet_t &info) {
  bool ret = true;

  if (IS_OK(ydlidar::protocol::check_ct_packet_t(info))) {
    lidar_error_t err = ydlidar::protocol::convert_ct_packet_to_error(info);

    if (err != NoError) {
      ret = false;
    }
  }

  return ret;
}

/*-------------------------------------------------------------
						turnOn
-------------------------------------------------------------*/
bool  CYdLidar::turnOn() {
  if (isScanning && lidarPtr->isScanning()) {
    return true;
  }

  uint32_t startTs = getms();
  // start scan...
  result_t op_result = lidarPtr->startScan();

  if (!IS_OK(op_result)) {
    op_result = lidarPtr->startScan();

    if (!IS_OK(op_result)) {
      fprintf(stderr, "[CYdLidar] Failed to start scan mode: %x, %s\n", op_result,
              ydlidar::protocol::DescribeError(lidarPtr->getDriverError()));
      lidarPtr->stop();
      isScanning = false;
      return false;
    }
  }

  point_interval_time = lidarPtr->getPointIntervalTime();
  package_transfer_time = lidarPtr->getPackageTransferTime();

  if (checkLidarAbnormal()) {
    fprintf(stderr,
            "[CYdLidar][%fs] Failed to turn on the Lidar, because %s.\n",
            (getms() - startTs) / 1000.0,
            ydlidar::protocol::DescribeError(lidarPtr->getDriverError()));
    lidarPtr->stop();
    isScanning = false;
    return false;
  }

  isScanning = true;
  lidarPtr->setAutoReconnect(m_AutoReconnect);
  printf("[YDLIDAR INFO][%fs] Now YDLIDAR is scanning ......\n",
         (getms() - startTs) / 1000.0);
  LOG_INFO("[%fs] Now YDLIDAR is scanning ......",(getms() - startTs) / 1000.0);
  fflush(stdout);
  return true;
}

/*-------------------------------------------------------------
						turnOff
-------------------------------------------------------------*/
bool  CYdLidar::turnOff() {
  if (lidarPtr) {
    lidarPtr->stop();
  }

  if (isScanning) {
    printf("[YDLIDAR INFO] Now YDLIDAR Scanning has stopped ......\n");
    LOG_INFO("Now YDLIDAR Scanning has stopped ......","");
  }

  isScanning = false;
  return true;
}


/*-------------------------------------------------------------
            checkLidarAbnormal
-------------------------------------------------------------*/
bool CYdLidar::checkLidarAbnormal() {
  int check_abnormal_count = 0;

  if (m_AbnormalCheckCount < 2) {
    m_AbnormalCheckCount = 2;
  }

  result_t op_result = RESULT_FAIL;

  while (check_abnormal_count < m_AbnormalCheckCount) {
    //Ensure that the voltage is insufficient or the motor resistance is high, causing an abnormality.
    if (check_abnormal_count > 1) {
      delay(check_abnormal_count * 1000);
    }

    LaserFan packages;
    packages.points.clear();
    op_result =  lidarPtr->grabScanData(&packages);

    if (IS_OK(op_result)) {
      return !IS_OK(op_result);
    }

    check_abnormal_count++;
  }

  return !IS_OK(op_result);
}

bool CYdLidar::getDeviceHealth(uint32_t timeout) {
  if (!lidarPtr) {
    return false;
  }

  uint32_t startTs = getms();
  result_t op_result;
  device_health healthinfo;
  op_result = lidarPtr->getHealth(healthinfo, timeout);

  if (IS_OK(op_result)) {
    printf("[YDLIDAR][%fs]:Lidar running correctly ! The health status: %s\n",
           (getms() - startTs) / 1000.0,
           (int)healthinfo.status == 0 ? "good" : "bad");
    LOG_INFO("[%fs]:Lidar running correctly ! The health status: %s",(getms() - startTs) / 1000.0,
             (int)healthinfo.status == 0 ? "good" : "bad");

    if (healthinfo.status == 2) {
      fprintf(stderr,
              "Error, YDLIDAR internal error detected. Please reboot the device to retry.\n");
      LOG_ERROR("Error, YDLIDAR internal error detected. Please reboot the device to retry.","");
      return false;
    } else {
      return true;
    }

  } else {
//    fprintf(stderr, "Error, cannot retrieve Yd Lidar health code: %x\n", op_result);
    return false;
  }
}

bool CYdLidar::getDeviceInfo(uint32_t timeout) {
  if (!lidarPtr) {
    return false;
  }

  uint32_t startTs = getms();
  device_info devinfo;
  result_t op_result = lidarPtr->getDeviceInfo(devinfo, timeout);

  if (!IS_OK(op_result)) {
    fprintf(stderr, "get Device Information Error: %s\n",
            ydlidar::protocol::DescribeError(lidarPtr->getDriverError()));
    fflush(stderr);
    return false;
  }

  std::string model = format("S2-Pro[%d]", devinfo.model);
  uint8_t Major = (uint8_t)(devinfo.firmware_version >> 8);
  uint8_t Minjor = (uint8_t)(devinfo.firmware_version & 0xff);
  printf("[YDLIDAR][%fs] Device Info:\n"
         "Firmware version: %u.%u\n"
         "Hardware version: %u\n"
         "Model: %s\n"
         "Serial: ",
         (getms() - startTs) / 1000.0,
         Major,
         Minjor,
         (unsigned int)devinfo.hardware_version,
         model.c_str());

  for (int i = 0; i < 16; i++) {
    printf("%01X", devinfo.serialnum[i] & 0xff);
  }

  printf("\n");
  m_LidarVersion.hardware = devinfo.hardware_version;
  m_LidarVersion.soft_major = Major;
  m_LidarVersion.soft_minor = Minjor / 10;
  m_LidarVersion.soft_patch = Minjor % 10;
  memcpy(&m_LidarVersion.sn[0], &devinfo.serialnum[0], 16);
  checkScanFrequency();
  //checkZeroOffsetAngle();
  printf("[YDLIDAR INFO] Current Sampling Rate : %dK\n", sample_rate);
  LOG_INFO(" Current Sampling Rate : %dK",sample_rate)
  return true;
}

/*-------------------------------------------------------------
                        checkScanFrequency
-------------------------------------------------------------*/
bool CYdLidar::checkScanFrequency() {
  uint32_t startTs = getms();
  float frequency = 6.0f;
  scan_frequency_t _scan_frequency;
  float hz = 0.f;
  m_ScanFrequency += frequency_offset;
  result_t ans = lidarPtr->getScanFrequency(_scan_frequency) ;

  if (IS_OK(ans)) {
    frequency = _scan_frequency.frequency / 100.f;
    hz = m_ScanFrequency - frequency;

    if (hz > 0) {
      while (hz > 0.95) {
        lidarPtr->setScanFrequencyAdd(_scan_frequency);
        hz = hz - 1.0;
      }

      while (hz > 0.09) {
        lidarPtr->setScanFrequencyAddMic(_scan_frequency);
        hz = hz - 0.1;
      }

      frequency = _scan_frequency.frequency / 100.0f;
    } else {
      while (hz < -0.95) {
        lidarPtr->setScanFrequencyDis(_scan_frequency);
        hz = hz + 1.0;
      }

      while (hz < -0.09) {
        lidarPtr->setScanFrequencyDisMic(_scan_frequency);
        hz = hz + 0.1;
      }

      frequency = _scan_frequency.frequency / 100.0f;
    }
  }

  ans = lidarPtr->getScanFrequency(_scan_frequency);

  if (IS_OK(ans)) {
    frequency = _scan_frequency.frequency / 100.0f;
    m_ScanFrequency = frequency;
  }

  m_ScanFrequency -= frequency_offset;
  fixed_size = sample_rate * 1000 / (m_ScanFrequency - 0.1);
  printf("[YDLIDAR INFO][%fs] Current Scan Frequency: %fHz\n",
         (getms() - startTs) / 1000.0, m_ScanFrequency);
  LOG_INFO("[%fs] Current Scan Frequency: %fHz",(getms() - startTs) / 1000.0, m_ScanFrequency);
  return true;
}

bool CYdLidar::checkZeroOffsetAngle() {
  bool ret = false;
  zero_offset_angle = 0.0;
  uint32_t startTs = getms();
  result_t ans;
  offset_angle_t angle;
  ans = lidarPtr->getZeroOffsetAngle(angle);

  if (IS_OK(ans)) {
    zero_offset_angle = angle.angle / 4.0;
    printf("[YDLIDAR INFO][%fs] Obtained Zero Offset Angle[%f°] \n",
           (getms() - startTs) / 1000.0, zero_offset_angle);
    LOG_INFO("[%fs] Obtained Zero Offset Angle[%f°] ",(getms() - startTs) / 1000.0, zero_offset_angle);
    fflush(stdout);
    ret = true;
  } else {
    printf("[YDLIDAR ERROR][%fs] Failed to Get Zero Offset Angle[%f°] \n",
           (getms() - startTs) / 1000.0, zero_offset_angle);
    LOG_ERROR("[%fs] Failed to Get Zero Offset Angle[%f°] ",(getms() - startTs) / 1000.0, zero_offset_angle);
    fflush(stdout);
  }

  return ret;
}

bool CYdLidar::checkStatus() {
  if (!checkCOMMs()) {
    return false;
  }

  bool ret  = getDeviceHealth();

  if (!ret) {
    ret = getDeviceHealth(100);
  }

  if (!getDeviceInfo() && !ret) {
    ret = getDeviceInfo();
  } else {
    ret = true;
  }

  return ret;
}

/*-------------------------------------------------------------
						checkCOMMs
-------------------------------------------------------------*/
bool  CYdLidar::checkCOMMs() {
  if (!lidarPtr) {
    printf("YDLidar SDK initializing\n");
    LOG_INFO("YDLidar SDK initializing","");
    // create the driver instance
    lidarPtr = new YDlidarDriver();

    if (!lidarPtr) {
      fprintf(stderr, "Create Driver fail\n");
      LOG_ERROR("Create Driver fail","");
      return false;

    }

    printf("YDLidar SDK has been initialized\n");
    LOG_INFO("YDLidar SDK has been initialized","");
    LOG_INFO("SDK Version: %s",YDlidarDriver::getSDKVersion().c_str())
    printf("[YDLIDAR]:SDK Version: %s\n", YDlidarDriver::getSDKVersion().c_str());
    fflush(stdout);
  }

  if (isConnected) {
    return true;
  }

  // Is it COMX, X>4? ->  "\\.\COMX"
  if (m_SerialPort.size() >= 3) {
    if (tolower(m_SerialPort[0]) == 'c' && tolower(m_SerialPort[1]) == 'o' &&
        tolower(m_SerialPort[2]) == 'm') {
      // Need to add "\\.\"?
      if (m_SerialPort.size() > 4 || m_SerialPort[3] > '4') {
        m_SerialPort = std::string("\\\\.\\") + m_SerialPort;
      }
    }
  }

  // make connection...
  result_t op_result = lidarPtr->connect(m_SerialPort.c_str(), m_SerialBaudrate);

  if (!IS_OK(op_result)) {
    fprintf(stderr,
            "[CYdLidar] Error, cannot bind to the specified serial port[%s] and baudrate[%d]\n",
            m_SerialPort.c_str(), m_SerialBaudrate);
    LOG_ERROR("Error, cannot bind to the specified serial port[%s] and baudrate[%d]",m_SerialPort.c_str(), m_SerialBaudrate);
    return false;
  }

  printf("[YDLIDAR INFO] Connection established in %s[%d]:\n",
         m_SerialPort.c_str(),
         m_SerialBaudrate);
  LOG_INFO("Connection established in %s[%d]:",m_SerialPort.c_str(),
           m_SerialBaudrate);
  fflush(stdout);
  printf("LiDAR successfully connected\n");
  LOG_INFO("LiDAR successfully connected","");
  isConnected = true;
  return true;
}


/*-------------------------------------------------------------
                        checkHardware
-------------------------------------------------------------*/
bool CYdLidar::checkHardware() {
  if (!lidarPtr) {
    return false;
  }

  if (isScanning && lidarPtr->isScanning()) {
    return true;
  }

  return false;
}

/*-------------------------------------------------------------
						initialize
-------------------------------------------------------------*/
bool CYdLidar::initialize() {
  if (!checkCOMMs()) {
      LOG_ERROR("Error initializing YDLIDAR scanner","");
    fprintf(stderr, "[CYdLidar::initialize] Error initializing YDLIDAR scanner.\n");
    fflush(stderr);
    return false;
  }

  if (!checkStatus()) {
      LOG_ERROR("Error initializing YDLIDAR check status under [%s] and [%d].",m_SerialPort.c_str(), m_SerialBaudrate);
    fprintf(stderr,
            "[CYdLidar::initialize] Error initializing YDLIDAR check status under [%s] and [%d].",
            m_SerialPort.c_str(), m_SerialBaudrate);
    fflush(stderr);
    return false;
  }

  LOG_INFO("LiDAR init success!","");
  printf("LiDAR init success!\n");
  fflush(stdout);
  return true;
}

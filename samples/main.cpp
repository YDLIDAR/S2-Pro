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
#include <iostream>
#include <string>
#include <memory>
#include <fstream>

using namespace std;
using namespace ydlidar;

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_driver.lib")
#endif

int main(int argc, char *argv[]) {

  ydlidar::init(argc, argv);

  std::string port;
  std::string baud;
  int baudrate = 115200;
  printf("__   ______  _     ___ ____    _    ____  \n");
  printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\ \n");
  printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) | \n");
  printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <  \n");
  printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\ \n");
  printf("\n");
  fflush(stdout);

  std::map<std::string, std::string> lidars = YDlidarDriver::lidarPortList();

  if (lidars.size() == 1) {
    std::map<string, string>::iterator iter = lidars.begin();
    port = iter->second;
  } else {
    printf("Please enter the lidar serial port:");
    std::cin >> port;
    printf("Please enter the lidar serial baud rate:");
    std::cin >> baud;
    baudrate = atoi(baud.c_str());
  }

  if (!ydlidar::ok()) {
    return 0;
  }

  float frequency = 6.0;

  CYdLidar laser;
  //////////////////////string property/////////////////
  /// lidar port
  laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
  /// ignore array
  std::string ignore_array;
  ignore_array.clear();
  laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(),
                    ignore_array.size());

  //////////////////////int property/////////////////
  /// lidar baudrate
  laser.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));

  /// abnormal count
  int optval = 4;
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
  /// intensity
  b_optvalue = false;
  laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));

  b_optvalue = true;
  laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));

  //////////////////////float property/////////////////
  /// unit: °
  float f_optvalue = 180.0f;
  laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
  f_optvalue = -180.0f;
  laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
  /// unit: m
  f_optvalue = 8.f;
  laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
  f_optvalue = 0.05f;
  laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
  /// unit: Hz
  laser.setlidaropt(LidarPropScanFrequency, &frequency, sizeof(float));

  //畸变矫正初始化
  laser.setEnableCorrection(true);
// {true, 87, 107, 1, 1},
// {true, 334, 354, 1, 1},
// {true, 208, 228, 1, 1},
// {true, 36, 51, 1.5, -1},
// {true, 284, 304, 1.5, -1},
// {true, 150, 170, 1.5, -1}};
  EaiCorrectItem item;
  item.enable = true;
  item.left_angle = 87;
  item.right_angle = 107;
  item.duty = 1;
  item.adj_dir = 1;
  laser.addCorrectionItem(item);
  item.enable = true;
  item.left_angle = 334;
  item.right_angle = 354;
  item.duty = 1;
  item.adj_dir = 1;
  laser.addCorrectionItem(item);
  item.enable = true;
  item.left_angle = 208;
  item.right_angle = 228;
  item.duty = 1;
  item.adj_dir = 1;
  laser.addCorrectionItem(item);
  item.enable = true;
  item.left_angle = 36;
  item.right_angle = 51;
  item.duty = 1.5;
  item.adj_dir = -1;
  laser.addCorrectionItem(item);
  item.enable = true;
  item.left_angle = 284;
  item.right_angle = 304;
  item.duty = 1.5;
  item.adj_dir = -1;
  laser.addCorrectionItem(item);
  item.enable = true;
  item.left_angle = 150;
  item.right_angle = 170;
  item.duty = 1.5;
  item.adj_dir = -1;
  laser.addCorrectionItem(item);


  bool ret = laser.initialize();

  if (ret) {
      laser.turnOn();
  }

  LaserScan scan;
  //日志文件
  ofstream ofs;
  ofs.open("data.log", ios::out | ios::trunc);

  while (ret && ydlidar::ok())
  {
      bool hardError;
      scan.points.clear();

      if (laser.doProcessSimple(scan, hardError))
      {
          fprintf(stdout, "Scan received: %u ranges in %f HZ\n",
                  (unsigned int)scan.points.size(), 1.0 / scan.config.scan_time);
          fflush(stdout);

          for (size_t i=0; i<scan.points.size(); ++i)
          {
              const LaserPoint& p = scan.points.at(i);
              ofs << i << " " << p.angle * 180.0f / M_PI << " " << p.range << endl;
          }
          ofs << endl;
      }
      else
      {
          printf("[YDLIDAR ERROR]: %s\n",
                 ydlidar::protocol::DescribeError(laser.getDriverError()));
          LOG_ERROR("%s",ydlidar::protocol::DescribeError(laser.getDriverError()));
          fflush(stdout);
      }
  }

  laser.turnOff();
  laser.disconnecting();

  return 0;
}

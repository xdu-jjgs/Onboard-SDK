/*! @file camera-stream-disease-detect.cpp
 *  @version 1.0.0
 *  @date Apr 16th 2023
 *
 *  @brief
 *  disease detection with YOLOv5-Lite
 *
 *  @Copyright (c) 2023 xiaojianzhong
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * */


#include "dji_vehicle.hpp"
#include "dji_linux_helpers.hpp"
#include "v5lite.h"
#include <iostream>
#include <pthread.h>

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace DJI::OSDK;

void show_rgb(CameraRGBImage img, void *p)
{
  V5lite V5lite("./config.yaml");
  V5lite.LoadEngine();

  std::string name = std::string(reinterpret_cast<char *>(p));
  std::cout << "#### Got image from:\t" << name << std::endl;
  cv::Mat mat(img.height, img.width, CV_8UC3, img.rawData.data(), img.width*3);

  V5lite.InferenceImage(mat);
}

int main(int argc, char** argv)
{
  bool f = false;
  bool m = false;
  char c = 0;
  std::cout << "Please enter the type of camera stream you want to view"
	    << std::endl
            << "m: Main Camera"
	    << std::endl
            << "f: FPV  Camera"
	    << std::endl;
  std::cin >> c;

  switch(c)
  {
    case 'm':
      m = true;
      break;
    case 'f':
      f = true;
      break;
    default:
      std::cout << "No camera selected" << std::endl;
      return 1;
  }

  LinuxSetup linuxEnvironment(argc, argv, true);
  Vehicle *vehicle = linuxEnvironment.getVehicle();
  const char *acm_dev = linuxEnvironment.getEnvironment()->getDeviceAcm().c_str();
  vehicle->advancedSensing->setAcmDevicePath(acm_dev);
  if (vehicle == NULL)
  {
    std::cout << "Vehicle not initialized, exiting." << std::endl;
    return -1;
  }

  char fpvName[] = "FPV_CAM";
  char mainName[] = "MAIN_CAM";

  bool camResult;
  if (f) {
    camResult = vehicle->advancedSensing->startFPVCameraStream(&show_rgb, &fpvName);
  } else if (m) {
    camResult = vehicle->advancedSensing->startMainCameraStream(&show_rgb, &mainName);
  }

  if(!camResult)
  {
    std::cout << "Failed to open selected camera" << std::endl;
    return 1;
  }

  sleep(120000000);

  if (f) {
    vehicle->advancedSensing->stopFPVCameraStream();
  } else if (m) {
    vehicle->advancedSensing->stopMainCameraStream();
  }
  return 0;
}

/*! @file disease-detect.cpp
 *  @version 1.0.0
 *  @date Apr 16th 2023
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
#include "v5lite.hpp"
#include <iostream>
#include <queue>
#include <pthread.h>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <jsoncpp/json/json.h>
#include <math.h>

#define FPV_CAM              false
#define MAIN_CAM             true
#define PIPELINE_IS_RELIABLE false
#define PIPELINE_ID          49155
#define BUFFER_SIZE          10240

Vehicle *vehicle;
bool running = true;
std::queue<std::vector<V5lite::DetectRes>> que;
std::mutex mtx;
V5lite *v5lite;
int width, height;


std::string jsname(){
  auto wktime = std::chrono::high_resolution_clock::to_time_t(std::chrono::high_resolution_clock::now());
  std::string jstxt_tmp="/home/zxj/Downloads/Onboard-SDK/savjs/";
  char wkname[20];
  std::strftime(wkname, sizeof(wkname), "%Y%m%d-%H:%M:%S", localtime(&wktime));
  std::string wkt=wkname;
  std::string jstxt=jstxt_tmp.append(wkt) += ".json";
  return jstxt;
}

const std::string jstxt = jsname();


void show_rgb(CameraRGBImage img, void *p) {
  if (!mtx.try_lock()) return;

  // FPV: h=720 w=1280
  // MAIN: h=960 w=1280
  cv::Mat mat(img.height, img.width, CV_8UC3, img.rawData.data(), img.width*3);
  cv::cvtColor(mat, mat, cv::COLOR_BGR2RGB);    //H20T
  //cv::imwrite("/home/zxj/Downloads/Onboard-SDK/savjs/input.jpg", mat);
  //cv::imshow("input", mat);
  width = mat.size().width;
  height = mat.size().height;
  DSTATUS("w=%d h=%d", width, height);

  auto rects = v5lite->InferenceImage(mat);

  if (running) {
    
    que.push(rects);
    
  }

  OsdkOsal_TaskSleepMs(10);
  mtx.unlock();
}

static void* OMSendTask_json(void *arg) {
  MopPipeline *OM_Pipeline = (MopPipeline *)arg;
  MopErrCode mopRet;
  
  char *sendBuf;
  sendBuf = (char *)OsdkOsal_Malloc(BUFFER_SIZE);
  if (sendBuf == NULL) {
    DERROR("[File-Service]  OsdkOsal_Malloc send buffer error");
    running = false;
    return NULL;
  }

  MopPipeline::DataPackType writePack = {.data = (uint8_t *)sendBuf, .length = BUFFER_SIZE};
  while (running) {
    if (que.empty()) {
      continue;
    }
    auto rects = que.front();
    que.pop();

    Json::Value pkg;
    Json::Value bbox;

    int offset = 0;
    //offset += sprintf_s(sendBuf + offset, BUFFER_SIZE, "%d,%d:", width, height);
    pkg["size"].append(width);
    pkg["size"].append(height);

    int i = 0;
    int max_det = 21;
    bbox[i].append(0);
    bbox[i].append(0);
    bbox[i].append(0);
    bbox[i].append(0);
    bbox[i].append(0);
    bbox[i].append(0);
    i++;
    for (const auto &rect: rects) {
      if (i >= max_det) {
        break;
      }
      //Json::Value box;
      bbox[i].append(int(rect.x));
      bbox[i].append(int(rect.y));
      bbox[i].append(int(rect.w));
      bbox[i].append(int(rect.h));
      bbox[i].append(int(rect.prob*100));
      //bbox[i].append(rect.prob);
      bbox[i].append(rect.classes);
      //pkg["bbox"].push_back({int(rect.x), int(rect.y), int(rect.w), int(rect.h), rect.prob, rect.classes});
      //bbox.append(box);
      i += 1;
    }

    //dirty work
    /*
    for (int s = 0; s < 10; s++){
      bbox[i+s].append(0);
      bbox[i+s].append(0);
      bbox[i+s].append(0);
      bbox[i+s].append(0);
      bbox[i+s].append(0);
      bbox[i+s].append(0);
    }
    i+=10;
    for (int s = 0; s < 100; s++){
      bbox[i+s].append(rand()%500);
      bbox[i+s].append(rand()%1000);
      bbox[i+s].append(rand()%100);
      bbox[i+s].append(rand()%100);
      bbox[i+s].append(rand()%100);
      bbox[i+s].append(0);
    }
    */

    pkg["bbox"] = bbox;
    Json::FastWriter sw;
    string s=sw.write(pkg);
    memcpy( sendBuf, &s[0], s.size());

    writePack.data = (uint8_t *)sendBuf;
    writePack.length = s.size();

    MopErrCode mopRet = OM_Pipeline->sendData(writePack, &writePack.length);

    auto js_time = std::chrono::high_resolution_clock::to_time_t(std::chrono::high_resolution_clock::now());
    char jt[20];
    strftime(jt, sizeof(jt), "%Y%m%d-%H:%M:%S", localtime(&js_time));
    string wrtime = jt;
    pkg["time"] = wrtime;
    Json::FastWriter wt;
    ofstream os;
    
    os.open(jstxt, std::ios::out | std::ios::app);
    if (!os.is_open()){
      cout << "write stop" << endl;
    }
    os << wt.write(pkg);
    os.close();

    if (mopRet == MOP_PASSED) {
      DSTATUS("[File-Service] upload request ack %s", sendBuf);
    } else if (mopRet == MOP_TIMEOUT) {
      DERROR("[File-Service] send timeout");
    } else if (mopRet == MOP_CONNECTIONCLOSE) {
      DERROR("[File-Service] connection close");
      running = false;
    } else {
      DERROR("[File-Service] send error");
    }
    
  }
  OsdkOsal_Free(sendBuf);
  return NULL;
}

static void* OMSendTask(void *arg) {
  MopPipeline *OM_Pipeline = (MopPipeline *)arg;
  MopErrCode mopRet;
  char *sendBuf;

  sendBuf = (char *)OsdkOsal_Malloc(BUFFER_SIZE);
  if (sendBuf == NULL) {
    DERROR("[File-Service]  OsdkOsal_Malloc send buffer error");
    running = false;
    return NULL;
  }

  MopPipeline::DataPackType writePack = {.data = (uint8_t *)sendBuf, .length = BUFFER_SIZE};

  while (running) {
    if (que.empty()) {
      continue;
    }
    auto rects = que.front();
    que.pop();

    int offset = 0;
    offset += sprintf(sendBuf + offset, "%d,%d:", width, height);

    int i = 0;
    int max_det = 20;
    for (const auto &rect: rects) {
      if (i >= max_det) {
        break;
      }
      offset += sprintf(sendBuf + offset, "%d,%d,%d,%d,%.2f,%d;", int(rect.x), int(rect.y), int(rect.w), int(rect.h), rect.prob, rect.classes);
      i += 1;
    }
    writePack.data = (uint8_t *)sendBuf;
    assert(offset > 0);
    writePack.length = offset - 1;

    MopErrCode mopRet = OM_Pipeline->sendData(writePack, &writePack.length);
    if (mopRet == MOP_PASSED) {
      DSTATUS("[File-Service] upload request ack %s", sendBuf);
    } else if (mopRet == MOP_TIMEOUT) {
      DERROR("[File-Service] send timeout");
    } else if (mopRet == MOP_CONNECTIONCLOSE) {
      DERROR("[File-Service] connection close");
      running = false;
    } else {
      DERROR("[File-Service] send error");
    }

  }
  OsdkOsal_Free(sendBuf);
  return NULL;
}

static void *MopServerTask(void *p) {
  PipelineType type = PIPELINE_IS_RELIABLE ? RELIABLE : UNRELIABLE;
  PipelineID id = PIPELINE_ID;

  for (;;) {
    MopServer *server = new MopServer();
    MopPipeline *OM_Pipeline = NULL;
    MopErrCode acceptRet = MOP_PASSED;
    if (((acceptRet = server->accept(id, type, OM_Pipeline)) != MOP_PASSED) || (OM_Pipeline == NULL)) {
      DERROR("MOP server accept failed, ret : %d", acceptRet);
      delete server;
      return NULL;
    } else {
      running = true;
      DSTATUS("accept successfully");
    }

    pthread_t sendTask;
    int sendRst = pthread_create(&sendTask, NULL, OMSendTask_json, OM_Pipeline);
    if (sendRst != 0) {
      DERROR("Send task create failed!\n");
    } else {
      DSTATUS("Send task create success!\n");
    }
    pthread_join(sendTask, NULL);

    if (server->close(id) != MOP_PASSED) {
      DERROR("MOP Pipeline disconnect pipeline(%d) failed", id);
    } else {
      running = false;
      DSTATUS("Disconnect mop pipeline id(%d) successfully", id);
    }
    delete server;
  }
  return NULL;
}

int main(int argc, char** argv) {
  assert(argc == 4);

  LinuxSetup linuxEnvironment(argc, argv, true);
  vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL) {
    std::cout << "Vehicle not initialized, exiting." << std::endl;
    return -1;
  }
  const char *acm_dev = linuxEnvironment.getEnvironment()->getDeviceAcm().c_str();
  vehicle->advancedSensing->setAcmDevicePath(acm_dev);

  v5lite = new V5lite(argv[2]);
  v5lite->LoadEngine(argv[3]);

  bool camRst;
  if (FPV_CAM) {
    camRst = vehicle->advancedSensing->startFPVCameraStream(&show_rgb, NULL);
  } else if (MAIN_CAM) {
    camRst = vehicle->advancedSensing->startMainCameraStream(&show_rgb, NULL);
  }
  if (!camRst) {
    std::cout << "Failed to open selected camera" << std::endl;
    return 1;
  }

  pthread_t serverTask;
  int srvRst = pthread_create(&serverTask, NULL, MopServerTask, NULL);
  if (srvRst != 0) {
    DERROR("Server task create failed!\n");
  } else {
    DSTATUS("Server task create success!\n");
  }
  pthread_join(serverTask, NULL);

  if (FPV_CAM) {
    vehicle->advancedSensing->stopFPVCameraStream();
  } else if (MAIN_CAM) {
    vehicle->advancedSensing->stopMainCameraStream();
  }

  return 0;
}

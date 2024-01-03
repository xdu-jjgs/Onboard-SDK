#include "dji_vehicle.hpp"
#include "dji_linux_helpers.hpp"
#include <string>
#include <pthread.h>
#include <mutex>
#include <condition_variable>
#include <opencv2/opencv.hpp>
#include <jsoncpp/json/json.h>

#define FPV_CAM              false
#define MAIN_CAM             true
#define PIPELINE_IS_RELIABLE false
#define PIPELINE_ID          49155
#define SEND_BUFFER_SIZE     1024
#define RECV_BUFFER_SIZE     10240

Vehicle *vehicle;
bool connected = false;
int width, height;
std::mutex mtx;
int fd_cpp2py, fd_py2cpp;

int output_size;

MopPipeline *pipeline;
uint8_t* send_buffer;
uint8_t* recv_buffer;
MopPipeline::DataPackType writePack = {(uint8_t*)send_buffer, SEND_BUFFER_SIZE};
MopPipeline::DataPackType readPack = {(uint8_t*)recv_buffer, RECV_BUFFER_SIZE};

cv::VideoCapture cap("/home/zxj/Desktop/input.mp4");
void show_rgb(CameraRGBImage img, void*) {
  if (!mtx.try_lock()) return;

  cv::Mat mat(img.height, img.width, CV_8UC3, img.rawData.data(), img.width*3);
  // cap.read(mat);
  cv::cvtColor(mat, mat, cv::COLOR_BGR2RGB);
  // cv::imwrite("/home/zxj/Desktop/input.jpg", mat);
  // cv::imshow("input", mat);
  // cv::waitKey(1);
  vector<unsigned char> data;
  cv::imencode(".jpg", mat, data);
  DSTATUS("write to cpp2py %d bytes", data.size());
  write(fd_cpp2py, &data[0], data.size());

  width = mat.size().width;
  height = mat.size().height;
  DSTATUS("w=%d h=%d", width, height);

  if (connected) {
    writePack.data = (uint8_t*)send_buffer;
    writePack.length = output_size;

    MopErrCode mopRet = pipeline->sendData(writePack, &writePack.length);

    if (mopRet == MOP_PASSED) {
      DSTATUS("[File-Service] send request ack %s", send_buffer);
    } else if (mopRet == MOP_TIMEOUT) {
      DERROR("[File-Service] send timeout");
    } else if (mopRet == MOP_CONNECTIONCLOSE) {
      DERROR("[File-Service] connection close");
      connected = false;
    } else {
      DERROR("[File-Service] send error");
    }
  }

  // OsdkOsal_TaskSleepMs(200);
  mtx.unlock();
}

static void* recv_output_task(void*) {
  while (true) {
    output_size = read(fd_py2cpp, send_buffer, SEND_BUFFER_SIZE);
    DSTATUS("recv from py2cpp %d bytes", output_size);
  }
  return NULL;
}

static void* OMSendTask(void*) {
  return NULL;
}

int colorid = 1;

static void* OMRecvTask(void*) {
  char* write_buffer;
  write_buffer = (char*)OsdkOsal_Malloc(RECV_BUFFER_SIZE);
  int write_size = 0;

  while (connected) {
    memset(recv_buffer, 0, RECV_BUFFER_SIZE);
    readPack.data = (uint8_t*)recv_buffer;
    readPack.length = RECV_BUFFER_SIZE;

    MopErrCode mopRet = pipeline->recvData(readPack, &readPack.length);
    if (readPack.length > 0) {
      memcpy(write_buffer + write_size, recv_buffer, readPack.length);
      write_size += readPack.length;
    } else if (write_size > 0 && readPack.length == 0) {
      std::ofstream ofs;
      std::stringstream ss;
      ss << "/home/zxj/Downloads/YOLOv5-Lite-master/tracker/reid/target";
      ss << colorid;
      ss << ".npy";
      ofs.open(ss.str(), ios::out|ios::binary);
      ofs.write(write_buffer, write_size);
      ofs.close();
      write_size = 0;
      ++colorid;
    }

    if (mopRet == MOP_PASSED) {
      DSTATUS("[File-Service] recv data %d bytes", readPack.length);
    } else if (mopRet == MOP_TIMEOUT) {
      DERROR("[File-Service] recv timeout");
    } else if (mopRet == MOP_CONNECTIONCLOSE) {
      DERROR("[File-Service] connection close");
      connected = false;
    } else {
      DERROR("[File-Service] recv error");
    }
  }
  return NULL;
}

static void* mop_server_task(void*) {
  PipelineType type = PIPELINE_IS_RELIABLE ? RELIABLE : UNRELIABLE;
  PipelineID id = PIPELINE_ID;

  send_buffer = (uint8_t*)OsdkOsal_Malloc(SEND_BUFFER_SIZE);
  if (send_buffer == NULL) {
    DERROR("[File-Service]  OsdkOsal_Malloc send buffer error");
    return NULL;
  }
  recv_buffer = (uint8_t*)OsdkOsal_Malloc(RECV_BUFFER_SIZE);
  if (recv_buffer == NULL) {
    DERROR("[File-Service]  OsdkOsal_Malloc recv buffer error");
    return NULL;
  }

  for (;;) {
    MopServer *server = new MopServer();
    MopErrCode acceptRet = MOP_PASSED;
    if (((acceptRet = server->accept(id, type, pipeline)) != MOP_PASSED) || (pipeline == NULL)) {
      DERROR("MOP server accept failed, ret : %d", acceptRet);
      delete server;
      break;
    } else {
      connected = true;
      DSTATUS("accept successfully");
    }

    pthread_t sendTask;
    if (pthread_create(&sendTask, NULL, OMSendTask, NULL) != 0) {
      DERROR("send task: failed to create");
      break;
    } else {
      DSTATUS("send task: created");
    }

    pthread_t recvTask;
    if (pthread_create(&recvTask, NULL, OMRecvTask, NULL) != 0) {
      DERROR("recv task: failed to create");
      break;
    } else {
      DSTATUS("recv task: created");
    }

    pthread_join(sendTask, NULL);
    pthread_join(recvTask, NULL);

    if (server->close(id) != MOP_PASSED) {
      DERROR("MOP Pipeline disconnect pipeline(%d) failed", id);
    } else {
      connected = false;
      DSTATUS("Disconnect mop pipeline id(%d) successfully", id);
    }
    delete server;
  }

  OsdkOsal_Free(send_buffer);
  OsdkOsal_Free(recv_buffer);
  return NULL;
}

int main(int argc, char** argv) {
  assert(argc == 2);

  LinuxSetup linuxEnvironment(argc, argv, true);
  vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL) {
    DERROR("failed to initialize vehicle");
  }
  const char *acm_dev = linuxEnvironment.getEnvironment()->getDeviceAcm().c_str();
  vehicle->advancedSensing->setAcmDevicePath(acm_dev);

  /*
  for (int i = 0; i < 320; i++) {
    cv::Mat mat;
    cap.read(mat);
  }
  */

  if ((fd_cpp2py = open("/home/zxj/Desktop/cpp2py", O_WRONLY)) != 0) {
    DERROR("cpp2py fifo: failed to open");
  } else {
    DSTATUS("cpp2py fifo: opened");
  }
  if ((fd_py2cpp = open("/home/zxj/Desktop/py2cpp", O_RDONLY)) != 0) {
    DERROR("py2cpp fifo: failed to open");
  } else {
    DSTATUS("py2cpp fifo: opened");
  }

  bool camRst;
  if (FPV_CAM) {
    camRst = vehicle->advancedSensing->startFPVCameraStream(&show_rgb, NULL);
  } else if (MAIN_CAM) {
    camRst = vehicle->advancedSensing->startMainCameraStream(&show_rgb, NULL);
  }
  if (!camRst) {
    DERROR("failed to open selected camera");
  }

  pthread_t mop_server_thrd;
  int mop_server_rst = pthread_create(&mop_server_thrd, NULL, mop_server_task, NULL);
  if (mop_server_rst != 0) {
    DERROR("Server task create failed!\n");
  } else {
    DSTATUS("Server task create success!\n");
  }

  pthread_t recv_output_thrd;
  int recv_output_rst = pthread_create(&recv_output_thrd, NULL, recv_output_task, NULL);
  if (recv_output_rst != 0) {
    DERROR("recv output task create failed!\n");
  } else {
    DSTATUS("recv output task create success!\n");
  }

  pthread_join(mop_server_thrd, NULL);
  pthread_join(recv_output_thrd, NULL);

  if (FPV_CAM) {
    vehicle->advancedSensing->stopFPVCameraStream();
  } else if (MAIN_CAM) {
    vehicle->advancedSensing->stopMainCameraStream();
  }

  return 0;
}

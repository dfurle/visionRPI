#include "variables.h"
#include "clock.h"

// threads.cpp
pthread_t VideoCap_t;
void* VideoCap(void* arg);


pthread_t videoSave_t;
void* VideoSave(void* arg);

// tcpserver.cpp
pthread_t opentcp_t;
void* opentcp(void* arg);

// tcpserver.cpp
pthread_t videoServer_t;
void* videoServer(void* arg);

inline bool checkErr(int rc, std::string name) {
  if (rc != 0) {
    printf("%s thread fail %d\n", name.c_str(), rc);
    return false;
  } else
    return true;
}

/* valid names:
    "USB"
    "SERVER"
    "DRIVE"
    "PID"
*/
bool startThread(std::string name, void* params) {
  int rc = 1;
  if (!name.compare("SERVER")) {
    rc = pthread_create(&videoServer_t, NULL, videoServer, NULL);
    return checkErr(rc, name);
  }
  if (!name.compare("VIDEO")) {
    rc = pthread_create(&VideoCap_t, NULL, VideoCap, NULL);
    return checkErr(rc, name);
  }
  if (!name.compare("TCP")) {
    rc = pthread_create(&opentcp_t, NULL, opentcp, NULL);
    return checkErr(rc, name);
  }
  if(!name.compare("SAVE")){
    rc = pthread_create(&opentcp_t, NULL, opentcp, params);
    return checkErr(rc, name);
  }
  return false;
}

void* VideoCap(void* args) {
  cv::VideoCapture vcap;
  if (Switches::cameraInput == 2) {
    printf("Not Using Camera\n");
    // printf("ERR: function was disabled\n");
    // exit(1);
  } else {
    while (!vcap.open(Switches::cameraInput)) {
      std::cout << "cant connect" << std::endl;
      usleep(10000000);
    }
    printf("Using Camera: %d\n",Switches::cameraInput);
  }
  printf("  setting brightness\n");
  vcap.set(cv::CAP_PROP_BRIGHTNESS, 100);
  printf("  setting auto exposure\n");
  vcap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
  printf("  setting exposure\n");
  vcap.set(cv::CAP_PROP_EXPOSURE, Var::EXPOSURE);
  usleep(1000);
  printf("  exposure at: %f\n",vcap.get(cv::CAP_PROP_EXPOSURE));
  vcap.set(cv::CAP_PROP_AUTOFOCUS, 0);
  vcap.set(cv::CAP_PROP_FRAME_WIDTH, Var::WIDTH);
  vcap.set(cv::CAP_PROP_FRAME_HEIGHT, Var::HEIGHT);
  Global::FrameWidth = vcap.get(cv::CAP_PROP_FRAME_WIDTH);
  Global::FrameHeight = vcap.get(cv::CAP_PROP_FRAME_HEIGHT);
  if(Switches::cameraInput == 2){
    Global::FrameHeight = 480;
    Global::FrameWidth = 640;
  }
  // vcap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('H','2','6','4'));
  int b = vcap.get(cv::CAP_PROP_FOURCC);
  char* fourcc = (char*) &b;
  fourcc[4] = 0;
  int cont = 0;
  printf("fourcc: %d |%s|\n",b,fourcc);
  if(Switches::cameraInput != 2){
    while (true) {
      if(vcap.grab()){
        Global::muteFrame.lock();
        vcap.retrieve(Global::frame);
        Global::newFrame = true;
        Global::muteFrame.unlock();
      }
      usleep(Var::waitAfterFrame);
    }
  }
}

void* VideoSave(void* arg){
  cv::Mat* img = (cv::Mat*) arg;
  int currentLog = 0;
  int fourcc = cv::VideoWriter::fourcc('M','J','P','G');
  int prevTime = 30;
  cv::VideoWriter out;
  Clock savingClock;

  if (Switches::SAVE){
    Global::muteImg.lock();
    out.write(*img);
    Global::muteImg.unlock();
    if(savingClock.getTimeAsSecs() >= 30.){
      printf("---SAVING---\n");
      savingClock.restart();
      out.release();
      std::string name = "./output";
      name += std::to_string(currentLog);
      name += ".avi";
      currentLog++;
      out.open(name,fourcc,30.,cv::Size(Var::WIDTH,Var::HEIGHT));
    } else {
      int time =  int(30.-savingClock.getTimeAsSecs());
      if(time != prevTime){
        printf("time till next save: %d\n",time);
        prevTime = time;
      }
    }
  }
}

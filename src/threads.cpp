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
    rc = pthread_create(&videoSave_t, NULL, VideoSave, params);
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
  } else {
    Global::FrameHeight = 480;
    Global::FrameWidth = 640;
    int num = 0;
    while(true){
      std::string imgText = "2022/BG";
      imgText.append(std::to_string(num));
      imgText.append(".jpg");
      if(++num >= Var::numImgs){
        num = 0;
      }

      Global::muteFrame.lock();
      printf("loading %s\n",imgText.c_str());
      Global::frame = cv::imread(imgText);
      if(Global::frame.empty()){
        printf("ERROR LOADING ABOVE FILE\n");
      }
      Global::newFrame = true;
      Global::muteFrame.unlock();
      sleep(Var::waitSeconds);
    }
  }
}

void* VideoSave(void* arg){
  printf("DO NOT USE SAVE FOR NOW, STILL WORKING ON IT\n");
  exit(1);

  cv::Mat* img = (cv::Mat*) arg;
  int currentLog = 0;
  int fourcc = cv::VideoWriter::fourcc('M','J','P','G');
  int prevTime = 30;
  cv::VideoWriter out;
  Clock timer;

  out.open("./output0.avi",fourcc,30.,cv::Size(Var::WIDTH,Var::HEIGHT));


  while(true){
    Global::muteImg.lock();
    out.write(*img);
    cv::imshow("debug",*img);
    cv::waitKey(0);
    Global::muteImg.unlock();

    if(timer.getTimeAsSecs() >= 30){
      timer.restart();
      printf("---SAVING---\n");
      out.release();

      std::string name = "./output";
      name += std::to_string(currentLog++);
      name += ".avi";
      out.open(name,fourcc,30.,cv::Size(Var::WIDTH,Var::HEIGHT));
    } else {
      int time =  int(30.-timer.getTimeAsSecs());
      if(time != prevTime){
        printf("Next Save In: %ds\n",time);
        prevTime = time;
      }
    }
    
  }
}

#include "variables.h"
#include "httpserver.h"
#include "clock.h"

// threads.cpp
pthread_t VideoCap_t;
void* VideoCap(void* arg);

// threads.cpp
pthread_t videoSave_t;
void* VideoSave(void* arg);

// tcpserver.cpp
pthread_t opentcp_t;
void* opentcp(void* arg);

// httpserver.cpp
pthread_t handleHttp_thread_t;
void* handleHttp_thread(void* arg);

inline bool checkErr(int rc, std::string name) {
  if (rc != 0) {
    printf("%s thread fail %d\n", name.c_str(), rc);
    return false;
  } else
    return true;
}

/**
 *  Valid Names:
 *   "USB"
 *   "SAVE"
 *   "PID"
 *   "HTTP"
 **/
bool startThread(std::string name, void* params) {
  int rc = 1;
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
  if(!name.compare("HTTP")){
    rc = pthread_create(&handleHttp_thread_t, NULL, handleHttp_thread, NULL);
    return checkErr(rc, name);
  }
  return false;
}

void* VideoCap(void* args) {
  // std::string cameraPipeline;
  // cameraPipeline ="v4l2src device=/dev/video0 extra-controls=\"c,exposure_auto=0,exposure_absolute=40\" ! ";
  // cameraPipeline+="video/x-raw, format=BGR, framerate=30/1, width=(int)640,height=(int)480 ! ";
  // cameraPipeline+="appsink";

  cv::VideoCapture vcap;
  if(!Switches::USECAM) {
    printf("Not Using Camera\n");
    // printf("ERR: function was disabled\n");
    // exit(1);
  } else {
    // while (!vcap.open(cameraPipeline)) {
    while (!vcap.open(0)) {
      std::cout << "cant connect" << std::endl;
      usleep(10000000);
    }
    printf("Using Camera: %d\n",0);

    printf("---===BEFORE===---\n");
    printf("auto exposure: %f\n",vcap.get(cv::CAP_PROP_AUTO_EXPOSURE));
    printf("exposure: %f\n",vcap.get(cv::CAP_PROP_EXPOSURE));
    printf("brightness: %f\n",vcap.get(cv::CAP_PROP_BRIGHTNESS));
    {
    int b = vcap.get(cv::CAP_PROP_FOURCC);
    char* fourcc = (char*) &b;
    fourcc[4] = 0;
    int cont = 0;
    printf("fourcc: %d |%s|\n",b,fourcc);
    }

    printf("---===Setting===---\n");
    printf("fourcc: %d\n",vcap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y','U','Y','V')));
    printf("auto exposure: %d\n",vcap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1));
    printf("exposure: %d\n",vcap.set(cv::CAP_PROP_EXPOSURE, 3));
    printf("autofocus: %d\n",vcap.set(cv::CAP_PROP_AUTOFOCUS, 0));
    printf("Frame: %f, %d\n",Global::FrameWidth,Var::WIDTH);
    printf("width: %d\n",vcap.set(cv::CAP_PROP_FRAME_WIDTH, Var::WIDTH));
    printf("height: %d\n",vcap.set(cv::CAP_PROP_FRAME_HEIGHT, Var::HEIGHT));
    Global::FrameWidth = vcap.get(cv::CAP_PROP_FRAME_WIDTH);
    Global::FrameHeight = vcap.get(cv::CAP_PROP_FRAME_HEIGHT);


    printf("---===AFTER===---\n");
    printf("auto exposure: %f\n",vcap.get(cv::CAP_PROP_AUTO_EXPOSURE));
    printf("exposure: %f\n",vcap.get(cv::CAP_PROP_EXPOSURE));
    printf("brightness: %f\n",vcap.get(cv::CAP_PROP_BRIGHTNESS));
    {
    int b = vcap.get(cv::CAP_PROP_FOURCC);
    char* fourcc = (char*) &b;
    fourcc[4] = 0;
    int cont = 0;
    printf("fourcc: %d |%s|\n",b,fourcc);
    }
  }

  if(Switches::USECAM){
    while (true) {
      if(vcap.grab()){
        Global::muteFrame.lock();
        vcap.retrieve(Global::frame);
        Global::newFrame = true;
        Global::muteFrame.unlock();
      }
      if(int(Global::FrameWidth) != Var::WIDTH){
        printf("width: %d\n",vcap.set(cv::CAP_PROP_FRAME_WIDTH, Var::WIDTH));
        printf("height: %d\n",vcap.set(cv::CAP_PROP_FRAME_HEIGHT, Var::HEIGHT));
        Global::FrameWidth = Var::WIDTH;
        Global::FrameHeight = Var::HEIGHT;
      }
      usleep(Var::waitAfterFrame);
    }
  } else {
    Global::FrameHeight = Var::HEIGHT;
    Global::FrameWidth = Var::WIDTH;
    int num = 0;
    while(true){
      std::string imgText;
      if(Var::WIDTH == 1280)
        imgText = "../2022-720p/BG";
      else
        imgText = "../2022/BG";
      imgText.append(std::to_string(num++));
      //imgText.append(std::to_string(1));
      imgText.append(".jpeg");

      Global::muteFrame.lock();
      // printf("loading %s\n",imgText.c_str());
      Global::frame = cv::imread(imgText);
      if(Global::frame.empty()){
        // printf("ERROR LOADING ABOVE FILE\n");
        num = 0;
        Global::muteFrame.unlock();
        continue;
      }
      Global::muteFrame.unlock();
      long total = 0;
      int waitSeconds = 4;
      while(waitSeconds * 1000000 > total){
        Global::muteFrame.lock();
        Global::newFrame = true;
        Global::muteFrame.unlock();
        usleep(33000);
        total += 33000;
      }
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

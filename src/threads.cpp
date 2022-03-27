#include "variables.h"
#include "clock.h"

// threads.cpp
std::thread* videoCap_t;
void VideoCap();

// threads.cpp
std::thread* videoSave_t;
void VideoSave();

// tcpserver.cpp
std::thread* opentcp_t;
void opentcp();

/**
 *  Valid Names:
 *   "VIDEO"
 *   "TCP"
 *   "SAVE"
 **/
bool startThread(std::string name, void* params) {
  if (!name.compare("VIDEO")) {
    videoCap_t = new std::thread(&VideoCap);
  }
  if (!name.compare("TCP")) {
    opentcp_t = new std::thread(&opentcp);
  }
  if(!name.compare("SAVE")){
    videoSave_t = new std::thread(&VideoSave);
  }
  return false;
}

void VideoCap() {
  // std::string cameraPipeline;
  // cameraPipeline ="v4l2src device=/dev/video0 extra-controls=\"c,exposure_auto=0,exposure_absolute=40\" ! ";
  // cameraPipeline+="video/x-raw, format=BGR, framerate=30/1, width=(int)640,height=(int)480 ! ";
  // cameraPipeline+="appsink";

  cv::VideoCapture vcap;
  if(!Switches::USECAM) {
    printf("Not Using Camera\n");
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
    ClockTimer timer(false);
    while (true) {
      timer.reset();
      if(vcap.grab()){
        timer.printTime("Grab");
        Global::muteFrame.lock();
        timer.printTime(" Lock");
        vcap.retrieve(Global::frame);
        timer.printTime(" Retrieve");
        cv::rotate(Global::frame, Global::frame, cv::ROTATE_90_CLOCKWISE);
        timer.printTime(" Rotate");
        Global::newFrame = true;
        Global::muteFrame.unlock();
      }
      timer.PTotal();
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

void VideoSave(){
  int currentLog = 1;
  int fourcc = cv::VideoWriter::fourcc('M','J','P','G');
  int prevTime = 30;
  cv::VideoWriter out;
  Clock timer;

  out.open("./output0.avi",fourcc,30.,cv::Size(Var::WIDTH,Var::HEIGHT));

  while(true){
    Global::muteImg.lock();
    if(Global::imgClean.size().area() != 0){
      out.write(Global::imgClean);
    }
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
    usleep(33000);
  }
}

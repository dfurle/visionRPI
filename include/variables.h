#pragma once

// #define RASPI

#include <ctime>
#include <iostream>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <vector>

#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <arpa/inet.h>
#include <cstdint>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <net/if.h>
#include <sstream>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <fstream>
#ifdef RASPI
  #include <linux/i2c-dev.h>
  #include <sys/types.h>
#endif





namespace Var {
extern int minR;
extern int maxR;
extern int minG;
extern int maxG;
extern int minB;
extern int maxB;

extern int          WIDTH;
extern int          HEIGHT;
extern int          EXPOSURE;
extern unsigned int waitAfterFrame;

extern double dist_cof[5];

extern double qualityLevel;
extern double minDistance;
extern int    blockSize;
extern bool   useHarisDetector;
extern double k;
extern int    maxCorners;

extern size_t maxTargets;
extern size_t avSize;

extern double IRLOffset;

// extern unsigned int videoPort;
extern int waitSeconds;
extern int numImgs;
} // namespace Var

class Target {
public:
  Target() { NullTargets(); };
  cv::RotatedRect rect;
  cv::Rect        boundingRect;
  double          area;
  int             id;
  cv::Point2d     points[4];
  void NullTargets() {
    // rect = cv::RotatedRect();
    // boundingRect = cv::Rect();
    area = 0.;
    id = -1;
    for (int i = 0; i < 4; i++) {
      points[i].x = 0;
      points[i].y = 0;
    }
  }
};


class Position{
public:
  Position(){ nullifyStruct(); }
  double dist;
  double robotAngle;
  void nullifyStruct() {
    dist = 0;
    robotAngle = 0;
  }
};

class mMutex{
public:
  pthread_mutex_t mutex;
  mMutex(){
    mutex = PTHREAD_MUTEX_INITIALIZER;
  }
  void lock(){
    pthread_mutex_lock(&mutex);
  }
  void unlock(){
    pthread_mutex_unlock(&mutex);
  }
};

namespace Global {
extern bool             newFrame;
extern double           FrameWidth, FrameHeight;
extern bool             interupt;
extern bool             dataValid;
extern double           gyroAngle;
extern double           gyroVelocity;
extern double           driveAngle;
extern double           turn;
extern double           P, I, D;
extern std::vector<Target> targets;
extern Position         position, positionAV;
extern int              buttonPress;
extern bool             videoError;
extern int              videoSocket;
extern const cv::Scalar BLUE, RED, YELLOW, GREEN;
// extern int sockets[2];
extern std::vector<int> imgSocket;
extern std::vector<int> thrSocket;

extern cv::Mat frame;
extern cv::Mat imgC, thresholdedC;

extern int httpStatus;

// extern pthread_mutex_t muteFrame;
// extern pthread_mutex_t mutePos;
// extern pthread_mutex_t muteImg;
extern mMutex muteFrame;
extern mMutex mutePos;
extern mMutex muteImg;

} // namespace Global

namespace Switches {
extern bool         SHOWORIG;
extern bool         SHOWHUE;
extern bool         SHOWTHRESH;
extern bool         SHOWTRACK;
extern bool         USEHTTP;
// extern bool         USESERVER;
// extern bool         USECOLOR;
extern bool         DOPRINT;
extern bool         FRAME;
extern bool         SAVE;
extern bool         DRAW;
extern int          printTime;
extern double       InitPID[];
extern int          cameraInput;
} // namespace Switches



bool startThread(std::string name, void* params = NULL);
void initSolvePnP();
void findAnglePnP(cv::Mat& img);

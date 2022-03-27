#pragma once

#include <ctime>
#include <iostream>
#include <thread>
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


namespace Var {
extern cv::Scalar minBGR;
extern cv::Scalar maxBGR;

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

} // namespace Var

class Target {
public:
  Target() { NullTargets(); };
  int             id;
  cv::RotatedRect minRect;
  cv::Rect        boundingRect;
  double          area;
  cv::Point2f     center;
  cv::Point2f     centerAim;
  cv::Point2f     points[4];
  void NullTargets() {
    // rect = cv::RotatedRect();
    // boundingRect = cv::Rect();
    area = 0.;
    id = -1;
    for (int i = 0; i < 4; i++) {
      points[i].x = 0;
      points[i].y = 0;
    }
    center.x = 0.;
    center.y = 0.;
    centerAim.x = 0.;
    centerAim.y = 0.;
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


namespace Global {
extern bool             newFrame;
extern double           FrameWidth, FrameHeight;
extern bool             interupt;
extern bool             dataValid;
extern std::vector<Target> targets;
extern Position         position, positionAV;
extern bool             videoError;
extern int              videoSocket;
extern const cv::Scalar BLUE, RED, YELLOW, GREEN;

extern std::vector<int> imgSocket;
extern std::vector<int> thrSocket;
extern std::vector<int> rPosSocket;

extern cv::Mat frame;
// extern cv::Mat imgC, thresholdedC, rPosC;
extern cv::Mat imgClean;

extern cv::Mat tvec_g, rvec_g;
extern bool useTR;

extern int httpStatus;

extern bool missPrev;

extern std::mutex muteFrame;
extern std::mutex mutePos;
extern std::mutex muteImg;
extern std::mutex muteHTTP;

} // namespace Global

namespace Switches {
extern bool         SHOWORIG;
extern bool         SHOWTHRESH;
extern bool         USEHTTP;
extern bool         DOPRINT;
extern bool         TFPRINT;
extern bool         FRAME;
extern bool         SAVE;
extern bool         DRAW;
extern bool         USECAM;
extern int          printTime;
} // namespace Switches


// In target.cpp
bool startThread(std::string name, void* params = NULL);
int findTarget(cv::Mat& img, cv::Mat& thresholded);
void initSolvePnP();
void findAnglePnP(cv::Mat& img, cv::Mat& rPos);

// In variables.cpp
// namespace str{
// bool cmp(std::string& s, std::string c);
// std::string substring(std::string& s, int i, int f);
// std::string substring(std::string& s, int i);
// bool contains(std::string& s, std::string c);
// std::vector<std::string> split(std::string s, std::string delim);
// std::string containsParam(std::vector<std::string>& v, std::string search);
// std::string getParam(std::vector<std::string>& v, std::string search);
// std::vector<char> ss_to_vec(std::stringstream& ss);
// }


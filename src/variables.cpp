#include "variables.h"


namespace Var {    //      2021
int minR = 0;   // 218   // 150
int maxR = 255; // 255
int minG = 194; // 209   // 195
int maxG = 255; // 255
int minB = 0;   // 222
int maxB = 120; // 241

int          WIDTH          = 640; // 1920 //1280 //640
int          HEIGHT         = 480; // 1080 //720  //480
//int          WIDTH          = 1280; // 1920 //1280 //640
//int          HEIGHT         = 720; // 1080 //720  //480
int          EXPOSURE       = 3; // edit in threads.cpp for now
unsigned int waitAfterFrame = 1000;

double dist_cof[5];

double IRLOffset = 0.;

int waitSeconds = 8;
int numImgs = 4;
} // namespace Var

namespace Global {
bool             newFrame = false;
double           FrameWidth, FrameHeight;
bool             interupt     = false;
bool             dataValid    = 0;
std::vector<Target> targets;
Position         position, positionAV;
bool             videoError  = false;
int              videoSocket = 0;
const cv::Scalar BLUE = cv::Scalar(255, 0, 0), RED = cv::Scalar(0, 0, 255), YELLOW = cv::Scalar(0, 255, 255), GREEN = cv::Scalar(0, 255, 0);

cv::Mat frame;
cv::Mat imgC, thresholdedC;

std::vector<int> imgSocket;
std::vector<int> thrSocket;

int httpStatus = 0;

mMutex muteFrame;
mMutex mutePos;
mMutex muteImg;
} // namespace Global

namespace Switches {
bool         SHOWORIG                   = false;
bool         SHOWHUE                    = false;
bool         SHOWTHRESH                 = false;
bool         SHOWTRACK                  = false;
bool         USEHTTP                    = false;
bool         DOPRINT                    = false;
bool         FRAME                      = true;
bool         SAVE                       = false;
bool         DRAW                       = true;
int          printTime                  = 0;
double       InitPID[]                  = {0, 0, 0};
int          cameraInput                = 0;
} // namespace Switches


namespace str{

bool cmp(std::string& s, std::string c){
  return s.compare(c) == 0;
}

/**
 * Returns substring between [i,f)
 */
std::string substring(std::string& s, int i, int f){
  return s.substr(i,f-i);
}
std::string substring(std::string& s, int i){
  return s.substr(i);
}

bool contains(std::string& s, std::string c){
  return s.find(c) != std::string::npos;
}

std::vector<std::string> split(std::string s, std::string delim){
  std::vector<std::string> nStr;
  int pos = s.find(delim);
  while(pos != std::string::npos){
    nStr.push_back(substring(s,0,pos));
    s = str::substring(s,pos+1);
    pos = s.find(delim);
  }
  if(s.length() != 0)
    nStr.push_back(s);
  return nStr;
}

std::string containsParam(std::vector<std::string>& v, std::string search){
  for(std::string s : v){
    if(contains(s,search))
      return s;
  }
  return "";
}

std::string getParam(std::vector<std::string>& v, std::string search){
  for(std::string s : v){
    if(contains(s,search)){
      return s.substr(s.find(' ')+1);
    }
  }
  return "";
}

std::vector<char> ss_to_vec(std::stringstream& ss){
  std::string s = ss.str();
  return std::vector<char>(s.begin(),s.end());
}
}

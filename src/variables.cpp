#include "variables.h"


namespace Var {
//               { B , G , R }
cv::Scalar minBGR{ 39,114,  0};
cv::Scalar maxBGR{255,255, 89};

// int          WIDTH          = 640; // 1920 //1280 //640
// int          HEIGHT         = 480; // 1080 //720  //480


// int          WIDTH          = 1280; // 1920 //1280 //640
// int          HEIGHT         = 720; // 1080 //720  //480
int          EXPOSURE       = 3; // edit in threads.cpp for now
unsigned int waitAfterFrame = 1000;

double dist_cof[5];

double IRLOffset = 0.;
} // namespace Var

namespace Global {
bool             newFrame = false;
bool             interupt     = false;
bool             dataValid    = 0;
std::vector<Target> targets;
Position         position, positionAV;
bool             videoError  = false;
int              videoSocket = 0;
const cv::Scalar BLUE = cv::Scalar(255, 0, 0), RED = cv::Scalar(0, 0, 255), YELLOW = cv::Scalar(0, 255, 255), GREEN = cv::Scalar(0, 255, 0);

cv::Size SIZE;

int FRAME_SIZES[6] = {
  640,
  480,
  1280,
  720,
  1920,
  1080
};

cv::Mat frame;
// cv::Mat imgC, thresholdedC, rPosC;
cv::Mat imgClean;

cv::Mat tvec_g(cv::Size(1,3),6);
cv::Mat rvec_g(cv::Size(1,3),6);
bool useTR;

std::vector<int> imgSocket;
std::vector<int> thrSocket;
std::vector<int> rPosSocket;

int httpStatus = 0;

bool missPrev = true;

std::mutex muteFrame;
std::mutex mutePos;
std::mutex muteImg;
std::mutex muteHTTP;
} // namespace Global

namespace Switches {
bool SHOWORIG;
bool SHOWTHRESH;
bool USEHTTP;
bool DOPRINT;
bool TFPRINT;
bool FRAME;
bool SAVE;
bool DRAW;
bool USECAM;
int  printTime;
int  resolution;
} // namespace Switches


// namespace str{

// bool cmp(std::string& s, std::string c){
//   return s.compare(c) == 0;
// }

// /**
//  * Returns substring between [i,f)
//  */
// std::string substring(std::string& s, int i, int f){
//   return s.substr(i,f-i);
// }
// std::string substring(std::string& s, int i){
//   return s.substr(i);
// }

// bool contains(std::string& s, std::string c){
//   return s.find(c) != std::string::npos;
// }

// std::vector<std::string> split(std::string s, std::string delim){
//   std::vector<std::string> nStr;
//   int pos = s.find(delim);
//   while(pos != std::string::npos){
//     nStr.push_back(substring(s,0,pos));
//     s = str::substring(s,pos+1);
//     pos = s.find(delim);
//   }
//   if(s.length() != 0)
//     nStr.push_back(s);
//   return nStr;
// }

// std::string containsParam(std::vector<std::string>& v, std::string search){
//   for(std::string s : v){
//     if(contains(s,search))
//       return s;
//   }
//   return "";
// }

// std::string getParam(std::vector<std::string>& v, std::string search){
//   for(std::string s : v){
//     if(contains(s,search)){
//       return s.substr(s.find(' ')+1);
//     }
//   }
//   return "";
// }

// std::vector<char> ss_to_vec(std::stringstream& ss){
//   std::string s = ss.str();
//   return std::vector<char>(s.begin(),s.end());
// }
// }

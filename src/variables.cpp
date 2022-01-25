#include "variables.h"



namespace Var {    //      2021
int minR = 0;   // 218   // 150
int maxR = 255; // 255
int minG = 100; // 209   // 195
int maxG = 255; // 255
int minB = 0;   // 222
int maxB = 180; // 241

int          WIDTH          = 640; // 1920//1280//640
int          HEIGHT         = 480; // 1080//720//480
int          EXPOSURE       = 20;
unsigned int waitAfterFrame = 1000;

double fx = 600;
double fy = 600;
double cx = 250;
double cy = 260;

double dist_cof[5];

double qualityLevel     = 0.05;
double minDistance      = 30;
int    blockSize        = 3;
bool   useHarisDetector = true;
double k                = 0.04;
int    maxCorners       = 4;

size_t maxTargets = 50;
size_t avSize     = 10;

double IRLOffset = 0.;

unsigned int videoPort = 4097;
} // namespace Var

namespace Global {
bool             newFrame = false;
double           FrameWidth, FrameHeight;
bool             interupt     = false;
bool             dataValid    = 0;
double           gyroAngle    = 0;
double           gyroVelocity = 0.0;
double           driveAngle   = 0;
double           turn         = 0.0;
double           P = 0.0, I = 0.0, D = 0.0;
std::vector<Target> targets;
int              buttonPress = 0;
bool             videoError  = false;
int              videoSocket = 0;
const cv::Scalar BLUE = cv::Scalar(255, 0, 0), RED = cv::Scalar(0, 0, 255), YELLOW = cv::Scalar(0, 255, 255), GREEN = cv::Scalar(0, 255, 0);

cv::Mat frame;

pthread_mutex_t frameMutex = PTHREAD_MUTEX_INITIALIZER;
// pthread_mutex_t targetMutex = PTHREAD_MUTEX_INITIALIZER;
// pthread_mutex_t positionMutex = PTHREAD_MUTEX_INITIALIZER;

} // namespace Global

namespace Switches {
bool         SHOWORIG                   = false;
bool         SHOWHUE                    = false;
bool         SHOWTHRESH                 = false;
bool         SHOWTRACK                  = false;
bool         USESERVER                  = false;
bool         USECOLOR                   = false;
bool         DOPRINT                    = false;
bool         FRAME                      = true;
bool         SAVE                       = false;
int          printTime                  = 0;
double       InitPID[]                  = {0, 0, 0};
int          cameraInput                = 0;
} // namespace Switches

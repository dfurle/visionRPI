#include "variables.h"

// threads.cpp
pthread_t MJPEG;
void* VideoCap(void* arg);

// threads.cpp
pthread_t USBSlaveThread;
void* USBSlave(void* arg);

// tcpserver.cpp
pthread_t tcpserver;
void* opentcp(void* arg);

// tcpserver.cpp
pthread_t videoServerThread;
void* videoServer(void* arg);

// drive.cpp
pthread_t PIDThread;
void* movePID(void* arg);

// drive.cpp
pthread_t DriveThread;
void* drive(void* arg);

inline bool checkErr(int rc, std::string name) {
  if (rc != 0) {
    printf("%s thread fail%d\n", name.c_str(), rc);
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
  if (!name.compare("USB")) {
    pthread_create(&USBSlaveThread, NULL, USBSlave, NULL);
    // int rc = pthread_setname_np(USBSlaveThread, "GyroThread");
    return checkErr(rc, name);
  }
  if (!name.compare("SERVER")) {
    pthread_create(&videoServerThread, NULL, videoServer, NULL);
    // int rc = pthread_setname_np(videoServerThread, "VideoThread");
    return checkErr(rc, name);
  }
  if (!name.compare("DRIVE")) {
    pthread_create(&DriveThread, NULL, drive, params);
    // int rc = pthread_setname_np(DriveThread, "DriveThread");
    return checkErr(rc, name);
  }
  if (!name.compare("PID")) {
    pthread_create(&PIDThread, NULL, movePID, NULL);
    // int rc = pthread_setname_np(PIDThread, "PIDThread");
    return checkErr(rc, name);
  }
  if (!name.compare("VIDEO")) {
    pthread_create(&MJPEG, NULL, VideoCap, NULL);
    // int rc = pthread_setname_np(MJPEG, "MJPEG Thread");
    return checkErr(rc, name);
  }
  if (!name.compare("TCP")) {
    pthread_create(&tcpserver, NULL, opentcp, &params);
    // int rc = pthread_setname_np(tcpserver, "tcpserver");
    return checkErr(rc, name);
  }
  return false;
}

void* VideoCap(void* args) {
  cv::VideoCapture vcap;
  if (Switches::cameraInput == 2) {
    // while (!vcap.open(0)) {
    //   std::cout << "cant connect" << std::endl;
    //   usleep(10000000);
    // }
    printf("Not Using Camera\n");
  } else {
    while (!vcap.open(Switches::cameraInput)) {
      std::cout << "cant connect" << std::endl;
      usleep(10000000);
    }
  }
  printf("setting brightness\n");
  vcap.set(cv::CAP_PROP_BRIGHTNESS, 100);
  printf("setting auto exposure\n");
  vcap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
  printf("setting exposure\n");
  vcap.set(cv::CAP_PROP_EXPOSURE, Var::EXPOSURE);
  vcap.set(cv::CAP_PROP_AUTOFOCUS, 0);
  vcap.set(cv::CAP_PROP_FRAME_WIDTH, Var::WIDTH);
  vcap.set(cv::CAP_PROP_FRAME_HEIGHT, Var::HEIGHT);
  Global::FrameWidth = vcap.get(cv::CAP_PROP_FRAME_WIDTH);
  Global::FrameHeight = vcap.get(cv::CAP_PROP_FRAME_HEIGHT);
  if(Switches::cameraInput == 2){
    Global::FrameHeight = 480;
    Global::FrameWidth = 640;
  }
  while (true) {
    pthread_mutex_lock(&Global::frameMutex);
    if (Switches::cameraInput != 2)
      vcap.read(Global::frame);
    pthread_mutex_unlock(&Global::frameMutex);
    Global::newFrame = true;
    usleep(Var::waitAfterFrame);
  }
}

void* USBSlave(void* arg) {
  printf("enter gyro slave\n");
  int ttyFid = open("/dev/ttyUSB0", O_RDWR);
  if (ttyFid == -1) {
    printf("Error unable to open port\n");
  }
  printf("enter readBus\n");
  char line[256];
  while (true) {
    for (int ii = 0; ii < 200; ii++) {
      int nb = read(ttyFid, &line[ii], 1);
      if (nb != 1) {
        printf("nb=%d\n", nb);
        Global::dataValid = false;
      } else
        Global::dataValid = true;
      if (nb < 0) {
        sleep(1);
        ii = 11;
        continue;
      }
      if (line[ii] == ',') {
        line[ii] = ' ';
      }
      if (line[ii] == '\n') {
        line[ii + 1] = 0;
        break;
      }
    }
    // printf("line=%s\n",line);
    float roll, pitch, yaw;
    float ACCX, ACCY, GYROZ, AAZ;
    // sscanf(line,"%f,%f,%f",&roll,&pitch,&yaw);
    sscanf(line, "%f %f %f %f %f %f %f", &ACCX, &ACCY, &roll, &pitch, &yaw, &GYROZ, &AAZ);
    if (GYROZ > 100)
      printf("yaw: %.2f; GYROZ: %.2f\n", yaw, GYROZ);
    Global::gyroAngle = yaw;
    Global::gyroVelocity = GYROZ;
  }
}
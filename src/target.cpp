#include "clock.h"
#include "drive.h"
#include "variables.h"
#include "parser.h"


// cv::VideoWriter out("output1.mjpg", -1, 30., cv::Size(Global::FrameWidth,Global::FrameWidth));

void createTrackbars() {
  const std::string trackbarWindowName = "Trackbars";
  cv::namedWindow(trackbarWindowName, 0);
  cv::createTrackbar("R_MIN", trackbarWindowName, &Var::minR, 255, NULL);
  cv::createTrackbar("R_MAX", trackbarWindowName, &Var::maxR, 255, NULL);
  cv::createTrackbar("G_MIN", trackbarWindowName, &Var::minG, 255, NULL);
  cv::createTrackbar("G_MAX", trackbarWindowName, &Var::maxG, 255, NULL);
  cv::createTrackbar("B_MIN", trackbarWindowName, &Var::minB, 255, NULL);
  cv::createTrackbar("B_MAX", trackbarWindowName, &Var::maxB, 255, NULL);
}

void ThresholdImage(const cv::Mat& original, cv::Mat& thresholded) {
  cv::inRange(original, cv::Scalar(Var::minR, Var::minG, Var::minB), cv::Scalar(Var::maxR, Var::maxG, Var::maxB), thresholded);
}

void morphOps(cv::Mat& thresh) {
  cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(8, 8));
  // dilate(thresh,thresh,dilateElement);
  // dilate(thresh,thresh,dilateElement);
}

// int findTarget(const cv::Mat& original, const cv::Mat& thresholded, Target* targets) {
int findTarget(const cv::Mat& original, const cv::Mat& thresholded) {
  int targetsFound = 0;
  // Clock total, between;
  ClockTimer timer;
  bool printTime = false;
  std::vector<cv::Vec4i> hierarchy;
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Point2f> corners;
  if (Switches::printTime == 2) {
    printTime = true;
    timer.reset();
    printf("begin findTarget\n");
  }

  // findContours(thresholded, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
  // findContours(thresholded, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
  cv::findContours(thresholded, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  timer.printTime(printTime," finding Contours");

  // for (std::vector<std::vector<cv::Point> >::iterator it = contours.begin(); it != contours.end();) {
  //   if (it->size() < 100) { // min contour
  //     it++;
  //     // it = contours.erase(it);
  //     // timer.printTime(printTime," removing contour");
  //   } else
  //     it++;
  // }

  timer.printTime(printTime," filter:Perim");
  if (contours.size() > Var::maxTargets) {
    timer.printTime(printTime," many targets");
    return targetsFound;
  } else if (contours.size() < 1) {
    timer.printTime(printTime," few targets");
    return targetsFound;
  }
  timer.printTime(printTime," filter:Size");

  std::vector<cv::RotatedRect> minRect(contours.size());
  std::vector<std::vector<cv::Point> > hull(contours.size());
  std::vector<std::vector<cv::Point> > art;
  std::vector<cv::Point> approx;
  // Target targets[Var::maxTargets];
  cv::Mat workingImage(Global::FrameHeight, Global::FrameWidth, CV_8UC1, cv::Scalar(0));
  // cv::Mat workingImageSq; // useless with 2022
  // int num = -1; // useless with 2022

  int numTargetsToFind = 3;
  std::pair<int,double> largestIDs[numTargetsToFind];
  double hullAreas[3];
  for(int i = 0; i < numTargetsToFind; i++){
    largestIDs[i] = std::pair<int,double>(-1,-1.);
  }

  if (!contours.empty() && !hierarchy.empty()) {
    for (int i = 0; i < (int)contours.size(); i++) {
      if (printTime)
        printf("i: %d\n", i);
      // targets[i].NullTargets();
      Global::targets.push_back(Target());
      Global::targets.back().id = i;
      if (hierarchy[i][2] != -1) {
        continue;
      }
      minRect[i] = cv::minAreaRect(cv::Mat(contours[i]));
      cv::Point2f rect_points[4];
      minRect[i].points(rect_points);
      std::copy(rect_points, rect_points + 4, Global::targets.back().points);
      Global::targets.back().rect = minRect[i];
      Global::targets.back().boundingRect = minRect[i].boundingRect();
      timer.printTime(printTime, " findRect");

      bool flag = false;
      int bounding = 20;
      for (int k = 0; k < 4; k++) {
        if (abs(rect_points[k].x - Global::FrameWidth / 2) > (Global::FrameWidth / 2 - bounding) || abs(rect_points[k].y - Global::FrameHeight / 2) > (Global::FrameHeight / 2 - bounding)){
          flag = true;
        }
      }
      if (flag) {
        if (printTime)
          printf("  SKIP: edge too close\n");
        continue;
      }

      // for (int j = 0; j < 4; j++) {
      //   line(original, rect_points[j], rect_points[(j + 1) % 4], Global::BLUE, 1, 8);
      //   circle(original, rect_points[j], 3, Global::RED, -1, 8, 0);
      // }
      timer.printTime(printTime," drawRect");

      Global::targets.back().area = cv::contourArea(contours[i]);

      // Might be useless? maybe good for differentiating between lights? idk
      // =========
      // cv::convexHull(contours[i], hull[i]);
      // timer.printTime(printTime," convexHull");
      // double ratioTest = cv::contourArea(hull[i]) / cv::contourArea(contours[i]);
      // // if (ratioTest < 3.25) { // TODO: before was 4 // for 2021
      // if (abs(ratioTest - 1) > 0.1) { // TODO: before was 4
      //   if (printTime){
      //     printf("  SKIP: Area-Ratio: %.2f\n", ratioTest);
	    //     printf("  x:%d, y:%d\n",contours[i][0].x,contours[i][0].y);
	    //   }
	    //   continue;
      // }
      // timer.printTime(printTime," ratioTest");


      // Probably useless, good with complex targets;
      // ========
      // approxPolyDP(hull[i], approx, cv::arcLength(hull[i], true) * 0.005, true); // 0.015
      // timer.printTime(printTime," afterPoly");
      // art.push_back(approx);

      // num = i; // used to work in 2021 with one target
      targetsFound++;
      timer.printTime(printTime," passed");

    } //---end contour loop i

    // printf("sizes: ");
    // for(int i = 0; i < 3; i++){
    //   printf("%d:%f, ",largestIDs[i].first,largestIDs[i].second);
    // }
    // printf("\n");

    if (printTime)
      printf("end: %d found\n",targetsFound);
    // for (unsigned int j = 0; j < art.size(); j++) {
    //   // drawContours(workingImage, art, j, cv::Scalar(255));
    //   // drawContours(original, art, j, Global::BLUE, 2);
    // }
    if(targetsFound >= numTargetsToFind){

      std::sort(Global::targets.begin(), Global::targets.end(), 
      [ ](const Target& lhs, const Target& rhs){
        return lhs.area > rhs.area;
      });

      Global::targets.erase(Global::targets.begin()+3,Global::targets.end());

      std::sort(Global::targets.begin(), Global::targets.end(), 
      [ ](const Target& lhs, const Target& rhs){
        return lhs.points[0].x < rhs.points[0].x;
      });

      // printf("largest1: \n");
      // for(int i = 0; i < Global::targets.size(); i++){
      //   printf("[area:%7.2f, x:%6.2f, y:%6.2f], \n",Global::targets[i].area,Global::targets[i].points[0].x,Global::targets[i].points[0].y);
      // }
      // printf("end\n");
      return 3;
    } else {
      return -1;
    }

    // imshow("workingImg",original);
    // cv::waitKey(0);



    // Useful for a single target, here it's many rectangles which the previous algorithm already finds
    // ====================
    // if (num != -1)
    //   workingImage(targets[num].boundingRect).copyTo(workingImageSq);
    // timer.printTime(printTime," drawMat");
    // if (targetsFound == 1) {
    //   cv::goodFeaturesToTrack(workingImageSq, corners, Var::maxCorners, Var::qualityLevel, Var::minDistance, cv::Mat(), Var::blockSize, Var::useHarisDetector, Var::k);
    //   for (unsigned int i = 0; i < corners.size(); i++) {
    //     corners[i].x = corners[i].x + targets[num].boundingRect.x;
    //     corners[i].y = corners[i].y + targets[num].boundingRect.y;
    //   }
    //   timer.printTime(printTime," goodFeaturesTrack");
    //   // OffSetX = 320-(corners[0].x+corners[1].x+corners[2].x+corners[3].x)/4.;
    //   Global::target.corners.clear();
    //   Global::target.corners.push_back(corners[0]);
    //   Global::target.corners.push_back(corners[1]);
    //   Global::target.corners.push_back(corners[2]);
    //   Global::target.corners.push_back(corners[3]);
    // }

  }

  // 2019
  // if(targetsFound==2){
  // line(original,tLeft->center, tRight->center, YELLOW, 1);
  // line(original,tLeft->center, tLeft->center, RED, 3);
  // line(original,tRight->center, tRight->center, RED, 3);
  // }
  if (printTime)
    timer.PTotal();


  return targetsFound;
}

int currentLog = 0;
int fourcc = cv::VideoWriter::fourcc('M','J','P','G');
int prevTime = 30;

void startSaving(cv::VideoWriter& out){
  std::string name = "./output";
  name += std::to_string(currentLog);
  name += ".avi";
  currentLog++;
  out.open(name,fourcc,30.,cv::Size(Var::WIDTH,Var::HEIGHT));
}

int main(int argc, const char* argv[]) {
  {
    Parser p(argc,argv);
    double printTime_d, cameraInput_d;
    double colLims[2][3];
    p.add_Parameter("-o" ,"--orig",Switches::SHOWORIG,false,"displays original camera input w/ lines");
    p.add_Parameter("-hu","--hue",Switches::SHOWHUE,false,"displays HSV of original image w/o lines");
    p.add_Parameter("-th","--threshold",Switches::SHOWTHRESH,false,"displays thresholded image (black & white)");
    p.add_Parameter("-tr","--track",Switches::SHOWTRACK,false,"displays sliders for RGB (or HSV depending on code)");
    p.add_Parameter("-s" ,"--server",Switches::USESERVER,false,"use server for reading image (B&W only. see below)");
    p.add_Parameter("-c" ,"--color",Switches::USECOLOR,false,"use color for the server");
    p.add_Parameter("-p" ,"--print",Switches::DOPRINT,false,"prints basic data");
    p.add_Parameter("-f" ,"--frame",Switches::FRAME,true,"prints of frames found");
    p.add_Parameter("-sv","--save",Switches::SAVE,false,"saves output to .avi file every 30s");
    p.add_Parameter("-pt","--ptime",printTime_d,0,"(1-2) prints time taken for each loop");
    p.add_Parameter("-P","--P",Switches::InitPID[0],0.0,"(0.0-1.0) Proportional value of PID");
    p.add_Parameter("-I","--I",Switches::InitPID[1],0.0,"(0.0-1.0) Integral     value of PID");
    p.add_Parameter("-D","--D",Switches::InitPID[2],0.0,"(0.0-1.0) Derivative   value of PID");
    p.add_Parameter("-cam","--camera",cameraInput_d,0,"(0-2) which camera port to use");
    p.add_Parameter("-nR","--minRed",colLims[0][0],Var::minR,"(0-255) lower end of thresh of Red or Hue");
    p.add_Parameter("-xR","--maxRed",colLims[1][0],Var::maxR,"(0-255) upper end of thresh of Red or Hue");
    p.add_Parameter("-nG","--minGreen",colLims[0][1],Var::minG,"(0-255) low ^ ^ of Green or Saturation");
    p.add_Parameter("-xG","--maxGreen",colLims[1][1],Var::maxG,"(0-255) up  ^ ^ of Green or Saturation");
    p.add_Parameter("-nB","--minBlue",colLims[0][2],Var::minB,"(0-255) low ^ ^ of Blue or Value");
    p.add_Parameter("-xB","--maxBlue",colLims[1][2],Var::maxB,"(0-255) up  ^ ^ of Blue or Value");

    p.add_Parameter("-fx","--fx",Var::fx,Var::fx,"---");
    p.add_Parameter("-fy","--fy",Var::fy,Var::fy,"---");
    p.add_Parameter("-cx","--cx",Var::cx,Var::cx,"---");
    p.add_Parameter("-cy","--cy",Var::cy,Var::cy,"---");


    p.add_Parameter("-d1","--d1",Var::dist_cof[0],1.,"---");
    p.add_Parameter("-d2","--d2",Var::dist_cof[1],1.,"---");
    p.add_Parameter("-d3","--d3",Var::dist_cof[2],1.,"---");
    p.add_Parameter("-d4","--d4",Var::dist_cof[3],1.,"---");
    p.add_Parameter("-d5","--d5",Var::dist_cof[4],1.,"---");
    printf("dist: %f\n",Var::dist_cof[0]);
    Var::dist_cof[0] *=  0.05106937569;
    Var::dist_cof[1] *= -0.0761728305;
    Var::dist_cof[2] *= -0.0002898593;
    Var::dist_cof[3] *= -0.0252227088;
    Var::dist_cof[4] *=  0.05262168077;


    if(p.checkParams(true))
      return 0;
    Switches::cameraInput = std::round(cameraInput_d);
    Switches::printTime = std::round(printTime_d);
    Var::minR = std::round(colLims[0][0]);
    Var::minG = std::round(colLims[0][1]);
    Var::minB = std::round(colLims[0][2]);
    Var::maxR = std::round(colLims[1][0]);
    Var::maxG = std::round(colLims[1][1]);
    Var::maxB = std::round(colLims[1][2]);
    printf("cam: %d | pt: %d\n",Switches::cameraInput, Switches::printTime);
  }
  cv::VideoWriter out;
  if (Switches::SAVE)
    startSaving(out);
    
  ClockTimer timer;
  Clock serverClock;
  Clock switchFrame;
  Clock savingClock;

  int aaa = 0;
  bool printTime = false;
  // bool firstTime = true;
  int frameCounter = 0, frameCounter2 = 0, frameCounterPrev = 0, missedFrames = 0;

  if (Switches::printTime == 1)
    printTime = true;
  timer.printTime(printTime,"getting input");

  // if(Switches::USESERVER && Global::videoSocket == 0){
  //   Switches::USESERVER = false;
  // }
  

  // start i2c connection:
  // int addr = 0x04;
  // int file_i2c;
  // char* filename = (char*)"/dev/i2c-1";
  // if ((file_i2c = open(filename, O_RDWR)) < 0) {
  //     printf("Failed to open the i2c bus\n");
  //     return false;
  // }
  // if (ioctl(file_i2c, I2C_SLAVE, addr) < 0) {
  //     printf("Failed to acquire bus access and/or talk to slave.\n");
  //     return false;
  // }

  cv::Mat img, HSV, gray, thresholded;
  Global::gyroAngle = 0;
  Global::driveAngle = 0;
  // int videoPort=4097;
  Position position, positionAV;
  std::vector<Position>::iterator it;
  std::vector<Position> posA;
  // Target targets[Var::maxTargets];
  // Init Threads--------------------

#ifdef RASPI
  // startThread("USB", NULL);
  // I don't remember if this is needed so I'll keep this for later script.sh
  // stty -F /dev/ttyUSB0 115200
  // stty -F /dev/ttyUSB0 -hupcl
#endif
  startThread("VIDEO", NULL);
  //int rc = pthread_create(&VideoCap_t, NULL, VideoCap, NULL);
  //rc = pthread_setname_np(VideoCap_t, "MJPEG Thread");
  
  startThread("TCP", &positionAV);
  


  // startThread("DRIVE", &positionAV);

  // startThread("PID", NULL);

  if (Switches::USESERVER) {
    startThread("SERVER", NULL);
  }

  // End Init Threads-----------------------------
  if (Switches::SHOWTRACK)
    createTrackbars();
  if (!img.isContinuous())
    img = img.clone();
  position.nullifyStruct();
  positionAV.nullifyStruct();
  timer.printTime(printTime,"Init Threads");

  Global::FrameHeight = 480;
  Global::FrameWidth = 640;
  initSolvePnP();

  serverClock.restart();

  // cv::VideoCapture vcap;
  // while (!vcap.open(Switches::cameraInput)) {
  //   std::cout << "cant connect" << std::endl;
  //   usleep(10000000);
  // }
  // printf("  setting brightness\n");
  // vcap.set(cv::CAP_PROP_BRIGHTNESS, 100);
  // printf("  setting auto exposure\n");
  // vcap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
  // printf("  setting exposure\n");
  // vcap.set(cv::CAP_PROP_EXPOSURE, Var::EXPOSURE);
  // usleep(1000);

  
  while (true) {
    timer.reset();
    //printf("locking\n");
    pthread_mutex_lock(&Global::frameMutex);
    Global::frame = cv::imread("2022/BG1.jpg");
    // if (Switches::cameraInput == 2) {
    //   // int num = (int(switchFrame.getTimeAsSecs() / 5.)) % 11 + 1;
    //   int num = 1;
    //   if (num != aaa) {
    //     std::string imgText = "2022/BG";
    //     imgText.append(std::to_string(num));
    //     imgText.append(".jpg");
    //     aaa = num;
    //     printf("%s\n", imgText.c_str());
    //     Global::frame = cv::imread(imgText);
    //   }
    // }
    if(true){
    // if (vcap.read(Global::frame)) { // check for empty frame
    // if (!Global::frame.empty() && Global::newFrame) { // check for empty frame
      Global::frame.copyTo(img);
      pthread_mutex_unlock(&Global::frameMutex);
      timer.printTime(printTime,"Get Frame");
      frameCounter++;

      // cv::cvtColor(img, HSV, CV_BGR2HSV);
      // timer.printTime(printTime," to HSV");
      // thresholded = ThresholdImage(HSV); // switch between HSV or RGB, see what works

      //cv::cvtColor(img,gray,CV_BGR2GRAY);
      //timer.printTime(printTime," to gray");
      
      // TODO: lower resolution of thresholding

      



      ThresholdImage(img,thresholded);
      timer.printTime(printTime," thresholded");

      // morphOps(thresholded);
      // timer.printTime(printTime," apply morphs");

      int targetsFound = findTarget(img, thresholded); // FIND THE TARGETS
      timer.printTime(printTime," findTarget");

      //if (targetsFound != 1)
        printf("targetsFound: %d\n", targetsFound);

      // if (targetsFound == 1) { // TARGET HAS BEEN FOUND----============---------------------==========-------------
      if (targetsFound >= 3) {
        // if (firstTime) {
        //   initSolvePnP(img);
        //   firstTime = false;
        // }
        findAnglePnP(img, position); // SOLVE FOR POSITION AND ROTATION
        timer.printTime(printTime," solvePnP");
        posA.push_back(position);
        if (posA.size() > Var::avSize)
          posA.erase(posA.begin());
        positionAV.nullifyStruct();

        // avaraging--------
        int cntr = 0;
        for (it = posA.end() - 3; it != posA.end(); it++) {
          cntr++;
          positionAV.x += (*it).x;
          positionAV.z += (*it).z;
          positionAV.alpha1 += (*it).alpha1;
          positionAV.alpha2 += (*it).alpha2;
          positionAV.dist += (*it).dist;
          positionAV.OffSetx += (*it).OffSetx;
        }
        for (it = posA.begin(); it != posA.end(); it++) {
          positionAV.alpha2 += (*it).alpha2;
        }
        positionAV.x /= cntr;
        positionAV.z /= cntr;
        positionAV.alpha1 /= cntr;
        positionAV.alpha2 /= posA.size();
        positionAV.dist /= cntr;
        positionAV.OffSetx /= cntr;
        //------------------
        timer.printTime(printTime," avaraging");

        if (Switches::DOPRINT) {
          printf("x=%6.2f, z=%6.2f, dist=%6.2f, alpha1=%6.2f, alpha2=%6.2f, speed=%4.2f, "
                 "turn=%5.2f, gyro=%7.2f, dataValid: %d\n",
                 positionAV.x,
                 positionAV.z,
                 positionAV.dist,
                 positionAV.alpha1,
                 positionAV.alpha2,
                 positionAV.speed,
                 positionAV.turn,
                 positionAV.gyro,
                 positionAV.dataValid);
          // printf("P=%2.3f I=%2.3f D=%2.3f\n",Global::P, Global::I, Global::D);
        }
      } else {
        missedFrames++;
      }

      // finished calculating
      if (Switches::SHOWORIG)
        imshow("Original", img);
      if (Switches::SHOWTHRESH)
        imshow("Thresholded", thresholded);
      if (Switches::SHOWHUE)
        imshow("HSV", HSV);
      if(Switches::SHOWORIG || Switches::SHOWTHRESH || Switches::SHOWHUE)
        timer.printTime(printTime," finished imshow");

      if (Switches::SAVE){
        out.write(img);
        if(savingClock.getTimeAsSecs() >= 30.){
          printf("---SAVING---\n");
          savingClock.restart();
          out.release();
          startSaving(out);
        } else {
          int time =  int(30.-savingClock.getTimeAsSecs());
          if(time != prevTime){
            printf("time till next save: %d\n",time);
            prevTime = time;
          }
        }
        timer.printTime(printTime," fin saving vid");
      }

      if (printTime) {
        timer.printTime("End");
        printf("\n");
      }
      if (Switches::SHOWORIG || Switches::SHOWHUE || Switches::SHOWTHRESH || Switches::SHOWTRACK) {
        cv::waitKey(5);
      }
      Global::newFrame = false;
    } // end check for new frame
    else
      pthread_mutex_unlock(&Global::frameMutex);


    frameCounter2++;
    if (frameCounter % 10 == 0 && frameCounter != frameCounterPrev) {
      frameCounterPrev = frameCounter;
      double dt = serverClock.getTimeAsSecs();
      if(Switches::FRAME){
        printf("------ Frame rate: %f fr/s (%f) \n", 10. / dt, frameCounter2 / dt);
        printf("------ Miss Frame: %d fr \n", missedFrames);
      }
      serverClock.restart();
      frameCounter2 = 0;
      missedFrames = 0;

      if (Switches::USESERVER && Global::videoSocket != 0 && !Global::videoError) {
        int bytes = 0;
        if (Switches::USECOLOR) {
          int imgSize = img.total() * img.elemSize();
          if (!img.isContinuous())
            img = img.clone();
          if ((bytes = send(Global::videoSocket, img.data, imgSize, MSG_NOSIGNAL)) < 0) {
            Global::videoError = true;
            printf("video error\n");
          }
        } else {
          int imgSize = thresholded.total() * thresholded.elemSize();
          // if (!thresholded.isContinuous())
          //   thresholded = thresholded.clone();
          if ((bytes = send(Global::videoSocket, thresholded.data, imgSize, MSG_NOSIGNAL)) < 0) {
            Global::videoError = true;
            printf("video error\n");
          }
        }
      }
    }
    usleep(1000);
  }
}

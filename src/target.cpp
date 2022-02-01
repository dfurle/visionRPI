#include "clock.h"
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
int findTarget(const cv::Mat& img, const cv::Mat& thresholded) {
  Global::targets.clear();
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
  std::vector<std::vector<cv::Point> > art;
  cv::Mat workingImage(Global::FrameHeight, Global::FrameWidth, CV_8UC1, cv::Scalar(0));

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

      for (int j = 0; j < 4; j++) {
        line(img, rect_points[j], rect_points[(j + 1) % 4], Global::BLUE, 1, 8);
        //circle(img, rect_points[j], 3, Global::RED, -1, 8, 0);
      }
      timer.printTime(printTime," drawRect");

      Global::targets.back().area = cv::contourArea(contours[i]);

      // Might be useless? maybe good for differentiating between lights? idk
      // =========
      // // std::vector<std::vector<cv::Point> > hull(contours.size());
      // <std::vector<cv::Point> hull;
      // cv::convexHull(contours[i], hull);
      // timer.printTime(printTime," convexHull");
      // double ratioTest = cv::contourArea(hull) / cv::contourArea(contours[i]);
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
      // std::vector<cv::Point> approx;
      // approxPolyDP(hull[i], approx, cv::arcLength(hull[i], true) * 0.005, true); // 0.015
      // timer.printTime(printTime," afterPoly");
      // art.push_back(approx);

      targetsFound++;
      timer.printTime(printTime," passed");

    } //---end contour loop i

    if (printTime)
      printf("end: %d found\n",targetsFound);

    // for (unsigned int j = 0; j < art.size(); j++) {
    //   // drawContours(workingImage, art, j, cv::Scalar(255));
    //   // drawContours(original, art, j, Global::BLUE, 2);
    // }
    if(targetsFound >= 3){
      std::sort(Global::targets.begin(), Global::targets.end(), 
      [ ](const Target& lhs, const Target& rhs){
        return lhs.area > rhs.area;
      });
      Global::targets.erase(Global::targets.begin()+3,Global::targets.end());
      std::sort(Global::targets.begin(), Global::targets.end(), 
      [ ](const Target& lhs, const Target& rhs){
        return lhs.points[0].x < rhs.points[0].x;
      });
      return 3;
    } else {
      return -1;
    }


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

  if (printTime)
    timer.PTotal();


  return targetsFound;
}

int main(int argc, const char* argv[]) {
  {
    Parser p(argc,argv);
    double printTime_d, cameraInput_d;
    double colLims[2][3];
    p.add_Parameter("-o" ,"--orig",Switches::SHOWORIG,false,"displays original camera input w/ lines");
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
    
  ClockTimer timer;
  Clock serverClock;

  bool printTime = false;
  int frameCounter = 0, frameCounter2 = 0, frameCounterPrev = 0, missedFrames = 0;

  std::cout << CV_VERSION << std::endl;

  if (Switches::printTime == 1)
    printTime = true;
  timer.printTime(printTime,"getting input");

  /* ---=== Start I2C Connection ===--- */
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

  cv::Mat img, thresholded;
  std::vector<Position> posA;

#ifdef RASPI
  // startThread("USB", NULL);
  // I don't remember if this is needed so I'll keep this for later script.sh
  // stty -F /dev/ttyUSB0 115200
  // stty -F /dev/ttyUSB0 -hupcl
#endif

  startThread("VIDEO");
  startThread("TCP");
  if (Switches::SAVE)
    startThread("SAVE",&img);
  if (Switches::USESERVER)
    startThread("SERVER");


  if (Switches::SHOWTRACK)
    createTrackbars();
  if (!img.isContinuous())
    img = img.clone();
  Global::position.nullifyStruct();
  Global::positionAV.nullifyStruct();
  timer.printTime(printTime,"Init Threads");

  // might be best to wait until camera is up?
  initSolvePnP();

  serverClock.restart();

  while (true) {
    timer.reset();
    Global::muteFrame.lock();
    if (!Global::frame.empty() && (Global::newFrame || Switches::cameraInput == 2)) {
      Global::muteImg.lock();
      Global::frame.copyTo(img);
      Global::muteFrame.unlock();
      timer.printTime(printTime,"Get Frame");
      frameCounter++;

      // TODO: lower resolution of thresholding

      ThresholdImage(img,thresholded);
      timer.printTime(printTime," thresholded");

      int targetsFound = findTarget(img, thresholded); // FIND THE TARGETS
      timer.printTime(printTime," findTarget");

      if (targetsFound < 3)
        printf("  targetsFound: %d\n", targetsFound);

      if (targetsFound >= 3) {
        /* ---=== Getting Position and Rotation ===--- */
        findAnglePnP(img);
        timer.printTime(printTime," solvePnP");
        posA.push_back(Global::position);
        if (posA.size() > Var::avSize)
          posA.erase(posA.begin());
        Global::positionAV.nullifyStruct();

        /* ---=== Averaging ===--- */
        int cntr = 0;
        for (auto it = posA.begin(); it != posA.end(); it++) {
          cntr++;
          Global::positionAV.dist += (*it).dist;
          Global::positionAV.robotAngle += (*it).robotAngle;
        }
        Global::positionAV.dist /= cntr;
        Global::positionAV.robotAngle /= cntr;
        timer.printTime(printTime," avaraging");

        if (Switches::DOPRINT) {
          printf("dist=%6.2f, robotAngle=%6.2f, dataValid: %d\n",
                 Global::positionAV.dist,
                 Global::positionAV.robotAngle,
                 Global::dataValid);
        }
      } else {
        missedFrames++;
      }

      /* ---=== Finished with frame, output it if needed ===--- */
      if (Switches::SHOWORIG)
        imshow("Original", img);
      if (Switches::SHOWTHRESH)
        imshow("Thresholded", thresholded);

      if (Switches::SHOWORIG || Switches::SHOWTHRESH || Switches::SHOWTRACK) {
        cv::waitKey(5);
      }
      if(Switches::SHOWORIG || Switches::SHOWTHRESH)
        timer.printTime(printTime," finished imshow");

      if (printTime) {
        timer.printTime("End");
        printf("\n");
      }
      Global::newFrame = false;
      // Global::dataValid = 1;
      Global::muteImg.unlock();
    } // end check for new frame
    else
      Global::muteFrame.unlock();

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

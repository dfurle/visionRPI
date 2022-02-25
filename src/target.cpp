#include "clock.h"
#include "variables.h"
#include "parser.h"


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

int findTarget(cv::Mat& img, cv::Mat& thresholded) {
  Global::targets.clear();
  int targetsFound = 0;
  // Clock total, between;
  ClockTimer timer;
  bool printTime = false;
  std::vector<cv::Vec4i> hierarchy;
  std::vector<std::vector<cv::Point> > contours;
  if (Switches::printTime == 2) {
    printTime = true;
    timer.reset();
    printf("begin findTarget\n");
  }

  // findContours(thresholded, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
  // findContours(thresholded, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
  cv::findContours(thresholded, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  timer.printTime(printTime," finding Contours");

  for (auto it = contours.begin(); it != contours.end();) {
    // printf("size: %f\n",cv::contourArea(*it));
    if (cv::contourArea(*it) < 100) { // min contour
      it = contours.erase(it);
      timer.printTime(printTime," removing contour");
    } else
      it++;
  }

  timer.printTime(printTime," filter:Perim");
  if (contours.size() > 50) {
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
      // std::copy(rect_points, rect_points + 4, Global::targets.back().points);
      Global::targets.back().rect = minRect[i];
      cv::Rect mRect = minRect[i].boundingRect();
      mRect.x -= 3;
      mRect.y -= 3;
      mRect.height += 6;
      mRect.width += 6;
      Global::targets.back().boundingRect = mRect;
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
        Global::targets.erase(Global::targets.end()-1);
        continue;
      }

      /*
      if(Switches::DRAW){
        for (int j = 0; j < 4; j++) {
          cv::line(img, rect_points[j], rect_points[(j + 1) % 4], Global::BLUE, 1, 8);
        }
        //circle(img, rect_points[j], 3, Global::RED, -1, 8, 0);
      }
      */
      timer.printTime(printTime," drawRect");

      Global::targets.back().area = cv::contourArea(contours[i]);

      // Might be useless? maybe good for differentiating between lights? idk
      // =========
      /*
      std::vector<cv::Point> hull;
      cv::convexHull(contours[i], hull);
      timer.printTime(printTime," convexHull");
      double ratioTest = cv::contourArea(hull) / cv::contourArea(contours[i]);
      //if (ratioTest < 3.25) { // TODO: before was 4 // for 2021
      if (abs(ratioTest - 1) > 0.1) { // TODO: before was 4
        if (printTime){
          printf("  SKIP: Area-Ratio: %.2f\n", ratioTest);
	        printf("  x:%d, y:%d\n",contours[i][0].x,contours[i][0].y);
	      }
	      continue;
      }
      printf("hull: %d\n",hull.size());
      for(cv::Point p : hull){
        std::cout << p << std::endl;
        cv::circle(img, p, 2, Global::RED, -1, 8, 0);
      }
      timer.printTime(printTime," ratioTest");
      */
      
      // Probably useless, good with complex targets;
      // ========
      /*
      std::vector<cv::Point> approx;
      //cv::approxPolyDP(hull, approx, cv::arcLength(hull, true) * 0.005, true); // 0.015
      //cv::approxPolyDP(hull, approx, 3, true);
      cv::approxPolyDP(contours[i], approx, 5, true);
      timer.printTime(printTime," afterPoly");
      //printf("approx: %d\n",approx.size());
      //for(cv::Point p : approx){
        //cv::circle(img, p, 2, Global::RED, -1, 8, 0);
        //std::cout << p << std::endl;
      //}
      art.push_back(approx);
      //
      if(approx.size() == 4){
        for(int i = 0; i < 4; i++){
          Global::targets.back().points[i] = approx[i];
        }
      }
      */

      targetsFound++;
      timer.printTime(printTime," passed");

    } //---end contour loop i

    if (printTime)
      printf("end: %d found\n",targetsFound);

    /*
    if(Switches::DRAW){
      for (int j = 0; j < 3; j++) {
        drawContours(img, art, j, Global::BLUE, 2);
      }
      //circle(img, rect_points[j], 3, Global::RED, -1, 8, 0);
    }
    */

    /*
    for (unsigned int j = 0; j < art.size(); j++) {
      drawContours(img, art, j, cv::Scalar(255));
      //drawContours(img, art, j, Global::BLUE, 2);
    }
    */


    // Useful for a single target, here it's many rectangles which the previous algorithm already finds
    // ====================
    ///*
    double qualityLevel     = 0.05;
    double minDistance      = 5;
    int    blockSize        = 3;
    bool   useHarisDetector = true;
    double k                = 0.04;
    int    maxCorners       = 4;
    cv::Mat workingImageSq[3];
    thresholded(Global::targets[0].boundingRect).copyTo(workingImageSq[0]);
    thresholded(Global::targets[1].boundingRect).copyTo(workingImageSq[1]);
    thresholded(Global::targets[2].boundingRect).copyTo(workingImageSq[2]);

    timer.printTime(printTime," drawMat");
    if (targetsFound == 3) {
      for(int mi = 0; mi < 3; mi++){
        std::vector<cv::Point> corners;
        cv::goodFeaturesToTrack(workingImageSq[mi], corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, useHarisDetector, k);
        // printf("corners: %d\n",corners.size());
        if(corners.size() == 4){
          for(int i = 0; i < 4; i++){
            // std::cout << corners[i] << std::endl;
            corners[i].x = corners[i].x + Global::targets[mi].boundingRect.x;
            corners[i].y = corners[i].y + Global::targets[mi].boundingRect.y;
            Global::targets[mi].points[i] = corners[i];
            cv::circle(img, corners[i], 2, Global::RED, -1, 8, 0);
          }
        }
      }
      timer.printTime(printTime," goodFeaturesTrack");
    }
    //*/
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

  }

  if (printTime)
    timer.PTotal();


  return targetsFound;
}


void readParameters(){
  std::ifstream file;
  file.open("../settings.txt", std::ios::in);
  if(file){
    std::string line;
    while(std::getline(file,line)){
      int split = line.find(' ');
      std::string param = line.substr(0,split);
      std::string value = line.substr(split+1);
      std::cout << param << std::endl;
      std::cout << value << std::endl;
             if(str::cmp(param,"minR")){
        Var::minR = std::stoi(value);
      } else if(str::cmp(param,"maxR")){
        Var::maxR = std::stoi(value);
      } else if(str::cmp(param,"minG")){
        Var::minG = std::stoi(value);
      } else if(str::cmp(param,"maxG")){
        Var::maxG = std::stoi(value);
      } else if(str::cmp(param,"minB")){
        Var::minB = std::stoi(value);
      } else if(str::cmp(param,"maxB")){
        Var::maxB = std::stoi(value);
      }
    }
  }
  file.close();
}

void writeParameters(){
  std::ofstream file;
  file.open("../settings.txt", std::ios::out);
  if(file){
    file << "minR " + std::to_string(Var::minR) + "\n";
    file << "maxR " + std::to_string(Var::maxR) + "\n";
    file << "minG " + std::to_string(Var::minG) + "\n";
    file << "maxG " + std::to_string(Var::maxG) + "\n";
    file << "minB " + std::to_string(Var::minB) + "\n";
    file << "maxB " + std::to_string(Var::maxB) + "\n";
  }
  file.close();
}

int main(int argc, const char* argv[]) {
  readParameters();
  {
    Parser p(argc,argv);
    double printTime_d, cameraInput_d;
    p.add_Parameter("-o" ,"--orig",Switches::SHOWORIG,false,"displays original camera input w/ lines");
    p.add_Parameter("-th","--threshold",Switches::SHOWTHRESH,false,"displays thresholded image (black & white)");
    p.add_Parameter("-tr","--track",Switches::SHOWTRACK,false,"displays sliders for RGB");
    p.add_Parameter("-http" ,"--http",Switches::USEHTTP,false,"use http server for streaming video");
    p.add_Parameter("-p" ,"--print",Switches::DOPRINT,false,"prints basic data");
    p.add_Parameter("-f" ,"--frame",Switches::FRAME,true,"prints of frames found");
    p.add_Parameter("-d","--draw",Switches::DRAW,true,"draws the lines on original img file");
    p.add_Parameter("-pt","--ptime",printTime_d,0,"(1-2) prints time taken for each loop");
    p.add_Parameter("-P","--P",Switches::InitPID[0],0.0,"(0.0-1.0) Proportional value of PID");
    p.add_Parameter("-I","--I",Switches::InitPID[1],0.0,"(0.0-1.0) Integral     value of PID");
    p.add_Parameter("-D","--D",Switches::InitPID[2],0.0,"(0.0-1.0) Derivative   value of PID");
    p.add_Parameter("-cam","--camera",cameraInput_d,0,"(0-2) which camera port to use");

    Var::dist_cof[0] =  0.05106937569;
    Var::dist_cof[1] = -0.0761728305;
    Var::dist_cof[2] = -0.0002898593;
    Var::dist_cof[3] = -0.0252227088;
    Var::dist_cof[4] =  0.05262168077;

    if(p.checkParams(true))
      return 0;
    Switches::cameraInput = std::round(cameraInput_d);
    Switches::printTime = std::round(printTime_d);
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
  cv::Mat rPos(cv::Size(500,1000),CV_8UC3);
  std::vector<Position> posA;
  int checkSum;

#ifdef RASPI
  // startThread("USB", NULL);
  // I don't remember if this is needed so I'll keep this for later script.sh
  // stty -F /dev/ttyUSB0 115200
  // stty -F /dev/ttyUSB0 -hupcl
#endif

  startThread("VIDEO");
  startThread("TCP");
  if(Switches::USEHTTP)
    startThread("HTTP");
  if (Switches::SAVE)
    startThread("SAVE",NULL);
  // if (Switches::USESERVER)
  //   startThread("SERVER");


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
    if (!Global::frame.empty() && Global::newFrame) {
      timer.printTime(printTime,"Get Frame");
      Global::frame.copyTo(img);
      Global::muteFrame.unlock();
      timer.printTime(printTime,"Copy Frame");
      frameCounter++;

      // TODO: lower resolution of thresholding, but do multiple times
      // cv::resize(img,scaled,cv::Size(640/2,480/2),INTER_LINEAR);

      ThresholdImage(img,thresholded);
      timer.printTime(printTime," thresholded");

      int targetsFound = findTarget(img,thresholded); // FIND THE TARGETS
      timer.printTime(printTime," findTarget");

      if (targetsFound < 3)
        printf("  targetsFound: %d\n", targetsFound);

      if (targetsFound >= 3) {
        /* ---=== Getting Position and Rotation ===--- */
        findAnglePnP(img,rPos);
        timer.printTime(printTime," solvePnP");
        posA.push_back(Global::position);
        if (posA.size() > 10)
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

      if(Switches::USEHTTP){
        Global::muteImg.lock();
        if(Global::httpStatus == 0){
          img.copyTo(Global::imgC);
          thresholded.copyTo(Global::thresholdedC);
          rPos.copyTo(Global::rPosC);
        }
        Global::muteImg.unlock();
      }
      timer.printTime(printTime," copy to glob");

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
        int checkSumLoc = 0;
        checkSumLoc += Var::minR + Var::maxR;
        checkSumLoc += Var::minG + Var::maxG;
        checkSumLoc += Var::minB + Var::maxB;
        if(checkSum != checkSumLoc){
          writeParameters();
          checkSum = 0;
          checkSum += Var::minR + Var::maxR;
          checkSum += Var::minG + Var::maxG;
          checkSum += Var::minB + Var::maxB;
        }
      }
      serverClock.restart();
      frameCounter2 = 0;
      missedFrames = 0;
    }
    usleep(1000);
  }
}

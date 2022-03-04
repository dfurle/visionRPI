#include "clock.h"
#include "variables.h"
#include "parser.h"

void ThresholdImage(const cv::Mat& original, cv::Mat& thresholded) {
  cv::inRange(original, cv::Scalar(Var::minR, Var::minG, Var::minB), cv::Scalar(Var::maxR, Var::maxG, Var::maxB), thresholded);
}

void morphOps(cv::Mat& thresh) {
  cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(8, 8));
  cv::erode(thresh,thresh,erodeElement);
  cv::dilate(thresh,thresh,dilateElement);
}

int findTarget(cv::Mat& img, cv::Mat& thresholded) {
  Global::targets.clear();
  int targetsFound = 0;
  // Clock total, between;
  #ifndef OPTIMIZE
  ClockTimer timer;
  bool printTime = false;
  #endif
  std::vector<cv::Vec4i> hierarchy;
  std::vector<std::vector<cv::Point> > contours;
  #ifndef OPTIMIZE
  if (Switches::printTime == 2) {
    printTime = true;
    timer.reset();
    printf("begin findTarget\n");
  }
  #endif

  // findContours(thresholded, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
  // findContours(thresholded, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
  cv::findContours(thresholded, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  #ifndef OPTIMIZE
  timer.printTime(printTime," finding Contours");
  #endif

  for (auto it = contours.begin(); it != contours.end();) {
    //printf("size: %f\n",cv::contourArea(*it));
    if (cv::contourArea(*it) < 45) { // min contour
      it = contours.erase(it);
      #ifndef OPTIMIZE
      timer.printTime(printTime," removing contour");
      #endif
    } else
      it++;
  }

  #ifndef OPTIMIZE
  timer.printTime(printTime," filter:Perim");
  #endif
  if (contours.size() > 50) {
    #ifndef OPTIMIZE
    timer.printTime(printTime," many targets");
    #endif
    return targetsFound;
  } else if (contours.size() < 1) {
    #ifndef OPTIMIZE
    timer.printTime(printTime," few targets");
    #endif
    return targetsFound;
  }
  #ifndef OPTIMIZE
  timer.printTime(printTime," filter:Size");
  #endif

  std::vector<cv::RotatedRect> minRect(contours.size());
  std::vector<std::vector<cv::Point> > art;
  cv::Mat workingImage(Global::FrameHeight, Global::FrameWidth, CV_8UC1, cv::Scalar(0));

  if (!contours.empty() && !hierarchy.empty()) {
    for (int i = 0; i < (int)contours.size(); i++) {
      #ifndef OPTIMIZE
      if (printTime)
        printf("i: %d\n", i);
      #endif
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
      // mRect.x -= 3;
      // mRect.y -= 3;
      // mRect.height += 6;
      // mRect.width += 6;

      std::vector<cv::Point2f> rp;
      rp.push_back(rect_points[0]);
      rp.push_back(rect_points[1]);
      rp.push_back(rect_points[2]);
      rp.push_back(rect_points[3]);

      cv::Point2f rCenter;
      for(int i = 0; i < 4; i++){
        rCenter.x += rp[i].x;
        rCenter.y += rp[i].y;
      }
      rCenter.x/=4.;
      rCenter.y/=4.;

      
      // printf("-----BEGIN-------- %d\n",i);
      // printf(" no sort\n");
      // for(int i = 0; i < 4; i++)
      //   printf("id: %d : %.1f,%.1f\n",i,rp[i].x,rp[i].y);

      std::sort(rp.begin(), rp.end(), 
      [ ](const cv::Point2f& lhs, const cv::Point2f& rhs){
        return lhs.x < rhs.x;
      });
      // printf(" x sort\n");
      // for(int i = 0; i < 4; i++)
      //   printf("id: %d : %.1f,%.1f\n",i,rp[i].x,rp[i].y);

      std::sort(rp.begin(), rp.begin()+2, 
      [ rCenter ](const cv::Point2f& lhs, const cv::Point2f& rhs){
        return abs(lhs.x - rCenter.x) < abs(rhs.x - rCenter.x);
      });
      // printf(" y-1 sort\n");
      // for(int i = 0; i < 4; i++)
      //   printf("id: %d : %.1f,%.1f\n",i,rp[i].x,rp[i].y);

      std::sort(rp.begin()+2, rp.end(), 
      [ rCenter ](const cv::Point2f& lhs, const cv::Point2f& rhs){
        return abs(lhs.x - rCenter.x) < abs(rhs.x - rCenter.x);
      });
      // printf(" y-2 sort\n");
      // for(int i = 0; i < 4; i++)
      //   printf("id: %d : %.1f,%.1f\n",i,rp[i].x,rp[i].y);

      // printf("-----END-------- %d\n",i);


      std::vector<cv::Point2f> np;
      np.push_back(cv::Point2f(rp[0].x,rp[0].y));
      np.push_back(cv::Point2f(rp[2].x,rp[2].y));
      np.push_back(cv::Point2f(rp[0].x,rp[1].y));
      np.push_back(cv::Point2f(rp[2].x,rp[3].y));
      // for(int i = 0; i < 4; i++){
      //   printf("id: %d :r %.1f,%.1f\n",i,rect_points[i].x,rect_points[i].y);
      //   printf("id: %d :n %.1f,%.1f\n",i,np[i].x,np[i].y);
      // }

      std::sort(np.begin(), np.end(), 
      [ ](const cv::Point2f& lhs, const cv::Point2f& rhs){
        return lhs.x < rhs.x;
      });
      std::sort(np.begin(), np.begin()+2, 
      [ ](const cv::Point2f& lhs, const cv::Point2f& rhs){
        return lhs.y < rhs.y;
      });
      std::sort(np.begin()+2, np.end(), 
      [ ](const cv::Point2f& lhs, const cv::Point2f& rhs){
        return lhs.y < rhs.y;
      });
      std::copy(np.begin(), np.end(), Global::targets.back().points);
      rect_points[0] = np[0];
      rect_points[1] = np[1];
      rect_points[2] = np[3];
      rect_points[3] = np[2];






      Global::targets.back().boundingRect = mRect;
      #ifndef OPTIMIZE
      timer.printTime(printTime, " findRect");
      #endif

      bool flag = false;
      int bounding = 20;
      for (int k = 0; k < 4; k++) {
        if (abs(rect_points[k].x - Global::FrameWidth / 2) > (Global::FrameWidth / 2 - bounding) || abs(rect_points[k].y - Global::FrameHeight / 2) > (Global::FrameHeight / 2 - bounding)){
          flag = true;
        }
      }
      if (flag) {
        #ifndef OPTIMIZE
        if (printTime)
          printf("  SKIP: edge too close\n");
        #endif
        Global::targets.erase(Global::targets.end()-1);
        continue;
      }

      
      #ifndef OPTIMIZE
      if(Switches::DRAW){
        for (int j = 0; j < 4; j++) {
          cv::line(img, rect_points[j], rect_points[(j + 1) % 4], Global::BLUE, 1, 8);
          circle(img, rect_points[j], 3, Global::RED, -1, 8, 0);
        }
      }
      #endif
      
      #ifndef OPTIMIZE
      timer.printTime(printTime," drawRect");
      #endif

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
      #ifndef OPTIMIZE
      timer.printTime(printTime," passed");
      #endif

    } //---end contour loop i

    #ifndef OPTIMIZE
    if (printTime)
      printf("end: %d found\n",targetsFound);
    #endif

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
      targetsFound = 3;
    }


    // Useful for a single target, here it's many rectangles which the previous algorithm already finds
    // ====================
    /*
    double qualityLevel     = 0.05;
    double minDistance      = 5;
    int    blockSize        = 5;
    bool   useHarisDetector = false;
    double k                = 0.04;
    int    maxCorners       = 4;

    if (targetsFound == 3) {
      cv::Mat workingImageSq[3];
      thresholded(Global::targets[0].boundingRect).copyTo(workingImageSq[0]);
      thresholded(Global::targets[1].boundingRect).copyTo(workingImageSq[1]);
      thresholded(Global::targets[2].boundingRect).copyTo(workingImageSq[2]);

      timer.printTime(printTime," drawMat");
      for(int mi = 0; mi < 3; mi++){
        // minDistance = Global::targets[mi].area / 5. * 0.8; // * (2./(2*5))
        // printf("dist: %f * %f\n",Global::targets[mi].area,  1 / 5. * 0.8);
        std::vector<cv::Point> corners;
        cv::goodFeaturesToTrack(workingImageSq[mi], corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, useHarisDetector, k);
        // cv::Mat rgbThresh;
        // cv::cvtColor(thresholded,rgbThresh,cv::COLOR_GRAY2BGR);
        // printf("corners: %d\n",corners.size());
        if(corners.size() >= 4){
          for(int i = 0; i < 4; i++){
            // std::cout << corners[i] << std::endl;
            corners[i].x = corners[i].x + Global::targets[mi].boundingRect.x;
            corners[i].y = corners[i].y + Global::targets[mi].boundingRect.y;
            Global::targets[mi].points[i] = corners[i];
            if(Switches::DRAW){
              // cv::circle(rgbThresh, corners[i], 2, Global::RED, -1, 8, 0);
              cv::circle(img, corners[i], 2, Global::RED, -1, 8, 0);
            }
          }
        }
        // cv::imshow("rgb",rgbThresh);
        // cv::imshow("img",img);
        // cv::waitKey(0);
      }
      timer.printTime(printTime," goodFeaturesTrack");
    }

    std::sort(Global::targets.begin(), Global::targets.end(), 
    [ ](const Target& lhs, const Target& rhs){
      return lhs.points[0].x < rhs.points[0].x;
    });
    */

  }

  #ifndef OPTIMIZE
  if (printTime)
    timer.PTotal();
  #endif


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
    p.add_Parameter("-http" ,"--http",Switches::USEHTTP,true,"use http server for streaming video");
    p.add_Parameter("-p" ,"--print",Switches::DOPRINT,false,"prints basic data");
    p.add_Parameter("-f" ,"--frame",Switches::FRAME,true,"prints of frames found");
    p.add_Parameter("-d","--draw",Switches::DRAW,true,"draws the lines on original img file");
    p.add_Parameter("-cam","--camera",Switches::USECAM,true,"which camera port to use");
    p.add_Parameter("-pt","--ptime",printTime_d,0,"(1-2) prints time taken for each loop");

    #ifdef OPTIMIZE
    Switches::USEHTTP = false;
    Switches::DRAW = false;
    #endif

    Var::dist_cof[0] = 0;
    Var::dist_cof[1] = 0;
    Var::dist_cof[2] = 0;
    Var::dist_cof[3] = 0;
    Var::dist_cof[4] = 0;

    if(p.checkParams(true))
      return 0;
    Switches::printTime = std::round(printTime_d);
    printf("pt: %d\n", Switches::printTime);
  }

  Clock serverClock;
  int frameCounter = 0, frameCounter2 = 0, frameCounterPrev = 0, missedFrames = 0;

  #ifndef OPTIMIZE
  ClockTimer timer;
  bool printTime = false;
  if (Switches::printTime == 1)
    printTime = true;
  timer.printTime(printTime,"getting input");
  #endif

  cv::Mat img, thresholded;
  cv::Mat rPos(cv::Size(250,1000),CV_8UC3);
  std::vector<Position> posA;
  int checkSum;

  startThread("VIDEO");
  startThread("TCP");
  if(Switches::USEHTTP)
    startThread("HTTP");
  if (Switches::SAVE)
    startThread("SAVE",NULL);


  if (!img.isContinuous())
    img = img.clone();
  Global::position.nullifyStruct();
  Global::positionAV.nullifyStruct();
  timer.printTime(printTime,"Init Threads");

  // might be best to wait until camera is up?
  initSolvePnP();

  serverClock.restart();

  while (true) {
    #ifndef OPTIMIZE
    if (Switches::printTime == 1)
      printTime = true;
    else
      printTime = false;
    timer.reset();
    #endif
    Global::muteFrame.lock();
    if (!Global::frame.empty() && Global::newFrame) {
      #ifndef OPTIMIZE
      timer.printTime(printTime,"Get Frame");
      #endif
      Global::frame.copyTo(img);
      Global::muteFrame.unlock();
      #ifndef OPTIMIZE
      timer.printTime(printTime,"Copy Frame");
      #endif
      frameCounter++;

      // TODO: lower resolution of thresholding, but do multiple times
      // cv::resize(img,scaled,cv::Size(640/2,480/2),INTER_LINEAR);

      ThresholdImage(img,thresholded);
      #ifndef OPTIMIZE
      timer.printTime(printTime," thresholded");
      #endif

      // morphOps(thresholded);
      #ifndef OPTIMIZE
      timer.printTime(printTime," morphOps");
      #endif

      int targetsFound = findTarget(img,thresholded); // FIND THE TARGETS
      #ifndef OPTIMIZE
      timer.printTime(printTime," findTarget");
      #endif
      // #ifndef OPTIMIZE
      if (targetsFound < 3)
        printf("  targetsFound: %d\n", targetsFound);
      // #endif

      if (targetsFound >= 3) {
        /* ---=== Getting Position and Rotation ===--- */
        findAnglePnP(img,rPos);
        #ifndef OPTIMIZE
        timer.printTime(printTime," solvePnP");
        #endif
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
        #ifndef OPTIMIZE
        timer.printTime(printTime," avaraging");
        #endif

        // #ifndef OPTIMIZE
        if (Switches::DOPRINT) {
          printf("dist=%6.2f, robotAngle=%6.2f, dataValid: %d\n",
                 Global::positionAV.dist,
                 Global::positionAV.robotAngle,
                 Global::dataValid);
        }
        // #endif
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

      #ifndef OPTIMIZE
      timer.printTime(printTime," copy to glob");

      /* ---=== Finished with frame, output it if needed ===--- */
      if (Switches::SHOWORIG)
        imshow("Original", img);
      if (Switches::SHOWTHRESH)
        imshow("Thresholded", thresholded);
      
      if (Switches::SHOWORIG || Switches::SHOWTHRESH) {
        cv::waitKey(5);
        timer.printTime(printTime," finished imshow");
      }

      if (printTime) {
        timer.printTime("End");
        printf("\n");
      }
      #endif
      Global::newFrame = false;
    } // end check for new frame
    else
      Global::muteFrame.unlock();

    // #ifndef OPTIMIZE
    frameCounter2++;
    if (frameCounter % 10 == 0 && frameCounter != frameCounterPrev) {
      frameCounterPrev = frameCounter;
      double dt = serverClock.getTimeAsSecs();
      if(Switches::FRAME){
        printf("------ Frame rate: %f fr/s (%f) \n", 10. / dt, frameCounter2 / dt);
        printf("------ Miss Frame: %d fr \n", missedFrames);
        #ifndef OPTIMIZE
        int checkSumLoc = 0;
        checkSumLoc += Var::minR + Var::maxR;
        checkSumLoc += Var::minG + Var::maxG;
        checkSumLoc += Var::minB + Var::maxB;
        if(checkSum != checkSumLoc){
          // writeParameters();
          checkSum = 0;
          checkSum += Var::minR + Var::maxR;
          checkSum += Var::minG + Var::maxG;
          checkSum += Var::minB + Var::maxB;
        }
        #endif
      }
      serverClock.restart();
      frameCounter2 = 0;
      missedFrames = 0;
    }
    // #endif
    usleep(1000);
  }
}

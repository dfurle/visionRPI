#include "clock.h"
#include "variables.h"
#include "parser.h"

void ThresholdImage(const cv::Mat& original, cv::Mat& thresholded, int color) {
  if(color == Target::COLOR::RED)
    cv::inRange(original, cv::Scalar(Var::rminR, Var::rminG, Var::rminB), cv::Scalar(Var::rmaxR, Var::rmaxG, Var::rmaxB), thresholded);
  else if(color == Target::COLOR::BLUE)
    cv::inRange(original, cv::Scalar(Var::bminR, Var::bminG, Var::bminB), cv::Scalar(Var::bmaxR, Var::bmaxG, Var::bmaxB), thresholded);
}

void morphOps(cv::Mat& thresh) {
  cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  // cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(8, 8));
  cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
  cv::erode(thresh,thresh,erodeElement);
  cv::dilate(thresh,thresh,dilateElement);
}

int findTarget(cv::Mat& img, cv::Mat& thresholded, int color) {
  Global::targets.clear();
  int targetsFound = 0;
  ClockTimer timer(Switches::printTime == 2);
  std::vector<cv::Vec4i> hierarchy;
  std::vector<std::vector<cv::Point> > contours;
  if (Switches::printTime == 2) {
    timer.reset();
    printf("begin findTarget\n");
  }

  cv::findContours(thresholded, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  timer.printTime(" finding Contours");

  for (auto it = contours.begin(); it != contours.end();) {
    //printf("size: %f\n",cv::contourArea(*it));
    if (cv::contourArea(*it) < 45) { // min contour
      it = contours.erase(it);
      timer.printTime(" removing contour");
    } else
      it++;
  }

  timer.printTime(" filter:Perim");
  if (contours.size() > 50) {
    timer.printTime(" many targets");
    return targetsFound;
  } else if (contours.size() < 1) {
    timer.printTime(" few targets");
    return targetsFound;
  }
  timer.printTime(" filter:Size");

  std::vector<std::vector<cv::Point> > art;
  cv::Mat workingImage(Global::FrameHeight, Global::FrameWidth, CV_8UC1, cv::Scalar(0));

  if (!contours.empty() && !hierarchy.empty()) {
    for (int i = 0; i < (int)contours.size(); i++) {
      if (timer.doPrint)
        printf("i: %d\n", i);
      // targets[i].NullTargets();
      Target target;
      target.id = i;
      if (hierarchy[i][2] != -1) {
        continue;
      }
      target.rect = cv::minAreaRect(contours[i]);
      // cv::Point2f rect_points[4];
      // std::copy(rect_points, rect_points + 4, target.points);
      // target.boundingRect = target.rect.boundingRect();
      target.boundingRect = cv::boundingRect(contours[i]);
      // target.rect.points(target.points);
      cv::Rect r = target.boundingRect;
      target.points[0] = cv::Point2f(r.x,r.y);
      target.points[1] = cv::Point2f(r.x+r.width,r.y);
      target.points[2] = cv::Point2f(r.x+r.width,r.y+r.height);
      target.points[3] = cv::Point2f(r.x,r.y+r.height);
      target.area = cv::contourArea(contours[i]);
      target.color = color;

      // if(Switches::DRAW)
      //   cv::putText(img,std::to_string(i),target.points[2],cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(0,255,255));


      // would rather use rotated rect area...
      float percentAreaTrue = 0.3;
      float percentAreaCalc = (target.boundingRect.area() - target.area)/(target.boundingRect.area());
      float diff = abs(percentAreaCalc - percentAreaTrue);
      if(diff > 0.1){
        if (timer.doPrint)
          printf("  SKIP: area ratio\n");
        continue;
      }
      timer.printTime( " findRect");


      float widthHeightRatio = target.boundingRect.width/double(target.boundingRect.height);
      // printf("  widhtHeight: %f\n",widthHeightRatio);
      if(abs(widthHeightRatio - 1) > 0.2){
        if (timer.doPrint)
          printf("  SKIP: width-height ratio\n");
        continue;
      }

      bool flag = false;
      int bounding = 20;
      for (int k = 0; k < 4; k++) {
        if (abs(target.points[k].x - Global::FrameWidth / 2) > (Global::FrameWidth / 2 - bounding) || abs(target.points[k].y - Global::FrameHeight / 2) > (Global::FrameHeight / 2 - bounding)){
          flag = true;
        }
      }
      if (flag) {
        if (timer.doPrint)
          printf("  SKIP: edge too close\n");
        continue;
      }

      
      if(Switches::DRAW){
        for (int j = 0; j < 4; j++) {
          cv::line(img, target.points[j], target.points[(j + 1) % 4], Global::BLUE, 1, 8);
          cv::circle(img, target.points[j], 3, Global::RED, -1, 8, 0);
        }
      }
      
      timer.printTime(" drawRect");
      targetsFound++;
      timer.printTime(" passed");
      Global::targets.push_back(target);
    } //---end contour loop i

    // for(int i = 0; i < contours.size(); i++){
    //   cv::drawContours(img,contours,i,cv::Scalar(0,255,0));
    // }

    if (timer.doPrint)
      printf("end: %d found\n",targetsFound);
  }
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
             if(str::cmp(param,"rminR")){
        Var::rminR = std::stoi(value);
      } else if(str::cmp(param,"rmaxR")){
        Var::rmaxR = std::stoi(value);
      } else if(str::cmp(param,"rminG")){
        Var::rminG = std::stoi(value);
      } else if(str::cmp(param,"rmaxG")){
        Var::rmaxG = std::stoi(value);
      } else if(str::cmp(param,"rminB")){
        Var::rminB = std::stoi(value);
      } else if(str::cmp(param,"rmaxB")){
        Var::rmaxB = std::stoi(value);

      } else if(str::cmp(param,"bminR")){
        Var::bminR = std::stoi(value);
      } else if(str::cmp(param,"bmaxR")){
        Var::bmaxR = std::stoi(value);
      } else if(str::cmp(param,"bminG")){
        Var::bminG = std::stoi(value);
      } else if(str::cmp(param,"bmaxG")){
        Var::bmaxG = std::stoi(value);
      } else if(str::cmp(param,"bminB")){
        Var::bminB = std::stoi(value);
      } else if(str::cmp(param,"bmaxB")){
        Var::bmaxB = std::stoi(value);
      }
    }
  }
  file.close();
}

void writeParameters(){
  std::ofstream file;
  file.open("../settings.txt", std::ios::out);
  if(file){
    file << "minR " + std::to_string(Var::rminR) + "\n";
    file << "maxR " + std::to_string(Var::rmaxR) + "\n";
    file << "minG " + std::to_string(Var::rminG) + "\n";
    file << "maxG " + std::to_string(Var::rmaxG) + "\n";
    file << "minB " + std::to_string(Var::rminB) + "\n";
    file << "maxB " + std::to_string(Var::rmaxB) + "\n";

    file << "minR " + std::to_string(Var::bminR) + "\n";
    file << "maxR " + std::to_string(Var::bmaxR) + "\n";
    file << "minG " + std::to_string(Var::bminG) + "\n";
    file << "maxG " + std::to_string(Var::bmaxG) + "\n";
    file << "minB " + std::to_string(Var::bminB) + "\n";
    file << "maxB " + std::to_string(Var::bmaxB) + "\n";
  }
  file.close();
}

int main(int argc, const char* argv[]) {
  std::cout << argv[0] << std::endl;
  std::cout << cv::getBuildInformation() << std::endl;
  readParameters();
  {
    Parser p(argc,argv);
    double printTime_d, cameraInput_d;
    p.add_Parameter("-o" ,"--orig",Switches::SHOWORIG,false,"displays original camera input w/ lines");
    p.add_Parameter("-th","--threshold",Switches::SHOWTHRESH,false,"displays thresholded image (black & white)");
    p.add_Parameter("-http" ,"--http",Switches::USEHTTP,true,"use http server for streaming video");
    p.add_Parameter("-sv" ,"--save",Switches::SAVE,false,"use http server for streaming video");
    p.add_Parameter("-p" ,"--print",Switches::DOPRINT,false,"prints basic data");
    p.add_Parameter("-f" ,"--frame",Switches::FRAME,true,"prints of frames found");
    p.add_Parameter("-d","--draw",Switches::DRAW,true,"draws the lines on original img file");
    p.add_Parameter("-cam","--cam",Switches::USECAM,true,"which camera port to use");
    p.add_Parameter("-pt","--ptime",printTime_d,0,"(1-2) prints time taken for each loop");

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

  ClockTimer timer(Switches::printTime == 1);
  timer.printTime("getting input");

  cv::Mat img, imgR, imgB, image, thresholdedR, thresholdedB;
  cv::Mat rPos(cv::Size(250,1000),CV_8UC3);
  std::vector<Position> posA;
  int checkSum;

  startThread("VIDEO");
  startThread("TCP");
  if(Switches::USEHTTP)
    startThread("HTTP");
  if (Switches::SAVE)
    startThread("SAVE");


  if (!img.isContinuous())
    img = img.clone();
  Global::position.nullifyStruct();
  Global::positionAV.nullifyStruct();
  timer.printTime("Init Threads");

  serverClock.restart();

  while (true) {
    timer.doPrint = Switches::printTime == 1;
    timer.reset();
    Global::muteFrame.lock();
    if (!Global::frame.empty() && Global::newFrame) {
      timer.printTime("Get Frame");
      Global::frame.copyTo(img);
      Global::muteFrame.unlock();
      img.copyTo(image);
      timer.printTime("Copy Frame");
      frameCounter++;

      // #define USESCALING

      cv::cvtColor(img,imgR,cv::COLOR_RGB2HSV_FULL);
      cv::cvtColor(img,imgB,cv::COLOR_BGR2HSV_FULL);
      timer.printTime(" cvt HSV");

      #ifdef USESCALING
      cv::Mat scaled;
      cv::Mat scaledT;
      int scale = 4;
      cv::resize(img,scaled,cv::Size(Var::WIDTH/scale,Var::HEIGHT/scale),cv::INTER_LINEAR);
      timer.printTime(" scale1");

      ThresholdImage(scaled,scaledT);
      timer.printTime(" thresh");
      morphOps(scaledT);
      timer.printTime(" morph");

      std::vector<cv::Vec4i> hierarchy;
      std::vector<std::vector<cv::Point> > contours;
      cv::findContours(scaledT, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      cv::Point min(scaledT.cols,scaledT.rows);
      cv::Point max(0,0);
      int cntrs = 0;
      for(auto c : contours){
        cntrs++;
        for(auto p : c){
          if(p.x < min.x){
            min.x = p.x;
          }
          if(p.y < min.y){
            min.y = p.y;
          }
          if(p.x > max.x){
            max.x = p.x;
          }
          if(p.y > max.y){
            max.y = p.y;
          }
        }
      }
      min = min * scale;
      max = max * scale;
      cv::Rect minRect = cv::Rect(min,cv::Size(max-min));
      timer.printTime(" find min/max");

      if(minRect.width > 0 && minRect.height > 0){
        cv::Mat imgCut, threshCut;
        img(minRect).copyTo(imgCut);
        timer.printTime(" copyTo");

        ThresholdImage(imgCut,threshCut);
        timer.printTime(" thresh cut");
        // morphOps(threshCut);
        // timer.printTime(" morph cut");
        thresholded = cv::Mat(cv::Size(Var::WIDTH,Var::HEIGHT), CV_8UC1);
        thresholded.setTo(0);
        threshCut.copyTo(thresholded(cv::Rect(min,threshCut.size())));
        timer.printTime(" copyto");
      } else {
        timer.printTime(" failed scale");
        thresholded = cv::Mat(cv::Size(Var::WIDTH,Var::HEIGHT), CV_8UC1);
        thresholded.setTo(0);
        timer.printTime(" set0");
        // ThresholdImage(img,thresholded);
        // timer.printTime(" thr heavy");

        // morphOps(thresholded);
        // timer.printTime(" morph heavy");
      }
      #else
      ThresholdImage(imgR,thresholdedR,Target::COLOR::RED);
      timer.printTime(" thrR");
      ThresholdImage(imgB,thresholdedB,Target::COLOR::BLUE);
      timer.printTime(" thrB");

      // morphOps(thresholdedR);
      // timer.printTime(" morph");
      // morphOps(thresholdedB);
      // timer.printTime(" morph");
      #endif

      int targetsFoundR = findTarget(img,thresholdedR, Target::COLOR::RED);  // FIND THE TARGETS
      int targetsFoundB = findTarget(img,thresholdedB, Target::COLOR::BLUE); // FIND THE TARGETS
      timer.printTime(" findTarget");
      if (targetsFoundR < 1)
        printf("  RtargetsFound: %d\n", targetsFoundR);
      if (targetsFoundB < 1)
        printf("  BtargetsFound: %d\n", targetsFoundB);

      if (targetsFoundR >= 1 || targetsFoundB >= 1) {
        Target target = Global::targets[0];
        printf("positions points: (%.2f,%.2f), (%.2f,%.2f)\n",target.points[0].x,target.points[0].y,target.points[2].x,target.points[2].y);
        double xDiff = target.points[0].x - target.points[2].x;
        xDiff *= 1.; // (size in pixels)/(distance in ft);
        printf("xDiff: %f\n",xDiff);
        double centerDiff = (Global::FrameWidth/2.) - (target.points[0].x + xDiff/2.);
        Global::mutePos.lock();
        Global::position.dist = xDiff;
        Global::position.robotAngle = centerDiff;
        Global::mutePos.unlock();


        if (Switches::DOPRINT) {
          printf("dist=%6.2f, robotAngle=%6.2f, dataValid: %d\n",
                 Global::position.dist,
                 Global::position.robotAngle,
                 Global::dataValid);
        }
      } else {
        missedFrames++;
      }

      if(Switches::USEHTTP || Switches::SAVE){
        Global::muteImg.lock();
        if(Global::httpStatus == 0){
          img.copyTo(Global::imgC);
          image.copyTo(Global::imgClean);
          thresholdedR.copyTo(Global::thresholdedRC);
          thresholdedB.copyTo(Global::thresholdedBC);
        }
        Global::muteImg.unlock();
      }

      timer.printTime(" copy to glob");
 
      /* ---=== Finished with frame, output it if needed ===--- */
      if (Switches::SHOWORIG)
        imshow("Original", img);
      if (Switches::SHOWTHRESH){
        imshow("ThresholdedR", thresholdedR);
        imshow("ThresholdedB", thresholdedB);
      }
      
      if (Switches::SHOWORIG || Switches::SHOWTHRESH) {
        cv::waitKey(5);
        timer.printTime(" finished imshow");
      }

      timer.PTotal();
      if(timer.doPrint) printf("-------\n");
      timer.printProportion();
      if(timer.doPrint) printf("-------\n");
      Global::newFrame = false;
    } // end check for new frame
    else
      Global::muteFrame.unlock();

    frameCounter2++;
    if (frameCounter % 10 == 0 && frameCounter != frameCounterPrev) {
      frameCounterPrev = frameCounter;
      double dt = serverClock.getTimeAsSecs();
      if(Switches::FRAME){
        printf("------ Frame rate: %.2f fr/s (%.1f) \n", 10. / dt, frameCounter2 / dt);
        // printf("------ Miss Frame: %d fr \n", missedFrames);
        // int checkSumLoc = 0;
        // checkSumLoc += Var::minR + Var::maxR;
        // checkSumLoc += Var::minG + Var::maxG;
        // checkSumLoc += Var::minB + Var::maxB;
        // if(checkSum != checkSumLoc){
        //   // writeParameters();
        //   checkSum = 0;
        //   checkSum += Var::minR + Var::maxR;
        //   checkSum += Var::minG + Var::maxG;
        //   checkSum += Var::minB + Var::maxB;
        // }
      }
      serverClock.restart();
      frameCounter2 = 0;
      missedFrames = 0;
    }
    // #endif
    usleep(1000);
  }
}

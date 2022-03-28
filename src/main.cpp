#include "clock.h"
#include "variables.h"
#include "parser.h"
#include "http.h"

inline void ThresholdImage(const cv::Mat& original, cv::Mat& thresholded);
inline void morphOps(cv::Mat& thresh);
void scaleAndThresh(cv::Mat& img, cv::Mat& thresh, ClockTimer& timer);

int main(int argc, const char* argv[]) {
  std::cout << cv::getBuildInformation() << std::endl;
  {
    Parser p(argc,argv);
    double printTime_d, res_d;
    p.add_Parameter("-o" ,"--orig",Switches::SHOWORIG,false,"displays original camera input w/ lines");
    p.add_Parameter("-th","--threshold",Switches::SHOWTHRESH,false,"displays thresholded image (black & white)");
    p.add_Parameter("-http" ,"--http",Switches::USEHTTP,true,"use http server for streaming video");
    p.add_Parameter("-p" ,"--print",Switches::DOPRINT,false,"prints basic data");
    p.add_Parameter("-tp" ,"--tfprint",Switches::TFPRINT,false,"prints the (targetsFound: 0)");
    p.add_Parameter("-sv" ,"--save",Switches::SAVE,false,"prints basic data");
    p.add_Parameter("-f" ,"--frame",Switches::FRAME,true,"prints of frames found");
    p.add_Parameter("-d","--draw",Switches::DRAW,true,"draws the lines on original img file");
    p.add_Parameter("-cam","--camera",Switches::USECAM,true,"which camera port to use");
    p.add_Parameter("-pt","--ptime",printTime_d,0,"(1-2) prints time taken for each loop");
    p.add_Parameter("-res","--resolution",res_d,0,"(1-2) prints time taken for each loop");

    Var::dist_cof[0] = 0;
    Var::dist_cof[1] = 0;
    Var::dist_cof[2] = 0;
    Var::dist_cof[3] = 0;
    Var::dist_cof[4] = 0;

    if(p.checkParams(true))
      return 0;
    Switches::printTime = std::round(printTime_d);
    Switches::resolution = std::round(res_d);
    Global::SIZE = cv::Size(Global::FRAME_SIZES[2*Switches::resolution],Global::FRAME_SIZES[2*Switches::resolution+1]);
  }

  Clock serverClock;
  int frameCounter = 0, frameCounter2 = 0, frameCounterPrev = 0, missedFrames = 0;

  ClockTimer timer(Switches::printTime == Global::PrintTimes::MAIN);
  timer.printTime("getting input");

  cv::Mat img, thresh, rPos(cv::Size(250,1000),CV_8UC3);
  cv::Mat imgC, threshC, rPosC;
  std::vector<Position> posA;
  int checkSum;

  startThread("VIDEO");
  startThread("TCP");
  if (Switches::SAVE)
    startThread("SAVE",NULL);
    
  HTTP http;
  if(Switches::USEHTTP){
    http.start(5050);
    http.addFile("/","../index.html","text/html");
    http.addImg("/video_stream1",&imgC);
    http.addImg("/video_stream2",&threshC);
    http.addImg("/video_stream3",&rPosC);
    http.addPut(Var::minBGR[0],"minB");
    http.addPut(Var::minBGR[1],"minG");
    http.addPut(Var::minBGR[2],"minR");
    http.addPut(Var::maxBGR[0],"maxB");
    http.addPut(Var::maxBGR[1],"maxG");
    http.addPut(Var::maxBGR[2],"maxR");
    http.addPut(Switches::DRAW,"Draw");
    http.addPut(Switches::DOPRINT,"Print");
    http.addPut(Switches::TFPRINT,"TFPrint");
    http.addPut(Switches::FRAME,"Frame");
    http.addPut(Switches::printTime,"PrintTime");
    http.startStreamingServer();
  }


  if (!img.isContinuous())
    img = img.clone();
  Global::position.nullifyStruct();
  Global::positionAV.nullifyStruct();
  timer.printTime("Init Threads");

  // might be best to wait until camera is up?
  initSolvePnP();

  serverClock.restart();

  while (true) {
    timer.doPrint = Switches::printTime == Global::PrintTimes::MAIN;
    timer.reset();
    Global::muteFrame.lock();
    if (!Global::frame.empty() && Global::newFrame) {
      timer.printTime("Get Frame");
      Global::frame.copyTo(img);
      Global::muteFrame.unlock();
      Global::muteImg.lock();
      if(Switches::SAVE)
        img.copyTo(Global::imgClean);
      Global::muteImg.unlock();
      timer.printTime("Copy Frame");
      frameCounter++;

      // #define USESCALING
      #ifdef USESCALING
      scaleAndThresh(img,thresh,timer);
      #else
      ThresholdImage(img,thresh);
      timer.printTime(" thr");

      morphOps(thresh);
      timer.printTime(" morph");
      #endif

      int targetsFound = findTarget(img,thresh); // FIND THE TARGETS
      timer.printTime(" findTarget");
      if (targetsFound < 3 && Switches::TFPRINT)
        printf("  targetsFound: %d\n", targetsFound);

      if (targetsFound >= 3) {
        /* ---=== Getting Position and Rotation ===--- */
        findAnglePnP(img,rPos);
        timer.printTime(" solvePnP");
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
        timer.printTime(" avaraging");

        if (Switches::DOPRINT) {
          printf("dist=%6.2f, robotAngle=%6.2f, dataValid: %d\n",
                 Global::positionAV.dist,
                 Global::positionAV.robotAngle,
                 Global::dataValid);
        }
        Global::missPrev = false;
      } else {
        Global::missPrev = true;
        missedFrames++;
      }

      if(Switches::USEHTTP || Switches::SAVE){
        Global::muteImg.lock();
        if(Global::httpStatus == 0){
          img.copyTo(imgC);
          thresh.copyTo(threshC);
          rPos.copyTo(rPosC);
        }
        Global::muteImg.unlock();
      }

      timer.printTime(" copy to glob");

      /* ---=== Finished with frame, output it if needed ===--- */
      if (Switches::SHOWORIG)
        cv::imshow("Original", img);
      if (Switches::SHOWTHRESH)
        cv::imshow("Thresholded", thresh);
      
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
      }
      serverClock.restart();
      frameCounter2 = 0;
      missedFrames = 0;
    }
    usleep(1000);
  }
}

inline void ThresholdImage(const cv::Mat& original, cv::Mat& thresholded) {
  cv::inRange(original, Var::minBGR, Var::maxBGR, thresholded);
}

inline void morphOps(cv::Mat& thresh) {
  cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  // cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(8, 8));
  cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
  cv::erode(thresh,thresh,erodeElement);
  cv::dilate(thresh,thresh,dilateElement);
}

void scaleAndThresh(cv::Mat& img, cv::Mat& thresh, ClockTimer& timer){
  cv::Mat scaled;
  cv::Mat scaledT;
  int scale = 4;
  cv::resize(img,scaled,Global::SIZE/scale,cv::INTER_LINEAR);
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
    thresh = cv::Mat(Global::SIZE, CV_8UC1);
    thresh.setTo(0);
    threshCut.copyTo(thresh(cv::Rect(min,threshCut.size())));
    timer.printTime(" copyto");
  } else {
    timer.printTime(" failed scale");
    thresh = cv::Mat(Global::SIZE, CV_8UC1);
    thresh.setTo(0);
    timer.printTime(" set0");
    ThresholdImage(img,thresh);
    timer.printTime(" thr heavy");

    morphOps(thresh);
    timer.printTime(" morph heavy");
  }
}


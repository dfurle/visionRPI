#include "variables.h"
#include "clock.h"

int findTarget(cv::Mat& img, cv::Mat& thresholded) {
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
    if (cv::contourArea(*it) < 45) {
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

  if (!contours.empty() && !hierarchy.empty()) {
    for (int i = 0; i < (int)contours.size(); i++) {
      if (timer.doPrint)
        printf("i: %d\n", i);
      if (hierarchy[i][2] != -1) {
        continue;
      }
      Target t;
      t.id = i;
      t.minRect = cv::minAreaRect(cv::Mat(contours[i]));
      t.minRect.points(t.points);
      t.boundingRect = t.minRect.boundingRect();
      t.area = cv::contourArea(contours[i]);


      for(int i = 0; i < 4; i++){
        t.center.x += t.points[i].x;
        t.center.y += t.points[i].y;
      }
      t.center.x/=4.;
      t.center.y/=4.;
      t.centerAim.x = (t.center.x / Var::WIDTH /2) - 1;
      t.centerAim.y = (t.center.y / Var::HEIGHT/2) - 1;

      std::vector<cv::Point2f> rp(t.points, t.points+4);
      std::sort(rp.begin(), rp.end(), 
      [ ](const cv::Point2f& lhs, const cv::Point2f& rhs){
        return lhs.x < rhs.x;
      });
      std::sort(rp.begin(), rp.begin()+2, 
      [ t ](const cv::Point2f& lhs, const cv::Point2f& rhs){
        return abs(lhs.x - t.center.x) < abs(rhs.x - t.center.x);
      });
      std::sort(rp.begin()+2, rp.end(), 
      [ t ](const cv::Point2f& lhs, const cv::Point2f& rhs){
        return abs(lhs.x - t.center.x) < abs(rhs.x - t.center.x);
      });

      std::vector<cv::Point2f> np;
      np.push_back(cv::Point2f(rp[0].x,rp[0].y));
      np.push_back(cv::Point2f(rp[2].x,rp[2].y));
      np.push_back(cv::Point2f(rp[0].x,rp[1].y));
      np.push_back(cv::Point2f(rp[2].x,rp[3].y));

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
        return lhs.y > rhs.y;
      });
      std::copy(np.begin(), np.end(), t.points);
      timer.printTime(" findRect");

      bool outOfBounds = false;
      int bounding = 20;
      for (int k = 0; k < 4; k++) {
        if (abs(t.points[k].x - Global::FrameWidth / 2) > (Global::FrameWidth / 2 - bounding) || abs(t.points[k].y - Global::FrameHeight / 2) > (Global::FrameHeight / 2 - bounding)){
          outOfBounds = true;
        }
      }
      if (outOfBounds) {
        if (timer.doPrint)
          printf("  SKIP: edge too close\n");
        continue;
      }

      
      if(Switches::DRAW){
        for (int j = 0; j < 4; j++) {
          cv::line(img, t.points[j], t.points[(j + 1) % 4], Global::RED, 1, 8);
          cv::circle(img, t.points[j], 3, Global::RED, -1, 8, 0);
        }
        timer.printTime(" drawRect");
      }


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
          t.points[i] = approx[i];
        }
      }
      */

      targetsFound++;
      timer.printTime(" passed");
      Global::targets.push_back(t);

    } //---end contour loop i

    if (timer.doPrint)
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

  timer.PTotal();


  return targetsFound;
}

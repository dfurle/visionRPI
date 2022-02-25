#include "variables.h"
#include "clock.h"
#define PI 3.141592

double radius = 2.2239583;

int numTargets = 16;

double tape_size  = 5.0  /12.;
double space_size = 5.5 /12.;

double tAngleL = (tape_size/2.)/(radius);

double angleBetween = (tape_size+space_size)/radius;
double tCenters[3] = {-angleBetween,0,angleBetween};

double stripHeight = (2.2/12.);

std::vector<cv::Point3f> mod3d;
std::vector<cv::Point3f> mod3d_center;
cv::Point2d center;
cv::Mat camera_matrix;
cv::Mat dist_coeffs;

std::vector<std::vector<cv::Point3f>> objectPoints;
std::vector<std::vector<cv::Point2f>> imagePoints;
cv::Size size;
cv::Mat new_cam_matrix;
cv::Mat distCoeffs;
std::vector<cv::Mat> rvecs;
std::vector<cv::Mat> tvecs;
int count = 0;


std::vector<cv::Point3f> axis3D;
std::vector<cv::Point2f> axis2D;
std::vector<cv::Point3f> circle3D;
std::vector<cv::Point2f> circle2D;
std::vector<cv::Point3f> base3D;
std::vector<cv::Point2f> base2D;


void initSolvePnP() {
  mod3d.clear();
  for(int i = 0; i < 3; i++){ 
    mod3d.push_back(cv::Point3f(radius*sin(tCenters[i]-tAngleL),0,           radius*cos(tCenters[i]-tAngleL)));
    mod3d.push_back(cv::Point3f(radius*sin(tCenters[i]-tAngleL),-stripHeight,radius*cos(tCenters[i]-tAngleL)));
    mod3d.push_back(cv::Point3f(radius*sin(tCenters[i]+tAngleL),0,           radius*cos(tCenters[i]+tAngleL)));
    mod3d.push_back(cv::Point3f(radius*sin(tCenters[i]+tAngleL),-stripHeight,radius*cos(tCenters[i]+tAngleL)));
  }

  // double setback = 2;
  // double mult = 0.5;
  // mod3d.push_back(cv::Point3f(radius*sin(tCenters[0]-tAngleL),0,           radius*cos(tCenters[0]-tAngleL)-(setback/12.*mult)));
  // mod3d.push_back(cv::Point3f(radius*sin(tCenters[0]-tAngleL),-stripHeight,radius*cos(tCenters[0]-tAngleL)-(setback/12.*mult)));
  // mod3d.push_back(cv::Point3f(radius*sin(tCenters[0]+tAngleL),0,           radius*cos(tCenters[0]+tAngleL)-(setback/12.)));
  // mod3d.push_back(cv::Point3f(radius*sin(tCenters[0]+tAngleL),-stripHeight,radius*cos(tCenters[0]+tAngleL)-(setback/12.)));

  // mod3d.push_back(cv::Point3f(radius*sin(tCenters[1]-tAngleL),0,           radius*cos(tCenters[1]-tAngleL)));
  // mod3d.push_back(cv::Point3f(radius*sin(tCenters[1]-tAngleL),-stripHeight,radius*cos(tCenters[1]-tAngleL)));
  // mod3d.push_back(cv::Point3f(radius*sin(tCenters[1]+tAngleL),0,           radius*cos(tCenters[1]+tAngleL)));
  // mod3d.push_back(cv::Point3f(radius*sin(tCenters[1]+tAngleL),-stripHeight,radius*cos(tCenters[1]+tAngleL)));

  // mod3d.push_back(cv::Point3f(radius*sin(tCenters[2]-tAngleL),0,           radius*cos(tCenters[2]-tAngleL)-(setback/12.)));
  // mod3d.push_back(cv::Point3f(radius*sin(tCenters[2]-tAngleL),-stripHeight,radius*cos(tCenters[2]-tAngleL)-(setback/12.)));
  // mod3d.push_back(cv::Point3f(radius*sin(tCenters[2]+tAngleL),0,           radius*cos(tCenters[2]+tAngleL)-(setback/12.*mult)));
  // mod3d.push_back(cv::Point3f(radius*sin(tCenters[2]+tAngleL),-stripHeight,radius*cos(tCenters[2]+tAngleL)-(setback/12.*mult)));




  mod3d_center.clear();
  for(int i = 0; i < 3; i++){
    // mod3d_center.push_back(cv::Point3d(radius*sin(tCenters[i]),-stripHeight/2.,radius*cos(tCenters[i])));
    mod3d_center.push_back(cv::Point3f(radius*sin(tCenters[i]),0,radius*cos(tCenters[i])));
  }

  
  axis3D.clear();
  axis3D.push_back(cv::Point3f(0.5f, 0, 0));
  axis3D.push_back(cv::Point3f(0, 0.5f, 0));
  axis3D.push_back(cv::Point3f(0, 0, -0.5f));
  axis3D.push_back(cv::Point3f(0, 0, 0));
  axis3D.push_back(cv::Point3f(0.5f, 0, 0+radius));
  axis3D.push_back(cv::Point3f(0, 0.5f, 0+radius));
  axis3D.push_back(cv::Point3f(0, 0, -0.5f+radius));
  axis3D.push_back(cv::Point3f(0, 0, 0+radius));
  axis3D.push_back(mod3d_center[0]);
  axis3D.push_back(mod3d_center[1]);
  axis3D.push_back(mod3d_center[2]);

  circle3D.clear();
  for(int i = 0; i < numTargets; i++){
    double mangle = 2*M_PI*(i/double(numTargets));
    // double mheight = -stripHeight/2.;
    double mheight = 0;
    circle3D.push_back(cv::Point3f(radius*sin(mangle),mheight  ,radius*cos(mangle)));
    circle3D.push_back(cv::Point3f(radius*sin(mangle),mheight-1,radius*cos(mangle)));
    circle3D.push_back(cv::Point3f(radius*sin(mangle),mheight-2,radius*cos(mangle)));
  }

  base3D.clear();
  for(int i = 0; i < numTargets; i++){
    double mangle = 2*M_PI*(i/double(numTargets));
    // double mheight = -stripHeight/2.;
    double mheight = 0;
    base3D.push_back(cv::Point3d(radius*sin(mangle),mheight,radius*cos(mangle)));
    base3D.push_back(cv::Point3d(radius*sin(mangle),mheight-8,radius*cos(mangle)));
  }


  // 2021
  // mod3d.push_back(cv::Point3d(+tTop / 2.0, -tHeight, 0.0)); // top right
  // mod3d.push_back(cv::Point3d(-tTop / 2.0, -tHeight, 0.0)); // top left
  // mod3d.push_back(cv::Point3d(-tStrip / 2.0, 0.0, 0.0));    // bottom left
  // mod3d.push_back(cv::Point3d(+tStrip / 2.0, 0.0, 0.0));    // bottom right

  center = cv::Point2d(Var::WIDTH / 2., Var::HEIGHT / 2.);
  size = cv::Size(Var::WIDTH, Var::HEIGHT);

  camera_matrix = (cv::Mat_<double>(3, 3) << 
		  size.width, 0, center.x,
		  0, size.height, center.y,
		  0, 0, 1);
  // dist_coeffs = NULL;
  // dist_coeffs = cv::Mat::zeros(1, 5, cv::DataType<double>::type);
  dist_coeffs = (cv::Mat_<double>(1, 5) << Var::dist_cof[0],Var::dist_cof[1],Var::dist_cof[2],Var::dist_cof[3],Var::dist_cof[4]);
}

bool pointsInBounds(std::vector<cv::Point2f> vec){
  for(auto it = vec.begin(); it != vec.end(); it++){
    if(it->x < 0 || it->x > Global::FrameWidth || it->y < 0 || it->y > Global::FrameHeight){
      printf("FAILED LINES OUT OF BOUNDS\n");
      return false;
    }
  }
  return true;
}

void findAnglePnP(cv::Mat& img, cv::Mat& rPos){
  center = cv::Point2d(Var::WIDTH / 2., Var::HEIGHT / 2.);
  size = cv::Size(Var::WIDTH, Var::HEIGHT);

  camera_matrix = (cv::Mat_<double>(3, 3) << 
		  size.width, 0, center.x,
		  0, size.height, center.y,
		  0, 0, 1);

  // std::cout << "cam: " << std::endl;
  // std::cout << camera_matrix << std::endl;
  // dist_coeffs = NULL;
  dist_coeffs = (cv::Mat_<double>(1, 5) << Var::dist_cof[0],Var::dist_cof[1],Var::dist_cof[2],Var::dist_cof[3],Var::dist_cof[4]);
  // dist_coeffs = NULL;
  std::vector<cv::Point2f> img2dpoints;
  ClockTimer timer;
  bool printTime = false;
  if (Switches::printTime == 3) {
    printTime = true;
    timer.reset();
    printf("begin findTarget\n");
  }

  // For 2022
  for(int t = 0; t < 3; t++){

    std::vector<cv::Point2d> pts;
    pts.push_back(Global::targets[t].points[0]);
    pts.push_back(Global::targets[t].points[1]);
    pts.push_back(Global::targets[t].points[2]);
    pts.push_back(Global::targets[t].points[3]);

    std::sort(pts.begin(), pts.end(), 
    [ ](const cv::Point2d& lhs, const cv::Point2d& rhs){
      return lhs.x < rhs.x;
    });


    if(pts[0].y < pts[1].y){
      img2dpoints.push_back(pts[0]);
      img2dpoints.push_back(pts[1]);
    } else {
      img2dpoints.push_back(pts[1]);
      img2dpoints.push_back(pts[0]);
    }
    if(pts[2].y > pts[3].y){
      img2dpoints.push_back(pts[3]);
      img2dpoints.push_back(pts[2]);
    } else {
      img2dpoints.push_back(pts[2]);
      img2dpoints.push_back(pts[3]);
    }
  }
  timer.printTime(printTime," added pts");

  /* ---===debugging drawing===--- */
  /*
  for(int i=0; i < (int) mod3d.size(); i++) {
    cv::circle(img, img2dpoints[i], i, cv::Scalar(0,255,0), 2);
    //printf("\n: x:%f\n",mod3d[i].x);
    //printf(": y:%f\n",mod3d[i].y);
    //printf(": z:%f\n",mod3d[i].z);
    //cv::circle(img, cv::Point2d((mod3d[i].x)*100+center.x,(-mod3d[i].y)*100+center.y), i, cv::Scalar(255,255,255), 2);
    //cv::circle(img, cv::Point2d((mod3d[i].z-2)*100+center.x,(-mod3d[i].y)*100+center.y*1.25), 1, cv::Scalar(255,255,0), 2);
    //cv::circle(img, cv::Point2d((mod3d[i].x)*100+center.x,(mod3d[i].z-2)*100+center.y*1.5), i, cv::Scalar(255,255,0), 2);
  }
  */

  cv::Mat rvec;
  cv::Mat tvec;
  cv::Mat rMat;

  // cv::solvePnP(mod3d, img2dpoints, camera_matrix, dist_coeffs, rvec, tvec);
  // cv::solvePnP(mod3d, img2dpoints, camera_matrix, dist_coeffs, rvec, tvec, true); //test this out!? plug in old r and t vec values
  cv::solvePnP(mod3d, img2dpoints, camera_matrix, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
  // cv::solvePnP(mod3d, img2dpoints, camera_matrix, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_EPNP);
  timer.printTime(printTime," solvePnP");

  cv::Rodrigues(rvec, rMat);
  timer.printTime(printTime," rodrig");

  // 2022
  // transvec is the transposing vector of target, x=0 y=1 z=2; x=dir y=height z=depth; rotation of robot matters
  double* transvec = tvec.ptr<double>();
  printf("tv: %5.2f %5.2f %5.2f\n",transvec[0],transvec[1],transvec[2]);

  cv::Mat tmp = cv::Mat(-rMat.t() * tvec);
  cv::Point3d xWorld = cv::Point3d(tmp.at<double>(0), tmp.at<double>(1), tmp.at<double>(2));

  printf("wx: %5.2f %5.2f %5.2f\n",xWorld.x,xWorld.y,xWorld.z);
  // transvec[0] -= Var::IRLOffset;
  double distance = sqrt(xWorld.x * xWorld.x + xWorld.z * xWorld.z);
  double robotAngle = atan2(transvec[0], transvec[2]);


  cv::Size res(500,1000);
  rPos.setTo(cv::Scalar(0,0,0));
  double relative_size = 100./radius;
  cv::Point center(res.width/2.,0);
  cv::circle(rPos,center,radius*relative_size,cv::Scalar(255,255,255),0.08*relative_size);
  cv::line(rPos, center, cv::Point(center.x,res.height), cv::Scalar(255,255,255));
  for(int i = 0; i < 18; i++){
    if(i%5==0)
      cv::line(rPos, cv::Point(center.x-(center.x*0.05),center.y+(i*relative_size)), cv::Point(center.x+(center.x*0.05),center.y+(i*relative_size)), cv::Scalar(255,255,0),0.07*relative_size);
    else
      cv::line(rPos, cv::Point(center.x-(center.x*0.05),center.y+(i*relative_size)), cv::Point(center.x+(center.x*0.05),center.y+(i*relative_size)), cv::Scalar(255,255,255),0.09*relative_size);
  }
  cv::Point robot_rPos(center.x+(xWorld.x*relative_size),center.y+xWorld.z*relative_size);
  cv::circle(rPos,robot_rPos, 0.5*relative_size, cv::Scalar(0,0,255),cv::FILLED);
  double alpha = atan2(xWorld.x, xWorld.z) - robotAngle;
  cv::line(rPos,robot_rPos,cv::Point(robot_rPos.x+(relative_size*sin(alpha)),robot_rPos.y-(relative_size*cos(alpha))), cv::Scalar(120,120,255),0.07*relative_size);
  cv::line(rPos,robot_rPos,cv::Point(robot_rPos.x+(15*relative_size*sin(alpha)),robot_rPos.y-(15*relative_size*cos(alpha))), cv::Scalar(120,120,255),0.04*relative_size);

  //distance += (1-0.904)*distance - 0.433;
  //distance += (1-0.992)*distance - 0.0327;

  Global::mutePos.lock();
  Global::position.dist = distance;
  Global::position.robotAngle = robotAngle * (180. / PI);
  Global::mutePos.unlock();
  timer.printTime(printTime," maths");

  std::stringstream streamDist;
  streamDist << std::fixed << std::setprecision(2) << distance << "ft";
  std::string sD = streamDist.str();

  std::stringstream streamAng;
  streamAng << std::fixed << std::setprecision(2) << robotAngle * (180./PI) << "deg";
  std::string sA = streamAng.str();
  cv::putText(rPos,sD,cv::Point(res.width*0.75,res.height*0.90),cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(255,255,255));
  cv::putText(rPos,sA,cv::Point(res.width*0.65,res.height*0.95),cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(255,255,255));

  // TODO AXIS

  axis2D.clear();
  circle2D.clear();
  base2D.clear();

  std::vector<cv::Point2f> reprojectedPoints;
  cv::projectPoints(mod3d, rvec, tvec, camera_matrix, dist_coeffs, reprojectedPoints);

  cv::projectPoints(axis3D, rvec, tvec, camera_matrix, dist_coeffs, axis2D);
  cv::projectPoints(circle3D, rvec, tvec, camera_matrix, dist_coeffs, circle2D);
  //cv::projectPoints(base3D, rvec, tvec, camera_matrix, dist_coeffs,base2D);
  timer.printTime(printTime," projectPts");

  if(Switches::DRAW){
    if(pointsInBounds(axis2D)){
      for(int i = 0; i < reprojectedPoints.size(); i++)
        cv::circle(img, reprojectedPoints[i], 3, cv::Scalar(255,255,255),cv::FILLED, cv::LINE_8);

      cv::line(img, axis2D[3], axis2D[2], cv::Scalar(255, 0, 0), 2); // z-blue
      cv::line(img, axis2D[3], axis2D[0], cv::Scalar(0, 0, 255), 2); // x-red
      cv::line(img, axis2D[3], axis2D[1], cv::Scalar(0, 255, 0), 2); // y-green
      cv::line(img, axis2D[7], axis2D[6], cv::Scalar(255, 0, 0), 2); // z-blue
      cv::line(img, axis2D[7], axis2D[4], cv::Scalar(0, 0, 255), 2); // x-red
      cv::line(img, axis2D[7], axis2D[5], cv::Scalar(0, 255, 0), 2); // y-green
      for (size_t i = 0; i < axis2D.size(); i++)
        cv::circle(img, axis2D[i], 3, cv::Scalar(255, 0, 255), cv::FILLED, cv::LINE_8);
      
      //for(int i = 0; i < base2D.size(); i+=2){
      //  cv::line(img, base2D[i], base2D[(i+2)%base2D.size()], cv::Scalar(0,100,100), 1);
      //  cv::line(img, base2D[i], base2D[i+1], cv::Scalar(0,100,100), 1);
      //  cv::line(img, base2D[i+1], base2D[(i+1+2)%base2D.size()], cv::Scalar(0,100,100), 1);
      //}
      for(int i = circle2D.size()/4+3; i < 3*circle2D.size()/4-3; i+=3){
        cv::line(img, circle2D[i], circle2D[(i+0+3)%circle2D.size()], cv::Scalar(0,100,100), 1);
        cv::line(img, circle2D[i], circle2D[i+1], cv::Scalar(0,100,100), 1);
        cv::line(img, circle2D[i+1], circle2D[(i+1+3)%circle2D.size()], cv::Scalar(0,100,100), 1);
        cv::line(img, circle2D[i+1], circle2D[i+2], cv::Scalar(0,100,100), 1);
        cv::line(img, circle2D[i+2], circle2D[(i+2+3)%circle2D.size()], cv::Scalar(0,100,100), 1);
      }
      for(int i = 0; i < circle2D.size()/4+3; i+=3){
        cv::line(img, circle2D[i], circle2D[(i+0+3)%circle2D.size()], cv::Scalar(0,255,255), 1);
        cv::line(img, circle2D[i], circle2D[i+1], cv::Scalar(0,255,255), 1);
        cv::line(img, circle2D[i+1], circle2D[(i+1+3)%circle2D.size()], cv::Scalar(0,255,255), 1);
        cv::line(img, circle2D[i+1], circle2D[i+2], cv::Scalar(0,255,255), 1);
        cv::line(img, circle2D[i+2], circle2D[(i+2+3)%circle2D.size()], cv::Scalar(0,255,255), 1);
      }
      for(int i = 3*circle2D.size()/4-3; i < circle2D.size(); i+=3){
        cv::line(img, circle2D[i], circle2D[(i+0+3)%circle2D.size()], cv::Scalar(0,255,255), 1);
        cv::line(img, circle2D[i], circle2D[i+1], cv::Scalar(0,255,255), 1);
        cv::line(img, circle2D[i+1], circle2D[(i+1+3)%circle2D.size()], cv::Scalar(0,255,255), 1);
        cv::line(img, circle2D[i+1], circle2D[i+2], cv::Scalar(0,255,255), 1);
        cv::line(img, circle2D[i+2], circle2D[(i+2+3)%circle2D.size()], cv::Scalar(0,255,255), 1);
      }
    }
  }
  timer.printTime(printTime," drew lines");

}

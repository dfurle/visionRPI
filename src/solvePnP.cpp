#include "variables.h"
#include "clock.h"
#define PI 3.141592

double radius = 2.;
double tCenters[3] = {-M_PI/8., 0, M_PI/8.};
double tAngleL = (5/12.)/(2*radius);
double stripHeight = (2/12.);

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
std::vector<cv::Point3f> lines3D;
std::vector<cv::Point2f> lines2D;
std::vector<cv::Point3f> plane3D;
std::vector<cv::Point2f> plane2D;
std::vector<cv::Point3f> circle3D;
std::vector<cv::Point2f> circle2D;


void initSolvePnP() {
  mod3d.clear();
  for(int i = 0; i < 3; i++){
    
    mod3d.push_back(cv::Point3d(radius*sin(tCenters[i]-tAngleL),0,           radius*cos(tCenters[i]-tAngleL)));
    mod3d.push_back(cv::Point3d(radius*sin(tCenters[i]-tAngleL),-stripHeight,radius*cos(tCenters[i]-tAngleL)));
    mod3d.push_back(cv::Point3d(radius*sin(tCenters[i]+tAngleL),0,           radius*cos(tCenters[i]+tAngleL)));
    mod3d.push_back(cv::Point3d(radius*sin(tCenters[i]+tAngleL),-stripHeight,radius*cos(tCenters[i]+tAngleL)));
  }
  for(int i = 0; i < 3; i++){
    mod3d_center.push_back(cv::Point3d(radius*sin(tCenters[i]),-stripHeight/2.,radius*cos(tCenters[i])));
  }


  axis3D.push_back(cv::Point3d(0.5f, 0, 0));
  axis3D.push_back(cv::Point3d(0, 0.5f, 0));
  axis3D.push_back(cv::Point3d(0, 0, -0.5f));
  axis3D.push_back(cv::Point3d(0, 0, 0));

  lines3D.push_back(mod3d_center[0]);
  lines3D.push_back(mod3d_center[1]);
  lines3D.push_back(mod3d_center[2]);
  lines3D.push_back(cv::Point3d(0, 0, 0));


  plane3D.push_back(mod3d_center[0]);
  plane3D.push_back(cv::Point3d(mod3d_center[0].x*1.5,mod3d_center[0].y,mod3d_center[0].z*1.5));
  plane3D.push_back(cv::Point3d(mod3d_center[1].x*1.5,mod3d_center[1].y,mod3d_center[1].z*1.5));
  plane3D.push_back(cv::Point3d(mod3d_center[2].x*1.5,mod3d_center[2].y,mod3d_center[2].z*1.5));
  plane3D.push_back(mod3d_center[2]);
  plane3D.push_back(mod3d_center[1]);
  plane3D.push_back(cv::Point3d(mod3d_center[0].x,mod3d_center[0].y+1.f,mod3d_center[0].z));
  plane3D.push_back(cv::Point3d(mod3d_center[1].x,mod3d_center[1].y+1.f,mod3d_center[1].z));
  plane3D.push_back(cv::Point3d(mod3d_center[2].x,mod3d_center[2].y+1.f,mod3d_center[2].z));


  for(int i = 0; i < 12; i++){
    circle3D.push_back(cv::Point3d(radius*sin(2*M_PI*(i/12.)),-stripHeight/2.,radius*cos(2*M_PI*(i/12.))));
  }


  // 2021
  // mod3d.push_back(cv::Point3d(+tTop / 2.0, -tHeight, 0.0)); // top right
  // mod3d.push_back(cv::Point3d(-tTop / 2.0, -tHeight, 0.0)); // top left
  // mod3d.push_back(cv::Point3d(-tStrip / 2.0, 0.0, 0.0));    // bottom left
  // mod3d.push_back(cv::Point3d(+tStrip / 2.0, 0.0, 0.0));    // bottom right

  center = cv::Point2d(Global::FrameWidth / 2., Global::FrameHeight / 2.);
  size = cv::Size(Global::FrameWidth, Global::FrameHeight);

  camera_matrix = (cv::Mat_<double>(3, 3) << 
		  Var::fx, 0, Var::cx,
		  0, Var::fy, Var::cy,
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

void findAnglePnP(cv::Mat& img) {
  dist_coeffs = (cv::Mat_<double>(1, 5) << Var::dist_cof[0],Var::dist_cof[1],Var::dist_cof[2],Var::dist_cof[3],Var::dist_cof[4]);
  std::vector<cv::Point2f> img2dpoints;
  ClockTimer timer;
  bool printTime = false;
  if (Switches::printTime == 3) {
    printTime = true;
    timer.reset();
    printf("begin findTarget\n");
  }

  // For 2022, placeholder
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
  //for(int i=0; i < (int) mod3d.size(); i++) {
    //circle(img, img2dpoints[i], i, cv::Scalar(0,255,0), 2);
    //circle(img, cv::Point2d(mod3d[i].x*100+center.x,-mod3d[i].y*100+center.y), i*5, cv::Scalar(255,255,0), 2);
  //}

  cv::Mat rvec;
  cv::Mat tvec;
  cv::Mat rMat;

  // cv::solvePnP(mod3d, img2dpoints, camera_matrix, dist_coeffs, rvec, tvec);
  // cv::solvePnP(mod3d, img2dpoints, camera_matrix, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_P3P); // default
  // cv::solvePnP(mod3d, img2dpoints, camera_matrix, dist_coeffs, rvec, tvec, true); //test this out!? plug in old r and t vec values
  cv::solvePnP(mod3d, img2dpoints, camera_matrix, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
  // cv::solvePnP(mod3d, img2dpoints, camera_matrix, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_DLS);
  // cv::solvePnP(mod3d, img2dpoints, camera_matrix, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_UPNP);
  timer.printTime(printTime," solvePnP");

  cv::Rodrigues(rvec, rMat);
  timer.printTime(printTime," rodrig");

  // 2022
  // transvec is the transposing vector of target, x=0 y=1 z=2; x=dir y=height z=depth; rotation of robot matters
  double* transvec = tvec.ptr<double>();
  // printf("tv: %5.2f %5.2f %5.2f\n",transvec[0],transvec[1],transvec[2]);
  double* xWorld = cv::Mat(-rMat.t() * tvec).ptr<double>();
  // printf("wx: %5.2f %5.2f %5.2f\n",xWorld[0],xWorld[1],xWorld[2]);
  // transvec[0] -= Var::IRLOffset;
  double distance = sqrt(xWorld[0] * xWorld[0] + xWorld[2] * xWorld[2]);
  // printf("dist %f\n",distance);
  double robotAngle = atan2(transvec[0], transvec[2]);
  // angle between dist vector and robot facing forward.


  Global::mutePos.lock();
  Global::position.dist = distance;
  Global::position.robotAngle = robotAngle * (180. / PI);
  Global::mutePos.unlock();
  timer.printTime(printTime," maths");

  // TODO AXIS

  cv::projectPoints(axis3D, rvec, tvec, camera_matrix, dist_coeffs, axis2D);
  cv::projectPoints(lines3D, rvec, tvec, camera_matrix, dist_coeffs, lines2D);
  cv::projectPoints(plane3D, rvec, tvec, camera_matrix, dist_coeffs, plane2D);
  cv::projectPoints(circle3D, rvec, tvec, camera_matrix, dist_coeffs, circle2D);
  timer.printTime(printTime," projectPts");


  if(pointsInBounds(lines2D)){
    for (size_t i = 0; i < lines2D.size(); i++)
      cv::circle(img, lines2D[i], 5, cv::Scalar(255, 0, 255), cv::FILLED, cv::LINE_8);
    cv::line(img, lines2D[0], lines2D[3], cv::Scalar(255,255,255), 2);
    cv::line(img, lines2D[1], lines2D[3], cv::Scalar(255,255,255), 2);
    cv::line(img, lines2D[2], lines2D[3], cv::Scalar(255,255,255), 2);
  }

  if(pointsInBounds(plane2D)){
    cv::line(img, plane2D[0], plane2D[1], cv::Scalar(255,255,255), 1);
    cv::line(img, plane2D[1], plane2D[2], cv::Scalar(255,255,255), 1);
    cv::line(img, plane2D[2], plane2D[3], cv::Scalar(255,255,255), 1);
    cv::line(img, plane2D[3], plane2D[4], cv::Scalar(255,255,255), 1);
    cv::line(img, plane2D[4], plane2D[5], cv::Scalar(255,255,255), 1);
    cv::line(img, plane2D[5], plane2D[0], cv::Scalar(255,255,255), 1);

    cv::line(img, plane2D[0], plane2D[6], cv::Scalar(255,255,255), 1);
    cv::line(img, plane2D[6], plane2D[7], cv::Scalar(255,255,255), 1);
    cv::line(img, plane2D[7], plane2D[8], cv::Scalar(255,255,255), 1);
    cv::line(img, plane2D[8], plane2D[4], cv::Scalar(255,255,255), 1);
  }

  if(pointsInBounds(axis2D)){
    for (size_t i = 0; i < axis2D.size(); i++)
      cv::circle(img, axis2D[i], 5, cv::Scalar(255, 0, 255), cv::FILLED, cv::LINE_8);
    cv::line(img, axis2D[3], axis2D[0], cv::Scalar(0, 0, 255), 2); // x-red
    cv::line(img, axis2D[3], axis2D[1], cv::Scalar(0, 255, 0), 2); // y-green
    cv::line(img, axis2D[3], axis2D[2], cv::Scalar(255, 0, 0), 2); // z-blue
  }


  if(pointsInBounds(circle2D)){
    for(int i = 0; i < 12; i++){
      cv::line(img, circle2D[i], circle2D[(i+1)%12], cv::Scalar(0,255,255), 1);
    }
  }
  timer.printTime(printTime," drew lines");

}

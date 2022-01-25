#include "variables.h"
#include "clock.h"
#define PI 3.141592

//double i2cm = 2.54;
//double tTop = 26.0 * i2cm;
//double tStrip = 14.5 * i2cm;
//double tHeight = 13.0 * i2cm;

//double i2cm = 2.54; // 2.54
//double tTop = 39.26 * i2cm; // 37.7
//double tStrip = 19.5 * i2cm; // 19.5
//double tHeight = 17.0 * i2cm; // 17.0

double i2cm = 0.08333; // 2.54
double tTop = 39.26 * i2cm; // 37.7
double tStrip = 19.5 * i2cm; // 19.5
double tHeight = 17.0 * i2cm; // 17.0

double radius = 2.;
double tCenters[3] = {-M_PI/8., 0, M_PI/8.};
double tAngleL = (5/12.)/(2*radius);
double stripHeight = (2/12.);

std::vector<cv::Point3f> mod3d;
std::vector<cv::Point3f> mod3d_center;
cv::Point2d center;
double focal_length;
double focal_length_r;
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

// void initSolvePnP(const cv::Mat& img) {
void initSolvePnP() {
  mod3d.clear();
  for(int i = 0; i < 3; i++){
    mod3d.push_back(cv::Point3d(radius*sin(tCenters[i]-tAngleL),0,           radius*cos(tCenters[i]-tAngleL)));
    mod3d.push_back(cv::Point3d(radius*sin(tCenters[i]+tAngleL),-stripHeight,radius*cos(tCenters[i]+tAngleL)));
  }
  for(int i = 0; i < 3; i++){
    mod3d_center.push_back(cv::Point3d(radius*sin(tCenters[i]),-stripHeight/2.,radius*cos(tCenters[i])));
  }


  // 2021
  // mod3d.push_back(cv::Point3d(+tTop / 2.0, -tHeight, 0.0)); // top right
  // mod3d.push_back(cv::Point3d(-tTop / 2.0, -tHeight, 0.0)); // top left
  // mod3d.push_back(cv::Point3d(-tStrip / 2.0, 0.0, 0.0));    // bottom left
  // mod3d.push_back(cv::Point3d(+tStrip / 2.0, 0.0, 0.0));    // bottom right

  center = cv::Point2d(Global::FrameWidth / 2., Global::FrameHeight / 2.);         // use the found center
  focal_length = Global::FrameWidth;
  focal_length_r = Global::FrameHeight;
  camera_matrix = (cv::Mat_<double>(3, 3) << focal_length, 0, center.x, 0, focal_length_r, center.y, 0, 0, 1);
  dist_coeffs = cv::Mat::zeros(5, 1, cv::DataType<double>::type);
  size = cv::Size(Global::FrameWidth, Global::FrameHeight);

  // GOOD PARAMETERS
  // camera_matrix = (cv::Mat_<double>(3,3) << 
                  //975.7315698328372, 0, 493.3360246900978, 0, 950.6116106554631, 391.9446438208861, 0, 0, 1);
  // dist_coeffs = (cv::Mat_<double>(1,5) << 0.1021387513883792, -0.1523456610014192, -0.0005797186028405386, -0.05044541761231074, 0.105243361554088);

  camera_matrix = (cv::Mat_<double>(3, 3) << 
		  Var::fx, 0, Var::cx, // 250 
		  0, Var::fy, Var::cy, // 260
		  0, 0, 1);
  dist_coeffs = (cv::Mat_<double>(1, 5) << Var::dist_cof[0],Var::dist_cof[1],Var::dist_cof[2],Var::dist_cof[3],Var::dist_cof[4]);

  camera_matrix = (cv::Mat_<double>(3, 3) << 
		  645.596980910074, 0, 290.4932471901438,
		  0, 645.5122982218642, 203.6380503569828,
		  0, 0, 1);
  dist_coeffs = (cv::Mat_<double>(1, 5) << 0.06182313013513041, 0.8114744623058454, -0.03986208147842847, -0.01346659639660972, -2.948899879946209);

  
  /*
  camera_matrix = (cv::Mat_<double>(3, 3) << 
		  640., 0, 320.,
		  0, 480., 240.,
		  0, 0, 1);
  dist_coeffs = (cv::Mat_<double>(1, 5) << 0., 0., 0., 0., 0.);
  */
  /*
  camera_matrix = (cv::Mat_<double>(3, 3) << 
    849.0214336999998/2, 0, 614.9722083831161/2,
    0, 849.1408003927772/2, 416.9607829490898/2,
    0, 0, 1);
  dist_coeffs = (cv::Mat_<double>(1, 5) << 0.02064680663006807, -0.03550275009991384, 0.009063551238267893, -0.003526262163251531, -0.0705243203157833);
  dist_coeffs /= 2.f;
  */


}

void findAnglePnP(cv::Mat& img) {
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
    img2dpoints.push_back(Global::targets[t].points[2]);
    img2dpoints.push_back(Global::targets[t].points[0]);
  }
  timer.printTime(printTime," added pts");

  // debugging drawing
  // for(int i=0; i < (int) mod3d.size(); i++) {
  //   circle(img, img2dpoints[i], i, cv::Scalar(0,255,0), 2);
  //   circle(img, cv::Point2d(mod3d[i].x*100+center.x,-mod3d[i].y*100+center.y), i*5,
  //           cv::Scalar(255,255,0), 2);
  // }

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

  /* since first param is input and were not using second param anywhere */
  // cv::Mat rotationVecTest;
  // cv::Rodrigues(rMat.t(),rotationVecTest);
  // cv::Mat tvecT = -rMat.t()*tvec;

  // 2022
  // transvec is the transposing vector of target, x=0 y=1 z=2; x=dir y=height z=depth; rotation of robot matters
  double* transvec = tvec.ptr<double>();
  double* xWorld = cv::Mat(-rMat.t() * tvec).ptr<double>();
  // transvec[0] -= Var::IRLOffset;
  double distance = sqrt(xWorld[0] * xWorld[0] + xWorld[2] * xWorld[2]);
  double robotAngle = atan2(transvec[0], transvec[2]);
  // angle between dist vector and robot facing forward.


  Global::mutePos.lock();
  Global::position.dist = distance;
  Global::position.robotAngle = robotAngle;
  Global::mutePos.unlock();
  timer.printTime(printTime," maths");
  // printf("position: %f, %f\n",position.dist,position.robotAngle);

  // TODO AXIS
  std::vector<cv::Point3f> axis3D;
  std::vector<cv::Point2f> axis2D;
  axis3D.push_back(cv::Point3d(0.5f, 0, 0));
  axis3D.push_back(cv::Point3d(0, 0.5f, 0));
  axis3D.push_back(cv::Point3d(0, 0, -0.5f));
  axis3D.push_back(cv::Point3d(0, 0, 0));

  std::vector<cv::Point3f> lines3D;
  std::vector<cv::Point2f> lines2D;
  lines3D.push_back(mod3d_center[0]);
  lines3D.push_back(mod3d_center[1]);
  lines3D.push_back(mod3d_center[2]);
  lines3D.push_back(cv::Point3d(0, 0, 0));
  
  cv::projectPoints(axis3D, rvec, tvec, camera_matrix, dist_coeffs, axis2D);
  cv::projectPoints(lines3D, rvec, tvec, camera_matrix, dist_coeffs, lines2D);
  timer.printTime(printTime," projectPts");


  for (size_t i = 0; i < axis2D.size(); i++)
    cv::circle(img, axis2D[i], 5, cv::Scalar(255, 0, 255), cv::FILLED, cv::LINE_8);
  cv::line(img, axis2D[3], axis2D[0], cv::Scalar(0, 0, 255), 2); // x-red
  cv::line(img, axis2D[3], axis2D[1], cv::Scalar(0, 255, 0), 2); // y-green
  cv::line(img, axis2D[3], axis2D[2], cv::Scalar(255, 0, 0), 2); // z-blue
  timer.printTime(printTime," drew axis");


  for (size_t i = 0; i < lines2D.size(); i++)
    cv::circle(img, lines2D[i], 5, cv::Scalar(255, 0, 255), cv::FILLED, cv::LINE_8);
  timer.printTime(printTime," drew dots");
  cv::line(img, lines2D[0], lines2D[3], cv::Scalar(255,255,255), 2);
  cv::line(img, lines2D[1], lines2D[3], cv::Scalar(255,255,255), 2);
  cv::line(img, lines2D[2], lines2D[3], cv::Scalar(255,255,255), 2);
  timer.printTime(printTime," drew lines");

  cv::circle(img, axis2D[3], 5, cv::Scalar(255, 0, 255), cv::FILLED, cv::LINE_8);
  timer.printTime(printTime," drew center");

}

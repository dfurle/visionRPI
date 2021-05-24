#include "variables.h"
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

std::vector<cv::Point3f> mod3d;
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
  mod3d.push_back(cv::Point3d(+tTop / 2.0, -tHeight, 0.0)); // top right
  mod3d.push_back(cv::Point3d(-tTop / 2.0, -tHeight, 0.0)); // top left
  mod3d.push_back(cv::Point3d(-tStrip / 2.0, 0.0, 0.0));    // bottom left
  mod3d.push_back(cv::Point3d(+tStrip / 2.0, 0.0, 0.0));    // bottom right
  center = cv::Point2d(Global::FrameWidth / 2, Global::FrameHeight / 2);         // use the found center
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

// TODO: maybe change "Position* position" to "Position& position"?
// TODO: remove "Targets& target", because it is in "Global::target"
void findAnglePnP(cv::Mat& img, Targets& target, Position* position) {
// void findAnglePnP(cv::Mat img, Targets& target, Position* position) {
  std::vector<cv::Point2f> img2dpoints;

  while (true) {
    int change = 0;
    for (int i = 0; i < 3; i++) {
      if (target.corners[i].y > target.corners[i + 1].y) {
        change = 1;
        cv::Point2f tmp = target.corners[i];
        target.corners[i] = target.corners[i + 1];
        target.corners[i + 1] = tmp;
      }
    }
    if (change == 0)
      break;
  }
  if (target.corners[0].x < target.corners[1].x) {
    img2dpoints.push_back(target.corners[1]);
    img2dpoints.push_back(target.corners[0]);
  } else {
    img2dpoints.push_back(target.corners[0]);
    img2dpoints.push_back(target.corners[1]);
  }
  if (target.corners[2].x < target.corners[3].x) {
    img2dpoints.push_back(target.corners[2]);
    img2dpoints.push_back(target.corners[3]);
  } else {
    img2dpoints.push_back(target.corners[3]);
    img2dpoints.push_back(target.corners[2]);
  }
  /*debugging drawing*/
  /*for(int i=0; i < (int) mod3d.size(); i++) {
    circle(im, mod2d[i], i, cv::Scalar(0,255,0), 2);
    circle(im, cv::Point2d(mod3d[i].x*3+center.x,mod3d[i].y*3+center.y+20), i*5,
  cv::Scalar(255,255,0), 2);
  }*/

  cv::Mat rvec;
  cv::Mat tvec;
  cv::Mat rMat;

  // cv::solvePnP(mod3d, img2dpoints, camera_matrix, dist_coeffs, rvec, tvec);
  // cv::solvePnP(mod3d, img2dpoints, camera_matrix, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_P3P); // default
  // cv::solvePnP(mod3d, img2dpoints, camera_matrix, dist_coeffs, rvec, tvec, true); //test this out!? plug in old r and t vec values
  cv::solvePnP(mod3d, img2dpoints, camera_matrix, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
  // cv::solvePnP(mod3d, img2dpoints, camera_matrix, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_DLS);
  // cv::solvePnP(mod3d, img2dpoints, camera_matrix, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_UPNP);

  cv::Rodrigues(rvec, rMat);

  /* since first param is input and were not using second param anywhere */
  // cv::Mat rotationVecTest;
  // cv::Rodrigues(rMat.t(),rotationVecTest);
  // cv::Mat tvecT = -rMat.t()*tvec;

  // transvec is the transposing vector of target, x=0 y=1 z=2; x=dir y=height z=depth; rotation of robot matters
  double* transvec = tvec.ptr<double>();
  cv::Mat xWorldd = -rMat.t() * tvec;
  double* xWorld = xWorldd.ptr<double>();
  transvec[0] -= Var::IRLOffset;
  double distance = sqrt(xWorld[0] * xWorld[0] + xWorld[2] * xWorld[2]);
  double alpha1 = atan2(transvec[0], transvec[2]);
  // angle between dist vector and robot facing forward.

  // xWorld is the irl coords of cam vs target; rotation of robot does NOT matter
  double alpha2 = atan2(xWorld[0], -xWorld[2]);
  // angle between dist vector and target facing forward.

  cv::Point2d tc((img2dpoints[2].x + img2dpoints[3].x) / 2.0, ((img2dpoints[2].y + img2dpoints[3].y) + (img2dpoints[1].y + img2dpoints[0].y)) / 4.0);

  // writing data
  position->x = sin(alpha2) * distance;
  position->z = cos(alpha2) * distance;
  position->x = xWorld[0];
  position->z = -xWorld[2];
  position->dist = distance;
  position->alpha1 = alpha1 * (180. / PI);
  position->alpha2 = alpha2 * (180. / PI);

  // TODO AXIS
  std::vector<cv::Point3f> axis3D;
  std::vector<cv::Point2f> axis2D;
  axis3D.push_back(cv::Point3d(1.f, 0, 0));
  axis3D.push_back(cv::Point3d(0, 1.f, 0));
  axis3D.push_back(cv::Point3d(0, 0, -1.f));
  axis3D.push_back(cv::Point3d(0, 0, 0));

  
  axis3D.push_back(mod3d[0]);
  axis3D.push_back(mod3d[1]);
  axis3D.push_back(mod3d[2]);
  axis3D.push_back(mod3d[3]);

  axis3D.push_back(cv::Point3d(mod3d[0].x,mod3d[0].y,-1.f));
  axis3D.push_back(cv::Point3d(mod3d[1].x,mod3d[1].y,-1.f));
  axis3D.push_back(cv::Point3d(mod3d[2].x,mod3d[2].y,-1.f));
  axis3D.push_back(cv::Point3d(mod3d[3].x,mod3d[3].y,-1.f));
  
  //axis3D.push_back(cv::Point3d(25., 0, 0));
  //axis3D.push_back(cv::Point3d(0, 25., 0));
  //axis3D.push_back(cv::Point3d(0, 0, 25.));
  //axis3D.push_back(cv::Point3d(0, 0, 0));
  
  cv::projectPoints(axis3D, rvec, tvec, camera_matrix, dist_coeffs, axis2D);

  // std::cout << " rvec = \n" << rvec << std::endl;
  // std::cout << " tvec = \n" << tvec << std::endl;
  // std::cout << " axis 2D = \n" << axis2D << std::endl;
  // std::cout << " tc = \n" << tc << std::endl;

  // for (size_t i = 0; i < img2dpoints.size(); i++)
  //   circle(img, img2dpoints[i], 7, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8);

  for (size_t i = 0; i < axis2D.size(); i++)
    circle(img, axis2D[i], 5, cv::Scalar(255, 0, 255), cv::FILLED, cv::LINE_8);
  circle(img, axis2D[3], 5, cv::Scalar(255, 255, 255), cv::FILLED, cv::LINE_8);

  cv::line(img, axis2D[3], axis2D[0], cv::Scalar(0, 0, 255), 2); // x-red
  cv::line(img, axis2D[3], axis2D[1], cv::Scalar(0, 255, 0), 2); // y-green
  cv::line(img, axis2D[3], axis2D[2], cv::Scalar(255, 0, 0), 2); // z-blue


  cv::line(img, axis2D[4], axis2D[5], cv::Scalar(255, 255, 255), 2);
  cv::line(img, axis2D[5], axis2D[6], cv::Scalar(255, 255, 255), 2);
  cv::line(img, axis2D[6], axis2D[7], cv::Scalar(255, 255, 255), 2);
  cv::line(img, axis2D[7], axis2D[4], cv::Scalar(255, 255, 255), 2);

  cv::line(img, axis2D[4], axis2D[8], cv::Scalar(255, 255, 255), 2);
  cv::line(img, axis2D[5], axis2D[9], cv::Scalar(255, 255, 255), 2);
  cv::line(img, axis2D[6], axis2D[10], cv::Scalar(255, 255, 255), 2);
  cv::line(img, axis2D[7], axis2D[11], cv::Scalar(255, 255, 255), 2);

  cv::line(img, axis2D[8], axis2D[9], cv::Scalar(255, 255, 0), 2);
  cv::line(img, axis2D[9], axis2D[10], cv::Scalar(255, 255, 0), 2);
  cv::line(img, axis2D[10], axis2D[11], cv::Scalar(255, 255, 0), 2);
  cv::line(img, axis2D[11], axis2D[8], cv::Scalar(255, 255, 0), 2);

  // cv::circle(im,center,4,cv::Scalar(255,255,255),2);

  /* todo camera calibration? */

  // rvecs.push_back(rvec);
  // tvecs.push_back(tvec);
  // objectPoints.push_back(mod3d);
  // imagePoints.push_back(img2dpoints);
  // printf("counter: %d\n",count++);
  // if(count >= 25){
  //   double reprojectionError = cv::calibrateCamera(objectPoints, imagePoints, size, new_cam_matrix, distCoeffs, rvecs, tvecs);
  //   printf("reproj: %f\n",reprojectionError);
  //   std::cout << "new cam mat: " << new_cam_matrix << std::endl;
  //   std::cout << "distCoeffs: " << distCoeffs << std::endl;
  //   // reproj~ 0.42
  //   /*
  //   new cam mat:
  //   [589.7459123027554, 0, 273.3815151315198;
  //   0, 490.5723437651063, 216.6938652072021;
  //   0, 0, 1]
  //   distCoeffs:
  //   [1.127042809255204, -10.37013923197316, 0.07598159234258514, -0.007075099337870662, 26.21197331983544]
  //   */
  // }
}

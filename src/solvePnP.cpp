#include "variables.h"
#define PI 3.141592

double i2cm = 2.54;
double tTop = 39.0 * i2cm;
double tStrip = 19.5 * i2cm;
double tHeight = 18.0 * i2cm;

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

void initSolvePnP(cv::Mat img) {
  mod3d.clear();
  mod3d.push_back(cv::Point3d(+tTop / 2.0, -tHeight, 0.0)); // top right
  mod3d.push_back(cv::Point3d(-tTop / 2.0, -tHeight, 0.0)); // top left
  mod3d.push_back(cv::Point3d(-tStrip / 2.0, 0.0, 0.0));    // bottom left
  mod3d.push_back(cv::Point3d(+tStrip / 2.0, 0.0, 0.0));    // bottom right
  center = cv::Point2d(img.cols / 2, img.rows / 2);         // use the found center
  focal_length = img.cols;
  focal_length_r = img.rows;
  camera_matrix = (cv::Mat_<double>(3, 3) << focal_length, 0, center.x, 0, focal_length_r, center.y, 0, 0, 1);
  dist_coeffs = cv::Mat::zeros(4, 1, cv::DataType<double>::type);
  size = cv::Size(Global::FrameWidth, Global::FrameHeight);

  // GOOD PARAMETERS
  // camera_matrix = (cv::Mat_<double>(3,3) << 975.7315698328372, 0, 493.3360246900978, 0, 950.6116106554631, 391.9446438208861, 0, 0, 1);
  // dist_coeffs = (cv::Mat_<double>(1,5) << 0.1021387513883792, -0.1523456610014192, -0.0005797186028405386, -0.05044541761231074, 0.105243361554088);

  camera_matrix = (cv::Mat_<double>(3, 3) << 975.7315698328372 / 2, 0, 493.3360246900978 / 2, 0, 950.6116106554631 / 2, 391.9446438208861 / 2, 0, 0, 1);
  dist_coeffs = (cv::Mat_<double>(1, 5) << 0.1021387513883792 / 2, -0.1523456610014192 / 2, -0.0005797186028405386 / 2, -0.05044541761231074 / 2, 0.105243361554088 / 2);
}

void findAnglePnP(cv::Mat img, Targets target, Position* position) {
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
  cv::solvePnP(mod3d, img2dpoints, camera_matrix, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_P3P); // default
  // cv::solvePnP(mod3d, img2dpoints, camera_matrix, dist_coeffs, rvec, tvec, true); //test this out!? plug in old r and t vec values
  // cv::solvePnP(mod3d, img2dpoints, camera_matrix, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
  // cv::solvePnP(mod3d, img2dpoints, camera_matrix, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_DLS);
  // cv::solvePnP(mod3d, img2dpoints, camera_matrix, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_UPNP);

  cv::Rodrigues(rvec, rMat);

  /* since first param is input and were not using second param anywhere */
  // cv::Mat rotationVecTest;
  // cv::Rodrigues(rMat.t(),rotationVecTest);
  // cv::Mat tvecT = -rMat.t()*tvec;

  // transvec is the transposing vector of target, x=0 y=1 z=2; x=dir y=height z=depth; rotation of robot matters
  double* transvec = tvec.ptr<double>();
  transvec[0] -= Var::IRLOffset;
  double distance = sqrt(transvec[0] * transvec[0] + transvec[2] * transvec[2]);
  double alpha1 = atan2(transvec[0], transvec[2]);
  // angle between dist vector and robot facing forward.

  // xWorld is the irl coords of cam vs target; rotation of robot does NOT matter
  cv::Mat xWorldd = -rMat.t() * tvec;
  double* xWorld = xWorldd.ptr<double>();
  double alpha2 = atan2(xWorld[0], -xWorld[2]);
  // angle between dist vector and target facing forward.

  cv::Point2d tc((img2dpoints[2].x + img2dpoints[3].x) / 2.0, ((img2dpoints[2].y + img2dpoints[3].y) + (img2dpoints[1].y + img2dpoints[0].y)) / 4.0);

  // writing data
  position->x = sin(alpha2) * distance;
  position->z = cos(alpha2) * distance;
  position->dist = distance;
  position->alpha1 = alpha1 * (180. / PI);
  position->alpha2 = alpha2 * (180. / PI);

  // TODO AXIS
  std::vector<cv::Point3f> axis3D;
  std::vector<cv::Point2f> axis2D;
  axis3D.push_back(cv::Point3d(25., 0, 0));
  axis3D.push_back(cv::Point3d(0, 25., 0));
  axis3D.push_back(cv::Point3d(0, 0, 25.));
  axis3D.push_back(cv::Point3d(0, 0, 0));
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

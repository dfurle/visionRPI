#include "variables.h"
#include "clock.h"

double radius = 2.2239583;

int numTargets = 16;

double tape_size  = 5.0  /12.;
double space_size = 5.5 /12.;

double tAngleL = (tape_size/2.)/(radius);

double angleBetween = (tape_size+space_size)/radius;
double tCenters[3] = {-angleBetween,0,angleBetween};

double stripHeight = (2./12.);

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


  mod3d_center.clear();
  for(int i = 0; i < 3; i++){
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
  camera_matrix = (cv::Mat_<double>(3, 3) << 
    580.4984062368188, 0, 325.3680926895594,
    0, 580.4542213657525, 267.8603909876159,
    0, 0, 1);
    
  dist_coeffs = (cv::Mat_<double>(1, 5) << 
    -0.02884002148680569,
    0.08108439904507091,
    0.009277215393240128,
    0.000317625197072589,
    -0.4184398847498976);


  camera_matrix = (cv::Mat_<double>(3, 3) << 
    1122.668685412353, 0, 631.338038581117,
    0, 1072.878260659514, 308.0890669710171,
    0, 0, 1);
    
  dist_coeffs = (cv::Mat_<double>(1, 5) << 
    0.5288239655258603,
    -7.815535053795306,
    0.04467497612220633,
    0.004964838135067267,
    25.12841292762858);

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
  if(Var::WIDTH==640){
    camera_matrix = (cv::Mat_<double>(3, 3) << 
      580.4984062368188, 0, 325.3680926895594,
      0, 580.4542213657525, 267.8603909876159,
      0, 0, 1);
      
    dist_coeffs = (cv::Mat_<double>(1, 5) << 
      -0.02884002148680569,
      0.08108439904507091,
      0.009277215393240128,
      0.000317625197072589,
      -0.4184398847498976);
  } else {
    camera_matrix = (cv::Mat_<double>(3, 3) << 
      1122.668685412353, 0, 631.338038581117,
      0, 1072.878260659514, 308.0890669710171,
      0, 0, 1);
      
    dist_coeffs = (cv::Mat_<double>(1, 5) << 
      0.5288239655258603,
      -7.815535053795306,
      0.04467497612220633,
      0.004964838135067267,
      25.12841292762858);
  }

  std::vector<cv::Point2f> img2dpoints;
  ClockTimer timer(Switches::printTime == 3);
  if (Switches::printTime == 3) {
    timer.reset();
    printf("begin solvePnP\n");
  }

  for(int t = 0; t < 3; t++){
    img2dpoints.push_back(Global::targets[t].points[0]);
    img2dpoints.push_back(Global::targets[t].points[1]);
    img2dpoints.push_back(Global::targets[t].points[2]);
    img2dpoints.push_back(Global::targets[t].points[3]);
  }
  // For 2022
  /*
  for(int t = 0; t < 3; t++){
    std::vector<cv::Point2f> pts;
    pts.push_back(Global::targets[t].points[0]);
    pts.push_back(Global::targets[t].points[1]);
    pts.push_back(Global::targets[t].points[2]);
    pts.push_back(Global::targets[t].points[3]);

    std::sort(pts.begin(), pts.end(), 
    [ ](const cv::Point2f& lhs, const cv::Point2f& rhs){
      return lhs.x < rhs.x;
    });
    std::sort(pts.begin(), pts.begin()+2, 
    [ ](const cv::Point2f& lhs, const cv::Point2f& rhs){
      return lhs.y < rhs.y;
    });
    std::sort(pts.begin()+2, pts.end(), 
    [ ](const cv::Point2f& lhs, const cv::Point2f& rhs){
      return lhs.y < rhs.y;
    });
    img2dpoints.push_back(pts[0]);
    img2dpoints.push_back(pts[1]);
    img2dpoints.push_back(pts[2]);
    img2dpoints.push_back(pts[3]);


    // if(pts[0].y < pts[1].y){
    //   img2dpoints.push_back(pts[0]);
    //   img2dpoints.push_back(pts[1]);
    // } else {
    //   img2dpoints.push_back(pts[1]);
    //   img2dpoints.push_back(pts[0]);
    // }
    // if(pts[2].y > pts[3].y){
    //   img2dpoints.push_back(pts[3]);
    //   img2dpoints.push_back(pts[2]);
    // } else {
    //   img2dpoints.push_back(pts[2]);
    //   img2dpoints.push_back(pts[3]);
    // }
  }
  */
  timer.printTime(" added pts");

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

  
  // cv::solvePnP(mod3d, img2dpoints, camera_matrix, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
  
  Global::muteHTTP.lock();
  // printf("tr: %d\n",Global::useTR);
  if(Global::useTR){
    rvec = Global::rvec_g;
    tvec = Global::tvec_g;
  } else {
    cv::solvePnP(mod3d, img2dpoints, camera_matrix, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
  }
  Global::muteHTTP.unlock();


  timer.printTime(" solvePnP");

  cv::Rodrigues(rvec, rMat);
  timer.printTime(" rodrig");

  // 2022
  // transvec is the transposing vector of target, x=0 y=1 z=2; x=dir y=height z=depth; rotation of robot matters
  double* transvec = tvec.ptr<double>();
  double* rotvec = rvec.ptr<double>();
  cv::Mat tmp = cv::Mat(-rMat.t() * tvec);
  cv::Point3d xWorld = cv::Point3d(tmp.at<double>(0), tmp.at<double>(1), tmp.at<double>(2));

  // printf("tv: %5.2f %5.2f %5.2f\n",transvec[0],transvec[1],transvec[2]);
  // printf("rv: %5.2f %5.2f %5.2f\n",rotvec[0]*(180./M_PI),rotvec[1]*(180./M_PI),rotvec[2]*(180./M_PI));
  // printf("wx: %5.2f %5.2f %5.2f\n",xWorld.x,xWorld.y,xWorld.z);
  
  // transvec[0] -= Var::IRLOffset;
  double distance = sqrt(xWorld.x * xWorld.x + xWorld.z * xWorld.z);
  double robotAngle = atan2(transvec[0], transvec[2]);


  cv::Size res(250,1000);
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
  double alpha = robotAngle + atan2(xWorld.x, xWorld.z);
  cv::line(rPos,robot_rPos,cv::Point(robot_rPos.x-(relative_size*sin(alpha)),robot_rPos.y-(relative_size*cos(alpha))), cv::Scalar(120,120,255),0.07*relative_size);
  cv::line(rPos,robot_rPos,cv::Point(robot_rPos.x-(15*relative_size*sin(alpha)),robot_rPos.y-(15*relative_size*cos(alpha))), cv::Scalar(120,120,255),0.04*relative_size);

  //distance += (1-0.904)*distance - 0.433;
  //distance += (1-0.992)*distance - 0.0327;
  // distance += (1-1.07)*distance-0.423;
  Global::mutePos.lock();
  Global::position.dist = distance;
  Global::position.robotAngle = robotAngle * (180. / M_PI);
  Global::mutePos.unlock();
  timer.printTime(" maths");

  std::stringstream streamDist;
  streamDist << std::fixed << std::setprecision(2) << distance << "ft";
  std::string sD = streamDist.str();

  std::stringstream streamAng;
  streamAng << std::fixed << std::setprecision(2) << robotAngle * (180./M_PI) << "deg";
  std::string sA = streamAng.str();
  cv::putText(rPos,sD,cv::Point(res.width*0.6,res.height*0.94),cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(255,255,255));
  cv::putText(rPos,sA,cv::Point(res.width*0.5,res.height*0.98),cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(255,255,255));
  if(Switches::DRAW){
    cv::putText(img,sD,cv::Point(Var::WIDTH*0.8,Var::HEIGHT*0.9),cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(255,255,255));
    cv::putText(img,sA,cv::Point(Var::WIDTH*0.75,Var::HEIGHT*0.95),cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(255,255,255));
  }

  // TODO AXIS

  axis2D.clear();
  circle2D.clear();
  base2D.clear();

  std::vector<cv::Point2f> reprojectedPoints;
  cv::projectPoints(mod3d, rvec, tvec, camera_matrix, dist_coeffs, reprojectedPoints);

  cv::projectPoints(axis3D, rvec, tvec, camera_matrix, dist_coeffs, axis2D);
  cv::projectPoints(circle3D, rvec, tvec, camera_matrix, dist_coeffs, circle2D);
  //cv::projectPoints(base3D, rvec, tvec, camera_matrix, dist_coeffs,base2D);
  timer.printTime(" projectPts");

  if(Switches::DRAW){
    if(pointsInBounds(axis2D)){
      //for(int i = 0; i < reprojectedPoints.size(); i++)
        //cv::circle(img, reprojectedPoints[i], 3, cv::Scalar(255,255,255),cv::FILLED, cv::LINE_8);

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
  timer.printTime(" drew lines");

}

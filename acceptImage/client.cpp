/**
 * OpenCV video streaming over TCP/IP
 * Client: Receives video from server and display it
 * by Steve Tuenkam
 */

#include "opencv2/opencv.hpp"
#include <sys/socket.h> 
#include <arpa/inet.h>
#include <unistd.h>

using namespace cv;

bool USECOLOR = false;


int main(int argc, char** argv){
  if(argc>2){
    std::string useColor = argv[argc-1];
    if(useColor == "-c")
      USECOLOR = true;
    else
      USECOLOR = false;
    printf("USECOLOR: %d\n",USECOLOR);
  }
  else
    USECOLOR=false;

  //--------------------------------------------------------
  //networking stuff: socket , connect
  //--------------------------------------------------------
  int         sokt;
  char*       serverIP;
  int         serverPort;

  serverIP   = argv[1];
  serverPort = atoi(argv[2]);

  struct  sockaddr_in serverAddr;
  socklen_t           addrLen = sizeof(struct sockaddr_in);

  if ((sokt = socket(PF_INET, SOCK_STREAM, 0)) < 0) {
    std::cerr << "socket() failed" << std::endl;
  }

  serverAddr.sin_family = PF_INET;
  serverAddr.sin_addr.s_addr = inet_addr(serverIP);
  serverAddr.sin_port = htons(serverPort);
  int ic = 0;
  while (connect(sokt, (sockaddr*)&serverAddr, addrLen) < 0) {
    std::cerr << "connect() failed! retrying: " << ic << std::endl;
    ic++;
    sleep(3);
  }



  //----------------------------------------------------------
  //OpenCV Code
  //----------------------------------------------------------

  //#color

  Mat img;

  if(USECOLOR)
    img = Mat::zeros(480 , 640, CV_8UC3);
  else
    img = Mat::zeros(480 , 640, CV_8UC1);

  int imgSize = img.total() * img.elemSize();
  uchar *iptr = img.data;
  int bytes = 0;
  int key;
    

  //make img continuos

  if ( ! img.isContinuous() ) { 
    img = img.clone();
  }
        
  std::cout << "Image Size:" << imgSize << std::endl;


  // namedWindow("CV Video Client",1);
  int debug = 0;
  while (key != 'q') {
    printf("\ndebug%d\n\n",debug++);

    if ((bytes = recv(sokt, iptr, imgSize , MSG_WAITALL)) == -1) {
      std::cerr << "recv failed, received bytes = " << bytes << std::endl;
    }
    printf("bytes rec: %d\n",bytes);
    printf("\ndebug%d\n\n",debug++);
    std::cout << img.data << std::endl;
    printf("\ndebug%d\n\n",debug++);
    cv::imshow("CVVidClient", img); 
      
    printf("\ndebug%d\n\n",debug++);
    if (key = cv::waitKey(10) >= 0) break;
  }   

  close(sokt);

  return 0;
}	

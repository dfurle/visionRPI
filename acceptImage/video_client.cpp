#include "opencv2/opencv.hpp"
#include <sys/socket.h> 
#include <arpa/inet.h>
#include <unistd.h>

bool USECOLOR = false;

int main(int argc, char* argv[]){

  if(argc > 2){
    std::string useColor = argv[argc-1];
    if(useColor == "-c")
      USECOLOR = true;
  } else if(argc == 2){
    printf("please use %s IP -c\n", argv[0]);
    return 1;
  }
  printf("USECOLOR: %d\n",USECOLOR);

  int sockfd;
  char* host;
  int port = 4097;

  host = argv[1];
  // port = atoi(argv[2]);

  struct sockaddr_in addr;
  socklen_t addrLen = sizeof(struct sockaddr_in);

  sockfd = socket(PF_INET, SOCK_STREAM, 0);

  addr.sin_family = PF_INET;
  addr.sin_addr.s_addr = inet_addr(host);
  addr.sin_port = htons(port);

  int ic = 0;
  while (connect(sockfd, (sockaddr*)&addr, addrLen) < 0) {
    std::cerr << "connect() failed! retrying: " << ic << std::endl;
    ic++;
    sleep(3);
  }

  cv::Mat img = cv::Mat::zeros(480 , 640, CV_8UC1);

  if(USECOLOR)
    img = cv::Mat::zeros(480 , 640, CV_8UC3);

  int imgSize = img.total() * img.elemSize();
  uchar *iptr = img.data;
  int bytes = 0;
  int key;

  if (!img.isContinuous())
    img = img.clone();
  
  std::cout << "Image Size:" << imgSize << std::endl;

  cv::namedWindow("Video Client");

  while(key != 'q') {
    if ((bytes = recv(sockfd, iptr, imgSize , MSG_WAITALL)) == -1) {
      std::cerr << "recv failed, received bytes = " << bytes << std::endl;
    }
    cv::imshow("Video Client", img);
    
    if (key = cv::waitKey(10) >= 0) break;
  }   

  close(sockfd);

  return 0;
}	

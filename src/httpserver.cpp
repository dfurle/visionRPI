#include "httpserver.h"

pthread_t stream_thread_t;
void* stream_thread(void* arg);

void* handleHttp_thread(void* arg){
  int ID = 0;
  bool interrupt = false;
  struct HOST host = TCPServer::create_socket(5050);
  pthread_create(&stream_thread_t, NULL, stream_thread, NULL);

  printf("listening\n");
  listen(host.sockfd, 5);

  Client client;
  client.ID = ID++;
  client.host = host;
  printf("handleConnection\n");
  client.handleConnection();
  return 0;
}

int Client::handleConnection(){
  const int MAXLINE = 10000;
  char from_client[MAXLINE];
  // int counter = 0;
  bool bIsFirstFrame = true;
  while (true) {
    // printf("waiting accept\n");
    if((socket = accept(host.sockfd, (sockaddr*)&host.addr, (socklen_t*)&host.addrlen)) < 0){
      printf("socket accept error\n");
    }
    // printf("accepted\n");
    // printf("ID: %d c: %d\n",ID,counter++);
    bzero(from_client, MAXLINE);
    read(socket, from_client, MAXLINE);

    // For Debugging
    // printf("---===RECEIVED===---\n");
    // printf("%s\n",from_client);

    handleRequest(std::string(from_client));
    // close socket?
  }
  return 0;
}

void Client::handleRequest(std::string req){
  std::string action;
  std::string hstr = str::substring(req,0,req.find("\r\n\r\n"));
  std::string cstr = str::substring(req,req.find("\r\n\r\n")+4);

  std::vector<std::string> header = str::split(hstr,"\n");

  action = str::split(header[0]," ")[0];
  // OR (less processing)
  // action = substring(header,0,header.find(' '));


  if(str::cmp(action,"GET")){
    // printf("---===RECEIVED===---\n");
    // std::cout << req << std::endl;
    if(handleGET(header)){
      close(socket);
    }
  } else if(str::cmp(action,"PUT")){
    std::vector<std::string> content = str::split(cstr,"\n");
    int contentLength = std::stoi(str::getParam(header,"Content-Length"));
    // printf("%d byte message\n",contentLength);
    if(content.size() == 0){
      // printf("reading again\n");
      // read again, contentLength bytes
      char from_client[contentLength];
      read(socket, from_client, contentLength);
      content = str::split(std::string(from_client),"\n");
    }
    handlePUT(header,content);
    close(socket);
  } else {
    printf("Incorrect HTTP format or misunderstood.\n");
  }
}

int Client::handleGET(std::vector<std::string> header){
  std::string req = str::split(header[0]," ")[1];
  int res;

  if(str::cmp(req,"/")){
    sendAll("text/html",getReqFile("../index.html"));
  } else if(str::cmp(req,"/favicon.ico")){
    sendAll("image/png",getReqFile("../img.png"));
  } else if(str::cmp(req,"/jquery.js")){
    sendAll("text/javascript",getReqFile("/home/pi/jquery.js"));
  } else if(str::cmp(req,"/initialData")){
    std::stringstream wss;
    wss << Var::rminR << ',' << Var::rmaxR << ',';
    wss << Var::rminG << ',' << Var::rmaxG << ',';
    wss << Var::rminB << ',' << Var::rmaxB << ',';
    wss << Var::bminR << ',' << Var::bmaxR << ',';
    wss << Var::bminG << ',' << Var::bmaxG << ',';
    wss << Var::bminB << ',' << Var::bmaxB << ',';
    if(Var::WIDTH == 1280){
      wss << "1" << ',';
    } else {
      wss << "0" << ',';
    }
    wss << Switches::DOPRINT << ',';
    wss << Switches::FRAME << ',';
    wss << Switches::DRAW << ',';
    wss << Switches::printTime;
    sendAll("text/plain",str::ss_to_vec(wss));
  } else if(str::contains(req,"/video_stream")){
    std::stringstream ssheader;
    ssheader << "HTTP/1.1 200 OK\r\n" ;
    ssheader << "Connection: keep-alive\r\n";
    // ssheader << "Connection: close\r\n";
    // ssheader << "Max-Age: 0\r\n";
    // ssheader << "Expires: 0\r\n";
    ssheader << "Cache-Control: no-cache, private\r\n";
    ssheader << "Pragma: no-cache\r\n";
    ssheader << "Content-Type: multipart/x-mixed-replace; boundary=--frame\r\n\r\n";
    res = sendData(str::ss_to_vec(ssheader));
    // printf("---===SENT===---\n");
    // printf("%s",header.data());
  
    if(str::cmp(req,"/video_stream1")){
      printf("VIDEO STREAM ONE PUSH %d\n",socket);
      Global::imgSocket.push_back(socket);
    }
    if(str::cmp(req,"/video_stream2")){
      printf("VIDEO STREAM TWO PUSH %d\n",socket);
      Global::thrRSocket.push_back(socket);
    }
    if(str::cmp(req,"/video_stream3")){
      printf("VIDEO STREAM THREE PUSH %d\n",socket);
      Global::thrBSocket.push_back(socket);
    }
  }
  if(str::contains(req,"/video_stream")){
    return 0;
  } else {
    return 1;
  }
}

void Client::handlePUT(std::vector<std::string> header, std::vector<std::string> content){
  int vals[12];
  double dvals[5];
  sendAll("text/plain",std::vector<char>(),true);
  if(content.size() == 0){
    return;
  }
  std::vector<std::string> rgb_vals = str::split(content[0],",");
  std::vector<std::string> tvec_vals = str::split(content[1],",");
  std::vector<std::string> rvec_vals = str::split(content[2],",");
  std::vector<std::string> strswitches = str::split(content[4],",");
  std::vector<bool> switches;
  for(int i = 0; i < 12; i++){
    vals[i] = std::stoi(rgb_vals[i]);
  }
  for(std::string s : strswitches){
    switches.push_back(std::stoi(s));
  }

  Global::muteHTTP.lock();
  int resval = switches[1];
  Switches::DOPRINT = switches[2];
  Switches::FRAME = switches[3];
  Switches::DRAW = switches[4];
  Switches::printTime = std::stoi(content[3]);
  printf("Switches print: %d\n",Switches::printTime);
  Var::rminR = vals[0];
  Var::rmaxR = vals[1];
  Var::rminG = vals[2];
  Var::rmaxG = vals[3];
  Var::rminB = vals[4];
  Var::rmaxB = vals[5];

  Var::bminR = vals[6];
  Var::bmaxR = vals[7];
  Var::bminG = vals[8];
  Var::bmaxG = vals[9];
  Var::bminB = vals[10];
  Var::bmaxB = vals[11];
  if(resval){
    Var::WIDTH = 1280;
    Var::HEIGHT = 720;
  } else {
    Var::WIDTH = 640;
    Var::HEIGHT = 480;
  }
  Global::muteHTTP.unlock();
}

std::vector<char> Client::getReqFile(std::string fname){
  std::ifstream file;
  file.open(fname,std::ios::binary | std::ios::in);
  std::vector<char> data;
  if(file){
    std::streampos size = file.tellg();
    file.seekg(0, std::ios::end);
    size = file.tellg() - size;
    char* memblock = new char[size];
    file.seekg(0, std::ios::beg);
    file.read(memblock, size);
    data = std::vector<char>(memblock,memblock+size);
    delete[] memblock;
  }
  file.close();
  return data;
}

int Client::sendData(std::vector<char> data){
  char *pdata = (char *)data.data();
  int length = data.size();
  int numSent;
  while (length > 0) {
    numSent = send(socket, pdata, length, MSG_NOSIGNAL);
    if (numSent == -1) return -1;
    pdata += numSent;
    length -= numSent;
  }
  return 0;
}

void Client::sendAll(std::string mimetype, std::vector<char> data, bool silence){
  if(!silence)
    printf("---===SEND(BEG)===---\n");
  std::stringstream wsss;
  wsss << "HTTP/1.1 200 OK\r\n"
       //<< "Connection: keep-alive\r\n"
       << "Cache-Control: no-cache, private\r\n"
       << "Pragma: no-cache\r\n"
       << "Content-Type: " << mimetype << "\r\n"
       << "Content-Length: " << data.size() << "\r\n";
  // for(std::string s : additional)
  //   wsss << s << "\r\n";
  wsss << "\r\n";
  std::vector<char> header = str::ss_to_vec(wsss);
  if(!silence)
    printf("%s",header.data());
  int res = sendData(header);
  if(data.size() != 0)
    res = sendData(data);
  if(!silence)
    printf("---===SEND(END)===---\n");
}

int g_sendData(std::vector<char> data, int socket){
  char *pdata = (char *)data.data();
  int length = data.size();
  int numSent;
  while (length > 0) {
    numSent = send(socket, pdata, length, MSG_NOSIGNAL);
    if (numSent == -1) return -1;
    pdata += numSent;
    length -= numSent;
  }
  return 0;
}
int g_sendData(std::vector<uchar> data, int socket){
  char *pdata = (char *)data.data();
  int length = data.size();
  int numSent;
  while (length > 0) {
    numSent = send(socket, pdata, length, MSG_NOSIGNAL);
    if (numSent == -1) return -1;
    pdata += numSent;
    length -= numSent;
  }
  return 0;
}

int sendIMG(std::vector<uchar>& imgBuf, int socket){
  std::stringstream ssheader;
  int res = 0;

  ssheader.str("");
  ssheader.clear();
  ssheader << "\r\n\r\n--frame\r\n";
  ssheader << "Content-Type: " << "image/jpg" << "\r\n";
  ssheader << "Content-Length: " << imgBuf.size() << "\r\n\r\n";
  std::vector<char> header = str::ss_to_vec(ssheader);
  res = g_sendData(header,socket);
  if(res == -1) return -1;
  res = g_sendData(imgBuf, socket);
  if(res == -1) return -1;
  return 0;
}

void* stream_thread(void* arg){
  std::vector<uchar> imgBuffer;
  std::vector<uchar> threshRBuffer;
  std::vector<uchar> threshBBuffer;
  std::vector<uchar> rPosBuffer;
  while(1){
    if(Global::imgC.empty()){
      usleep(33000);
      continue;
    }
    Global::muteImg.lock();
    Global::httpStatus = 1;
    Global::muteImg.unlock();

    cv::imencode(".jpeg", Global::imgC,imgBuffer);
    cv::imencode(".jpeg", Global::thresholdedRC,threshRBuffer);
    cv::imencode(".jpeg", Global::thresholdedBC,threshBBuffer);

    Global::muteImg.lock();
    Global::httpStatus = 0;
    Global::muteImg.unlock();

    int res = 0;
    
    if(!Global::imgSocket.empty() && !Global::thrRSocket.empty() && !Global::thrBSocket.empty()){
      for(auto it = Global::imgSocket.begin(); it != Global::imgSocket.end();){
        if(sendIMG(imgBuffer,*it) == -1) {
          close(*it);
          Global::imgSocket.erase(it);
        } else
          it++;
      }
      for(auto it = Global::thrRSocket.begin(); it != Global::thrRSocket.end();){
        if(sendIMG(threshRBuffer,*it) == -1){
          close(*it);
          Global::thrRSocket.erase(it);
        } else
          it++;
      }
      for(auto it = Global::thrBSocket.begin(); it != Global::thrBSocket.end();){
        if(sendIMG(threshBBuffer,*it) == -1){
          close(*it);
          Global::thrBSocket.erase(it);
        } else
          it++;
      }
    }
    usleep(100000);
  }
}

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
    printf("---===RECEIVED===---\n");
    std::cout << req << std::endl;
    if(handleGET(header)){
      close(socket);
    }
  } else if(str::cmp(action,"PUT")){
    std::vector<std::string> content = str::split(cstr,"\n");
    for(std::string s : content){
      std::cout << s << std::endl;
    }
    int contentLength = std::stoi(str::getParam(header,"Content-Length"));
    // printf("%d byte message\n",contentLength);
    if(content.size() == 0){
      // printf("reading again\n");
      // read again, contentLength bytes
      char from_client[1000];
      read(socket, from_client, 1000);
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
    wss << Var::minR << ',' << Var::maxR << ',';
    wss << Var::minG << ',' << Var::maxG << ',';
    wss << Var::minB << ',' << Var::maxB << ',';
    for(int i = 0; i < 5; i++){
      wss << Var::dist_cof[i] << ',';
    }
    sendAll("text/plain",str::ss_to_vec(wss));
  } else if(str::contains(req,"/video_stream")){
    std::stringstream ssheader;
    ssheader << "HTTP/1.1 200 OK\r\n" ;
    ssheader << "Connection: keep-alive\r\n";
    //ssheader << "Connection: close\r\n";
    ssheader << "Max-Age: 0\r\n";
    ssheader << "Expires: 0\r\n";
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
      Global::thrSocket.push_back(socket);
    }
  }
  if(str::contains(req,"/video_stream")){
    return 0;
  } else {
    return 1;
  }
}

void Client::handlePUT(std::vector<std::string> header, std::vector<std::string> content){
  int vals[6];
  double dvals[5];
  sendAll("text/plain",std::vector<char>(),true);
  if(content.size() != 2){
    return;
  }
  std::vector<std::string> rgb_vals = str::split(content[0],",");
  std::vector<std::string> dco_vals = str::split(content[1],",");
  for(int i = 0; i < 6; i++){
    vals[i] = std::stoi(rgb_vals[i]);
  }
  for(int i = 0; i < 5; i++){
    dvals[i] = std::stod(dco_vals[i]);
  }
  Var::minR = vals[0];
  Var::maxR = vals[1];
  Var::minG = vals[2];
  Var::maxG = vals[3];
  Var::minB = vals[4];
  Var::maxB = vals[5];
  Var::dist_cof[0] = dvals[0];
  Var::dist_cof[1] = dvals[1];
  Var::dist_cof[2] = dvals[2];
  Var::dist_cof[3] = dvals[3];
  Var::dist_cof[4] = dvals[4];
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
    numSent = send(socket, pdata, length, 0);
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
    numSent = send(socket, pdata, length, 0);
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
    numSent = send(socket, pdata, length, 0);
    if (numSent == -1) return -1;
    pdata += numSent;
    length -= numSent;
  }
  return 0;
}

void sendIMG(std::vector<uchar>& imgBuf, int socket){
  std::stringstream ssheader;
  int res = 0;

  ssheader.str("");
  ssheader.clear();
  ssheader << "--frame\r\n";
  ssheader << "Content-Type: " << "image/jpg" << "\r\n";
  ssheader << "Content-Length: " << imgBuf.size() << "\r\n\r\n";
  std::vector<char> header = str::ss_to_vec(ssheader);
  res = g_sendData(header,socket);

  res = g_sendData(imgBuf, socket);
  ssheader.str("");
  ssheader.clear();
  ssheader << "\r\n\r\n";
  std::vector<char> footer = str::ss_to_vec(ssheader);
  res = g_sendData(footer, socket);
}

void* stream_thread(void* arg){
  std::vector<uchar> imgBuffer;
  std::vector<uchar> threshBuffer;
  while(1){
    if(Global::imgC.empty()){
      usleep(33000);
      continue;
    }
    Global::muteImg.lock();
    Global::httpStatus = 1;
    Global::muteImg.unlock();

    cv::imencode(".jpeg", Global::imgC,imgBuffer);
    cv::imencode(".jpeg", Global::thresholdedC,threshBuffer);

    Global::muteImg.lock();
    Global::httpStatus = 0;
    Global::muteImg.unlock();
    
    if(!Global::imgSocket.empty()){
      for(int s : Global::imgSocket){
        sendIMG(imgBuffer,s);
      }
    }
    if(!Global::thrSocket.empty()){
      for(int s : Global::thrSocket){
        sendIMG(threshBuffer,s);
      }
    }
    usleep(100000);
  }
}

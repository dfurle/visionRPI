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


std::string Client::requestedFile(std::string str){
  std::string HTTPHeader = str.substr(0,str.find('\n'));
  int locDivider = str.find(' ');
  std::string action = HTTPHeader.substr(0,locDivider);
  std::string file = HTTPHeader.substr(0,str.find(' ',locDivider+1)).substr(locDivider+1);
  // printf("action: |%s|\n",action.c_str());
  // printf("file  : |%s|\n",file.c_str());
  if(action.compare("GET") == 0){
    return file;
  } else {
    return "";
  }
}

std::vector<char> Client::getReqFile(std::string fname, bool binary){
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

/**
 * Sends the header:
 * 
 *  HTTP/1.1 200 OK
 *  Cache-Control: no-cache, private
 *  Pragma: no-cache
 *  Content-Type: mimetype
 *  Content-Length: size
 * 
 * 
 * @param mimetype type of data to be sent, ex: "text/plain" or "image/png"
 * @param size size of the data to be sent
 * @param additional vector of strings, any additional headers added
 **/
int Client::sendHeader(std::string mimetype, int size, std::vector<std::string> additional){
  std::stringstream wsss;
  wsss << "HTTP/1.1 200 OK\r\n"
       //<< "Connection: keep-alive\r\n"
       << "Cache-Control: no-cache, private\r\n"
       << "Pragma: no-cache\r\n"
       << "Content-Type: " << mimetype << "\r\n"
       << "Content-Length: " << size << "\r\n";
  for(std::string s : additional)
    wsss << s << "\r\n";
  wsss << "\r\n";
  std::string headers = wsss.str();
  printf("%s",headers.c_str());
  return sendData((void*)headers.data(), headers.size());
}

int Client::sendData(void* data, int length){
  char *pdata = (char *) data;
  int numSent;
  while (length > 0) {
    numSent = send(socket, pdata, length, 0);
    if (numSent == -1) return -1;
    pdata += numSent;
    length -= numSent;
  }
  printf("%s\n",pdata);
  return 0;
}

void Client::sendAll(std::string mimetype, std::vector<char> data){
  printf("---===SEND(BEG)===---\n");
  int res = sendHeader(mimetype,data.size());
  if(data.size() != 0)
    res = sendData((void*)data.data(), data.size());
  printf("---===SEND(END)===---\n");
}

/**
 * Returns substring between [i,f)
 */
std::string substring(std::string s, int i, int f){
  return s.substr(i,f-i);
}

void Client::readRequest(std::string req){
  int wpos = req.find("PUT");
  if(wpos != std::string::npos){
    int dataPos = req.find("\r\n\r\n");
    std::string data;
    int vals[6];
    double dvals[5];
    if(dataPos+4 == req.length()){
      printf("data was not yet sent\n");
      char from_client[1024];
      bzero(from_client, 1024);
      read(socket, from_client, 1024);
      data = std::string(from_client);
    } else {
      printf("data was already sent\n");
      data = substring(req,dataPos+5,req.length()-1);
    }
    sendAll("text/plain",std::vector<char>());

    std::cout << "|" << data << "|" << std::endl;
    int prevPos = 1;
    for(int i = 0; i < 6; i++){
      int pos = data.find(',',prevPos);
      std::string substrStr = substring(data,prevPos,pos);
      printf("%d:%s\n",i,substrStr.c_str());
      vals[i] = std::atoi(substrStr.c_str());
      prevPos = pos+1;
    }
    for(int i = 0; i < 5; i++){
      int pos = data.find(',',prevPos);
      std::string substrStr = substring(data,prevPos,pos);
      dvals[i] = std::stod(substrStr);
      prevPos = pos+1;
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
}


int Client::handleConnection(){
  const int MAXLINE = 10000;
  char from_client[MAXLINE];
  int counter = 0;
  bool bIsFirstFrame = true;
  while (true) {
    printf("waiting accept\n");
    if((socket = accept(host.sockfd, (sockaddr*)&host.addr, (socklen_t*)&host.addrlen)) < 0){
      printf("socket accept error\n");
    }
    printf("accepted\n");
    printf("ID: %d c: %d\n",ID,counter++);
    bzero(from_client, MAXLINE);
    read(socket, from_client, MAXLINE);
    printf("---===RECEIVED===---\n");
    printf("%s\n",from_client);
    readRequest(std::string(from_client));

    std::string req = requestedFile(std::string(from_client));
    int res;

    if(req.compare("/") == 0){
      std::vector<char> data = getReqFile("../index.html");
      sendAll("text/html",data);
    } else if(req.compare("/favicon.ico") == 0){
      std::vector<char> data = getReqFile("../img.png",true);
      sendAll("image/png",data);
    } else if(req.compare("/jquery.js") == 0){
      std::vector<char> data = getReqFile("/home/pi/jquery.js");
      sendAll("text/javascript",data);
    } else if(req.compare("/initialData") == 0){
      std::stringstream wss;
      wss << Var::minR << ',';
      wss << Var::maxR << ',';
      wss << Var::minG << ',';
      wss << Var::maxG << ',';
      wss << Var::minB << ',';
      wss << Var::maxB << ',';
      wss << Var::dist_cof[0] << ',';
      wss << Var::dist_cof[1] << ',';
      wss << Var::dist_cof[2] << ',';
      wss << Var::dist_cof[3] << ',';
      wss << Var::dist_cof[4];
      sendHeader("text/plain",wss.str().length());
      int res = sendData((void*)wss.str().data(), wss.str().length());

    } else if(req.compare("/video_stream1") == 0 || req.compare("/video_stream2") == 0){
      std::string header;
      std::stringstream ssheader;
      ssheader << "HTTP/1.1 200 OK\r\n" ;
      ssheader << "Connection: keep-alive\r\n";
      //ssheader << "Connection: close\r\n";
      ssheader << "Max-Age: 0\r\n";
      ssheader << "Expires: 0\r\n";
      ssheader << "Cache-Control: no-cache, private\r\n";
      ssheader << "Pragma: no-cache\r\n";
      ssheader << "Content-Type: multipart/x-mixed-replace; boundary=--frame\r\n\r\n";
      header = ssheader.str();

      res = sendData((void*)header.data(), header.length());
      printf("---===SENT===---\n");
      printf("%s",header.c_str());
      printf("success: %d\n",res);
    
      if(req.compare("/video_stream1") == 0){
        printf("VIDEO STREAM ONE PUSH %d\n",socket);
        Global::imgSocket.push_back(socket);
      }
      if(req.compare("/video_stream2") == 0){
        printf("VIDEO STREAM TWO PUSH %d\n",socket);
        Global::thrSocket.push_back(socket);
      }
    }

    if(req.compare("/video_stream1") == 0 || req.compare("/video_stream2") == 0){
      
    } else {
      close(socket);
    }
  }
  return 0;
}


int g_sendData(void* data, int length, int socket){
  char *pdata = (char *) data;
  int numSent;
  while (length > 0) {
    numSent = send(socket, pdata, length, 0);
    if (numSent == -1) return -1;
    pdata += numSent;
    length -= numSent;
  }
  return 0;
}


void* stream_thread(void* arg){
  std::stringstream ssheader;
  std::string header;
  int res = 0;
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
        try{
          ssheader.str("");
          ssheader.clear();
          ssheader << "--frame\r\n";
          ssheader << "Content-Type: " << "image/jpg" << "\r\n";
          ssheader << "Content-Length: " << imgBuffer.size() << "\r\n\r\n";
          header = ssheader.str();
          res = g_sendData((void*)header.data(), header.length(),s);

          res = g_sendData((void*)imgBuffer.data(), imgBuffer.size(),s);
          ssheader.str("");
          ssheader.clear();
          ssheader << "\r\n\r\n";
          res = g_sendData((void*)header.data(), header.length(),s);
        } catch(const std::exception& e) {
          std::cout << "ERROR" << e.what() << std::endl;
        } catch(...){
          std::cout << "some other error" << std::endl;
        }
      }
    }
    if(!Global::thrSocket.empty()){
      for(int s : Global::thrSocket){
        try{
          ssheader.str("");
          ssheader.clear();
          ssheader << "--frame\r\n";
          ssheader << "Content-Type: " << "image/jpg" << "\r\n";
          ssheader << "Content-Length: " << threshBuffer.size() << "\r\n\r\n";
          header = ssheader.str();
          res = g_sendData((void*)header.data(), header.length(),s);

          res = g_sendData((void*)threshBuffer.data(), threshBuffer.size(),s);
          ssheader.str("");
          ssheader.clear();
          ssheader << "\r\n\r\n";
          res = g_sendData((void*)header.data(), header.length(),s);
        } catch(const std::exception& e) {
          std::cout << "ERROR" << e.what() << std::endl;
        } catch(...){
          std::cout << "some other error" << std::endl;
        }
      }
    }

    usleep(100000);
  }
}

void* client_thread(void* arg) {
  struct CLIENT* client = (CLIENT*)arg;

  int socket = client->socket;

  const int MAXLINE = 32; // random value for buffer just in case?
  char from_client[MAXLINE];
  const int MLEN = 1024;
  char to_client[MLEN];
  while (true) {
    // Read from client
    bzero(from_client, MAXLINE);
    int lenbuf = read(socket, from_client, sizeof(from_client));
    // bzero(to_client, MLEN);

    
    // sprintf(to_client, "%.2f,%.2f,%d\n", Global::position->dist, Global::position->robotAngle, Global::dataValid);
    // mutex are most likely required, test performance
    Global::mutePos.lock();
    snprintf(to_client, sizeof(to_client), "%.2f,%.2f,%d\n", Global::position.dist, Global::position.robotAngle, Global::dataValid);
    Global::mutePos.unlock();
    std::string send_to = to_client;

    // int bytesSent = send(socket, to_client, sizeof(to_client), MSG_NOSIGNAL);
    int bytesSent = send(socket, send_to.c_str(), sizeof(send_to), MSG_NOSIGNAL);
    if(bytesSent < 0){
      printf("thread: %d failed\n",client->ID);
      close(socket);
      return 0;
    }
    // printf("message sent: %s",to_client);
    // printf("Bytes sent: %d/%d, but only needed %d\n",bytesSent,sizeof(to_client), strlen(to_client));
    usleep(100);
  }
  close(client->socket);
  return 0;
}

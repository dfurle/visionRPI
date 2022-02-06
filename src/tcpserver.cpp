#include "tcpserver.h"

#define HOSTNAMELENGTH 128
#define MAXCLIENTS 32
#define RIO_ID 5957 // 1220

bool interrupt = false;

pthread_t client_thread_t[MAXCLIENTS];
void* client_thread(void* arg);

HOST TCPServer::create_socket(int port){
  struct HOST host;
  host.port = port;
  host.sockfd = socket(AF_INET, SOCK_STREAM,0);

  // struct sockaddr_in addr;
  host.addr.sin_family = AF_INET;
  host.addr.sin_addr.s_addr = htonl(INADDR_ANY);
  host.addr.sin_port = htons(host.port);
  host.addrlen = sizeof(host.addr);

  int on = 1;
  setsockopt(host.sockfd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
  bind(host.sockfd, (sockaddr*)&host.addr, host.addrlen);
  printf("socket created: %d\n", host.sockfd);
  return host;
}

int TCPServer::getClient(HOST &host){
  listen(host.sockfd, 5);
  return accept(host.sockfd, (sockaddr*)&host.addr, (socklen_t*)&host.addrlen);
}


void* opentcp(void* arg) {
  int ID = 0;
  struct HOST host = TCPServer::create_socket(RIO_ID);
  while(!interrupt){
    int sd_client = TCPServer::getClient(host);
    if (ID >= MAXCLIENTS)
      continue;
    struct CLIENT client;
    client.ID = ID++;
    client.socket = sd_client;
    printf("connecting to client now\n");
    int ok = pthread_create(&client_thread_t[ID], 0, client_thread, &client);
    // int rc = pthread_setname_np(client_thread_t[ID], "client_thread");
    ok = pthread_detach(client_thread_t[ID]);
    if (ok != 0)
      printf("clientThread error\n");
    usleep(1000);
  }
  return 0;
}


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

int Client::sendData(void* data, int length){
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

void Client::sendAll(std::string mimetype, std::vector<char> data, int flag){
  std::stringstream wsss;
  wsss << "HTTP/1.1 200 OK\r\n"
       //<< "Connection: keep-alive\r\n"
       << "Cache-Control: no-cache, private\r\n"
       << "Pragma: no-cache\r\n"
       << "Content-Type: " << mimetype << "\r\n"
       << "Content-Length: " << data.size() << "\r\n"
       << "\r\n";
  std::string headers = wsss.str();
  int res = sendData((void*)headers.data(), headers.size());
  res = sendData((void*)data.data(), data.size());

  printf("---===SEND(BEG)===---\n");
  printf("%s%s\n",headers.c_str(),data.data());
  printf("---===SEND(END)===---\n");
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
    std::string s(from_client);
    int wpos = s.find("PUT");
    if(wpos != std::string::npos){
      int vals[6];
      sendAll("text/plain",std::vector<char>());
      int dataPos = s.find("\r\n\r\n");
      dataPos += 5;
      std::string data = s.substr(dataPos);
      data = data.substr(0,data.length()-1);
      // std::cout << "|" << data << "|" << std::endl;
      int prevPos = 0;
      for(int i = 0; i < 6; i++){
        int pos = data.find(',',prevPos);
        std::string substrStr = data.substr(prevPos,pos-prevPos);
        vals[i] = std::atoi(substrStr.c_str());
        prevPos = pos+1;
      }
      for(int i = 0; i < 6; i++){
        printf("val: %d is %d\n",i,vals[i]);
      }
      Var::minR = vals[0];
      Var::maxR = vals[1];
      Var::minG = vals[2];
      Var::maxG = vals[3];
      Var::minB = vals[4];
      Var::maxB = vals[5];
    }

    std::string req = requestedFile(std::string(from_client));
    int res;

    if(req.compare("/") == 0){
      std::vector<char> data = getReqFile("../index.html");
      sendAll("text/html",data);
    } else if(req.compare("/favicon.ico") == 0){
      std::vector<char> data = getReqFile("../img.png",true);
      sendAll("image/png",data);
    } else if(req.compare("/initialData") == 0){
      // std::vector<char> data = getReqFile("../img.png",true);
      std::stringstream wss;
      wss << Var::minR << ',';
      wss << Var::maxR << ',';
      wss << Var::minG << ',';
      wss << Var::maxG << ',';
      wss << Var::minB << ',';
      wss << Var::maxB;
      std::stringstream wsss;
      wsss << "HTTP/1.1 200 OK\r\n"
          << "Cache-Control: no-cache, private\r\n"
          << "Pragma: no-cache\r\n"
          << "Content-Type: " << "text/plain" << "\r\n"
          << "Content-Length: " << wss.str().length() << "\r\n"
          << "\r\n";
      std::string headers = wsss.str();
      int res = sendData((void*)headers.data(), headers.size());
      res = sendData((void*)wss.str().data(), wss.str().length());
    } else if(req.compare("/video_stream1") == 0){
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
    
      Global::sockets[0] = socket; 
    } else if(req.compare("/video_stream2") == 0){
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
    
      Global::sockets[1] = socket;
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
  while(1){
    Global::muteImg.lock();
    if(Global::sockets[0] != -1){
      ssheader.str("");
      ssheader.clear();
      ssheader << "--frame\r\n";
      ssheader << "Content-Type: " << "image/jpg" << "\r\n";
      ssheader << "Content-Length: " << Global::imgBuffer.size() << "\r\n\r\n";
      header = ssheader.str();
      res = g_sendData((void*)header.data(), header.length(),Global::sockets[0]);

      res = g_sendData((void*)Global::imgBuffer.data(), Global::imgBuffer.size(),Global::sockets[0]);
      ssheader.str("");
      ssheader.clear();
      ssheader << "\r\n\r\n";
      res = g_sendData((void*)header.data(), header.length(),Global::sockets[0]);

      // printf("---===VideoStreamSent===---\n");
      // std::cout << header << "<frame here>" << "\r\n\r\n";
      // printf("---===VideoStreamEnd===---\n");
    }
    if(Global::sockets[1] != -1){
      ssheader.str("");
      ssheader.clear();
      ssheader << "--frame\r\n";
      ssheader << "Content-Type: " << "image/jpg" << "\r\n";
      ssheader << "Content-Length: " << Global::threshBuffer.size() << "\r\n\r\n";
      header = ssheader.str();
      res = g_sendData((void*)header.data(), header.length(),Global::sockets[1]);

      res = g_sendData((void*)Global::threshBuffer.data(), Global::threshBuffer.size(),Global::sockets[1]);
      ssheader.str("");
      ssheader.clear();
      ssheader << "\r\n\r\n";
      res = g_sendData((void*)header.data(), header.length(),Global::sockets[1]);

      // printf("---===VideoStreamSent===---\n");
      // std::cout << header << "<frame here>" << "\r\n\r\n";
      // printf("---===VideoStreamEnd===---\n");
    }
    Global::muteImg.unlock();

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

void* videoServer(void* arg) {
  struct HOST host = TCPServer::create_socket(Var::videoPort);
  while(true){
    Global::videoSocket = TCPServer::getClient(host);
    printf("video socket: %d\n", Global::videoSocket);
    while (!Global::videoError)
      sleep(1);
    close(Global::videoSocket);
    Global::videoSocket = 0;
    Global::videoError = false;
    sleep(5);
  }
  printf("error videoServer\n");
  return 0;
}

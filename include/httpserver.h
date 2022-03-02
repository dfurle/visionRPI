#pragma once
#include "variables.h"
#include "tcpserver.h"

class Client{
public:
  int ID;
  int socket;
  HOST host;

  Client(){};
  int handleConnection();

  std::vector<char> getReqFile(std::string file);
  int sendData(std::vector<char> data);
  void sendAll(std::string mimetype, std::vector<char> data, bool silence = true);

  void handleRequest(std::string req);
  int  handleGET(std::vector<std::string> header);
  void handlePUT(std::vector<std::string> header, std::vector<std::string> content);
};

int g_sendData(void* data, int length, int socket);
int sendIMG(std::vector<uchar>& imgBuf, int socket);

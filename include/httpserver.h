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
  std::string requestedFile(std::string str);
  void readRequest(std::string req);
  std::vector<char> getReqFile(std::string file, bool binary = false);
  int sendHeader(std::string mimetype, int size, std::vector<std::string> additional = {});
  int sendData(void* data, int length);
  void sendAll(std::string mimetype, std::vector<char> data);
};
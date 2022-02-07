#pragma once
#include "variables.h"
// #include <netdb.h>

struct HOST{
  int port;
  int sockfd;
  struct sockaddr_in addr;
  int addrlen;
};
struct CLIENT {
  int ID;
  int socket;
  // Position* pos;
};

class TCPServer {
public:
  TCPServer() {}
  static HOST create_socket(int port);
  static int getClient(HOST &host);
};

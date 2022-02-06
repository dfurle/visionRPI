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


class Client{
public:
  int ID;
  int socket;
  HOST host;

  Client(){};
  int handleConnection();
  std::string requestedFile(std::string str);
  std::vector<char> getReqFile(std::string file, bool binary = false);
  std::string sendHeader(std::string mimetype, int length, int flag = 0);
  int sendData(void* data, int length);
  void sendAll(std::string mimetype, std::vector<char> data, int flag = 0);

};
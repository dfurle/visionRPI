#include "variables.h"
#include <netdb.h>
#include <sys/socket.h>

class TCPServer {
public:
  TCPServer() {}
  // static int tcp_open_th(unsigned int PORT,char* hostname);
  static int open_local_th(int PORT);
  // static int open_remote_th(int PORT,char* hostname);
  static int tcp_send_th(int sockfd, int* DATA, int lenDATA);
  // static int tcp_get_th(int sd_current,int *DATA,int lenDATA); // may be needed some time. dont delete
  static int tcp_listen3(int sd, char* host_name, int len, int* sd_current);
  // static int tcp_listen_th(int sd);
};

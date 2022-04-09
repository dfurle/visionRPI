#include "tcpserver.h"

#define HOSTNAMELENGTH 128
#define MAXCLIENTS 128
#define RIO_ID 5800

bool interrupt = false;

std::thread* client_thread_t[MAXCLIENTS];
void client_thread(CLIENT* client);

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


void opentcp() {
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
    client_thread_t[ID] = new std::thread(&client_thread,&client);
    client_thread_t[ID]->detach();
    usleep(1000);
  }
}

void client_thread(CLIENT* client) {
  int socket = client->socket;

  const int MAXLINE = 32; // random value for buffer just in case?
  char from_client[MAXLINE];
  while (true) {
    // Read from client
    bzero(from_client, MAXLINE);
    int lenbuf = read(socket, from_client, sizeof(from_client));

    // Generate data string
    char to_client[64];
    Global::mutePos.lock();
    snprintf(to_client, 64, "%.2f,%.2f,%d\n", Global::position.dist, Global::position.robotAngle, Global::dataValid);
    Global::dataValid = 0;
    Global::mutePos.unlock();
    std::string send_to = to_client;

    // Send string
    int bytesSent = send(socket, send_to.c_str(), send_to.length(), MSG_NOSIGNAL);
    if(bytesSent < 0){
      printf("thread: %d failed\n",client->ID);
      close(socket);
      return;
    }
    usleep(100);
  }
  close(socket);
}
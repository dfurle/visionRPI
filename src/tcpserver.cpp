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

// void* videoServer(void* arg) {
//   struct HOST host = TCPServer::create_socket(Var::videoPort);
//   while(true){
//     Global::videoSocket = TCPServer::getClient(host);
//     printf("video socket: %d\n", Global::videoSocket);
//     while (!Global::videoError)
//       sleep(1);
//     close(Global::videoSocket);
//     Global::videoSocket = 0;
//     Global::videoError = false;
//     sleep(5);
//   }
//   printf("error videoServer\n");
//   return 0;
// }

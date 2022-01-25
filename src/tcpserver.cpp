#include "tcpserver.h"

#define HOSTNAMELENGTH 128
#define MAXCLIENTS 32
#define RIO_ID 5957 // 1220

bool interrupt = false;

pthread_t client_thread_t[MAXCLIENTS];
static void* client_thread(void* arg);

HOST TCPServer::create_socket(int port){
  struct HOST host;
  host.port = port;
  host.sockfd = socket(AF_INET, SOCK_STREAM,0);

  // struct sockaddr_in addr;
  host.addr.sin_family = AF_INET;
  host.addr.sin_addr.s_addr = htonl(INADDR_ANY);
  host.addr.sin_port = htons(host.port);

  int on = 1;
  setsockopt(host.sockfd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
  bind(host.sockfd, (sockaddr*)&host.addr, sizeof(host.addr));
  printf("socket created: %d\n", host.sockfd);
  return host;
}

int TCPServer::getClient(HOST &host){
  listen(host.sockfd, 5);
  return accept(host.sockfd, (sockaddr*)&host.addr, (socklen_t*)&host.addr);
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

static void* client_thread(void* arg) {
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

    
    // sprintf(to_client, "%.2f,%.2f,%d\n", pos->dist, pos->robotAngle, pos->dataValid);
    // sprintf(to_client, "%.2f,%.2f,%d\n", Global::position->dist, Global::position->robotAngle, Global::position->dataValid);
    // mutex are most likely required, test performance
    Global::mutePos.lock();
    snprintf(to_client, sizeof(to_client), "%.2f,%.2f,%d\n", Global::position.dist, Global::position.robotAngle, Global::position.dataValid);
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

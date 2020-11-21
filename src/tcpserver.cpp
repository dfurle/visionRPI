#include "tcpserver.h"

#define MAXLINE 80 // check
#define MLEN 8192  // check
#define HOSTNAMELENGTH 128
#define MAXCLIENTS 32

bool interrupt = false;

pthread_t client_thread[MAXCLIENTS];
static void* run_client_thread(void* arg);

struct HOST {
  int PORT;
  char NAME[HOSTNAMELENGTH];
};
struct CLIENT {
  int ID;
  int tcpPort;
  Position* pos;
};

/*
int TCPServer::tcp_open_th(unsigned int PORT,char* hostname){ // required
  int sd;
  if (PORT<0) {
    PORT=-PORT;
    sd=open_remote_th(PORT,hostname);
  }else
  sd=open_local_th(PORT);
  return sd;
}
*/
/*--------------------------------------------------------------------*/
int TCPServer::open_local_th(int PORT) {
  struct sockaddr_in sinn;
  int on = 1;
  int sockfd;
  if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    return -1;
  if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) < 0)
    perror("setsockopt(SO_REUSEADDR) failed");
  memset(&sinn, 0, sizeof(sinn));
  sinn.sin_family = AF_INET;
  sinn.sin_addr.s_addr = htonl(INADDR_ANY);
  sinn.sin_port = htons(PORT);
  if (bind(sockfd, (struct sockaddr*)&sinn, sizeof(sinn)) == -1)
    return -1;
  return sockfd;
}
/*--------------------------------------------------------------------*/
/*
int TCPServer::open_remote_th(int PORT,char* hostname){
  struct sockaddr_in pin;
  struct hostent* hp;
  int sd_current;
  if ((hp = gethostbyname(hostname)) == 0)
    return -1;
  memset(&pin, 0, sizeof(pin));
  pin.sin_family = AF_INET;
  pin.sin_addr.s_addr = ((struct in_addr *)(hp->h_addr))->s_addr;
  pin.sin_port = htons(PORT);
  if ((sd_current = socket(AF_INET, SOCK_STREAM, 0)) == -1 )
    return -1;
  if (connect(sd_current,(struct sockaddr *)  &pin, sizeof(pin)) == -1)
    return -1;
  return sd_current;
}
*/
/*====================================================================*/
int TCPServer::tcp_send_th(int sockfd, int* DATA, int lenDATA) {
  // whats the point of turning DATA into int*
  int bytesSent;
  char* buf;
  buf = (char*)DATA;
  while (lenDATA > 0) {
    bytesSent = send(sockfd, buf, lenDATA, 0);
    if (bytesSent <= 0)
      return -1;
    lenDATA -= bytesSent;
    buf += bytesSent;
  }
  return lenDATA;
}
/*====================================================================*/
/*
int TCPServer::tcp_get_th(int sockfd,int *DATA,int lenDATA){ // may be needed some time, dont delete
  int bytesRead = 0;
  char *buf;
  buf=(char*)DATA;
  while(lenDATA>0){
    bytesRead=recv(sockfd,buf,lenDATA, 0);
    if (bytesRead <=0) return -1;
    lenDATA -= bytesRead;
    buf += bytesRead;
  }
  return lenDATA;
}
*/
/*====================================================================*/
int TCPServer::tcp_listen3(int sockfd, char* host_name, int len, int* sd_current) { // required
  struct sockaddr_in pin;
  struct hostent* hp;
  socklen_t addrlen;
  int rem_port;
  if (listen(sockfd, 5) == -1)
    return -1;
  addrlen = sizeof(pin);
  if ((*sd_current = accept(sockfd, (struct sockaddr*)&pin, &addrlen)) == -1)
    return -1;
  rem_port = ntohs(pin.sin_port);

  if ((hp = gethostbyaddr(&pin.sin_addr, sizeof(pin.sin_addr), AF_INET)) == 0)
    strncpy(host_name, inet_ntoa(pin.sin_addr), len);
  else
    strncpy(host_name, hp->h_name, len);
  host_name[len - 1] = 0;
  return rem_port;
}
/*====================================================================*/
/*
int TCPServer::tcp_listen_th(int sockfd){
  char rem_host[160];
  int sd_current = -1;
  tcp_listen3(sockfd, rem_host, 160-1, &sd_current);
  return sd_current;
}
*/

//---------******-------******--------====*******============*******==============------********-----------******------------------

void* opentcp(void* arg) { // initial thread
  Position* pos = (Position*)arg;
  HOST host;
  unsigned int ID = 0;
  int sock_main = 0; // check
  int TCP_FLAG = 0;  // check
  strncpy(host.NAME, "127.0.0.1", HOSTNAMELENGTH);
  host.PORT = 6969;
  while (TCP_FLAG == 0) {
    // sock_main = TCPServer::tcp_open_th(host.PORT,NULL);
    sock_main = TCPServer::open_local_th(host.PORT);
    TCP_FLAG = 1;
    if (sock_main < 0)
      sleep(1);
  }
  printf("socket: %d\n", sock_main);
  while (!interrupt) {
    char host_name[HOSTNAMELENGTH];
    int sd_clnt = 0;
    TCPServer::tcp_listen3(sock_main, host_name, HOSTNAMELENGTH, &sd_clnt); // check
    if (ID >= MAXCLIENTS)
      continue;

    struct CLIENT client;
    client.ID = ID++;
    client.tcpPort = sd_clnt;
    client.pos = pos;

    int ret = pthread_create(&client_thread[ID], 0, run_client_thread, &client);
    char th_name[64];
    sprintf(th_name, "client_thread_%d", ID);
    // ret = pthread_setname_np(client_thread[ID], th_name);
    ret = pthread_detach(client_thread[ID]);
    if (ret != 0)
      printf("clientThread error\n");
  }
  return 0;
}

static void* run_client_thread(void* arg) {
  struct CLIENT* client = (struct CLIENT*)arg;

  int tcpPort = client->tcpPort;
  Position* pos = client->pos;

  char mesg[MLEN];
  char buf[MAXLINE];
  char line[MAXLINE];
  int ib, il; // index counters
  bool flag;
  int lenbuf = 0;
  while (true) {
    bzero(line, MAXLINE);
    flag = false;
    il = 0;
    while (!flag) {
      if (lenbuf == 0) {
        bzero(buf, MAXLINE);
        lenbuf = read(tcpPort, buf, sizeof(buf));
        if (lenbuf <= 0)
          return 0;
      }
      //----------------------------------
      ib = 0;
      while (lenbuf > 0) {
        line[il++] = buf[ib];
        lenbuf--;
        if (buf[ib] == '\n' || buf[ib] == 0) {
          flag = true;
          memcpy(buf, &buf[ib + 1], lenbuf);
          line[il++] = 0;
          break;
        }
        ib++;
      }
      //-------------------------------
    }
    if (atoi(buf) != -1) // check whether this is line or buf
      Global::buttonPress = atoi(buf);
    bzero(mesg, MLEN);
    sprintf(&mesg[strlen(mesg)], "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,", pos->x, pos->z, pos->dist, pos->alpha1, pos->alpha2, pos->OffSetx, pos->speed, pos->turn, pos->gyro, pos->P, pos->I, pos->D);
    if (strlen(mesg) > MLEN)
      exit(1);
    sprintf(&mesg[strlen(mesg)], "\n");

    int rc = TCPServer::tcp_send_th(tcpPort, (int*)mesg, strlen(mesg));
    if (rc != 0)
      return 0;
    if (strncmp("exit", line, 4) == 0)
      break;
  }

  close(client->tcpPort);
  return 0;
}

void* videoServer(void* arg) {
  char host[80];
  sprintf(host, "%s", "127.0.0.1");
  // int sock = tcp_open_th(Var::videoPort,host); // check whether local open works
  int sock = TCPServer::open_local_th(Var::videoPort);
  while (1) {
    // int sockfd = tcp_listen_th(sock);
    char rem_host[160];
    TCPServer::tcp_listen3(sock, rem_host, 160 - 1, &Global::videoSocket); // check
    printf("socket2: %d\n", Global::videoSocket);
    while (!Global::videoError)
      sleep(1);
    close(Global::videoSocket);
    Global::videoSocket = 0;
    Global::videoError = false;
  }
  printf("error videoServer\n");
  return 0;
}

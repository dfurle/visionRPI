#include <netdb.h> 
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h> 
#include <sys/socket.h> 
#define MAX 8192 

int func(int sockfd){ 
  char buff[MAX];
  char *cmd="get dist";
  while(1){
    write(sockfd, cmd, strlen(cmd)+1);
    bzero(buff, sizeof(buff));
    int lenbuf = read(sockfd, buff, sizeof(buff));
    printf("%s\n", buff);	
    if ((strncmp(buff, "exit", 4)) == 0) {
      printf("Client Exit...\n");
      break;
    }
    usleep(100000);
  }
}
  
int main(int argc, const char* argv[]){ 
  char HOST[256];
  sprintf(HOST,"127.0.0.1");
  int PORT=6969;
  
  if (argc>1) sprintf(HOST,"%s",argv[1]);
  if (argc>2) PORT=atoi(argv[2]);

  int sockfd; 
  struct sockaddr_in servaddr;

  // socket create and varification 
  sockfd = socket(AF_INET, SOCK_STREAM, 0);

  servaddr.sin_family = AF_INET; 
  servaddr.sin_addr.s_addr = inet_addr(HOST); 
  servaddr.sin_port = htons(PORT); 
  
  connect(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr));
  
  func(sockfd); 
  
  close(sockfd); 
} 

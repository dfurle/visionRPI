#pragma once

#include "variables.h"


class Request{
public:
  enum {
    ERR,
    GET,
    PUT
  };
  int type = ERR;
  std::string where;

  std::unordered_map<std::string, std::string> headers;
  void parseHeader(std::string h);

  std::stringstream content;
  Request(int socket);
  void print(){
    std::cout << "(i) |" << type << ":" << where << "|\n";
    int i = 0;
    for(auto& key_val : headers){
      std::cout << " (" << i++ << ") |" << key_val.first << " : " << key_val.second << "|\n";
    }
  }
};

class Response{
public:
  std::string topper = "HTTP/1.1 200 OK\r\n";
  std::unordered_map<std::string, std::string> headers;
  std::string body;
  Response(){}
  void clearHeaders(){
    topper = "";
    headers.clear();
  }
  void addHeader(std::string n, std::string v){
    headers[n] = v;
  }
  void removeHeader(std::string n){
    headers.erase(n);
  }
  void setTopper(std::string s){
    topper = s + "\r\n";
  }
  std::string constructHeader(){
    std::string s = topper;
    for(auto& key_val : headers){
      s += key_val.first + ": " + key_val.second + "\r\n";
    }
    s += "\r\n";
    return s;
  }
  void setBody(std::string data, std::string t){
    body = data;
    headers["Content-Type"] = t;
    headers["Content-Length"] = std::to_string(data.length());
  }
  void setType(std::string t){
    headers["Content-Type"] = t;
  }
  bool sendAll(int sock){
    std::string h = constructHeader();
    if(send(sock, h.data(), h.length(), MSG_NOSIGNAL) == 0 && h.length() != 0)
      return true;
    if(send(sock, body.data(), body.length(), MSG_NOSIGNAL) == 0 && body.length() != 0)
      return true;
    return false;
  }
};

class HTTP{
public:
  int port;
  int sockfd;
private:
  struct sockaddr_in addr;
  int addrlen;
  std::thread* mainHandle;
  std::thread* streamHandle;

  struct Connection{
    std::string file;
    std::string type;
  };
  std::unordered_map<std::string, Connection> connections;

  struct IConnection{
    cv::Mat* data;
    std::vector<unsigned char> cdata;
    std::vector<int> sockets;
  };
  std::unordered_map<std::string, IConnection> iconnections;

  enum Type{
    BOOL, INT, DOUBLE
  };

  struct Value{
    void* val;
    int type;
    
  };

  std::unordered_map<std::string, Value> pconnections;

  Response res404;

public:
  HTTP();
  void start(int port);
  void handleRequests();
  std::string getReqFile(std::string fname);

  void streamer();
  void startStreamingServer();

  void addFile(std::string n, std::string f, std::string t);
  void addPut(bool& val, std::string name);
  void addPut(int& val, std::string name);
  void addPut(double& val, std::string name);
  void addImg(std::string n, cv::Mat* d);
  std::string initData();
  void handlePUT(std::stringstream& content);
};

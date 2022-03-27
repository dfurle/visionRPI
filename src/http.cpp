#include "http.h"


std::string HTTP::initData(){
  std::stringstream ss;
  for(auto& key_val : pconnections){
    if(key_val.second.type == Type::BOOL)
      ss << key_val.first << " " << *((bool*)key_val.second.val) << "\n";
    else if(key_val.second.type == Type::INT)
      ss << key_val.first << " " << *((int*)key_val.second.val) << "\n";
    else if(key_val.second.type == Type::DOUBLE)
      ss << key_val.first << " " << *((double*)key_val.second.val) << "\n";
  }
  std::cout << ss.str() << std::endl;
  return ss.str();
}


HTTP::HTTP(){
  res404.clearHeaders();
  res404.setTopper("HTTP/1.1 404 Not Found");
}

void HTTP::start(int port){
  this->port = port;
  sockfd = socket(AF_INET, SOCK_STREAM,0);

  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(port);
  addrlen = sizeof(addr);

  int on = 1;
  setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
  bind(sockfd, (sockaddr*)&addr, (socklen_t)addrlen);
  printf("socket created: %d\n", sockfd);

  listen(sockfd, 5);
  mainHandle = new std::thread(&HTTP::handleRequests, this);
}

Request::Request(int socket){
  std::string fullReq;
  char buf[1024];
  int readLen = 0;
  do{
    bzero(buf, 1024);
    readLen = read(socket, buf, 1024);
    fullReq += buf;
    if(std::string(buf).substr(readLen-4).compare("\r\n\r\n") == 0){
      break;
    }
  } while(readLen > 0);
  int splitPos = fullReq.find("\r\n\r\n");
  parseHeader(fullReq.substr(0,splitPos));
  if(headers.find("Content-Length") != headers.end()){
    do{
      bzero(buf, 1024);
      readLen = read(socket, buf, 1024);
      fullReq += buf;
      if(std::string(buf).substr(readLen-4).compare("\r\n\r\n") == 0){
        fullReq = fullReq.substr(0,fullReq.length()-4);
        break;
      }
    } while(readLen > 0);
    content = std::stringstream(fullReq.substr(splitPos+4));
  }
}

void Request::parseHeader(std::string h){
  std::stringstream hdr(h);
  std::string line;
  std::getline(hdr,line,'\n');
  int typeSplit = line.find(" ");
  std::string act = line.substr(0,typeSplit++);
  if(act.compare("GET") == 0){
    type = Request::GET;
  } else if (act.compare("PUT") == 0){
    type = Request::PUT;
  } else {
    type = Request::ERR;
  }
  act = line.substr(typeSplit,line.find(" ",typeSplit)-typeSplit);
  where = act;
  while(std::getline(hdr,line,'\n')){
    if(line[line.length()-1] == '\r')
      line = line.substr(0,line.length()-1);

    int pos = line.find(": ");
    std::string par = line.substr(0,pos);
    std::string val = line.substr(pos+2);
    headers[par] = val;
  }
}


void HTTP::handlePUT(std::stringstream& content){
  std::string s;
  while(std::getline(content,s,'\n')){
    std::string par = s.substr(0,s.find(" "));
    std::string val = s.substr(s.find(" ")+1);
    if(pconnections.find(par) == pconnections.end()){
      printf("parameter not found: |%s|\n",par.c_str());
      continue;
    }
    if(pconnections[par].type == Type::BOOL){
      *((bool*)pconnections[par].val) = std::stoi(val) == 1 ? true : false;
    } else if(pconnections[par].type == Type::INT){
      *((int*)pconnections[par].val) = std::stoi(val);
    } else if(pconnections[par].type == Type::DOUBLE) {
      *((double*)pconnections[par].val) = std::stod(val);
    }
  }
}

void HTTP::handleRequests(){
  while(true){
    int socket;
    if((socket = accept(this->sockfd, (sockaddr*)&this->addr, (socklen_t*)&this->addrlen)) < 0){
      printf("socket accept error\n");
      continue;
    }
    Request req(socket);
    // req.print();

    if(req.type == Request::GET){
      bool connectionExists = false;
      bool doClose = true;

      if(req.where.compare("/initialData") == 0){
        connectionExists = true;
        Response res;
        res.setBody(initData(),"text/plain");
        res.sendAll(socket);
      } else if(connections.find(req.where) != connections.end()){
        connectionExists = true;
        Response res;
        std::string data = getReqFile(connections[req.where].file);
        res.setBody(data,connections[req.where].type);
        std::cout << res.constructHeader() << std::endl;
        res.sendAll(socket);
      } else if(iconnections.find(req.where) != iconnections.end()){
        connectionExists = true;
        doClose = false;
        Response res;
        res.addHeader("Connection","keep-alive");
        res.addHeader("Cache-Control","no-cache, private");
        res.addHeader("Pragma","no-cache");
        res.setType("multipart/x-mixed-replace; boundary=--frame");
        std::cout << res.constructHeader() << std::endl;
        res.sendAll(socket);
        iconnections[req.where].sockets.push_back(socket);
      }

      if(!connectionExists){
        printf("\n!!!no such connection exists: |%s|!!!\n\n",req.where.c_str());
        res404.sendAll(socket);
      }
      if(doClose){
        close(socket);
      }
    } else if(req.type == Request::PUT){
      handlePUT(req.content);
      Response res;
      res.sendAll(socket);
      close(socket);
    }
  }
}

void HTTP::addFile(std::string n, std::string f, std::string t){
  Connection c;
  c.file = f;
  c.type = t;
  connections[n] = c;
}

void HTTP::addPut(bool& val, std::string name){
  Value v;
  v.val = (void*) &val;
  v.type = Type::BOOL;
  pconnections[name] = v;
}
void HTTP::addPut(int& val, std::string name){
  Value v;
  v.val = (void*) &val;
  v.type = Type::INT;
  pconnections[name] = v;
}
void HTTP::addPut(double& val, std::string name){
  Value v;
  v.val = (void*) &val;
  v.type = Type::DOUBLE;
  pconnections[name] = v;
}
void HTTP::addImg(std::string n, cv::Mat* d){
  IConnection c;
  c.data = d;
  iconnections[n] = c;
}

std::string HTTP::getReqFile(std::string fname){
  std::ifstream file;
  try{
    file.open(fname,std::ios::binary | std::ios::in);
  } catch(std::exception e){
    printf("THROWING ERROR!!! %s\n\n",fname.c_str());
    std::cout << e.what() << std::endl;
    throw std::invalid_argument("file not found");
  }
  std::string data;
  if(file.is_open()){
    printf("\nREADING FILE!!! %s\n\n",fname.c_str());
    std::streampos size = file.tellg();
    file.seekg(0, std::ios::end);
    size = file.tellg() - size;
    char* memblock = new char[size];
    file.seekg(0, std::ios::beg);
    file.read(memblock, size);
    data += memblock;
    delete[] memblock;
  }
  file.close();
  return data;
}

void HTTP::startStreamingServer(){
  streamHandle = new std::thread(&HTTP::streamer, this);
}

void HTTP::streamer(){
  while(true){
    Global::muteImg.lock();
    Global::httpStatus = 1;
    Global::muteImg.unlock();
    for(auto& key_val : iconnections){
      if(!key_val.second.data->empty()){
        cv::imencode(".jpeg",*key_val.second.data,key_val.second.cdata);
      }
    }
    Global::muteImg.lock();
    Global::httpStatus = 0;
    Global::muteImg.unlock();

    for(auto& key_val : iconnections){
      Response res;
      res.clearHeaders();
      res.setTopper("--frame");
      res.setBody(std::string(key_val.second.cdata.begin(),key_val.second.cdata.end()),"image/jpeg");
      for(auto sock = key_val.second.sockets.begin(); sock < key_val.second.sockets.end();){
        if(res.sendAll(*sock)){
          close(*sock);
          key_val.second.sockets.erase(sock);
        } else {
          sock++;
        }
      }
    }
    usleep(33000);
  }
}


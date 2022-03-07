#pragma once
#include <sys/time.h>
#include <stdlib.h>
#include <string>
#include <vector>

class Clock {
private:
  struct timeval time1, time2;

public:
  Clock() { gettimeofday(&time1, NULL); }
  void restart() { gettimeofday(&time1, NULL); }
  double getTimeAsSecs() {
    gettimeofday(&time2, NULL);
    return ((time2.tv_usec - time1.tv_usec) * 1.e-6 + (time2.tv_sec - time1.tv_sec));
  }
  double getTimeAsMillis() {
    gettimeofday(&time2, NULL);
    return ((time2.tv_usec - time1.tv_usec) * 1.e-3 + 1.e+3 * (time2.tv_sec - time1.tv_sec));
  }
  double getTimeAsMicros() {
    gettimeofday(&time2, NULL);
    return ((time2.tv_usec - time1.tv_usec) + 1.e+6 * (time2.tv_sec - time1.tv_sec));
  }
  void printTime(Clock& prev, std::string params) {
    printf(" %-20s %.2f tot: %.2f\n", params.c_str(), prev.getTimeAsMillis(), getTimeAsMillis());
    prev.restart();
  }
};

class ClockTimer {
public:
  Clock total;
  Clock between;
  bool doPrint;
  std::vector<std::string> betweenStr;
  std::vector<double> betweenTimes;
  
public:
  ClockTimer(bool dp){ doPrint = dp; }
  void PTotal(){
    if(doPrint){
      printf("--finalTime: %.2f\n==---------------------------==\n", total.getTimeAsMillis());
    }
  }
  void printTime(std::string str){
    if(doPrint){
      betweenTimes.push_back(between.getTimeAsMillis());
      betweenStr.push_back(str);
      printf(" %-20s %.2f tot: %.2f\n", str.c_str(), between.getTimeAsMillis(), total.getTimeAsMillis());
      between.restart();
    }
  }
  void printProportion(){
    std::vector<double> prop;
    double sum = total.getTimeAsMillis();
    for(double t : betweenTimes){
      prop.push_back(t/sum*100);
    }
    for(int i = 0; i < prop.size(); i++){
      printf(" %-20s %6.2f\n", betweenStr[i].c_str(), prop[i]);
    }
  }
  void reset(){
    total.restart();
    between.restart();
    betweenTimes.clear();
    betweenStr.clear();
  }

};
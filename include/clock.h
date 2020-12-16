#pragma once
#include <sys/time.h>
#include <stdlib.h>
#include <string>

class Clock {
private:
  struct timeval time1, time2;

public:
  Clock() { gettimeofday(&time1, NULL); }
  void restart() { gettimeofday(&time1, NULL); }
  double getTimeAsSecs() {
    gettimeofday(&time2, NULL);
    return ((time2.tv_usec - time1.tv_usec) * 1e-6 + (time2.tv_sec - time1.tv_sec));
  }
  double getTimeAsMillis() {
    gettimeofday(&time2, NULL);
    return ((time2.tv_usec - time1.tv_usec) * 1e-3 + 1e+3 * (time2.tv_sec - time1.tv_sec));
  }
  double getTimeAsMicros() {
    gettimeofday(&time2, NULL);
    return ((time2.tv_usec - time1.tv_usec) + 1e+6 * (time2.tv_sec - time1.tv_sec));
  }
  void printTime(Clock& prev, std::string params) {
    printf(" %-20s %.2f tot: %.2f\n", params.c_str(), prev.getTimeAsMillis(), getTimeAsMillis());
    prev.restart();
  }
};

class ClockTimer {
private:
  Clock total;
  Clock between;
  
public:
  ClockTimer(){}
  void PTotal(bool r = false){
    printf("--finalTime: %.2f\n==---------------------------==\n", total.getTimeAsMillis());
    if(r) reset();
  }
  void printTime(std::string str){
    printf(" %-20s %.2f tot: %.2f\n", str.c_str(), between.getTimeAsMillis(), total.getTimeAsMillis());
    between.restart();
  }
  void printTime(bool doPrint, std::string str){
    if(doPrint){
      printf(" %-20s %.2f tot: %.2f\n", str.c_str(), between.getTimeAsMillis(), total.getTimeAsMillis());
      between.restart();
    }
  }
  void reset(){
    total.restart();
    between.restart();
  }

};
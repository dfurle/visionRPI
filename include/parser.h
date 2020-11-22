#pragma once
#include <vector>
#include <string>
#include <algorithm>


class Parser{
public:
  class P_Switch{
  public:
    std::string s;
    std::string s_long;
    std::string explain;
    bool& value;
    bool df;
    P_Switch(std::string s, std::string s_long, std::string explain, bool& value, bool df):
    s(s),
    s_long(s_long),
    explain(explain),
    value(value),
    df(df)
    {
    }
  };
  class P_Value{
  public:
    std::string s;
    std::string s_long;
    std::string explain;
    double& value;
    double df;
    P_Value(std::string s, std::string s_long, std::string explain, double& value, double df):
    s(s),
    s_long(s_long),
    explain(explain),
    value(value),
    df(df)
    {
    }
  };

  std::vector<std::pair<std::string,double>> args;
  std::vector<P_Switch> switches;
  std::vector<P_Value> values;
  std::string banner;
  int max_short_size;
  bool HELP;
  

  Parser(int argc, const char* argv[]){
    banner = "-------------------------------------------------";
    max_short_size = 4;
    add_Parameter("-h","--help",HELP, false,"shows help menu");
    if(argc < 2){
      return;
    }
    for(int i = 1; i < argc; i++){
      std::string s(argv[i]);
      double value = 0.0;
      auto delim = s.find("=");
      if(delim != std::string::npos){
        char* e;
        value = std::strtod(s.substr(delim+1).c_str(),&e);
        s = s.substr(0,delim);
        if(*e != '\0'){
          printf("failed to change input to double| argument: \'%s\'\n",s.c_str());
          value = 0.0;
        }
      } else {
        value = 1.0;
      }
      this->args.push_back(std::pair<std::string,double>(s,value));
    }
  }

  void settings(size_t banner_size, int max_short_size){
    banner = std::string(banner_size,'-');
    this->max_short_size = max_short_size;
  }


  void printHelp(){
    int width_longs = 2;
    int width_explain_s = 5;
    int width_explain_v = 5;
    for(P_Switch s : switches){
      width_longs = std::max(width_longs,int(s.s_long.length()));
      width_explain_s = std::max(width_explain_s,int(s.explain.length()));
    }
    for(P_Value s : values){
      width_longs = std::max(width_longs,int(s.s_long.length()));
      width_explain_v = std::max(width_explain_v,int(s.explain.length()));
    }
    printf("%s\n",banner.c_str());
    printf("------ | HELP MENU: -h or --help | ------\n");
    printf("%s\nHELP: BOOLEANS\n%s\n",banner.c_str(),banner.c_str());
    for(P_Switch s : switches){
      if(s.s.length() == 0)
        printf("\t| %*s%-*s | %-*s | default: %s |\n",(4+max_short_size),"",width_longs,s.s_long.c_str(),width_explain_s,s.explain.c_str(),s.df ? "true " : "false");
      else if(s.s_long.length() == 0)
        printf("\t| %-*s%*s | %-*s | default: %s |\n",max_short_size,s.s.c_str(),(4+width_longs),"",width_explain_s,s.explain.c_str(),s.df ? "true " : "false");
      else
        printf("\t| %-*s or %-*s | %-*s | default: %s |\n",max_short_size,s.s.c_str(),width_longs,s.s_long.c_str(),width_explain_s,s.explain.c_str(),s.df ? "true " : "false");
    }
    printf("%s\nHELP: VARIABLES\n%s\n",banner.c_str(),banner.c_str());
    for(P_Value s : values){
      if(s.s.length() == 0)
        printf("\t| %*s%-*s | %-*s | default: %*s=%7.3f |\n",(4+max_short_size),"",width_longs,s.s_long.c_str(),width_explain_v,s.explain.c_str(),max_short_size,s.s.c_str(),s.df);
      else if(s.s_long.length() == 0)
        printf("\t| %-*s%*s | %-*s | default: %*s=%7.3f |\n",max_short_size,s.s.c_str(),(4+width_longs),"",width_explain_v,s.explain.c_str(),max_short_size,s.s.c_str(),s.df);
      else
        printf("\t| %-*s or %-*s | %-*s | default: %*s=%7.3f |\n",max_short_size,s.s.c_str(),width_longs,s.s_long.c_str(),width_explain_v,s.explain.c_str(),max_short_size,s.s.c_str(),s.df);
    }
    printf("%s\n",banner.c_str());
  }

  void printVals(){
    int width_longs = 2;
    int width_explain_s = 5;
    int width_explain_v = 5;
    for(P_Switch s : switches){
      width_longs = std::max(width_longs,int(s.s_long.length()));
      width_explain_s = std::max(width_explain_s,int(s.explain.length()));
    }
    for(P_Value s : values){
      width_longs = std::max(width_longs,int(s.s_long.length()));
      width_explain_v = std::max(width_explain_v,int(s.explain.length()));
    }
    printf("%s\nBOOLEANS\n%s\n",banner.c_str(),banner.c_str());
    for(P_Switch s : switches){
      if(s.s.length() == 0)
        printf("\t| %*s%-*s | %s |\n",(4+max_short_size),"",width_longs,s.s_long.c_str(),s.value ? "true " : "false");
      else if(s.s_long.length() == 0)
        printf("\t| %-*s%*s | %s |\n",max_short_size,s.s.c_str(),(width_longs+4),"",s.value ? "true " : "false");
      else
        printf("\t| %-*s or %-*s | %s |\n",max_short_size,s.s.c_str(),width_longs,s.s_long.c_str(),s.value ? "true " : "false");
    }
    printf("%s\nVARIABLES\n%s\n",banner.c_str(),banner.c_str());
    for(P_Value s : values){
      if(s.s.length() == 0)
        printf("\t| %*s%-*s | %08.3f |\n",(4+max_short_size),"",width_longs,s.s_long.c_str(),s.value);
      else if(s.s_long.length() == 0)
        printf("\t| %-*s%*s | %08.3f |\n",max_short_size,s.s.c_str(),(width_longs+4),"",s.value);
      else
        printf("\t| %-*s or %-*s | %08.3f |\n",max_short_size,s.s.c_str(),width_longs,s.s_long.c_str(),s.value);
    }
    printf("%s\n",banner.c_str());

  }


  void add_Parameter(std::string _short, std::string _long, bool& _variable, bool _default, std::string _explain){
    _variable = _default;
    P_Switch tmp(_short,_long,_explain,_variable,_default);
    switches.push_back(tmp);
  }
  void add_Parameter(std::string _short, std::string _long, double& _variable, double _default, std::string _explain){
    _variable = _default;
    P_Value tmp(_short,_long,_explain,_variable,_default);
    values.push_back(tmp);
  }
  bool checkParams(bool print_vals){
    for(int i = 0; i < switches.size(); i++){
      auto val = std::find_if(args.begin(),args.end(),[&](std::pair<std::string,double> arg){
        if(arg.first.compare(switches[i].s) == 0 || arg.first.compare(switches[i].s_long) == 0)
          return true;
        return false;
      });
      if(val != args.end())
        switches[i].value = !switches[i].value;
    }

    for(int i = 0; i < values.size(); i++){
      auto val = std::find_if(args.begin(),args.end(),[&](std::pair<std::string,double> arg){
        if(arg.first.compare(values[i].s) == 0 || arg.first.compare(values[i].s_long) == 0)
          return true;
        return false;
      });
      if(val != args.end())
        values[i].value = val[0].second;
    }
    if(HELP){
      printHelp();
      return true;
    }
    if(print_vals)
      printVals();
    return false;
  }
};
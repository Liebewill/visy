/* 
 * File:   Parameters.cpp
 * Author: daniele
 * 
 * Created on 14 marzo 2015, 18.22
 */

#include <map>

#include "Parameters.h"

namespace visy
{

  Parameters::Parameters (int argc, char** argv)
  {
    this->argc = argc;
    this->argv = argv;
  }

  Parameters::~Parameters ()
  {
  }

  void
  Parameters::putString (std::string name)
  {
    std::stringstream ss;
    ss << "--" << name;
    std::string value;
    pcl::console::parse_argument(argc, argv, ss.str().c_str(), value);
    this->strings.insert(std::pair<std::string, std::string>(name, value));
  }

  void
  Parameters::putBool (std::string name)
  {
    std::stringstream ss;
    ss << "-" << name;
    bool value = pcl::console::find_switch(argc, argv, ss.str().c_str());
    this->bools.insert(std::pair < std::string, bool>(name, value));
  }

  void
  Parameters::putFloat (std::string name)
  {
    std::stringstream ss;
    ss << "--" << name;
    float value;
    pcl::console::parse_argument(argc, argv, ss.str().c_str(), value);
    this->floats.insert(std::pair<std::string, float>(name, value));
  }

  void
  Parameters::putInt (std::string name)
  {
    std::stringstream ss;
    ss << "--" << name;
    int value;
    pcl::console::parse_argument(argc, argv, ss.str().c_str(), value);
    this->ints.insert(std::pair<std::string, int>(name, value));
  }

  bool
  Parameters::getBool (std::string name)
  {
    return this->bools[name];
  }

  float
  Parameters::getFloat (std::string name)
  {
    return this->floats[name];
  }

  int
  Parameters::getInt (std::string name)
  {
    return this->ints[name];
  }

  std::string
  Parameters::getString (std::string name)
  {
    return this->strings[name];
  }

  std::vector<float>
  Parameters::parseFloatArray (std::string s, std::string delimiter)
  {

    std::vector<float> array;
    size_t pos = 0;
    std::string token;
    while ((pos = s.find(delimiter)) != std::string::npos)
    {
      token = s.substr(0, pos);
      array.push_back(atof(token.c_str()));
      s.erase(0, pos + delimiter.length());
    }
    token = s.substr(0, pos);
    array.push_back(atof(token.c_str()));
    return array;
  }


}
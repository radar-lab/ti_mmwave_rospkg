#ifndef _PARAM_PARSER_CLASS_
#define _PARAM_PARSER_CLASS_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ti_mmwave_rospkg/mmWaveCLI.h"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <cstdio>
#include <sstream>
#include <string>
#include <vector>

namespace ti_mmwave_rospkg {

class ParameterParser : public nodelet::Nodelet{

  public:
  	
  	ParameterParser();
  	void ParamsParser(ti_mmwave_rospkg::mmWaveCLI &srv, ros::NodeHandle &n);
	void CalParams(ros::NodeHandle &nh);

  private:
    
    virtual void onInit();
    
    ti_mmwave_rospkg::mmWaveCLI srv;

};
}
#endif

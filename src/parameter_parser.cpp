#include "parameter_parser.h"

namespace ti_mmwave_rospkg {

PLUGINLIB_EXPORT_CLASS(ti_mmwave_rospkg::parameter_parser, nodelet::Nodelet);

parameter_parser::parameter_parser() {}

void parameter_parser::onInit() {
   ros::NodeHandle private_nh = getPrivateNodeHandle();
}

void parameter_parser::params_parser(ti_mmwave_rospkg::mmWaveCLI &srv, ros::NodeHandle &n) {
  size_t pos = 0;
  int i = 0;
  int txAntennas = 0;
  std::string cmd;
  std::string token;
  ROS_ERROR("%s",srv.response.resp.c_str());
  while ((pos = srv.request.comm.find(" ")) != std::string::npos) {
    token = srv.request.comm.substr(0, pos);
    if (!token.compare("chirpCfg")) txAntennas++;
    if (i == 0) {
      cmd = token;
    } else if (!cmd.compare("frameCfg")) {
      switch (i) {
        // case 1:
          // n.setParam("/ti_mmwave/chirpStartIdx", std::stoi(token)); break;
        // case 2:
          // n.setParam("/ti_mmwave/chirpEndIdx", std::stoi(token)); break;
        case 3:
          n.setParam("/ti_mmwave/numLoops", std::stoi(token)); break;
        case 4:
          n.setParam("/ti_mmwave/numFrames", std::stoi(token)); break;
        case 5:
          n.setParam("/ti_mmwave/framePeriodicity", std::stof(token)); break;
      }
    } else if (!cmd.compare("profileCfg")) {
      switch (i) {
        case 2:
          n.setParam("/ti_mmwave/startFreq", std::stof(token)); break;
        case 3:
          n.setParam("/ti_mmwave/idleTime", std::stof(token)); break;
        case 5:
          n.setParam("/ti_mmwave/rampEndTime", std::stof(token)); break;
        case 8:
          n.setParam("/ti_mmwave/freqSlopeConst", std::stof(token)); break;
        case 10:
          n.setParam("/ti_mmwave/numAdcSamples", std::stoi(token)); break;
        case 11:
          n.setParam("/ti_mmwave/digOutSampleRate", std::stof(token)); break;
      }
    }
    srv.request.comm.erase(0, pos + 1);
    i++;
  }
  n.setParam("/ti_mmwave/numTxAnt", txAntennas);
}

}
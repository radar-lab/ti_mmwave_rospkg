#include "ParameterParser.h"

namespace ti_mmwave_rospkg {

PLUGINLIB_EXPORT_CLASS(ti_mmwave_rospkg::ParameterParser, nodelet::Nodelet);

ParameterParser::ParameterParser() {}

void ParameterParser::onInit() {}

void ParameterParser::ParamsParser(ti_mmwave_rospkg::mmWaveCLI &srv, ros::NodeHandle &nh) {

    //   ROS_ERROR("%s",srv.request.comm.c_str());
    //   ROS_ERROR("%s",srv.response.resp.c_str());
    std::vector <std::string> v;
    std::string s = srv.request.comm.c_str(); 
    std::istringstream ss(s);
    std::string token;
    std::string req;
    int i = 0;
    while (std::getline(ss, token, ' ')) {
        v.push_back(token);
        if (i > 0) {
            if (!req.compare("profileCfg")) {
                switch (i) {
                    case 2:
                        nh.setParam("/ti_mmwave/startFreq", std::stof(token)); break;
                    case 3:
                        nh.setParam("/ti_mmwave/idleTime", std::stof(token)); break;
                    case 4:
                        nh.setParam("/ti_mmwave/adcStartTime", std::stof(token)); break;
                    case 5:
                        nh.setParam("/ti_mmwave/rampEndTime", std::stof(token)); break;
                    case 8:
                        nh.setParam("/ti_mmwave/freqSlopeConst", std::stof(token)); break;
                    case 10:
                        nh.setParam("/ti_mmwave/numAdcSamples", std::stoi(token)); break;
                    case 11:
                        nh.setParam("/ti_mmwave/digOutSampleRate", std::stof(token)); break;
                    case 14:
                        nh.setParam("/ti_mmwave/rxGain", std::stof(token)); break;
                }
            } else if (!req.compare("frameCfg")) {
                switch (i) {
                case 1:
                    nh.setParam("/ti_mmwave/chirpStartIdx", std::stoi(token)); break;
                case 2:
                    nh.setParam("/ti_mmwave/chirpEndIdx", std::stoi(token)); break;
                case 3:
                    nh.setParam("/ti_mmwave/numLoops", std::stoi(token)); break;
                case 4:
                    nh.setParam("/ti_mmwave/numFrames", std::stoi(token)); break;
                case 5:
                    nh.setParam("/ti_mmwave/framePeriodicity", std::stof(token)); break;
                }
            }
        } else req = token;
        i++;
    }
    }

void ParameterParser::CalParams(ros::NodeHandle &nh) {
    float c0 = 299792458;
    int chirpStartIdx;
    int chirpEndIdx;
    int numLoops;
    float framePeriodicity;
    float startFreq;
    float idleTime;
    float adcStartTime;
    float rampEndTime;
    float digOutSampleRate;
    float freqSlopeConst;
    float numAdcSamples;

    nh.getParam("/ti_mmwave/startFreq", startFreq);
    nh.getParam("/ti_mmwave/idleTime", idleTime);
    nh.getParam("/ti_mmwave/adcStartTime", adcStartTime);
    nh.getParam("/ti_mmwave/rampEndTime", rampEndTime);
    nh.getParam("/ti_mmwave/digOutSampleRate", digOutSampleRate);
    nh.getParam("/ti_mmwave/freqSlopeConst", freqSlopeConst);
    nh.getParam("/ti_mmwave/numAdcSamples", numAdcSamples);

    nh.getParam("/ti_mmwave/chirpStartIdx", chirpStartIdx);
    nh.getParam("/ti_mmwave/chirpEndIdx", chirpEndIdx);
    nh.getParam("/ti_mmwave/numLoops", numLoops);
    nh.getParam("/ti_mmwave/framePeriodicity", framePeriodicity);

    int ntx = chirpEndIdx - chirpStartIdx + 1;
    int nd = numLoops;
    int nr = numAdcSamples;
    float tfr = framePeriodicity * 1e-3;
    float fs = digOutSampleRate * 1e3;
    float kf = freqSlopeConst * 1e12;
    float adc_duration = nr / fs;
    float BW = adc_duration * kf;
    float PRI = (idleTime + rampEndTime) * 1e-6;
    float fc = startFreq * 1e9 + kf * (adcStartTime * 1e-6 + adc_duration / 2); 

    float vrange = c0 / (2 * BW);
    float max_range = nr * vrange;
    float max_vel = c0 / (2 * fc * PRI) / ntx;
    float vvel = max_vel / nd;

    nh.setParam("/ti_mmwave/num_TX", ntx);
    nh.setParam("/ti_mmwave/f_s", fs);
    nh.setParam("/ti_mmwave/f_c", fc);
    nh.setParam("/ti_mmwave/BW", BW);
    nh.setParam("/ti_mmwave/PRI", PRI);
    nh.setParam("/ti_mmwave/t_fr", tfr);
    nh.setParam("/ti_mmwave/max_range", max_range);
    nh.setParam("/ti_mmwave/range_resolution", vrange);
    nh.setParam("/ti_mmwave/max_doppler_vel", max_vel);
    nh.setParam("/ti_mmwave/doppler_vel_resolution", vvel);
}

}
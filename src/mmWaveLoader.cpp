#include "ros/ros.h"
#include "nodelet/loader.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "mmWave_Manager");
    
    nodelet::Loader manager(true);
    
    nodelet::M_string remap(ros::names::getRemappings());
    
    nodelet::V_string nargv;
    
    manager.load("mmWaveCommSrv", "ti_mmwave_rospkg/mmWaveCommSrv", remap, nargv);
    
    manager.load("mmWaveDataHdl", "ti_mmwave_rospkg/mmWaveDataHdl", remap, nargv);
    
    ros::spin();
    
    return 0;
}

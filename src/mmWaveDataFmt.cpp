#include "mmWaveDataFmt.hpp"

namespace ti_mmwave_rospkg
{

PLUGINLIB_EXPORT_CLASS(ti_mmwave_rospkg::mmWaveDataFmt, nodelet::Nodelet);

mmWaveDataFmt::mmWaveDataFmt() {}

void mmWaveDataFmt::onInit()
{
   ros::NodeHandle private_nh = getPrivateNodeHandle();
}

//TODO: callback for radar message 
void mmWaveDataFmt::something_here()
{

}

//TODO: callback for type
void mmWaveDataFmt::something_else_here()
{

}

}


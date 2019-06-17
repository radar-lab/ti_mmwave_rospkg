#include "mmWaveCommSrv.hpp"

namespace ti_mmwave_rospkg
{

PLUGINLIB_EXPORT_CLASS(ti_mmwave_rospkg::mmWaveCommSrv, nodelet::Nodelet);

mmWaveCommSrv::mmWaveCommSrv() {}

void mmWaveCommSrv::onInit()
{
    ros::NodeHandle private_nh = getPrivateNodeHandle();
    ros::NodeHandle private_nh2("~"); // follow namespace for multiple sensors
   
    private_nh2.getParam("command_port", mySerialPort);
    
    private_nh2.getParam("command_rate", myBaudRate);
    
    ROS_INFO("mmWaveCommSrv: command_port = %s", mySerialPort.c_str());
    ROS_INFO("mmWaveCommSrv: command_rate = %d", myBaudRate);
    
    commSrv = private_nh.advertiseService("/mmWaveCLI", &mmWaveCommSrv::commSrv_cb, this);
    
    NODELET_DEBUG("mmWaveCommsrv: Finished onInit function");
}


bool mmWaveCommSrv::commSrv_cb(mmWaveCLI::Request &req , mmWaveCLI::Response &res) {
    NODELET_DEBUG("mmWaveCommSrv: Port is \"%s\" and baud rate is %d", mySerialPort.c_str(), myBaudRate);

    /*Open Serial port and error check*/
    serial::Serial mySerialObject("", myBaudRate, serial::Timeout::simpleTimeout(1000));
    mySerialObject.setPort(mySerialPort.c_str());
    try {
        mySerialObject.open();
    } catch (std::exception &e1) {
        ROS_INFO("mmWaveCommSrv: Failed to open User serial port with error: %s", e1.what());
        ROS_INFO("mmWaveCommSrv: Waiting 20 seconds before trying again...");
        try {
            // Wait 20 seconds and try to open serial port again
            ros::Duration(20).sleep();
            mySerialObject.open();
        } catch (std::exception &e2) {
            ROS_ERROR("mmWaveCommSrv: Failed second time to open User serial port, error: %s", e1.what());
            NODELET_ERROR("mmWaveCommSrv: Port could not be opened. Port is \"%s\" and baud rate is %d", mySerialPort.c_str(), myBaudRate);
        return false;
        }
    }

   /*Read any previous pending response(s)*/
   while (mySerialObject.available() > 0)
   {
      mySerialObject.readline(res.resp, 1024, ":/>");
      ROS_INFO("mmWaveCommSrv: Received (previous) response from sensor: '%s'", res.resp.c_str());
      res.resp = "";
   }

   /*Send out command received from the client*/
   ROS_INFO("mmWaveCommSrv: Sending command to sensor: '%s'", req.comm.c_str());
   req.comm.append("\n");
   int bytesSent = mySerialObject.write(req.comm.c_str());

   /*Read output from mmwDemo*/
   mySerialObject.readline(res.resp, 1024, ":/>");
   ROS_INFO("mmWaveCommSrv: Received response from sensor: '%s'", res.resp.c_str());

   mySerialObject.close();
   
   return true;
}

}




/*
 * mmWaveCommSrv.cpp
 * Description: This file implements a ROS nodelet which will open up a serial port provided by the user
 *              at a certain baud rate (also provided by user) that will interface with the 1443EVM mmwDemo
 *              User UART to be used for board configuration.
 *                       
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/
#include "mmWaveCommSrv.hpp"

namespace ti_mmwave_rospkg
{

PLUGINLIB_EXPORT_CLASS(ti_mmwave_rospkg::mmWaveCommSrv, nodelet::Nodelet);

mmWaveCommSrv::mmWaveCommSrv() {}

void mmWaveCommSrv::onInit()
{
   ros::NodeHandle private_nh = getPrivateNodeHandle();
   
   private_nh.getParam("/mmWave_Manager/command_port", mySerialPort);
   
   private_nh.getParam("/mmWave_Manager/command_rate", myBaudRate);
   
   ROS_INFO("mmWaveCommSrv: command_port = %s", mySerialPort.c_str());
   ROS_INFO("mmWaveCommSrv: command_rate = %d", myBaudRate);
   
   commSrv = private_nh.advertiseService("mmWaveCLI", &mmWaveCommSrv::commSrv_cb, this);
   
   NODELET_DEBUG("mmWaveCommsrv: Finished onInit function");
}


bool mmWaveCommSrv::commSrv_cb(mmWaveCLI::Request &req , mmWaveCLI::Response &res)
{
   NODELET_DEBUG("mmWaveCommSrv: Port is \"%s\" and baud rate is %d", mySerialPort.c_str(), myBaudRate);

   /*Open Serial port and error check*/
   serial::Serial mySerialObject("", myBaudRate, serial::Timeout::simpleTimeout(1000));
   mySerialObject.setPort(mySerialPort.c_str());
   try
   {
      mySerialObject.open();
   } catch (std::exception &e1) {
      ROS_INFO("mmWaveCommSrv: Failed to open User serial port with error: %s", e1.what());
      ROS_INFO("mmWaveCommSrv: Waiting 20 seconds before trying again...");
      try
      {
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
      mySerialObject.readline(res.resp, 128, ":/>");
      ROS_INFO("mmWaveCommSrv: Received (previous) response from sensor: '%s'", res.resp.c_str());
      res.resp = "";
   }

   /*Send out command received from the client*/
   ROS_INFO("mmWaveCommSrv: Sending command to sensor: '%s'", req.comm.c_str());
   req.comm.append("\n");
   int bytesSent = mySerialObject.write(req.comm.c_str());

   /*Read output from mmwDemo*/
   mySerialObject.readline(res.resp, 128, ":/>");
   ROS_INFO("mmWaveCommSrv: Received response from sensor: '%s'", res.resp.c_str());

   mySerialObject.close();
   
   return true;
}

}




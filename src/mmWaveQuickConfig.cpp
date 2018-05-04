/*
 * mmWaveQuickConfig.cpp
 *
 * This file will load a .txt file specified at the command line
 * and parse it, sending each line to the mmWaveCLI service as a command.
 * Examples configurations are provided under the cfg folder.
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

#include "ros/ros.h"
#include "ti_mmwave_rospkg/mmWaveCLI.h"
#include <cstdlib>
#include <fstream>
#include <stdio.h>
#include <regex>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mmWaveQuickConfig");
  if (argc != 2)
  {
    ROS_INFO("mmWaveQuickConfig: usage: mmWaveQuickConfig /file_directory/params.cfg");
    return 1;
  }
  else
  {
    ROS_INFO("mmWaveQuickConfig: Configuring mmWave device using config file: %s", argv[1]);
  }
  
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<ti_mmwave_rospkg::mmWaveCLI>("/mmWaveCommSrv/mmWaveCLI");
  ti_mmwave_rospkg::mmWaveCLI srv;
  std::ifstream myParams;
  
  //wait for service to become available
  ros::service::waitForService("/mmWaveCommSrv/mmWaveCLI", 100000); 
  
  myParams.open(argv[1]);
  
  if( myParams.is_open() )
  {
    while( std::getline(myParams, srv.request.comm) )
    {
      // Remove Windows carriage-return if present
      srv.request.comm.erase(std::remove(srv.request.comm.begin(), srv.request.comm.end(), '\r'), srv.request.comm.end());

      // Ignore comment lines (first non-space char is '%') or blank lines
      if (std::regex_match(srv.request.comm, std::regex("^\\s*%.*") ) ||
          std::regex_match(srv.request.comm, std::regex("^\\s*") ))
      {
          ROS_INFO("mmWaveQuickConfig: Ignored blank or comment line: '%s'", srv.request.comm.c_str() );
      }

      // Send commands to mmWave sensor
      else
      {
        ROS_INFO("mmWaveQuickConfig: Sending command: '%s'", srv.request.comm.c_str() );
	int numTries = 0;

        // Try each command twice if first time fails (in case serial port connection had initial error)
        while (numTries < 2)
	{
          if( client.call(srv) )
          {
            if (std::regex_search(srv.response.resp, std::regex("Done") )) 
            {
              ROS_INFO("mmWaveQuickConfig: Command successful (mmWave sensor responded with 'Done')");
	      break;
            }
            else if (numTries == 0)
            {
              ROS_INFO("mmWaveQuickConfig: Command failed (mmWave sensor did not respond with 'Done')");
              ROS_INFO("mmWaveQuickConfig: Response: '%s'", srv.response.resp.c_str() );
            }
	    else
	    {
              ROS_ERROR("mmWaveQuickConfig: Command failed (mmWave sensor did not respond with 'Done')");
              ROS_ERROR("mmWaveQuickConfig: Response: '%s'", srv.response.resp.c_str() );
              return 1;
	    }
          }
          else
          {
            ROS_ERROR("mmWaveQuickConfig: Failed to call service mmWaveCLI");
            ROS_ERROR("%s", srv.request.comm.c_str() );
            return 1;
          }
	  numTries++;
	}
      }
    }

    myParams.close();
  }
  else
  {
     ROS_ERROR("mmWaveQuickConfig: Failed to open File %s", argv[1]);
     return 1;
  }

  ROS_INFO("mmWaveQuickConfig: mmWaveQuickConfig will now terminate. Done configuring mmWave device using config file: %s", argv[1]);
  return 0;
}

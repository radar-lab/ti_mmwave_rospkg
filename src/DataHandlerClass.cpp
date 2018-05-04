/*
 * DataHandlerClass.cpp
 *
 * This is the implementation of the DataHandlerClass.h
 * Three threads are spawned when start() is called.
 *  1) readIncomingData() thread
 *  2) sortIncomingData() thread
 *  3) syncedBufferSwap() thread
 *  
 * Together they implement a double-buffered read from the data serial port 
 * which sorts the data into the class's mmwDataPacket struct.
 *
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


#include <DataHandlerClass.h>
#include <pthread.h>
#include <algorithm>
#include "pcl_ros/point_cloud.h"
#include "sensor_msgs/PointField.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <cmath>


DataUARTHandler::DataUARTHandler(ros::NodeHandle* nh) : currentBufp(&pingPongBuffers[0]) , nextBufp(&pingPongBuffers[1]) 
{
    nodeHandle = nh;
    DataUARTHandler_pub = nodeHandle->advertise< sensor_msgs::PointCloud2 >("RScan", 100);
    maxAllowedElevationAngleDeg = 90; // Use max angle if none specified
    maxAllowedAzimuthAngleDeg = 90; // Use max angle if none specified
}

/*Implementation of setUARTPort*/
void DataUARTHandler::setUARTPort(char* mySerialPort)
{
    dataSerialPort = mySerialPort;
}

/*Implementation of setBaudRate*/
void DataUARTHandler::setBaudRate(int myBaudRate)
{
    dataBaudRate = myBaudRate;
}

/*Implementation of setMaxAllowedElevationAngleDeg*/
void DataUARTHandler::setMaxAllowedElevationAngleDeg(int myMaxAllowedElevationAngleDeg)
{
    maxAllowedElevationAngleDeg = myMaxAllowedElevationAngleDeg;
}

/*Implementation of setMaxAllowedAzimuthAngleDeg*/
void DataUARTHandler::setMaxAllowedAzimuthAngleDeg(int myMaxAllowedAzimuthAngleDeg)
{
    maxAllowedAzimuthAngleDeg = myMaxAllowedAzimuthAngleDeg;
}

/*Implementation of readIncomingData*/
void *DataUARTHandler::readIncomingData(void)
{
    
    int firstPacketReady = 0;
    uint8_t last8Bytes[8] = {0};
    
    /*Open UART Port and error checking*/
    serial::Serial mySerialObject("", dataBaudRate, serial::Timeout::simpleTimeout(100));
    mySerialObject.setPort(dataSerialPort);
    try
    {
        mySerialObject.open();
    } catch (std::exception &e1) {
        ROS_INFO("DataUARTHandler Read Thread: Failed to open Data serial port with error: %s", e1.what());
        ROS_INFO("DataUARTHandler Read Thread: Waiting 20 seconds before trying again...");
        try
        {
            // Wait 20 seconds and try to open serial port again
            ros::Duration(20).sleep();
            mySerialObject.open();
        } catch (std::exception &e2) {
            ROS_ERROR("DataUARTHandler Read Thread: Failed second time to open Data serial port, error: %s", e1.what());
            ROS_ERROR("DataUARTHandler Read Thread: Port could not be opened. Port is \"%s\" and baud rate is %d", dataSerialPort, dataBaudRate);
            pthread_exit(NULL);
        }
    }
    
    if(mySerialObject.isOpen())
        ROS_INFO("DataUARTHandler Read Thread: Port is open");
    else
        ROS_ERROR("DataUARTHandler Read Thread: Port could not be opened");
    
    /*Quick magicWord check to synchronize program with data Stream*/
    while(!isMagicWord(last8Bytes))
    {

        last8Bytes[0] = last8Bytes[1];
        last8Bytes[1] = last8Bytes[2];
        last8Bytes[2] = last8Bytes[3];
        last8Bytes[3] = last8Bytes[4];
        last8Bytes[4] = last8Bytes[5];
        last8Bytes[5] = last8Bytes[6];
        last8Bytes[6] = last8Bytes[7];
        mySerialObject.read(&last8Bytes[7], 1);
        
    }
    
    /*Lock nextBufp before entering main loop*/
    pthread_mutex_lock(&nextBufp_mutex);
    
    while(ros::ok())
    {
        /*Start reading UART data and writing to buffer while also checking for magicWord*/
        last8Bytes[0] = last8Bytes[1];
        last8Bytes[1] = last8Bytes[2];
        last8Bytes[2] = last8Bytes[3];
        last8Bytes[3] = last8Bytes[4];
        last8Bytes[4] = last8Bytes[5];
        last8Bytes[5] = last8Bytes[6];
        last8Bytes[6] = last8Bytes[7];
        mySerialObject.read(&last8Bytes[7], 1);
        
        nextBufp->push_back( last8Bytes[7] );  //push byte onto buffer
        
        //ROS_INFO("DataUARTHandler Read Thread: last8bytes = %02x%02x %02x%02x %02x%02x %02x%02x",  last8Bytes[7], last8Bytes[6], last8Bytes[5], last8Bytes[4], last8Bytes[3], last8Bytes[2], last8Bytes[1], last8Bytes[0]);
        
        /*If a magicWord is found wait for sorting to finish and switch buffers*/
        if( isMagicWord(last8Bytes) )
        {
            //ROS_INFO("Found magic word");
        
            /*Lock countSync Mutex while unlocking nextBufp so that the swap thread can use it*/
            pthread_mutex_lock(&countSync_mutex);
            pthread_mutex_unlock(&nextBufp_mutex);
            
            /*increment countSync*/
            countSync++;
            
            /*If this is the first packet to be found, increment countSync again since Sort thread is not reading data yet*/
            if(firstPacketReady == 0)
            {
                countSync++;
                firstPacketReady = 1;
            }
            
            /*Signal Swap Thread to run if countSync has reached its max value*/
            if(countSync == COUNT_SYNC_MAX)
            {
                pthread_cond_signal(&countSync_max_cv);
            }
            
            /*Wait for the Swap thread to finish swapping pointers and signal us to continue*/
            pthread_cond_wait(&read_go_cv, &countSync_mutex);
            
            /*Unlock countSync so that Swap Thread can use it*/
            pthread_mutex_unlock(&countSync_mutex);
            pthread_mutex_lock(&nextBufp_mutex);
            
            nextBufp->clear();
            memset(last8Bytes, 0, sizeof(last8Bytes));
              
        }
      
    }
    
    
    mySerialObject.close();
    
    pthread_exit(NULL);
}


int DataUARTHandler::isMagicWord(uint8_t last8Bytes[8])
{
    int val = 0, i = 0, j = 0;
    
    for(i = 0; i < 8 ; i++)
    {
    
       if( last8Bytes[i] == magicWord[i])
       {
          j++;
       }
    
    }
    
    if( j == 8)
    {
       val = 1;
    }
    
    return val;  
}

void *DataUARTHandler::syncedBufferSwap(void)
{
    while(ros::ok())
    {
        pthread_mutex_lock(&countSync_mutex);
    
        while(countSync < COUNT_SYNC_MAX)
        {
            pthread_cond_wait(&countSync_max_cv, &countSync_mutex);
            
            pthread_mutex_lock(&currentBufp_mutex);
            pthread_mutex_lock(&nextBufp_mutex);
            
            std::vector<uint8_t>* tempBufp = currentBufp;
        
            this->currentBufp = this->nextBufp;
            
            this->nextBufp = tempBufp;
            
            pthread_mutex_unlock(&currentBufp_mutex);
            pthread_mutex_unlock(&nextBufp_mutex);
            
            countSync = 0;
            
            pthread_cond_signal(&sort_go_cv);
            pthread_cond_signal(&read_go_cv);
            
        }
    
        pthread_mutex_unlock(&countSync_mutex);

    }

    pthread_exit(NULL);
    
}

void *DataUARTHandler::sortIncomingData( void )
{
    MmwDemo_Output_TLV_Types tlvType = MMWDEMO_OUTPUT_MSG_NULL;
    uint32_t tlvLen = 0;
    uint32_t headerSize;
    unsigned int currentDatap = 0;
    SorterState sorterState = READ_HEADER;
    int i = 0, tlvCount = 0, offset = 0;
    float maxElevationAngleRatioSquared;
    float maxAzimuthAngleRatio;
    
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> RScan(new pcl::PointCloud<pcl::PointXYZI>);
    
    //wait for first packet to arrive
    pthread_mutex_lock(&countSync_mutex);
    pthread_cond_wait(&sort_go_cv, &countSync_mutex);
    pthread_mutex_unlock(&countSync_mutex);
    
    pthread_mutex_lock(&currentBufp_mutex);
    
    while(ros::ok())
    {
        
        switch(sorterState)
        {
            
        case READ_HEADER:
            
            //make sure packet has at least first three fields (12 bytes) before we read them (does not include magicWord since it was already removed)
            if(currentBufp->size() < 12)
            {
               sorterState = SWAP_BUFFERS;
               break;
            }
            
            //get version (4 bytes)
            memcpy( &mmwData.header.version, &currentBufp->at(currentDatap), sizeof(mmwData.header.version));
            currentDatap += ( sizeof(mmwData.header.version) );
            
            //get totalPacketLen (4 bytes)
            memcpy( &mmwData.header.totalPacketLen, &currentBufp->at(currentDatap), sizeof(mmwData.header.totalPacketLen));
            currentDatap += ( sizeof(mmwData.header.totalPacketLen) );
            
            //get platform (4 bytes)
            memcpy( &mmwData.header.platform, &currentBufp->at(currentDatap), sizeof(mmwData.header.platform));
            currentDatap += ( sizeof(mmwData.header.platform) );      
            
            //if packet doesn't have correct header size (which is based on platform and SDK version), throw it away (does not include magicWord since it was already removed)
	    if((((mmwData.header.version >> 24) & 0xFF) < 1) || (((mmwData.header.version >> 16) & 0xFF) < 1))  //check if SDK version is older than 1.1
	    {
               //ROS_INFO("mmWave device firmware detected version: 0x%8.8X", mmwData.header.version);
	       headerSize = 28;
	    }
            else if((mmwData.header.platform & 0xFFFF) == 0x1443)
	    {
	       headerSize = 28;
	    }
	    else  // 1642
	    {
	       headerSize = 32;
	    }
            if(currentBufp->size() < headerSize)
            {
               sorterState = SWAP_BUFFERS;
               break;
            }
            
            //get frameNumber (4 bytes)
            memcpy( &mmwData.header.frameNumber, &currentBufp->at(currentDatap), sizeof(mmwData.header.frameNumber));
            currentDatap += ( sizeof(mmwData.header.frameNumber) );
            
            //get timeCpuCycles (4 bytes)
            memcpy( &mmwData.header.timeCpuCycles, &currentBufp->at(currentDatap), sizeof(mmwData.header.timeCpuCycles));
            currentDatap += ( sizeof(mmwData.header.timeCpuCycles) );
            
            //get numDetectedObj (4 bytes)
            memcpy( &mmwData.header.numDetectedObj, &currentBufp->at(currentDatap), sizeof(mmwData.header.numDetectedObj));
            currentDatap += ( sizeof(mmwData.header.numDetectedObj) );
            
            //get numTLVs (4 bytes)
            memcpy( &mmwData.header.numTLVs, &currentBufp->at(currentDatap), sizeof(mmwData.header.numTLVs));
            currentDatap += ( sizeof(mmwData.header.numTLVs) );
            
            //get subFrameNumber (4 bytes) (not used for XWR1443)
            if((mmwData.header.platform & 0xFFFF) != 0x1443)
	    {
               memcpy( &mmwData.header.subFrameNumber, &currentBufp->at(currentDatap), sizeof(mmwData.header.subFrameNumber));
               currentDatap += ( sizeof(mmwData.header.subFrameNumber) );
	    }

            //if packet lengths do not patch, throw it away
            if(mmwData.header.totalPacketLen == currentBufp->size() )
            {
               sorterState = CHECK_TLV_TYPE;
            }
            else sorterState = SWAP_BUFFERS;

            break;
            
        case READ_OBJ_STRUCT:
            
            i = 0;
            offset = 0;
            
            //get number of objects
            memcpy( &mmwData.numObjOut, &currentBufp->at(currentDatap), sizeof(mmwData.numObjOut));
            currentDatap += ( sizeof(mmwData.numObjOut) );
            
            //get xyzQFormat
            memcpy( &mmwData.xyzQFormat, &currentBufp->at(currentDatap), sizeof(mmwData.xyzQFormat));
            currentDatap += ( sizeof(mmwData.xyzQFormat) );
            
            RScan->header.seq = 0;
            //RScan->header.stamp = (uint32_t) mmwData.header.timeCpuCycles;
            RScan->header.frame_id = "base_radar_link";
            RScan->height = 1;
            RScan->width = mmwData.numObjOut;
            RScan->is_dense = 1;
            RScan->points.resize(RScan->width * RScan->height);
            
            // Calculate ratios for max desired elevation and azimuth angles
            if ((maxAllowedElevationAngleDeg >= 0) && (maxAllowedElevationAngleDeg < 90))
            {
                maxElevationAngleRatioSquared = tan(maxAllowedElevationAngleDeg * M_PI / 180.0);
                maxElevationAngleRatioSquared = maxElevationAngleRatioSquared * maxElevationAngleRatioSquared;
            }
            else
            {
                maxElevationAngleRatioSquared = -1;
            }
            if ((maxAllowedAzimuthAngleDeg >= 0) && (maxAllowedAzimuthAngleDeg < 90))
            {
                maxAzimuthAngleRatio = tan(maxAllowedAzimuthAngleDeg * M_PI / 180.0);
            }
            else
            {
                maxAzimuthAngleRatio = -1;
            }
            //ROS_INFO("maxElevationAngleRatioSquared = %f", maxElevationAngleRatioSquared);
            //ROS_INFO("maxAzimuthAngleRatio = %f", maxAzimuthAngleRatio);
            //ROS_INFO("mmwData.numObjOut before = %d", mmwData.numObjOut);


            //set some parameters for pointcloud
            while( i < mmwData.numObjOut )
            {
                //get object range index
                memcpy( &mmwData.objOut.rangeIdx, &currentBufp->at(currentDatap), sizeof(mmwData.objOut.rangeIdx));
                currentDatap += ( sizeof(mmwData.objOut.rangeIdx) );
                
                //get object doppler index
                memcpy( &mmwData.objOut.dopplerIdx, &currentBufp->at(currentDatap), sizeof(mmwData.objOut.dopplerIdx));
                currentDatap += ( sizeof(mmwData.objOut.dopplerIdx) );
                
                //get object peak intensity value
                memcpy( &mmwData.objOut.peakVal, &currentBufp->at(currentDatap), sizeof(mmwData.objOut.peakVal));
                currentDatap += ( sizeof(mmwData.objOut.peakVal) );
                
                //get object x-coordinate
                memcpy( &mmwData.objOut.x, &currentBufp->at(currentDatap), sizeof(mmwData.objOut.x));
                currentDatap += ( sizeof(mmwData.objOut.x) );
                
                //get object y-coordinate
                memcpy( &mmwData.objOut.y, &currentBufp->at(currentDatap), sizeof(mmwData.objOut.y));
                currentDatap += ( sizeof(mmwData.objOut.y) );
                
                //get object z-coordinate
                memcpy( &mmwData.objOut.z, &currentBufp->at(currentDatap), sizeof(mmwData.objOut.z));
                currentDatap += ( sizeof(mmwData.objOut.z) );
                
                //convert from Qformat to float(meters)
                float temp[4];
                
                temp[0] = (float) mmwData.objOut.x;
                temp[1] = (float) mmwData.objOut.y;
                temp[2] = (float) mmwData.objOut.z;
                //temp[4] = //doppler 
                
                for(int j = 0; j < 3; j++)
                {
                    if(temp[j] > 32767)
                        temp[j] -= 65535;
                    
                    temp[j] = temp[j] / pow(2,mmwData.xyzQFormat);
                 }   
                 
                // Convert intensity to dB
                temp[3] = 10 * log10(mmwData.objOut.peakVal + 1);  // intensity
                
                // Map mmWave sensor coordinates to ROS coordinate system
                RScan->points[i].x = temp[1];   // ROS standard coordinate system X-axis is forward which is the mmWave sensor Y-axis
                RScan->points[i].y = -temp[0];  // ROS standard coordinate system Y-axis is left which is the mmWave sensor -(X-axis)
                RScan->points[i].z = temp[2];   // ROS standard coordinate system Z-axis is up which is the same as mmWave sensor Z-axis
                RScan->points[i].intensity = temp[3];
               
                // Keep point if elevation and azimuth angles are less than specified max values
                // (NOTE: The following calculations are done using ROS standard coordinate system axis definitions where X is forward and Y is left)
                if (((maxElevationAngleRatioSquared == -1) ||
                     (((RScan->points[i].z * RScan->points[i].z) / (RScan->points[i].x * RScan->points[i].x +
                                                                    RScan->points[i].y * RScan->points[i].y)
                      ) < maxElevationAngleRatioSquared)
                    ) &&
                    ((maxAzimuthAngleRatio == -1) || (fabs(RScan->points[i].y / RScan->points[i].x) < maxAzimuthAngleRatio)) &&
		            (RScan->points[i].x != 0)
                   )
                {
                    //ROS_INFO("Kept point");
                    i++;
                }

                // Otherwise, remove the point
                else
                {
                    //ROS_INFO("Removed point");
                    mmwData.numObjOut--;
                }
            }

            // Resize point cloud since some points may have been removed
            RScan->width = mmwData.numObjOut;
            RScan->points.resize(RScan->width * RScan->height);
            
            //ROS_INFO("mmwData.numObjOut after = %d", mmwData.numObjOut);
            //ROS_INFO("DataUARTHandler Sort Thread: number of obj = %d", mmwData.numObjOut );
            
            DataUARTHandler_pub.publish(RScan);
            
            sorterState = CHECK_TLV_TYPE;
            
            break;
            
        case READ_LOG_MAG_RANGE:
            {
        
              i = 0;
            
              while (i++ < tlvLen - 1)
              {
                     //ROS_INFO("DataUARTHandler Sort Thread : Parsing Range Profile i=%d and tlvLen = %u", i, tlvLen);
              }
            
              currentDatap += tlvLen;
            
              sorterState = CHECK_TLV_TYPE;
            }
            
            break;
            
        case READ_NOISE:
            {
        
              i = 0;
            
              while (i++ < tlvLen - 1)
              {
                     //ROS_INFO("DataUARTHandler Sort Thread : Parsing Noise Profile i=%d and tlvLen = %u", i, tlvLen);
              }
            
              currentDatap += tlvLen;
            
              sorterState = CHECK_TLV_TYPE;
            }
           
            break;
            
        case READ_AZIMUTH:
            {
        
              i = 0;
            
              while (i++ < tlvLen - 1)
              {
                     //ROS_INFO("DataUARTHandler Sort Thread : Parsing Azimuth Profile i=%d and tlvLen = %u", i, tlvLen);
              }
            
              currentDatap += tlvLen;
            
              sorterState = CHECK_TLV_TYPE;
            }
            
            break;
            
        case READ_DOPPLER:
            {
        
              i = 0;
            
              while (i++ < tlvLen - 1)
              {
                     //ROS_INFO("DataUARTHandler Sort Thread : Parsing Doppler Profile i=%d and tlvLen = %u", i, tlvLen);
              }
            
              currentDatap += tlvLen;
            
              sorterState = CHECK_TLV_TYPE;
            }
            
            break;
            
        case READ_STATS:
            {
        
              i = 0;
            
              while (i++ < tlvLen - 1)
              {
                     //ROS_INFO("DataUARTHandler Sort Thread : Parsing Stats Profile i=%d and tlvLen = %u", i, tlvLen);
              }
            
              currentDatap += tlvLen;
            
              sorterState = CHECK_TLV_TYPE;
            }
            
            break;
        
        case CHECK_TLV_TYPE:
        
            //ROS_INFO("DataUARTHandler Sort Thread : tlvCount = %d, numTLV = %d", tlvCount, mmwData.header.numTLVs);
        
            if(tlvCount++ >= mmwData.header.numTLVs)
            {
                //ROS_INFO("DataUARTHandler Sort Thread : CHECK_TLV_TYPE state says tlvCount max was reached, going to switch buffer state");
                sorterState = SWAP_BUFFERS;
            }
            else
            {
               //get tlvType (32 bits) & remove from queue
                memcpy( &tlvType, &currentBufp->at(currentDatap), sizeof(tlvType));
                currentDatap += ( sizeof(tlvType) );
                
                //ROS_INFO("DataUARTHandler Sort Thread : sizeof(tlvType) = %d", sizeof(tlvType));
            
                //get tlvLen (32 bits) & remove from queue
                memcpy( &tlvLen, &currentBufp->at(currentDatap), sizeof(tlvLen));
                currentDatap += ( sizeof(tlvLen) );
                
                //ROS_INFO("DataUARTHandler Sort Thread : sizeof(tlvLen) = %d", sizeof(tlvLen));
                
                //ROS_INFO("DataUARTHandler Sort Thread : tlvType = %d, tlvLen = %d", (int) tlvType, tlvLen);
            
                switch(tlvType)
                {
                case MMWDEMO_OUTPUT_MSG_NULL:
                
                    break;
                
                case MMWDEMO_OUTPUT_MSG_DETECTED_POINTS:
                    //ROS_INFO("DataUARTHandler Sort Thread : Object TLV");
                    sorterState = READ_OBJ_STRUCT;
                    break;
                
                case MMWDEMO_OUTPUT_MSG_RANGE_PROFILE:
                    //ROS_INFO("DataUARTHandler Sort Thread : Range TLV");
                    sorterState = READ_LOG_MAG_RANGE;
                    break;
                
                case MMWDEMO_OUTPUT_MSG_NOISE_PROFILE:
                    //ROS_INFO("DataUARTHandler Sort Thread : Noise TLV");
                    sorterState = READ_NOISE;
                    break;
                
                case MMWDEMO_OUTPUT_MSG_AZIMUTH_STATIC_HEAT_MAP:
                    //ROS_INFO("DataUARTHandler Sort Thread : Azimuth Heat TLV");
                    sorterState = READ_AZIMUTH;
                    break;
                
                case MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP:
                    //ROS_INFO("DataUARTHandler Sort Thread : R/D Heat TLV");
                    sorterState = READ_DOPPLER;
                    break;
                
                case MMWDEMO_OUTPUT_MSG_STATS:
                    //ROS_INFO("DataUARTHandler Sort Thread : Stats TLV");
                    sorterState = READ_STATS;
                    break;
                
                case MMWDEMO_OUTPUT_MSG_MAX:
                    //ROS_INFO("DataUARTHandler Sort Thread : Header TLV");
                    sorterState = READ_HEADER;
                    break;
                
                default: break;
                }
            }
            
        break;
            
       case SWAP_BUFFERS:
       
            pthread_mutex_lock(&countSync_mutex);
            pthread_mutex_unlock(&currentBufp_mutex);
                            
            countSync++;
                
            if(countSync == COUNT_SYNC_MAX)
            {
                pthread_cond_signal(&countSync_max_cv);
            }
                
            pthread_cond_wait(&sort_go_cv, &countSync_mutex);
                
            pthread_mutex_unlock(&countSync_mutex);
            pthread_mutex_lock(&currentBufp_mutex);
                
            currentDatap = 0;
            tlvCount = 0;
                
            sorterState = READ_HEADER;
            
            break;
                
            
        default: break;
        }
    }
    
    
    pthread_exit(NULL);
}

void DataUARTHandler::start(void)
{
    
    pthread_t uartThread, sorterThread, swapThread;
    
    int  iret1, iret2, iret3;
    
    pthread_mutex_init(&countSync_mutex, NULL);
    pthread_mutex_init(&nextBufp_mutex, NULL);
    pthread_mutex_init(&currentBufp_mutex, NULL);
    pthread_cond_init(&countSync_max_cv, NULL);
    pthread_cond_init(&read_go_cv, NULL);
    pthread_cond_init(&sort_go_cv, NULL);
    
    countSync = 0;
    
    /* Create independent threads each of which will execute function */
    iret1 = pthread_create( &uartThread, NULL, this->readIncomingData_helper, this);
    if(iret1)
    {
     ROS_INFO("Error - pthread_create() return code: %d\n",iret1);
     ros::shutdown();
    }
    
    iret2 = pthread_create( &sorterThread, NULL, this->sortIncomingData_helper, this);
    if(iret2)
    {
        ROS_INFO("Error - pthread_create() return code: %d\n",iret1);
        ros::shutdown();
    }
    
    iret3 = pthread_create( &swapThread, NULL, this->syncedBufferSwap_helper, this);
    if(iret3)
    {
        ROS_INFO("Error - pthread_create() return code: %d\n",iret1);
        ros::shutdown();
    }
    
    ros::spin();

    pthread_join(iret1, NULL);
    ROS_INFO("DataUARTHandler Read Thread joined");
    pthread_join(iret2, NULL);
    ROS_INFO("DataUARTHandler Sort Thread joined");
    pthread_join(iret3, NULL);
    ROS_INFO("DataUARTHandler Swap Thread joined");
    
    pthread_mutex_destroy(&countSync_mutex);
    pthread_mutex_destroy(&nextBufp_mutex);
    pthread_mutex_destroy(&currentBufp_mutex);
    pthread_cond_destroy(&countSync_max_cv);
    pthread_cond_destroy(&read_go_cv);
    pthread_cond_destroy(&sort_go_cv);
    
    
}

void* DataUARTHandler::readIncomingData_helper(void *context)
{  
    return (static_cast<DataUARTHandler*>(context)->readIncomingData());
}

void* DataUARTHandler::sortIncomingData_helper(void *context)
{  
    return (static_cast<DataUARTHandler*>(context)->sortIncomingData());
}

void* DataUARTHandler::syncedBufferSwap_helper(void *context)
{  
    return (static_cast<DataUARTHandler*>(context)->syncedBufferSwap());
}

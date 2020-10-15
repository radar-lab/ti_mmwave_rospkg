#ifndef _DATA_HANDLER_CLASS_
#define _DATA_HANDLER_CLASS_

#include <ti_mmwave_rospkg/RadarScan.h>
#include "mmWave.h"
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <boost/shared_ptr.hpp>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pthread.h>
#include <algorithm>
#include "pcl_ros/point_cloud.h"
#include "sensor_msgs/PointField.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#define COUNT_SYNC_MAX 2

class DataUARTHandler{
    
public:
    
    /*Constructor*/
   //void DataUARTHandler(ros::NodeHandle* nh) : currentBufp(&pingPongBuffers[0]) , nextBufp(&pingPongBuffers[1]) {}
    DataUARTHandler(ros::NodeHandle* nh);
    
    void setFrameID(char* myFrameID);

    /*User callable function to set the UARTPort*/
    void setUARTPort(char* mySerialPort);
    
    /*User callable function to set the BaudRate*/
    void setBaudRate(int myBaudRate);

    /*User callable function to set maxAllowedElevationAngleDeg*/
    void setMaxAllowedElevationAngleDeg(int myMaxAllowedElevationAngleDeg);
    
    /*User callable function to set maxAllowedElevationAngleDeg*/
    void setMaxAllowedAzimuthAngleDeg(int myMaxAllowedAzimuthAngleDeg);

    void setNodeHandle(ros::NodeHandle* nh);
      
    /*User callable function to start the handler's internal threads*/
    void start(void);
    
    /*Helper functions to allow pthread compatability*/
    static void* readIncomingData_helper(void *context);
    
    static void* sortIncomingData_helper(void *context);
    
    static void* syncedBufferSwap_helper(void *context);
    
    /*Sorted mmwDemo Data structure*/
    mmwDataPacket mmwData;

private:

    int nr;
    int nd;
    int ntx;
    float fs;
    float fc;
    float BW;
    float PRI;
    float tfr;
    float max_range;
    float vrange;
    float max_vel;
    float vvel;

    char* frameID;
    /*Contains the name of the serial port*/
    char* dataSerialPort;
    
    /*Contains the baud Rate*/
    int dataBaudRate;
    
    /*Contains the max_allowed_elevation_angle_deg (points with elevation angles 
      outside +/- max_allowed_elevation_angle_deg will be removed)*/
    int maxAllowedElevationAngleDeg;
    
    /*Contains the max_allowed_azimuth_angle_deg (points with azimuth angles 
      outside +/- max_allowed_azimuth_angle_deg will be removed)*/
    int maxAllowedAzimuthAngleDeg;
    
    /*Mutex protected variable which synchronizes threads*/
    int countSync;
    
    /*Read/Write Buffers*/
    std::vector<uint8_t> pingPongBuffers[2];
    
    /*Pointer to current data (sort)*/
    std::vector<uint8_t>* currentBufp;
    
    /*Pointer to new data (read)*/
    std::vector<uint8_t>* nextBufp;
    
    /*Mutex protecting the countSync variable */
    pthread_mutex_t countSync_mutex;
    
    /*Mutex protecting the nextBufp pointer*/
    pthread_mutex_t nextBufp_mutex;
    
    /*Mutex protecting the currentBufp pointer*/
    pthread_mutex_t currentBufp_mutex;
    
    /*Condition variable which blocks the Swap Thread until signaled*/
    pthread_cond_t countSync_max_cv;
    
    /*Condition variable which blocks the Read Thread until signaled*/
    pthread_cond_t read_go_cv;
    
    /*Condition variable which blocks the Sort Thread until signaled*/
    pthread_cond_t sort_go_cv;
    
    /*Swap Buffer Pointers Thread*/
    void *syncedBufferSwap(void);
    
    /*Checks if the magic word was found*/
    int isMagicWord(uint8_t last8Bytes[8]);
    
    /*Read incoming UART Data Thread*/
    void *readIncomingData(void);
    
    /*Sort incoming UART Data Thread*/
    void *sortIncomingData(void);
    
    void visualize(const ti_mmwave_rospkg::RadarScan &msg);

    ros::NodeHandle* nodeHandle;
    
    ros::Publisher DataUARTHandler_pub;
    ros::Publisher radar_scan_pub;
    ros::Publisher marker_pub;
};

#endif 

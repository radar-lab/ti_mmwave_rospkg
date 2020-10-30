
/*
 * mmWave.h
 *
 * This file contains various defines used within this package.
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


#ifndef _TI_IWR14XX_
#define _TI_IWR14XX_

#include <iostream>
#include <iostream>
#include <cstdio>
#include "serial/serial.h"
#include "ros/ros.h"
#include <boost/thread.hpp>
#include <cstdint>

enum MmwDemo_Output_TLV_Types
{
    MMWDEMO_OUTPUT_MSG_NULL = 0,
    /*! @brief   List of detected points */
    MMWDEMO_OUTPUT_MSG_DETECTED_POINTS,

    /*! @brief   Range profile */
    MMWDEMO_OUTPUT_MSG_RANGE_PROFILE,

    /*! @brief   Noise floor profile */
    MMWDEMO_OUTPUT_MSG_NOISE_PROFILE,

    /*! @brief   Samples to calculate static azimuth  heatmap */
    MMWDEMO_OUTPUT_MSG_AZIMUTH_STATIC_HEAT_MAP,

    /*! @brief   Range/Doppler detection matrix */
    MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP,

    /*! @brief   Stats information */
    MMWDEMO_OUTPUT_MSG_STATS,

    /*! @brief   List of detected points side information */
    MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO,

    MMWDEMO_OUTPUT_MSG_MAX
};

enum SorterState{ READ_HEADER, 
    CHECK_TLV_TYPE,
    READ_OBJ_STRUCT, 
    READ_LOG_MAG_RANGE, 
    READ_NOISE, 
    READ_AZIMUTH, 
    READ_DOPPLER, 
    READ_STATS,
    SWAP_BUFFERS,
    READ_SIDE_INFO};

struct MmwDemo_output_message_header_t
    {
        /*! brief   Version: : MajorNum * 2^24 + MinorNum * 2^16 + BugfixNum * 2^8 + BuildNum   */
        uint32_t    version;

        /*! @brief   Total packet length including header in Bytes */
        uint32_t    totalPacketLen;

        /*! @brief   platform type */
        uint32_t    platform;
        
        /*! @brief   Frame number */
        uint32_t    frameNumber;

        /*! @brief   Time in CPU cycles when the message was created. For XWR16xx: DSP CPU cycles, for XWR14xx: R4F CPU cycles */
        uint32_t    timeCpuCycles;
        
        /*! @brief   Number of detected objects */
        uint32_t    numDetectedObj;

        /*! @brief   Number of TLVs */
        uint32_t    numTLVs;

        /*! @brief   Sub-frame Number (not used with XWR14xx) */
        uint32_t    subFrameNumber;
    };

// Detected object structure for mmWave SDK 1.x and 2.x
struct MmwDemo_DetectedObj
    {
        uint16_t   rangeIdx;     /*!< @brief Range index */
        uint16_t   dopplerIdx;   /*!< @brief Dopler index */
        uint16_t   peakVal;      /*!< @brief Peak value */
        int16_t  x;             /*!< @brief x - coordinate in meters. Q format depends on the range resolution */
        int16_t  y;             /*!< @brief y - coordinate in meters. Q format depends on the range resolution */
        int16_t  z;             /*!< @brief z - coordinate in meters. Q format depends on the range resolution */
    };
    
// Detected object structures for mmWave SDK 3.x (DPIF_PointCloudCartesian_t and DPIF_PointCloudSideInfo_t)

/**
* @brief
* Point cloud definition in Cartesian coordinate system - floating point format
*
*/
typedef struct DPIF_PointCloudCartesian_t
{
/*! @brief x - coordinate in meters */
float x;

/*! @brief y - coordinate in meters */
float y;

/*! @brief z - coordinate in meters */
float z;

/*! @brief Doppler velocity estimate in m/s. Positive velocity means target
* is moving away from the sensor and negative velocity means target
* is moving towards the sensor. */
float velocity;
}DPIF_PointCloudCartesian;

/**
* @brief
* Point cloud side information such as SNR and noise level
*
*/
typedef struct DPIF_PointCloudSideInfo_t
{
/*! @brief snr - CFAR cell to side noise ratio in dB expressed in 0.1 steps of dB */
int16_t snr;

/*! @brief y - CFAR noise level of the side of the detected cell in dB expressed in 0.1 steps of dB */
int16_t noise;
}DPIF_PointCloudSideInfo;


struct mmwDataPacket{
MmwDemo_output_message_header_t header;
uint16_t numObjOut;
uint16_t xyzQFormat; // only used for SDK 1.x and 2.x
MmwDemo_DetectedObj objOut; // only used for SDK 1.x and 2.x

DPIF_PointCloudCartesian_t newObjOut; // used for SDK 3.x
DPIF_PointCloudSideInfo_t sideInfo; // used for SDK 3.x
};

const uint8_t magicWord[8] = {2, 1, 4, 3, 6, 5, 8, 7};

#endif











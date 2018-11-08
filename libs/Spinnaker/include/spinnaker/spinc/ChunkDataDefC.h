//=============================================================================
// Copyright © 2017 FLIR Integrated Imaging Solutions, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of FLIR
// Integrated Imaging Solutions, Inc. ("Confidential Information"). You
// shall not disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with FLIR Integrated Imaging Solutions, Inc. (FLIR).
//
// FLIR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. FLIR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================
   
/* Auto-generated file. Do not modify. */

#ifndef PGR_SPINNAKER_CHUNKDATADEF_C_H
#define PGR_SPINNAKER_CHUNKDATADEF_C_H

#include "SpinnakerPlatformC.h"

/**
* @defgroup SpinnakerDefsC Spinnaker C Definitions
* @ingroup SpinnakerC
*
* Definitions for Spinnaker C
*/
/*@{*/

/**
* @defgroup ChunkDataDefC_h Chunk Data Structures
*/
/*@{*/

/**
* @brief The type of information that can be obtained from image chunk data.
*/

typedef struct _spinChunkData
{
    double m_blackLevel;
    int64_t m_frameID;
    double m_exposureTime;
    int64_t m_timestamp;
    int64_t m_exposureEndLineStatusAll;
    int64_t m_width;
    int64_t m_image;
    int64_t m_height;
    double m_gain;
    int64_t m_sequencerSetActive;
    int64_t m_cRC;
    int64_t m_offsetX;
    int64_t m_offsetY;
    int64_t m_serialDataLength;
    int64_t m_partSelector;
    int64_t m_pixelDynamicRangeMin;
    int64_t m_pixelDynamicRangeMax;
    int64_t m_timestampLatchValue;
    int64_t m_lineStatusAll;
    int64_t m_counterValue;
    double m_timerValue;
    int64_t m_scanLineSelector;
    int64_t m_encoderValue;
    int64_t m_linePitch;
    int64_t m_transferBlockID;
    int64_t m_transferQueueCurrentBlockCount;
    int64_t m_streamChannelID;
    double m_scan3dCoordinateScale;
    double m_scan3dCoordinateOffset;
    double m_scan3dInvalidDataValue;
    double m_scan3dAxisMin;
    double m_scan3dAxisMax;
    double m_scan3dTransformValue;
    double m_scan3dCoordinateReferenceValue;
} spinChunkData;

/*@}*/

/*@}*/

#endif // PGR_SPINNAKER_CHUNKDATADEF_C_H
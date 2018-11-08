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

#ifndef PGR_SPINNAKER_ICHUNKDATA_H
#define PGR_SPINNAKER_ICHUNKDATA_H

#include "SpinGenApi/GCTypes.h"
#include "SpinGenApi/INodeMap.h"
#include "CameraDefs.h"

namespace Spinnaker
{
    /**
    * @defgroup SpinnakerClasses Spinnaker Classes
    */
    /*@{*/

    /**
    * @defgroup IChunkData_h IChunkData Class
    */
    /*@{*/

    /**
    *@brief The Interface file for ChunkData.
    */

    class SPINNAKER_API IChunkData
    {
    public:
        virtual ~IChunkData(){};

        virtual void SetChunks(GenApi::INodeMap & pNodeMap) = 0;

        virtual float64_t GetBlackLevel() const = 0;
        virtual int64_t GetFrameID() const = 0;
        virtual float64_t GetExposureTime() const = 0;
        virtual int64_t GetTimestamp() const = 0;
        virtual int64_t GetExposureEndLineStatusAll() const = 0;
        virtual int64_t GetWidth() const = 0;
        virtual int64_t GetImage() const = 0;
        virtual int64_t GetHeight() const = 0;
        virtual float64_t GetGain() const = 0;
        virtual int64_t GetSequencerSetActive() const = 0;
        virtual int64_t GetCRC() const = 0;
        virtual int64_t GetOffsetX() const = 0;
        virtual int64_t GetOffsetY() const = 0;
        virtual int64_t GetSerialDataLength() const = 0;
        virtual int64_t GetPartSelector() const = 0;
        virtual int64_t GetPixelDynamicRangeMin() const = 0;
        virtual int64_t GetPixelDynamicRangeMax() const = 0;
        virtual int64_t GetTimestampLatchValue() const = 0;
        virtual int64_t GetLineStatusAll() const = 0;
        virtual int64_t GetCounterValue() const = 0;
        virtual float64_t GetTimerValue() const = 0;
        virtual int64_t GetScanLineSelector() const = 0;
        virtual int64_t GetEncoderValue() const = 0;
        virtual int64_t GetLinePitch() const = 0;
        virtual int64_t GetTransferBlockID() const = 0;
        virtual int64_t GetTransferQueueCurrentBlockCount() const = 0;
        virtual int64_t GetStreamChannelID() const = 0;
        virtual float64_t GetScan3dCoordinateScale() const = 0;
        virtual float64_t GetScan3dCoordinateOffset() const = 0;
        virtual float64_t GetScan3dInvalidDataValue() const = 0;
        virtual float64_t GetScan3dAxisMin() const = 0;
        virtual float64_t GetScan3dAxisMax() const = 0;
        virtual float64_t GetScan3dTransformValue() const = 0;
        virtual float64_t GetScan3dCoordinateReferenceValue() const = 0;

    protected:
        IChunkData() {};

    };
    /*@}*/

    /*@}*/

}
#endif // PGR_SPINNAKER_ICHUNKDATA_H
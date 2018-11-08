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

#ifndef PGR_SPINNAKER_IDEVICEEVENT_H
#define PGR_SPINNAKER_IDEVICEEVENT_H

#include "Event.h"
#include "SpinnakerPlatform.h"

namespace Spinnaker
{
    class IDeviceEvent : public virtual Event
    {
    public:
        virtual ~IDeviceEvent() {};
        virtual void OnDeviceEvent(Spinnaker::GenICam::gcstring eventName) = 0;
        virtual uint64_t GetDeviceEventId() const = 0;
        virtual GenICam::gcstring GetDeviceEventName() const = 0;

    protected:
        IDeviceEvent() {};
        IDeviceEvent(const IDeviceEvent&) {};
        IDeviceEvent& operator=(const IDeviceEvent&);
    };
}

#endif /* PGR_SPINNAKER_IDEVICEEVENT_H */
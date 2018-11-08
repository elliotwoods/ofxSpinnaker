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

#ifndef PGR_SPINNAKER_IINTERFACEVENT_H
#define PGR_SPINNAKER_IINTERFACEVENT_H

#include "Event.h"
#include "ArrivalEvent.h"
#include "RemovalEvent.h"
#include "SpinnakerPlatform.h"

namespace Spinnaker
{
    class SPINNAKER_API IInterfaceEvent : public virtual IArrivalEvent, public virtual IRemovalEvent
    {
    public:
        virtual ~IInterfaceEvent() {};
        virtual void OnDeviceArrival(uint64_t serialNumber) = 0;
        virtual void OnDeviceRemoval(uint64_t serialNumber) = 0;

    protected:
        IInterfaceEvent() {};
        IInterfaceEvent(const IInterfaceEvent&) {};
        IInterfaceEvent& operator=(const IInterfaceEvent&);
    };
}

#endif /* PGR_SPINNAKER_IINTERFACEVENT_H */
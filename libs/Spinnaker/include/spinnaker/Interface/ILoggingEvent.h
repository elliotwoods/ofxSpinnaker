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

#ifndef PGR_SPINNAKER_ILOGGINGEVENT_H
#define PGR_SPINNAKER_ILOGGINGEVENT_H

#include "SpinnakerDefs.h"
#include "SpinnakerPlatform.h"
#include "Event.h"
#include "LoggingEventDataPtr.h"

namespace Spinnaker
{
    class LoggingEventDataPtr;

    class ILoggingEvent : public virtual Event
    {
    public:
        virtual ~ILoggingEvent() {};
        virtual void OnLogEvent(LoggingEventDataPtr eventPtr) = 0;

    protected:
        ILoggingEvent() {};
        ILoggingEvent(const ILoggingEvent&) {};
        ILoggingEvent& operator=(const ILoggingEvent&);
    };
}

#endif /* PGR_SPINNAKER_ILOGGINGEVENT_H */
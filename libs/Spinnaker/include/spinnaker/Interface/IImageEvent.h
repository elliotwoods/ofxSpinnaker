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

#ifndef PGR_SPINNAKER_IIMAGEEVENT_H
#define PGR_SPINNAKER_IIMAGEEVENT_H

#include "Event.h"
#include "Spinnaker.h"
#include "SpinnakerPlatform.h"

namespace Spinnaker
{
    class IImageEvent : public virtual Event
    {
    public:
        virtual ~IImageEvent() {};
        virtual void OnImageEvent(ImagePtr image) = 0;

    protected:
        IImageEvent() {};
        IImageEvent(const IImageEvent&) {};
        IImageEvent& operator=(const IImageEvent&);
    };
}

#endif /* PGR_SPINNAKER_IIMAGEEVENT_H */
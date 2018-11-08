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

#ifndef PGR_SPINNAKER_DEVICEEVENT_H
#define PGR_SPINNAKER_DEVICEEVENT_H

#include "Interface/IDeviceEvent.h"

namespace Spinnaker
{
    /**
     *  @defgroup SpinnakerEventClasses Spinnaker Event Classes
     */
     /*@{*/

     /**
     *  @defgroup DeviceEvent_h DeviceEvent Class
     */
     /*@{*/

    /**
    * @brief A handler to device events.
    */

    class SPINNAKER_API DeviceEvent : public IDeviceEvent
    {
    public:
        /**
        * Default constructor.
        */
        DeviceEvent();

        /**
        * Virtual destructor.
        */
        virtual ~DeviceEvent();

        /**
        * Device event callback.
        *
        * @param eventName The name of the event
        */
        virtual void OnDeviceEvent(Spinnaker::GenICam::gcstring eventName) = 0;

        /**
        * Get the ID of the device event.
        *
        * @return The device event ID
        */
        uint64_t GetDeviceEventId() const;

        /**
        * Get the name of the device event.
        *
        * @return The device event name
        */
        GenICam::gcstring GetDeviceEventName() const;

    protected:
        /**
        * Assignment operator.
        */
        DeviceEvent& operator=( const DeviceEvent& );
    };
    /*@}*/

    /*@}*/
}

#endif // PGR_SPINNAKER_DEVICEEVENT_H
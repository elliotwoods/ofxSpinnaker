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

#ifndef FLIR_SPINNAKER_ISYSTEM_H
#define FLIR_SPINNAKER_ISYSTEM_H

#include "SpinnakerPlatform.h"
#include "SpinnakerDefs.h"
#include "InterfaceList.h"
#include "CameraList.h"
#include "LoggingEvent.h"

namespace Spinnaker
{
    // Forward declaration of implementation class
    class LoggingEvent;
    /**
    * @defgroup SpinnakerClasses Spinnaker Classes
    */
    /*@{*/

    /**
    * @defgroup ISystem_h ISystem Class
    */
    /*@{*/

    /**
    * @brief The interface file for System.
    */

    class SPINNAKER_API ISystem
    {
    public:

        virtual ~ISystem() {};

        virtual void ReleaseInstance() = 0;
        virtual InterfaceList GetInterfaces(bool updateInterface = true) = 0;
        virtual CameraList GetCameras(bool updateInterfaces = true, bool updateCameras = true) = 0;
        virtual bool UpdateCameras(bool updateInterfaces = true) = 0;
        virtual void RegisterInterfaceEvent(Event & evtToRegister, bool updateInterface = true) = 0;
        virtual void UnregisterInterfaceEvent(Event & evtToUnregister) = 0;
        virtual void RegisterLoggingEvent(LoggingEvent & handler) = 0;
        virtual void UnregisterAllLoggingEvent() = 0;
        virtual void UnregisterLoggingEvent(LoggingEvent & handler) = 0;
        virtual void SetLoggingEventPriorityLevel(SpinnakerLogLevel level) = 0;
        virtual SpinnakerLogLevel GetLoggingEventPriorityLevel() = 0;
        virtual bool IsInUse() = 0;
        virtual void SendActionCommand(unsigned int deviceKey, unsigned int groupKey, unsigned int groupMask, unsigned long long actionTime = 0, unsigned int* pResultSize = 0, ActionCommandResult results[] = NULL) = 0;
        virtual const LibraryVersion GetLibraryVersion() = 0;

    protected:
        ISystem() {};
        ISystem(const ISystem&) {};
        ISystem& operator=(const ISystem&);
    };

    /*@}*/

    /*@}*/
}

#endif //FLIR_SPINNAKER_ISYSTEM_H

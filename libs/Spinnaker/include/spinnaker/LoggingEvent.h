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

#ifndef PGR_SPINNAKER_LOGGINGEVENT_H
#define PGR_SPINNAKER_LOGGINGEVENT_H

#include "Interface/ILoggingEvent.h"
#include "LoggingEventDataPtr.h"

namespace Spinnaker
{
	class LoggingEventDataPtr;

	/**
	 *  @defgroup SpinnakerEventClasses Spinnaker Event Classes
	 */
	/*@{*/

	/**
	 *  @defgroup LoggingEvent_h LoggingEvent Class
	 */
	/*@{*/

    /**
     * @brief An event handler for capturing the device logging event.
     */

	class SPINNAKER_API LoggingEvent : public ILoggingEvent
	{
	public:

		/**
		* Default constructor.
		*/
		LoggingEvent();

		/**
		* Virtual destructor.
		*/
		~LoggingEvent();

		/**
		* The callback for the log event.
		*
		* @param eventPtr The logging event pointer
		*
		*/
		virtual void OnLogEvent(LoggingEventDataPtr eventPtr) = 0;

	protected:
		/**
		* Assignment operator.
		*/
		LoggingEvent& operator=(const LoggingEvent&);
	};

    /*@}*/

    /*@}*/
}

#endif // PGR_SPINNAKER_LOGGINGEvent_H
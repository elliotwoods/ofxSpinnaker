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

#ifndef PGR_LOGGINGEVENTDATAPTR_H
#define PGR_LOGGINGEVENTDATAPTR_H

#include "BasePtr.h"
#include "LoggingEventData.h"

namespace Spinnaker
{
	class LoggingEventData;

	/**
	 *  @defgroup SpinnakerEventClasses Spinnaker Event Classes
	 */
	/*@{*/

	/**
	 *  @defgroup LoggingEventDataPtr_h LoggingEventDataPtr Class	 
	 */
	/*@{*/

    /**
     * @brief A reference tracked pointer to the LoggingEvent object.
     */

	class SPINNAKER_API LoggingEventDataPtr : public BasePtr<LoggingEventData>
	{

	public:
		/**
		* Default Constructor
		*/
		LoggingEventDataPtr() throw()
			: BasePtr<LoggingEventData>()
		{
		}

		/**
		* Default Constructor
		*/
		LoggingEventDataPtr(const int /*nMustBeNull*/) throw()
			: BasePtr<LoggingEventData>()
		{
		}

		/**
		* Copy Constructor
		*/
		virtual LoggingEventDataPtr& operator=(const int nMustBeNull){ return dynamic_cast<LoggingEventDataPtr&>(BasePtr<LoggingEventData>::operator=(nMustBeNull)); };

		/**
		* Virtual Destructor
		*/
		virtual ~LoggingEventDataPtr(void){};
		
	};
    /*@}*/

    /*@}*/
}

#endif //PGR_LOGGINGEVENTDATAPTR_H
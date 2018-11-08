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

#ifndef PGR_SPINNAKER_EVENT_H
#define PGR_SPINNAKER_EVENT_H

#include "SpinnakerDefs.h"
#include "SpinnakerPlatform.h"
#include "SpinGenApi/GCString.h"

namespace Spinnaker
{
	/**
	 *  @defgroup SpinnakerEventClasses Spinnaker Event Classes
	 */
	/*@{*/

	/**
	 *  @defgroup Event_h Event Class
	 */
	 /*@{*/

    /**
    * @brief The base class for all event types.
    */

	class SPINNAKER_API Event
	{
	public:

		/**
		* Virtual Destructor
		*/
		virtual ~Event();

		/**
		* Sets the event type
		*
		* @param eventType The event type
		*
		*/
		void SetEventType(EventType eventType);

		/**
		* Gets the event type
		*
		* @return The event type
		*
		*/
		EventType GetEventType();

		/**
		* Gets the event payload data
		*
		* @return The event payload data
		*/
		const uint8_t* GetEventPayloadData();

		/**
		* Gets the event payload data size
		*
		* @return The event payload data size
		*/
		const size_t GetEventPayloadDataSize();

	protected:
		Event();
		Event& operator=( const Event& );
		friend class EventProcessor;
		friend class IDataStream;
		friend class Stream;
		struct EventData;
		EventData* m_pEventData;
		void SetEventPayload(uint8_t* offset, size_t length);
	};

	/*@}*/

	/*@}*/
}

#endif // PGR_SPINNAKER_EVENT_H
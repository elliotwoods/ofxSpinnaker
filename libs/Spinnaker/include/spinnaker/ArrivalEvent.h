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

#ifndef PGR_SPINNAKER_ARRIVALEVENT_H
#define PGR_SPINNAKER_ARRIVALEVENT_H

#include "Interface/IArrivalEvent.h"

namespace Spinnaker
{
	/**
	 *  @defgroup SpinnakerEventClasses Spinnaker Event Classes
	 */

	/*@{*/

	/**
	 *  @defgroup ArrivalEvent_h ArrivalEvent Class
	 */

	/*@{*/
	
	/**
	 * @brief An event handler for capturing the device arrival event.
	 */
	class SPINNAKER_API  ArrivalEvent : public IArrivalEvent
	{
	public:
		/**
		* Default constructor.
		*/
		ArrivalEvent();

		/**
		* Virtual destructor.
		*/
		virtual ~ArrivalEvent();

		/**
		* Callback to the device arrival event.		
		*/
		virtual void OnDeviceArrival(uint64_t serialNumber) = 0;

	protected:
		/**
		* Assignment operator.
		*/
		ArrivalEvent& operator=( const ArrivalEvent& );
	};

	/*@}*/

	/*@}*/
}

#endif // PGR_SPINNAKER_ARRIVALEVENT_H
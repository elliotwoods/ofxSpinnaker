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

#ifndef PGR_SPINNAKER_INTERFACEEVENT_H
#define PGR_SPINNAKER_INTERFACEEVENT_H

#include "Interface/IInterfaceEvent.h"

namespace Spinnaker
{
	/**
	 * @defgroup SpinnakerEventClasses Spinnaker Event Classes
	 */
	/*@{*/

	/**
	 *  @defgroup InterfaceEvent_h InterfaceEvent Class
	 */
	/*@{*/

    /**
    * @brief A handler to device arrival and removal events on all interfaces.
    */

    class SPINNAKER_API InterfaceEvent : public IInterfaceEvent
	{
	public:
		/**
		* Default constructor.
		*/
		InterfaceEvent();

		/**
		* Virtual destructor.
		*/
		virtual ~InterfaceEvent();

		/**
		* Device arrival event callback.
		*/
		virtual void OnDeviceArrival(uint64_t serialNumber) = 0;

		/**
		* Callback to the device removal event.
		*
		* @param serialNumber The serial number of the removed device
		*/
		virtual void OnDeviceRemoval(uint64_t serialNumber) = 0;

	protected:
		/**
		* Assignment operator.
		*/
		InterfaceEvent& operator=( const InterfaceEvent& );
	};
	/*@}*/

	/*@}*/
}

#endif // PGR_SPINNAKER_INTERFACEEVENT_H
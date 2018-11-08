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

#ifndef PGR_SPINNAKER_IMAGEEVENT_H
#define PGR_SPINNAKER_IMAGEEVENT_H

#include "Interface/IImageEvent.h"

namespace Spinnaker
{
	/**
	 *  @defgroup SpinnakerEventClasses Spinnaker Event Classes
	 */

	/*@{*/

	/**
	 *  @defgroup ImageEvent_h ImageEvent Class
	 */

	/*@{*/

    /**
    * @brief A handler for capturing image arrival events.
    */

	class SPINNAKER_API ImageEvent : public IImageEvent
	{
	public:
		/**
		* Default Constructor
		*/
		ImageEvent();

		/**
		* Virtual Destructor
		*/
		virtual ~ImageEvent();

		/**
		* Image event callback
		*
		* @param image The ImagePtr object
		*/
		virtual void OnImageEvent(ImagePtr image) = 0;

	protected:
		/**
		* Assignment operator.
		*/
		ImageEvent& operator=( const ImageEvent& );
	};

	/*@}*/

	/*@}*/
}

#endif // PGR_SPINNAKER_IMAGEEVENT_H
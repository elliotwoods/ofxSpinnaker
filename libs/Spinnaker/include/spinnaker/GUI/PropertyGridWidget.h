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

#pragma warning(push)
#pragma warning(disable : 4275) //non dll-interface class 'Gtk::VBox' used as base for dll-interface class
#pragma warning(disable : 4250) // warning C4250: inherits 'Gtk::Object::Gtk::Object::destroy_notify_' via dominance   This kills all gtk warnings. For some reason have to put it outsite of #pragma warning(push)
#pragma once
#include "Spinnaker.h"

namespace Spinnaker
{
	namespace GUI_GTKmm
	{
		/**
		 * @defgroup SpinnakerGUIGTKmm Spinnaker GUI GTKmm Factory Class
		 */

		/*@{*/

		/**
		 * @defgroup PropertyGridWidget_h Spinnaker GUI GTKmm Property Grid Widget Class
		 */

		/*@{*/

		class SPINNAKER_API PropertyGridWidget : public Gtk::VBox
		{
			public:

				/**
				* Default constructor.
				*/
				PropertyGridWidget(void);


				/**
				* Default destructor.
				*/
				~PropertyGridWidget(void);


				/**
				 * Connect the PropertyGridWidgetWindow to a Camera object.
				 *
				 * @return boolean indicating whether the operation was successful
				 */
				bool Connect(Spinnaker::CameraPtr camera);


				/**
				 * Connect the PropertyGridWidgetWindow to a NodeMap object.
				 *
				 * @return boolean indicating whether the operation was successful
				 */
				bool Connect(Spinnaker::GenApi::INodeMap& nodeMap);

				/**
				 * Disconnect the PropertyGridWidgetWindow from associated
				 * nodemap object.
				 *
				 */
				void Disconnect();


				/**
				 * Get a boolean indicating whether PropertyGridWidgetWindow is connected to a camera object.
				 *
				 * @return boolean indicating connection status.
				 */
				bool IsConnected();


				/**
				 * Immediately refresh all features listed in PropertyGridWidget.
				 *
				 */
				void Refresh();


				/**
				 * Get the window title.
				 *
				 * @return string representing the title.
				 */
				Glib::ustring GetID();


				/**
				 * Set the window title.
				 *
				 * @param string representing the title.
				 */
				void SetID(Glib::ustring id);


				/**
				 * Show the PropertyGridWidgetWindow.
				 *
				 */
				void Open();


				/**
				 * Hide the PropertyGridWidgetWindow.
				 *
				 */
				void Close();

			protected:
				struct PropertyGridWidgetData;
				PropertyGridWidgetData* m_pPropertyGridWidgetData;
		};

		/*@}*/

		/*@}*/
	}
}
#pragma warning(pop)
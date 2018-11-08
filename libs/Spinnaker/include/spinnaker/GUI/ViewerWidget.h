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
		 * @defgroup ViewerWidget_h Spinnaker GUI GTKmm Viewer Widget Class
		 */

		/*@{*/

		class SPINNAKER_API ViewerWidget : public Gtk::VBox
		{
		public:

			/**
			* Default constructor.
			*/
			ViewerWidget(void);


			/**
			 * Default destructor.
			 */
			~ViewerWidget(void);


			/**
			 * Connect ImageDrawingWindow to camera.
			 *
			 * @param pointer to a camera object
			 *
			 * @return boolean indicating whether the operation was successful
			 */
			bool Connect(Spinnaker::CameraPtr camera);


			/**
			 * Disconnect the ImageDrawingWindow from associated
			 * camera object.
			 *
			 */
			void Disconnect();


			/**
			 * Start image streaming.
			 *
			 */
			void StartStreaming();


			/**
			 * Get a boolean indicating whether Window is connected to a camera object.
			 *
			 * @return boolean indicating connection status.
			 */
			bool IsConnected(){return m_bIsConnected;};


			/**
			 * Stop image streaming.
			 *
			 */
			void StopStreaming();


			/**
			 * Set the window title.
			 *
			 * @param string representing the title.
			 */
			Glib::ustring GetID();

			/**
			 * Set the window title.
			 *
			 * @param string representing the title.
			 */
			void SetID(Glib::ustring id);


			/**
			 * Open the ImageDrawingWindow.
			 *
			 */
			void Open();

			/**
			 * Hide the ImageDrawingWindow.
			 *
			 */
			void Close();
		protected:
				struct ViewerWidgetData;
				ViewerWidgetData* m_pViewerWidgetData;
				bool m_bIsConnected;
		};

		/*@}*/

		/*@}*/
	}
}
#pragma warning(pop)
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
		 * @defgroup Viewer_h Spinnaker GUI GTKmm Viewer Class
		 */

		/*@{*/

		class ViewerWidget;
		class SPINNAKER_API Viewer
		{
		public:

			/**
			* Default constructor.
			*/
			Viewer(void);


			/**
			 * Default destructor.
			 */
			~Viewer(void);


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
			*Get a new Viewer widget
			*@return a Gtk::Widget* pointer for Viewer
			*/
			ViewerWidget* GetWidget();


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
				struct ViewerData;
				ViewerData* m_pViewerData;
				bool m_bIsConnected;
		};

		/*@}*/

		/*@}*/
	}
}

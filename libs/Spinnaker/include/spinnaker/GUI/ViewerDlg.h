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
	namespace GUI
	{
		/**
		 * @defgroup SpinnakerGUI Spinnaker GUI Classes
		 */

		/*@{*/

		/**
		 * @defgroup ViewerDlg_h Spinnaker GUI Viewer Dialog Class
		 */

		/*@{*/

		class SPINNAKER_API ViewerDlg
		{
		public:

			/**
			* Default constructor.
			*/
			ViewerDlg(void);


			/**
			 * Default destructor.
			 */
			~ViewerDlg(void);


			/**
			 * Connect ImageDrawingWindow to camera.
			 *
			 * @param pointer to a camera object
			 *
			 * @return boolean indicating whether the operation was successful
			 */
			bool Connect(Spinnaker::CameraPtr camera);


			/**
			 * Start image streaming.
			 *
			 */
			void StartStreaming();


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
			std::string GetTitle();


			/**
			 * Get the window title.
			 *
			 * @return string representing the title.
			 */
			void SetTitle(std::string title);


			/**
			 * Disconnect the ImageDrawingWindow from associated
			 * camera object.
			 *
			 */
			void Disconnect();


			/**
			 * Get a boolean indicating whether Window is connected to a camera object.
			 *
			 * @return boolean indicating connection status.
			 */
			bool IsConnected();


			/**
			 * Show the ImageDrawingWindow.
			 *
			 */
			void Open();


			/**
			 * Hide the ImageDrawingWindow.
			 *
			 */
			void Close();
		protected:

	#ifdef GUIFrame_GTK
			Spinnaker::GUI_GTKmm::Viewer* m_pGTKViewer;
	#else
			Spinnaker::GUI_WPF::ImageDrawingWindow* m_pWPFViewer;
	#endif
		};

		/*@}*/

		/*@}*/
	}
}
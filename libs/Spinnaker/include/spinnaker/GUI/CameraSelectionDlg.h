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

namespace Spinnaker
{
	namespace GUI
	{
		/**
		 * @defgroup SpinnakerGUI Spinnaker GUI Classes
		 */

		/*@{*/

		/**
		 * @defgroup CameraSelectionDlg_h Spinnaker GUI Camera Selection Dialog Class
		 */

		/*@{*/

		class SPINNAKER_API CameraSelectionDlg
		{
			public:

				/**
				* Default constructor.
				*/
				CameraSelectionDlg(void);


				/**
				* Default destructor.
				*/

				~CameraSelectionDlg(void);

				/**
				 * Register callback function for double clicked event on listed devices in CameraSelectionWindow
				 *
				 * @param function pointer to double clicked event
				 */
				void RegisterDoubleClickedCallback(Spinnaker::GUI_WPF::ClickedEventCallback pCallBackFunc);


				/**
				 * Register callback function for single clicked event on listed devices in CameraSelectionWindow
				 *
				 * @param function pointer to single clicked event
				 */
				void RegisterSingleClickedCallback(Spinnaker::GUI_WPF::ClickedEventCallback pCallBackFunc);


				/**
				 * Rescan devices and refresh CameraSelectionWindow.
				 *
				 */
				void Refresh();


				/**
				 * Show the CameraSelectionWindow.
				 *
				 */
				void Open();


				/**
				 * Hide the CameraSelectionWindow.
				 *
				 */
				void Close();


				/**
				 * Set the window title.
				 *
				 * @param string representing the title.
				 */
				void SetTitle(std::string title);


				/**
				 * Get the window title.
				 *
				 * @return string representing the title.
				 */
				std::string GetTitle();

			private:

	#ifdef GUIFrame_GTK
				Spinnaker::GUI_GTKmm::CameraSelection* m_pGTKCameraSelection;
	#else
				Spinnaker::GUI_WPF::CameraSelectionWindow* m_pWPFCameraSelection;
	#endif
		};

		/*@}*/

		/*@}*/
	}
}
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
		 * @defgroup PropertyGridDlg_h Spinnaker GUI Property Grid Class
		 */

		/*@{*/

		class SPINNAKER_API PropertyGridDlg
		{
			public:

				/**
				* Default constructor.
				*/
				PropertyGridDlg(void);


				/**
				* Default destructor.
				*/
				~PropertyGridDlg(void);

				/**
				 * Connect the PropertyGridWindow to a NodeMap object.
				 *
				 * @return boolean indicating whether the operation was successful
				 */
				bool Connect(Spinnaker::CameraPtr camera);


				/**
				 * Disconnect the PropertyGridWindow from associated
				 * nodemap object.
				 *
				 */
				void Disconnect();


				/**
				 * Get a boolean indicating whether PropertyGridWindow is connected to a camera object.
				 *
				 * @return boolean indicating connection status.
				 */
				bool IsConnected();


				/**
				 * Immediately refresh all features listed in PropertyGrid.
				 *
				 */
				void Refresh();



				/**
				 * Get the window title.
				 *
				 * @return string representing the title.
				 */
				std::string GetTitle();

				/**
				 * Set the window title.
				 *
				 * @param string representing the title.
				 */
				void SetTitle(std::string title);


				/**
				 * Show the PropertyGridWindow.
				 *
				 */
				void Open();

				/**
				 * Hide the PropertyGridWindow.
				 *
				 */
				void Close();

			protected:
	#ifdef GUIFrame_GTK
				Spinnaker::GUI_GTKmm::PropertyGrid* m_pGtkPropertyGrid;
	#else
				Spinnaker::GUI_WPF::PropertyGridWindow* m_pWPFPropertyGrid;
	#endif
		};

		/*@}*/

		/*@}*/
	}
}

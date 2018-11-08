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

#ifdef GUIFrame_GTK
	#include "GUI/SpinnakerGUI_GTKmm.h"
#else
	#include "GUI/SpinnakerGUI_WPF.h"
#endif

#include "ViewerDlg.h"
#include "PropertyGridDlg.h"
#include "CameraSelectionDlg.h"

namespace Spinnaker
{
	namespace GUI
	{
		/**
		 * @defgroup SpinnakerGUI Spinnaker GUI Classes
		 */

		/*@{*/

		/**
		 * @defgroup SpinnakerGUIFactory Spinnaker GUI Factory Class
		 */

		/*@{*/

		class SPINNAKER_API GUIFactory
		{
			public:

				/**
				 * Default constructor.
				 */
				GUIFactory(void);


				/**
				 * Default destructor.
				 */
				~GUIFactory(void);


				/**
				 * Statically Get Camera Selection Window from GUI Factory
				 *
				 * @return A pointer to CameraSelectionWindow. User has ownership of this pointer.
				 */
				static CameraSelectionDlg* GetCameraSelectionDlg();


				/**
				 * Statically Get PropertyGridWindow from GUI Factory.
				 *
				 * @return A pointer to PropertyGridWindow. User has ownership of this pointer.
				 */
				static PropertyGridDlg* GetPropertyGridDlg();


				/**
				 * Statically Get Image Drawing Window
				 *
				 * @return A pointer to ImageDrawingWindow. User has ownership of this pointer.
				 */
				static ViewerDlg* GetViewer();


				/**
				 * Connect GUI Factory to camera object
				 *
				 * @param pointer to a camera object
				 * @return boolean indicating whether operation was successful
				 */
				bool ConnectGUILib(Spinnaker::CameraPtr camera);


				/**
				 * Disconnect GUI Factory from camera object.
				 *
				 */
				void DisconnectGUILib();


				/**
				 * Get a boolean indicating whether GUI Factory is connected to a camera object.
				 *
				 * @return boolean indicating connection status.
				 */
				bool IsConnected(){return m_bisConnected;}


				/**
				 * Get a list of dialog names
				 *
				 * @return string list of dialog names
				 */
				std::list<std::string> GetDialogNameList();


				/**
				 * Get dialog with specific name
				 *
				 * @param string holding dialog name
				 */
				std::list<std::string> GetControlNameList();


				/**
				 * Get number of dialogs contained in GUI Factory
				 *
				 * @return number of dialogs
				 */
				int GetNumDlgs();


				/**
				 * Display dialog at specific name list index
				 *
				 * @param name index of dialog to be shown
				 */
				void ShowDlgByIndex(unsigned int index);


				/**
				 * Display dialog with specific name
				 *
				 * @param string holding dialog name
				 */
				void ShowDlgByName(std::string dialogName);

				/**
				 * Hide dialog at specific name list index
				 *
				 * @param name index of dialog to be shown
				 */
				void HideDlgByIndex(unsigned int index);


				/**
				 * Hide dialog with specific name
				 *
				 * @param string holding dialog name
				 */
				void HideDlgByName(std::string dialogName);


				/**
				 * Return number of controls contained in GUI Factory
				 *
				 * @return number of controls
				 */
				int GetNumCtrls();

			protected:
			static CameraSelectionDlg* m_pCameraSelectionDialog;
			static ViewerDlg* m_pViewer;
			static PropertyGridDlg* m_pPropertyGridDialog;

	#ifdef GUIFrame_GTK
			//GTK GUI API
			Spinnaker::GUI_GTKmm::GUIFactory* m_pGTKGUIAPI;
	#else
			//WPF GUI API
			Spinnaker::GUI_WPF::GUIFactory* m_pWPFGUIFactory;
	#endif
			bool m_bisConnected;
	//		CameraControlProxy* m_pCameraControlProxy;
		};

		/*@}*/

		/*@}*/
	}
}

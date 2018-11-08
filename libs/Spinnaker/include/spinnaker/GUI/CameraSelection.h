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
		 * @defgroup CameraSelection_h Spinnaker GUI GTKmm Camera Selection Class
		 */

		/*@{*/

		class CameraSelectionWidget;
		class SPINNAKER_API CameraSelection
		{
			public:

				/**
				 * Default constructor.
				 */
				CameraSelection(void);


				/**
				 * Default destructor.
				 */
				~CameraSelection(void);


				/**
				 * Callback signal for single clicked event on listed devices in CameraSelectionWindow
				 * The user needs to connect the signal to functions which have parameters of   void pFunction (bool, Spinnaker::Camera*, Spinnaker::Interface*);
				 * @param bool isCameraClicked,  Spinnaker::Camera* camera (returns NULL is isCameraClicked == false), Spinnaker::Interface* interface(returns NULL is isCameraClicked == true)
				 */
				sigc::signal<void, bool, Spinnaker::CameraPtr*, Spinnaker::Interface*> signal_single_clicked();


				/**
				 * Callback signal for double clicked event on listed devices in CameraSelectionWindow
				 * The user needs to connect the signal to functions which have parameters of   void pFunction (bool, Spinnaker::Camera*, Spinnaker::Interface*);
				 * @param bool isCameraClicked,  Spinnaker::Camera* camera (returns NULL is isCameraClicked == false), Spinnaker::Interface* interface(returns NULL is isCameraClicked == true)
				 */
				sigc::signal<void, bool, Spinnaker::CameraPtr*, Spinnaker::Interface*> signal_double_clicked();


				/**
				 * Callback signal for start or stop device clicked event on listed devices in CameraSelectionWidgetWindow
				 * The user needs to connect the signal to functions which have parameters of void pFunction (Spinnaker::Camera*);
				 * @param Spinnaker::Camera* pCamera
				 */
				sigc::signal<void, Spinnaker::CameraPtr> signal_startstop_device_clicked();

				/**
				 * Callback signal for configure device clicked event on listed devices in CameraSelectionWidgetWindow
				 * The user needs to connect the signal to functions which have parameters of void pFunction (Spinnaker::Camera*);
				 * @param Spinnaker::Camera* pCamera
				 */
				sigc::signal<void, Spinnaker::CameraPtr> signal_configure_device_clicked();

				/**
				 * Open the CameraSelectionWindow.
				 *
				 */
				void Open();


				/**
				 * Close the CameraSelectionWindow.
				 *
				 */
				void Close();


				/**
				 * Rescan devices and refresh CameraSelectionWindow.
				 *
				 */
				void Refresh();


				/**
				 *Get a new Camera Selection widget

				 *@return a Gtk::Widget* pointer for CameraSelection
				 */
				CameraSelectionWidget* GetWidget();


				/**
				 * Set the window title.
				 *
				 * @param Glib::ustring representing the title.
				 */
				void SetID(Glib::ustring id);


				/**
				 * Get the window title.
				 *
				 * @return Glib::ustring representing the title.
				 */
				Glib::ustring GetID();

			protected:
				struct CameraSelectionData;
				CameraSelectionData* m_pCameraSelectionData;
		};

		/*@}*/

		/*@}*/
	}
}
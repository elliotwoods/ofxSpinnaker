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
		 * @defgroup PropertyGrid_h Spinnaker GUI GTKmm Property Grid Class
		 */

		/*@{*/

		class PropertyGridWidget;
		class SPINNAKER_API PropertyGrid
		{
			public:

				/**
				* Default constructor.
				*/
				PropertyGrid(void);


				/**
				* Default destructor.
				*/
				~PropertyGrid(void);


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
				 *Get a new PropertyGrid widget

				 *@return a Gtk::Widget* pointer for PropertyGrid
				 */
				PropertyGridWidget* GetWidget();


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
				struct PropertyGridData;
				PropertyGridData* m_pPropertyGridData;
		};

		/*@}*/

		/*@}*/
	}
}

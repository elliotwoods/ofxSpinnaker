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

#ifndef FLIR_SPINNAKER_GUI_WPF_H
#define FLIR_SPINNAKER_GUI_WPF_H

#include <Spinnaker.h>

namespace Spinnaker
{
	namespace GUI_WPF
	{
		/**
		* @defgroup SpinnakerGUI Spinnaker GUI Classes
		*/

		/*@{*/

		/**
		* @defgroup SpinnakerGUIWPF Spinnaker GUI WPF Factory Class
		*/

		/*@{*/

		/**
		* Device clicked event callback function prototype. Defines the syntax of the
		* clicked callback function that is passed into RegisterDoubleClickedCallback()
		* and RegisterSingleClickedCallback().
		*
		* @param isCamera boolean indicating whether clicked device is a camera.
		* @param pCamera pointer to clicked camera object.
		* @param pInterface pointer to clicked interface object.
		*/
		typedef void(*ClickedEventCallback)(bool isCamera, Spinnaker::CameraPtr* pCamera, Spinnaker::InterfacePtr* pInterface);

		/**
		* Device firmware update event callback function prototype. Defines the syntax of the
		* firmware update callback function that is passed into RegisterFirmwareUpdateStartingCallback()
		* and RegisterFirmwareUpdateFinishedCallback().
		*
		* @param boolean indicating whether firmware update is starting.
		* @param boolean indicating whether firmware update is finished.
		* @param boolean indicating whether firmware update is successful.
		* @param char*	 containing error message returned from firmware update.
		* @param pointer to camera object being updated.
		*/
		typedef void(*FirmwareUpdateEventCallback)(bool isStarting, bool isFinished, bool isSuccessful, const char* pErrorMessage, Spinnaker::CameraPtr* pCamera);

		/**
		* Return type for ShowModal() method in CameraSelectionWindow
		*/
		struct DeviceInformationStruct
		{
			/** Whether the clicked device is camera. */
			bool isCamera;
			/** pointer to clicked camera object. */
			Spinnaker::CameraPtr* pCamera;
			/** pointer to clicked interface object. */
			Spinnaker::InterfacePtr* pInterface;
		};

		/**
		* The CameraSelectionWindow object represents a dialog that provides
		* a graphical interface that lists number of cameras and interfaces
		* available to the library.
		*/
		class SPINNAKER_API CameraSelectionWindow
		{

		public:

			/**
			* Default constructor.
			*/
			CameraSelectionWindow();

			/**
			* Default destructor.
			*/
			~CameraSelectionWindow();

			/**
			* Show the CameraSelectionWindow.
			*
			*/
			void Show();

			/**
			* Show the CameraSelectionWindow as modal.
			*
			* @param devInfoStruct DeviceInformationStruct containing information of selected device or interface object
			*
			* @return True = a valid selection was made. False = dialog was closed and no selection was made.
			*/
			bool ShowModal(DeviceInformationStruct* devInfoStruct);


			/**
			* Rescan devices and refresh CameraSelectionWindow.
			*
			*/
			void Refresh();

			/**
			* Hide the CameraSelectionWindow.
			*
			*/
			void Hide();

			/**
			* Set the window title.
			*
			* @param title string representing the title.
			*/
			void SetTitle(std::string title);

			/**
			* Get the window title.
			*
			* @return string representing the title.
			*/
			std::string GetTitle();

			/**
			* Register callback function for double clicked event on listed devices in CameraSelectionWindow
			*
			* @param pCallBackFunc function pointer to double clicked event
			*/
			void RegisterDoubleClickedCallback(ClickedEventCallback pCallBackFunc);

			/**
			* Register callback function for single clicked event on listed devices in CameraSelectionWindow
			*
			* @param pCallBackFunc function pointer to single clicked event
			*/
			void RegisterSingleClickedCallback(ClickedEventCallback pCallBackFunc);

			/**
			* Register callback function for start/stop context menu event in CameraSelectionWindow
			*
			* @param pCallBackFunc function pointer to startstop context menu clicked event
			*/
			void RegisterStartStopContextMenuCallback(ClickedEventCallback pCallBackFunc);

			/**
			* Register callback function for configure device context menu event in CameraSelectionWindow
			*
			* @param pCallBackFunc function pointer to configure device context menu clicked event
			*/
			void RegisterConfigureDeviceContextMenuCallback(ClickedEventCallback pCallBackFunc);

			/**
			* Register callback function for firmware update starting event in CameraSelectionWindow
			*
			* @param pCallBackFunc function pointer to firmware update event
			*/
			void RegisterFirmwareUpdateStartingCallback(FirmwareUpdateEventCallback pCallBackFunc);

			/**
			* Register callback function for firmware update finished event in CameraSelectionWindow
			*
			* @param pCallBackFunc function pointer to firmware update event
			*/
			void RegisterFirmwareUpdateFinishedCallback(FirmwareUpdateEventCallback pCallBackFunc);

			/**
			* Register callback function for disconnecting a connected device in CameraSelectionWindow through menu or context menu
			*
			* @param pCallBackFunc function pointer to disconnect event
			*/
			void RegisterDisconnectDeviceMenuCallback(ClickedEventCallback pCallBackFunc);

			/**
			* Unregister callback function for double clicked event in CameraSelectionWindow
			*
			* @param pCallBackFunc function pointer to double clicked event
			*/
			void UnregisterDoubleClickedCallback(ClickedEventCallback pCallBackFunc);

			/**
			* Unregister callback function for single clicked event in CameraSelectionWindow
			*
			* @param pCallBackFunc function pointer to single clicked event
			*/
			void UnregisterSingleClickedCallback(ClickedEventCallback pCallBackFunc);

			/**
			* Unregister callback function for StartStop context menu clicked event in CameraSelectionWindow
			*
			* @param pCallBackFunc function pointer to start/stop context menu event
			*/
			void UnregisterStartStopContextMenuCallback(ClickedEventCallback pCallBackFunc);

			/**
			* Unregister callback function for configure device context menu clicked event in CameraSelectionWindow
			*
			* @param pCallBackFunc function pointer to configure device context menu event
			*/
			void UnregisterConfigureDeviceContextMenuCallback(ClickedEventCallback pCallBackFunc);


			/**
			* Unregister callback function for firmware update starting event in CameraSelectionWindow
			*
			* @param pCallBackFunc function pointer to firmware update event
			*/
			void UnregisterFirmwareUpdateStartingCallback(FirmwareUpdateEventCallback pCallBackFunc);

			/**
			* Unregister callback function for firmware update finished event in CameraSelectionWindow
			*
			* @param pCallBackFunc function pointer to firmware update event
			*/
			void UnregisterFirmwareUpdateFinishedCallback(FirmwareUpdateEventCallback pCallBackFunc);

			/**
			* Unregister callback function for disconnecting a connected device in CameraSelectionWindow through menu or context menu
			*
			* @param pCallBackFunc function pointer to disconnect event
			*/
			void UnregisterDisconnectDeviceMenuCallback(ClickedEventCallback pCallBackFunc);
		private:
			struct CamSelectionData; // Forward declaration

			CamSelectionData* m_pCamSelectionData;
		};

		/**
		* The PropertyGridWindow object represents a dialog that provides
		* a graphical interface to view and control features on a camera
		* object.
		*/
		class SPINNAKER_API PropertyGridWindow
		{
		public:
			/**
			* Default constructor.
			*/
			PropertyGridWindow();

			/**
			* Default destructor.
			*/
			~PropertyGridWindow();


			/**
			* Connect the PropertyGridWindow to a NodeMap object.
			*
			* @return boolean indicating whether the operation was successful
			*/
			bool Connect(Spinnaker::CameraPtr* pCam);


			/**
			* Disconnect the PropertyGridWindow from associated
			* nodeMap object.
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
			void RefreshFeatures();

			/**
			* Show the PropertyGridWindow.
			*
			*/
			void Show();

			/**
			* Show the PropertyGridWindow as Modal.
			*
			*/
			void ShowModal();

			/**
			* Hide the PropertyGridWindow.
			*
			*/
			void Hide();

			/**
			* Check whether PropertyGridWindow is visible on screen.
			*
			* @return boolean indicating whether the window is visible
			*/
			bool IsVisible();

			/**
			* Set the window title.
			*
			* @param title string representing the title.
			*/
			void SetTitle(std::string title);

			/**
			* Get the window title.
			*
			* @return string representing the title.
			*/
			std::string GetTitle();

		private:
			struct PropertyGridData; // Forward declaration

			PropertyGridData* m_pPropertyGridData;
		};

		/**
		* The ImageDrawingWindow object represents a dialog that displays
		* live images from a connected camera object.
		*/
		class SPINNAKER_API ImageDrawingWindow
		{
		public:
			/**
			* Default constructor.
			*/
			ImageDrawingWindow();

			/**
			* Default destructor.
			*/
			~ImageDrawingWindow();


			/**
			* Connect ImageDrawingWindow to camera.
			*
			* @param pCam pointer to a camera object
			*
			* @return boolean indicating whether the operation was successful
			*/
			bool Connect(Spinnaker::CameraPtr* pCam);


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
			* Start image streaming.
			*
			*/
			void Start();

			/**
			* Stop image streaming.
			*/
			void Stop();

			/**
			* Single image acquisition.
			*
			*/
			void Snap();

			/**
			* Get a boolean indicating whether drawing Window is streaming images.
			*
			* @return boolean indicating streaming status.
			*/
			bool IsStreaming();

			/**
			* Show the ImageDrawingWindow.
			*
			*/
			void Show();

			/**
			* Show the ImageDrawingWindow as Modal.
			*
			*/
			void ShowModal();

			/**
			* Hide the ImageDrawingWindow.
			*
			*/
			void Hide();

			/**
			* Hide the ImageDrawingWindow.
			*
			* @return boolean indicating whether the window is visible
			*/
			bool IsVisible();

			/**
			* Set the window title.
			*
			* @param title string representing the title.
			*/
			void SetTitle(std::string title);

			/**
			* Get the window title.
			*
			* @return string representing the title.
			*/
			std::string GetTitle();

		private:
			struct ImageStreamingData; // Forward declaration

			ImageStreamingData* m_pImageStreamingData;
		};

		/**
		* The GUIFactory object represents a C++ object that provides access to
		* GUI objects such as CameraSelectionWindow, PropertyGridWindow and
		* dialogs defined in GUI XML
		*/
		class SPINNAKER_API GUIFactory
		{
		public:
			/**
			* Default constructor.
			*/
			GUIFactory();

			/**
			* Default destructor.
			*/
			~GUIFactory();

			/**
			* Connect GUI Factory to camera object
			*
			* @param camera pointer to a camera object
			* @return boolean indicating whether operation was successful
			*/
			bool ConnectGUILibrary(Spinnaker::CameraPtr* camera);

			/**
			* Disconnect GUI Factory from camera object.
			*
			*/
			void DisconnectGUILibrary();

			/**
			* Get a boolean indicating whether GUI Factory is connected to a camera object.
			*
			* @return boolean indicating connection status.
			*/
			bool IsConnected();

			/**
			* Get PropertyGridWindow from GUI Factory.
			*
			* @return A pointer to PropertyGridWindow. User has ownership of this pointer.
			*/
			PropertyGridWindow* GetPropertyGridWindow();

			/**
			* Get Camera Selection Window from GUI Factory
			*
			* @return A pointer to CameraSelectionWindow. User has ownership of this pointer.
			*/
			CameraSelectionWindow* GetCameraSelectionWindow();

			/**
			* Get Image Drawing Window
			*
			* @return A pointer to ImageDrawingWindow. User has ownership of this pointer.
			*/
			ImageDrawingWindow* GetImageDrawingWindow();

			/**
			* Get number of dialogs contained in GUI Factory
			*
			* @return number of dialogs
			*/
			int GetNumOfDialogs();

			/**
			* Get a list of dialog names
			*
			* @return string list of dialog names
			*/
			std::list<std::string> GetDialogNameList();

			/**
			* Display dialog with specific name
			*
			* @param dialogName string holding dialog name
			*/
			void ShowDialogByName(std::string dialogName);

			/**
			* Display dialog at specific name list index
			*
			* @param index name index of dialog to be shown
			*/
			void ShowDialogByIndex(int index);

			/**
			* Hide dialog by specific name
			*
			* @param dialogName name of dialog to be hidden
			*/
			void HideDialogByName(std::string dialogName);

			/**
			* Hide dialog at specific name list index
			*
			* @param index name index of dialog to be hidden
			*/
			void HideDialogByIndex(int index);

			/**
			* Return number of controls contained in GUI Factory
			*
			* @return number of controls
			*/
			int GetNumOfControls();

			/**
			* Return a string list of controls
			*
			* @return list of control names
			*/
			std::list<std::string> GetControlNameList();

		private:
			struct GUIFactoryData; // Forward declaration

			GUIFactoryData* m_pGUIFactoryData;
		};

		/*@}*/

		/*@}*/
	}
}

#endif // FLIR_SPINNAKER_GUI_WPF_H
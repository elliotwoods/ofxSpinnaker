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
   
/* Auto-generated file. Do not modify. */

#ifndef FLIR_SPINNAKER_TRANSPORTLAYERDEFS_H
#define FLIR_SPINNAKER_TRANSPORTLAYERDEFS_H

namespace Spinnaker
{
	/**
	* @defgroup SpinnakerQuickSpinClasses Spinnaker QuickSpin Classes
	*/
	/*@{*/

	/**	
	* @defgroup TransportLayerDefs_h TransportLayerDefs Class
	*/
	/*@{*/

	/**
	*@brief The enum definitions for TL Device nodes from the transport layer .xml files.
	*/

	enum StreamTypeEnum	/*!< Stream type of the device.*/
	{
		StreamType_Mixed,	/*!< Stream Type - Mixed*/
		StreamType_Custom,	/*!< Stream Type - Custom*/
		StreamType_GEV,	/*!< Stream Type - GEV*/
		StreamType_CL,	/*!< Stream Type - CL*/
		StreamType_IIDC,	/*!< Stream Type - IIDC*/
		StreamType_UVC,	/*!< Stream Type - UVC*/
		StreamType_CXP,	/*!< Stream Type - CXP*/
		StreamType_CLHS,	/*!< Stream Type - CLHS*/
		StreamType_U3V,	/*!< Stream Type - U3V*/
		StreamType_ETHERNET,	/*!< Stream Type - ETHERNET*/
		StreamType_PCI,	/*!< Stream Type - PCI*/
		NUMSTREAMTYPE
	};

	enum StreamDefaultBufferCountModeEnum	/*!< DEPRECATED; Replaced by StreamBufferCountMode. Controls access to setting the number of buffers used for the stream. Locked to Manual mode on 32-bit Windows due to memory constraints.*/
	{
		StreamDefaultBufferCountMode_Manual,	/*!< DEPRECATED. The number of buffers used for the stream are set by the user.*/
		StreamDefaultBufferCountMode_Auto,	/*!< DEPRECATED. The number of buffers used for the stream is automatically calculated.*/
		NUMSTREAMDEFAULTBUFFERCOUNTMODE
	};

	enum StreamBufferCountModeEnum	/*!< Controls access to setting the number of buffers used for the stream. Locked to Manual mode on 32-bit Windows due to memory constraints.*/
	{
		StreamBufferCountMode_Manual,	/*!< The number of buffers used for the stream are set by the user.*/
		StreamBufferCountMode_Auto,	/*!< The number of buffers used for the stream is automatically calculated based on the device frame rate.*/
		NUMSTREAMBUFFERCOUNTMODE
	};

	enum StreamBufferHandlingModeEnum	/*!< Available buffer handling modes of this data stream:*/
	{
		StreamBufferHandlingMode_OldestFirst,	/*!< The application always gets the buffer from the head of the output buffer queue (thus, the oldest available one). If the output buffer queue is empty, the application waits for a newly acquired buffer until the timeout expires.*/
		StreamBufferHandlingMode_OldestFirstOverwrite,	/*!< The application always gets the buffer from the head of the output buffer queue (thus, the oldest available one). If the output buffer queue is empty, the application waits for a newly acquired buffer until the timeout expires. If a new buffer arrives it will overwrite the existing buffer from the head of the queue (behaves like a circular buffer).*/
		StreamBufferHandlingMode_NewestFirst,	/*!< The application always gets the buffer from the tail of the output buffer queue (thus, the newest available one). If the output buffer queue is empty, the application waits for a newly acquired buffer until the timeout expires.*/
		StreamBufferHandlingMode_NewestFirstOverwrite,	/*!< DEPRECATED.  This is replaced by NewestOnly.*/
		StreamBufferHandlingMode_NewestOnly,	/*!< The application always gets the latest completed buffer (the newest one).  If the Output Buffer Queue is empty, the application waits for a newly acquired buffer until the timeout expires.  This buffer handling mode is typically used in a live display GUI where it is important that there is no lag between camera and display.*/
		NUMSTREAMBUFFERHANDLINGMODE
	};

	enum DeviceTypeEnum	/*!< Transport layer type of the device.*/
	{
		DeviceType_Mixed,	/*!< TL - Mixed*/
		DeviceType_Custom,	/*!< TL - Custom*/
		DeviceType_GEV,	/*!< TL - GEV*/
		DeviceType_CL,	/*!< TL - CL*/
		DeviceType_IIDC,	/*!< TL - IIDC*/
		DeviceType_UVC,	/*!< TL - UVC*/
		DeviceType_CXP,	/*!< TL - CXP*/
		DeviceType_CLHS,	/*!< TL - CLHS*/
		DeviceType_U3V,	/*!< TL - U3V*/
		DeviceType_ETHERNET,	/*!< TL - ETHERNET*/
		DeviceType_PCI,	/*!< TL - PCI*/
		NUMDEVICETYPE
	};

	enum DeviceAccessStatusEnum	/*!< Gets the access status the transport layer Producer has on the device.*/
	{
		DeviceAccessStatus_Unknown,	/*!< Unknown status*/
		DeviceAccessStatus_ReadWrite,	/*!< Full access*/
		DeviceAccessStatus_ReadOnly,	/*!< Read-only access*/
		DeviceAccessStatus_NoAccess,	/*!< Non-available devices*/
		NUMDEVICEACCESSSTATUS
	};

	enum GevCCPEnum	/*!< Controls the device access privilege of an application.*/
	{
		GevCCP_EnumEntry_GevCCP_OpenAccess,	/*!< Open access privilege.*/
		GevCCP_EnumEntry_GevCCP_ExclusiveAccess,	/*!< Exclusive access privilege.*/
		GevCCP_EnumEntry_GevCCP_ControlAccess,	/*!< Control access privilege.*/
		NUMGEVCCP
	};

	enum GUIXMLLocationEnum	/*!< Sets the location to load GUI XML.*/
	{
		GUIXMLLocation_Device,	/*!< Load XML from device*/
		GUIXMLLocation_Host,	/*!< Load XML from host*/
		NUMGUIXMLLOCATION
	};

	enum GenICamXMLLocationEnum	/*!< Sets the location to load GenICam XML.*/
	{
		GenICamXMLLocation_Device,	/*!< Load GenICam XML from device*/
		GenICamXMLLocation_Host,	/*!< Load GenICam XML from host*/
		NUMGENICAMXMLLOCATION
	};

	enum DeviceEndianessMechanismEnum	/*!< Identifies the endianness handling mode.*/
	{
		DeviceEndianessMechanism_Legacy,	/*!< Handling the device endianness according to GenICam Schema 1.0*/
		DeviceEndianessMechanism_Standard,	/*!< Handling the device endianness according to GenICam Schema 1.1 and later*/
		NUMDEVICEENDIANESSMECHANISM
	};

	enum DeviceCurrentSpeedEnum	/*!< The USB Speed that the device is currently operating at.*/
	{
		DeviceCurrentSpeed_UnknownSpeed,	/*!< Unknown-Speed.*/
		DeviceCurrentSpeed_LowSpeed,	/*!< Low-Speed.*/
		DeviceCurrentSpeed_FullSpeed,	/*!< Full-Speed.*/
		DeviceCurrentSpeed_HighSpeed,	/*!< High-Speed.*/
		DeviceCurrentSpeed_SuperSpeed,	/*!< Super-Speed.*/
		NUMDEVICECURRENTSPEED
	};

	enum POEStatusEnum	/*!< Reports and controls the interface's power over Ethernet status.*/
	{
		POEStatus_NotSupported,	/*!< Not Supported*/
		POEStatus_PowerOff,	/*!< Power is Off*/
		POEStatus_PowerOn,	/*!< Power is On*/
		NUMPOESTATUS
	};

	/*@}*/

	/*@}*/

}
#endif // FLIR_SPINNAKER_TRANSPORTLAYERDEFS_H
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

#include "QuickSpinC.h"

#ifndef FLIR_SPINNAKERC_TLDEVICE_H
#define FLIR_SPINNAKERC_TLDEVICE_H

/**	
* @defgroup TLDeviceC_h TLDevice Structures
* @ingroup CQuickSpin
*/
/*@{*/

typedef struct _quickSpinTLDevice
{
	quickSpinStringNode DeviceID;
	quickSpinStringNode DeviceSerialNumber;
	quickSpinStringNode DeviceVendorName;
	quickSpinStringNode DeviceModelName;
	quickSpinEnumerationNode DeviceType;
	quickSpinStringNode DeviceDisplayName;
	quickSpinEnumerationNode DeviceAccessStatus;
	quickSpinStringNode DeviceVersion;
	quickSpinStringNode DeviceUserID;
	quickSpinStringNode DeviceDriverVersion;
	quickSpinBooleanNode DeviceIsUpdater;
	quickSpinEnumerationNode GevCCP;
	quickSpinEnumerationNode GUIXMLLocation;
	quickSpinStringNode GUIXMLPath;
	quickSpinEnumerationNode GenICamXMLLocation;
	quickSpinStringNode GenICamXMLPath;
	quickSpinIntegerNode GevDeviceIPAddress;
	quickSpinIntegerNode GevDeviceSubnetMask;
	quickSpinIntegerNode GevDeviceMACAddress;
	quickSpinIntegerNode GevDeviceGateway;
	quickSpinIntegerNode DeviceLinkSpeed;
	quickSpinIntegerNode GevVersionMajor;
	quickSpinIntegerNode GevVersionMinor;
	quickSpinBooleanNode GevDeviceModeIsBigEndian;
	quickSpinIntegerNode GevDeviceReadAndWriteTimeout;
	quickSpinIntegerNode GevDeviceMaximumRetryCount;
	quickSpinIntegerNode GevDevicePort;
	quickSpinCommandNode GevDeviceDiscoverMaximumPacketSize;
	quickSpinIntegerNode GevDeviceMaximumPacketSize;
	quickSpinBooleanNode GevDeviceIsWrongSubnet;
	quickSpinBooleanNode DeviceMulticastMonitorMode;
	quickSpinEnumerationNode DeviceEndianessMechanism;
	quickSpinStringNode DeviceInstanceId;
	quickSpinEnumerationNode DeviceCurrentSpeed;
	quickSpinBooleanNode DeviceU3VProtocol;
} quickSpinTLDevice;

/*@}*/

#endif // FLIR_SPINNAKERC_TLDEVICE_H
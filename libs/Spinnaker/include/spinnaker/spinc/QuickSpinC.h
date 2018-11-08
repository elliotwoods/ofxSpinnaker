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

#ifndef PGR_SPINNAKER_QUICKSPIN_C_H
#define PGR_SPINNAKER_QUICKSPIN_C_H

#include "SpinnakerDefsC.h"
#include "QuickSpinDefsC.h"

#include "TransportLayerDeviceC.h"
#include "TransportLayerInterfaceC.h"
#include "TransportLayerStreamC.h"
#include "TransportLayerDefsC.h"

/**
 * @defgroup CQuickSpin Spinnaker C QuickSpin API
 *
 */
 /*@{*/

#ifdef __cplusplus
extern "C" {
#endif
	
	/**
	* @defgroup CQuickSpinAccess QuickSpin Access
	*
	* @brief The functions in this section initialize the various QuickSpin
	* structs for the C API.
	*/
	/*@{*/

	/*
	* Pre-fetches all nodes housed on the GenICam nodemap (device initialization required)
	* @see spinError
	*
	* @param hCamera The camera of the nodes to fetch
	* @param pQuickSpin The struct pointer in which the GenICam nodes are returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API quickSpinInit(spinCamera hCamera, quickSpin* pQuickSpin);

	/*
	* Pre-features all nodes housed on the GenICam, transport layer device, and transport layer stream nodemaps (device initialization required)
	* @see spinError
	*
	* @param hCamera The camera of the nodes to fetch
	* @param pQuickSpin The struct pointer in which the GenICam nodes are returned (accepts NULL)
	* @param pQuickSpinTLDevice The struct pointer in which the transport layer device nodes are returned (accepts NULL)
	* @param pQuickSpinTLStream The struct pointer in which the transport layer stream nodes are returned (accepts NULL)
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API quickSpinInitEx(spinCamera hCamera, quickSpin* pQuickSpin, quickSpinTLDevice* pQuickSpinTLDevice, quickSpinTLStream* pQuickSpinTLStream);

	/*
	* Pre-fetches all nodes housed on the transport layer device nodemap (device initialization not necessary)
	* @see spinError
	*
	* @param hCamera The camera of the nodes to fetch
	* @param pQuickSpinTLDevice The struct pointer in which the transport layer device nodes are returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API quickSpinTLDeviceInit(spinCamera hCamera, quickSpinTLDevice* pQuickSpinTLDevice);

	/*
	* Pre-fetches all nodes housed on the transport layer stream nodemap (device initialization not necessary)
	* @see spinError
	*
	* @param hCamera The camera of the nodes to fetch
	* @param pQuickSpinTLStream The struct pointer in which the transport layer stream nodes are returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API quickSpinTLStreamInit(spinCamera hCamera, quickSpinTLStream* pQuickSpinTLStream);

	/*
	* Pre-fetches all nodes housed on the transport layer interface nodemap (device initialization not necessary)
	* @see spinError
	*
	* @param hInterface The interface of the nodes to fetch
	* @param pQuickSpinTLInterface The struct pointer in which the transport layer interface nodes are returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API quickSpinTLInterfaceInit(spinInterface hInterface, quickSpinTLInterface* pQuickSpinTLInterface);

	/*@}*/
	
#ifdef __cplusplus
}
#endif	
	
/*@}*/

#endif // PGR_SPINNAKER_QUICKSPIN_C_H
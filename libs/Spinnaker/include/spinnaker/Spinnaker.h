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

#ifndef PGR_SPINNAKER_H
#define PGR_SPINNAKER_H

/**
*  @defgroup SpinnakerHeaders Spinnaker Headers
*/

/*@{*/

/**
 *  @defgroup Spinnaker_h Spinnaker.h
 *
 *  Global header file for Spinnaker.
 *
 *  By including this file, all required header files for full Spinnaker
 *  operation will be included automatically. It is recommended that this file
 *  be used instead of manually including individual header files.
 *
 *  We welcome your bug reports, suggestions, and comments:
 *  http://www.ptgrey.com/support/contact-us
 */

/*@{*/

/**
 * SpinnakerPlatform Include
 */
#include "SpinnakerPlatform.h"

/**
 * Spinnaker Definitions Include
 */
#include "SpinnakerDefs.h"

/** 
*  Spinnaker GenICam Wrapper Includes
*/
#include "SpinGenApi/GCBase.h"

#include "SpinGenApi/SpinnakerGenApi.h"

/**
 * Spinnaker Class Includes
 */
#include "System.h"
#include "SystemPtr.h"
#include "Exception.h"
#include "Interface.h"
#include "InterfaceList.h"
#include "InterfacePtr.h"
#include "CameraBase.h"
#include "CameraList.h"
#include "Camera.h"
#include "CameraPtr.h"
#include "Image.h"
#include "ImagePtr.h"
#include "ImageStatistics.h"

/**
 * Spinnaker Event Class Includes
 */
#include "Event.h"
#include "InterfaceEvent.h"
#include "ArrivalEvent.h"
#include "RemovalEvent.h"
#include "DeviceEvent.h"
#include "ImageEvent.h"
#include "LoggingEvent.h"
#include "LoggingEventDataPtr.h"

/*@}*/

/*@}*/

#endif // PGR_SPINNAKER_H
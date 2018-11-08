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

#ifndef FLIR_SPINNAKERC_TLSTREAM_H
#define FLIR_SPINNAKERC_TLSTREAM_H

/**	
* @defgroup TLStreamC_h TLStream Structures
* @ingroup CQuickSpin
*/
/*@{*/

typedef struct _quickSpinTLStream
{
	quickSpinStringNode StreamID;
	quickSpinEnumerationNode StreamType;
	quickSpinIntegerNode StreamTotalBufferCount;
	quickSpinIntegerNode StreamDefaultBufferCount;
	quickSpinIntegerNode StreamDefaultBufferCountMax;
	quickSpinEnumerationNode StreamDefaultBufferCountMode;
	quickSpinIntegerNode StreamBufferCountManual;
	quickSpinIntegerNode StreamBufferCountResult;
	quickSpinIntegerNode StreamBufferCountMax;
	quickSpinEnumerationNode StreamBufferCountMode;
	quickSpinEnumerationNode StreamBufferHandlingMode;
	quickSpinBooleanNode StreamCRCCheckEnable;
	quickSpinBooleanNode GevPacketResendMode;
	quickSpinIntegerNode GevMaximumNumberResendRequests;
	quickSpinIntegerNode GevPacketResendTimeout;
	quickSpinIntegerNode GevMaximumNumberResendBuffers;
	quickSpinIntegerNode GevTotalPacketCount;
	quickSpinIntegerNode GevFailedPacketCount;
	quickSpinIntegerNode GevResendPacketCount;
	quickSpinIntegerNode StreamFailedBufferCount;
	quickSpinIntegerNode StreamBufferUnderrunCount;
	quickSpinIntegerNode GevResendRequestCount;
	quickSpinIntegerNode StreamBlockTransferSize;
} quickSpinTLStream;

/*@}*/

#endif // FLIR_SPINNAKERC_TLSTREAM_H
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

#ifndef FLIR_SPINNAKER_TRANSPORTLAYERSTREAM_H
#define FLIR_SPINNAKER_TRANSPORTLAYERSTREAM_H

#include "SpinnakerPlatform.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include "TransportLayerDefs.h"
#include <string>

namespace Spinnaker
{
	/**
	* @defgroup SpinnakerQuickSpinClasses Spinnaker QuickSpin Classes
	*/
	/*@{*/

	/**
	* @defgroup TransportLayerStream_h TransportLayerStream Class
	*/
	/*@{*/

	/**
	*@brief Part of the QuickSpin API to provide access to camera information without having to first initialize the camera.
	*/

	class SPINNAKER_API TransportLayerStream
	{
	public:
		TransportLayerStream(GenApi::INodeMap *nodeMapTLDevice);
		~TransportLayerStream();

	protected:
		TransportLayerStream();

	private:
		TransportLayerStream(const TransportLayerStream&);
		TransportLayerStream& operator=(const TransportLayerStream&);
		TransportLayerStream& operator=(GenApi::INodeMap &);

	public:
		/**
		 * Description: Device unique ID for the data stream, e.g. a GUID.
		 * Visibility: Expert
		 */
		GenApi::IString &StreamID;

		/**
		 * Description: Stream type of the device.
		 * Visibility: Expert
		 */
		GenApi::IEnumerationT<StreamTypeEnum> &StreamType;

		/**
		 * Description: Counts the number of image buffers that arrived since stream started.
		 * Visibility: Expert
		 */
		GenApi::IInteger &StreamTotalBufferCount;

		/**
		 * Description: DEPRECATED; Replaced by StreamBufferCountManual. Controls the number of buffers to be used on this stream upon acquisition start when in manual mode.
		 * Visibility: Invisible
		 */
		GenApi::IInteger &StreamDefaultBufferCount;

		/**
		 * Description: DEPRECATED; Replaced by StreamBufferCountMax. Controls the maximum number of buffers that should be used on this stream. This value is calculated based on the available system memory.
		 * Visibility: Invisible
		 */
		GenApi::IInteger &StreamDefaultBufferCountMax;

		/**
		 * Description: DEPRECATED; Replaced by StreamBufferCountMode. Controls access to setting the number of buffers used for the stream. Locked to Manual mode on 32-bit Windows due to memory constraints.
		 * Visibility: Invisible
		 */
		GenApi::IEnumerationT<StreamDefaultBufferCountModeEnum> &StreamDefaultBufferCountMode;

		/**
		 * Description: Controls the number of buffers to be used on this stream upon acquisition start when in manual mode.
		 * Visibility: Expert
		 */
		GenApi::IInteger &StreamBufferCountManual;

		/**
		 * Description: Displays the number of buffers to be used on this stream upon acquisition start. Recalculated on acquisition start if in auto mode.
		 * Visibility: Expert
		 */
		GenApi::IInteger &StreamBufferCountResult;

		/**
		 * Description: Controls the maximum number of buffers that should be used on this stream. This value is calculated based on the available system memory.
		 * Visibility: Expert
		 */
		GenApi::IInteger &StreamBufferCountMax;

		/**
		 * Description: Controls access to setting the number of buffers used for the stream. Locked to Manual mode on 32-bit Windows due to memory constraints.
		 * Visibility: Expert
		 */
		GenApi::IEnumerationT<StreamBufferCountModeEnum> &StreamBufferCountMode;

		/**
		 * Description: Available buffer handling modes of this data stream:
		 * Visibility: Beginner
		 */
		GenApi::IEnumerationT<StreamBufferHandlingModeEnum> &StreamBufferHandlingMode;

		/**
		 * Description: Enables or disables CRC checks on received images.
		 * Visibility: Expert
		 */
		GenApi::IBoolean &StreamCRCCheckEnable;

		/**
		 * Description: Enables or disables the packet resend mechanism.
		 * Visibility: Expert
		 */
		GenApi::IBoolean &GevPacketResendMode;

		/**
		 * Description: Maximum number of resend requests per image. Each resend request consists of a span of consecutive packet IDs.
		 * Visibility: Expert
		 */
		GenApi::IInteger &GevMaximumNumberResendRequests;

		/**
		 * Description: Time in milliseconds to wait after the image trailer is received and before the image is completed by the driver.
		 * Visibility: Expert
		 */
		GenApi::IInteger &GevPacketResendTimeout;

		/**
		 * Description: This node is not used and has been deprecated.
		 * Visibility: Invisible
		 */
		GenApi::IInteger &GevMaximumNumberResendBuffers;

		/**
		 * Description: Displays number of packets received on this stream.
		 * Visibility: Expert
		 */
		GenApi::IInteger &GevTotalPacketCount;

		/**
		 * Description: Displays number of packets missed on this stream.
		 * Visibility: Expert
		 */
		GenApi::IInteger &GevFailedPacketCount;

		/**
		 * Description: Displays number of packets received after retransmit request on this stream.
		 * Visibility: Expert
		 */
		GenApi::IInteger &GevResendPacketCount;

		/**
		 * Description: Displays number of incomplete images with missing leader/trailer information.
		 * Visibility: Expert
		 */
		GenApi::IInteger &StreamFailedBufferCount;

		/**
		 * Description: Displays number of dropped images caused by driver running out of buffers.
		 * Visibility: Expert
		 */
		GenApi::IInteger &StreamBufferUnderrunCount;

		/**
		 * Description: Displays number of packets requested to be retransmitted on this stream.
		 * Visibility: Expert
		 */
		GenApi::IInteger &GevResendRequestCount;

		/**
		 * Description: Controls the image breakup size that should be used on this stream.
		 * Visibility: Expert
		 */
		GenApi::IInteger &StreamBlockTransferSize;


	protected:
		friend class CameraBase;
		friend class ICameraBase;
		friend class CameraInternal;

	};
	/*@}*/

	/*@}*/

}
#endif // FLIR_SPINNAKER_TRANSPORTLAYERSTREAM_H
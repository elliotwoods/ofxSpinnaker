//=============================================================================
// Copyright © 2018 FLIR Integrated Imaging Solutions, Inc. All Rights Reserved.
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

#ifndef FLIR_SPIN_VIDEO_C_H
#define FLIR_SPIN_VIDEO_C_H

/**
 * SpinnakerPlatform C Include
 */
#include "SpinnakerPlatformC.h"

/**
 * Spinnaker C Definition Includes
 */
#include "SpinnakerDefsC.h"

/**
 * @defgroup SpinnakerC Spinnaker C API
 */
/*@{*/

#ifdef __cplusplus
extern "C" {
#endif

    /**
    * @defgroup CSpinVideo SpinVideo Recording Access
    *
    * @brief The functions in this section provide access to video recording
    * capabilities, which include opening, building, and closing video
    * files.
    */
    /*@{*/

    /*
    * Opens an empty video file to create an uncompressed video
    * @see spinError
    *
    * @param phSpinVideo The spinVideo handle pointer in which the spin video recorder is returned
    * @param pName The filename to save the video as; do not include the extension
    * @param option The video options related to saving as uncompressed video; includes frame rate
    *
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinVideoOpenUncompressed(spinVideo* phSpinVideo, const char* pName, spinAVIOption option);

    /*
    * Opens an empty video file to create an MJPG file
    * @see spinError
    *
    * @param phSpinVideo The spinVideo handle pointer in which the spin video recorder is returned
    * @param pName The filename to save the video as; do not include the extension
    * @param option The video options related to saving as MJPG; includes frame rate and quality
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinVideoOpenMJPG(spinVideo* phSpinVideo, const char* pName, spinMJPGOption option);

    /*
    * Opens an empty video file to create an H264 MP4 file
    * @see spinError
    *
    * @param phSpinVideo The spinVideo handle pointer in which the spin video recorder is returned
    * @param pName The filename to save the video as; do not include the extension
    * @param option The video options related to saving as MJPG; includes frame rate, bitrate, height, and width
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinVideoOpenH264(spinVideo* phSpinVideo, const char* pName, spinH264Option option);

    /*
    * Appends an image to the end of an open video file
    * @see spinError
    *
    * @param hSpinVideo The spin video recorder to append the image to
    * @param hImage The image to append
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinVideoAppend(spinVideo hSpinVideo, spinImage hImage);

    /**
    * Set the maximum file size (in megabytes) of a AVI/MP4 file. A new AVI/MP4 file
    * is created automatically when file size limit is reached. Setting
    * a maximum size of 0 indicates no limit on file size.
    *
    * @param hSpinVideo The spin video recorder to append the image to
    * @param size The maximum video file size in MB.
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinVideoSetMaximumFileSize(spinVideo hSpinVideo, unsigned int size);

    /*
    * Closes a video file to complete its creation
    * @see spinError
    *
    * @param hSpinVideo The spin video recorder to close
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinVideoClose(spinVideo hSpinVideo);
    /*@}*/

    /*@}*/

#ifdef __cplusplus
}
#endif

/*@}*/

#endif /* FLIR_SPIN_VIDEO_C_H */
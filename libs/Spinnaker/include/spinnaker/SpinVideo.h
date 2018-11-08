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

#ifndef FLIR_SPIN_VIDEO_H
#define FLIR_SPIN_VIDEO_H

#include "SpinnakerPlatform.h"
#include "SpinVideoDefs.h"
#include "ImagePtr.h"

namespace Spinnaker
{
    namespace Video
    {
        /**
        *  @defgroup SpinnakerClasses Spinnaker Classes
        */
        /*@{*/

        /**
        *  @defgroup SpinVideo_h Spinnaker Video Class
        */
        /*@{*/

        /**
        * @brief Provides the functionality for the user to record images to an AVI/MP4 file.
        */

        class SPINNAKER_API SpinVideo
        {
        public:

            /**
             * Default constructor.
             */
            SpinVideo();

            /**
             * Default destructor.
             */
            virtual ~SpinVideo();

            /**
             * Open an video file in preparation for writing Images to disk.
             * The size of video files is limited to 2GB. The filenames are
             * automatically generated using the filename specified.
             *
             * @param pFileName The filename of the video file.
             * @param pOption Options to apply to the video file.
             *
             * @see Close()
             *
             */
            virtual void Open(
                const char* pFileName,
                AVIOption &  pOption);

            /**
             * Open an MJPEG video file in preparation for writing Images to disk.
             * The size of video files is limited to 2GB. The filenames are
             * automatically generated using the filename specified.
             *
             * @param pFileName The filename of the video file.
             * @param pOption MJPEG options to apply to the video file.
             *
             * @see Close()
             * @see MJPGOption
             *
             */
            virtual void Open(
                const char* pFileName,
                MJPGOption &  pOption);


            /**
            * Open an H264 MP4 video file in preparation for writing Images to disk.
            * The size of MP4 files is limited to 2GB. The filenames are
            * automatically generated using the filename specified.
            *
            * @param pFileName The filename of the MP4 video file.
            * @param pOption H264 options to apply to the MP4 video file.
            *
            * @see Close()
            * @see H264Option
            *
            */
            virtual void Open(
                const char* pFileName,
                H264Option &  pOption);


            /**
             * Append an image to the video file.
             *
             * @param pImage The image to append.
             *
             */
            virtual void Append(ImagePtr pImage);

            /**
             * Close the video file.
             *
             * @see Open()
             *
             */
            virtual void Close();

            /**
             * Set the maximum file size (in megabytes) of a AVI/MP4 file. A new video file
             * is created automatically when file size limit is reached. Setting
             * a maximum size of 0 indicates no limit on file size.
             *
             * @param size The maximum video file size in MB.
             *
             * @see Append(ImagePtr pImage)
             *
             */
            virtual void SetMaximumFileSize(unsigned int size);

        private:

            SpinVideo(const SpinVideo&);
            SpinVideo& operator=(const SpinVideo&);

            struct SpinVideoData; // Forward declaration

            SpinVideoData* m_pSpinVideoData;
        };

        /*@}*/

        /*@}*/
    }
}

#endif // FLIR_SPIN_VIDEO_H
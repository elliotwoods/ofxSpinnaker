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

#ifndef FLIR_SPINNAKER_CAMERABASE_H
#define FLIR_SPINNAKER_CAMERABASE_H

#include "Interface/ICameraBase.h"

namespace Spinnaker
{
    class ImagePtr;

    /**
    *  @defgroup SpinnakerClasses Spinnaker Classes
    */
    /*@{*/

    /**
    *  @defgroup CameraBase_h Camera Base Class
    */
    /*@{*/

    /**
    * @brief The base class for the camera object.
    */

    class SPINNAKER_API CameraBase : public ICameraBase
    {
    public:

        /**
        * Virtual Destructor
        */
        virtual ~CameraBase(void);

        /**
        * Connect to camera, retrieve XML and generate node map.
        * This function needs to be called before any camera related
        * API calls such as BeginAcquisition(), EndAcquisition(),
        * GetNodeMap(), GetNextImage().
        *
        * @see BeginAcquisition()
        * @see EndAcquisition()
        * @see GetNodeMap()
        * @see GetNextImage()
        */
        void Init();

        /**
        * Disconnect camera port and free GenICam node map and
        * GUI XML. Do not call more functions that access the remote
        * device such as WritePort/ReadPort after calling DeInit();
        * Events should also be unregistered before calling camera DeInit().
        * Otherwise an exception will be thrown in the DeInit() call and require
        * the user to unregister events before the camera can be re-initialized again.
        *
        * @see Init()
        * @see UnregisterEvent(Event & evtToUnregister)
        */
        void DeInit();

        /**
        * Checks if camera is initialized. This function needs to return
        * true in order to retrieve a valid NodeMap from the GetNodeMap()
        * call.
        *
        * @see GetNodeMap()
        *
        * @return If camera is initialized or not
        */
        bool IsInitialized();

        /**
        * Checks a flag to determine if camera is still valid for use.
        *
        * @return If camera is valid or not
        */
        bool IsValid();

        /**
        * Gets a reference to the node map that is generated from a GenICam
        * XML file.  The camera must be initialized by a call to Init() first
        * before a node map reference can be successfully acquired.
        *
        * @see Init()
        *
        * @return  A reference to the INodeMap.
        */
        GenApi::INodeMap & GetNodeMap() const;


        /**
        * Gets a reference to the node map that is generated from a GenICam
        * XML file for the GenTL Device module.  The camera does not need to be
        * initialized before acquiring this node map.
        *
        * @return  A reference to the INodeMap.
        */
        GenApi::INodeMap & GetTLDeviceNodeMap() const;

        /**
        * Gets a reference to the node map that is generated from a GenICam
        * XML file for the GenTL Stream module.  The camera does not need to be
        * initialized before acquiring this node map.
        *
        * @return  A reference to the INodeMap.
        */
        GenApi::INodeMap & GetTLStreamNodeMap() const;

        /**
        * Returns the access mode that the software has on the Camera.
        * The camera does not need to be initialized before calling this function.
        *
        * @see Init()
        *
        * @return An enumeration value indicating the access mode
        */
        GenApi::EAccessMode GetAccessMode() const;

        /*
        * Reads a remote port on a physical Camera. This function can be used to read
        * registers on Cameras.
        *
        * Caution: Only perform direct read/write to a register if the register isn't
        *          supported in the device nodemap. Otherwise the camera and nodemap
        *          may be left in an undefined state after the register read/write.
        *
        * @see WritePort()
        *
        * @param iAddress A 64 bit address to a register on the camera
        * @param pBuffer A pointer to a data buffer that will store result
        * @param iSize Number of bytes to read
        *
        */
        void ReadPort(uint64_t iAddress, void *pBuffer, size_t iSize);

        /*
        * Writes a remote port on a physical Camera. This function can be used to write
        * registers on Cameras.
        *
        * Caution: Only perform direct read/write to a register if the register isn't
        *          supported in the device nodemap. Otherwise the camera and nodemap
        *          may be left in an undefined state after the register read/write.
        *
        * @see ReadPort()
        *
        * @param iAddress A 64 bit address to a register on the camera
        * @param pBuffer A pointer to a data buffer that will be written
        * @param iSize Number of bytes to write
        *
        */
        void WritePort(uint64_t iAddress, const void *pBuffer, size_t iSize);

        /**
        * Starts the image acquisition engine. The camera must be initialized via
        * a call to Init() before starting an acquisition.
        *
        * @see Init()
        */
        void BeginAcquisition();

        /**
        * Stops the image acquisition engine.  If EndAcquisition() is called
        * without a prior call to BeginAcquisition() an error message
        * "Camera is not started" will be thrown. All Images that were
        * acquired using GetNextImage() need to be released first using image->Release()
        * before calling EndAcquisition().  All buffers in the input pool and
        * output queue will be discarded when EndAcquisition() is called.
        *
        *
        * @see Init()
        * @see BeginAcquisition()
        * @see GetNextImage( grabTimeout )
        * @see Image::Release()
        *
        */
        void EndAcquisition();

        /**
        * Gets the next image that was received by the transport layer.  This function
        * will block indefinitely until an image arrives. Most cameras support one stream
        * so the default streamID is 0 but if a camera supports multiple streams
        * the user can input the streamID to select from which stream to grab images
        *
        * @see Init()
        * @see BeginAcquisition()
        * @see EndAcquisition()
        *
        * @param grabTimeout a 64bit value that represents a timeout in milliseconds
        * @param streamID The stream to grab the image.
        * @return pointer to an Image object
        *
        */
        ImagePtr GetNextImage(uint64_t grabTimeout = EVENT_TIMEOUT_INFINITE, uint64_t streamID = 0);

        /**
        * This returns a unique id string that identifies the camera.  This is the
        * camera serial number.
        *
        * @return string that uniquely identifies the camera (serial number)
        */
        GenICam::gcstring GetUniqueID();

        /**
        * Returns true if the camera is currently streaming or false if it
        * is not.
        *
        * @see Init()
        *
        * @return returns true if camera is streaming and false otherwise.
        */
        bool IsStreaming() const;

        /**
        * Returns the GUI XML that can be passed into the Spinnaker GUI framework
        *
        * @return GenICam::gcstring that represents the uncompressed GUI XML file
        */
        GenICam::gcstring GetGuiXml() const;

        /**
        * Registers a specific event for the camera.  The camera has to be initialized first
        * with a call to Init() before registering for events.
        *
        * @see Init()
        *
        * @param evtToRegister The event to register for the camera
        *
        */
        void RegisterEvent(Event & evtToRegister);


        /**
        * Registers a specific event for the camera
        *
        * @see Init()
        *
        * @param evtToRegister The event to register for the camera
        * @param eventName The event name to register
        *
        */
        void RegisterEvent(Event & evtToRegister, const GenICam::gcstring& eventName);

        /**
        * Unregisters an event for the camera
        * Events should be unregistered first before calling camera DeInit().
        * Otherwise an exception will be thrown in the DeInit() call and require
        * the user to unregister events before the camera can be re-initialized again.
        *
        * @see DeInit()
        *
        * @param evtToUnregister The event to unregister from the camera
        *
        */
        void UnregisterEvent(Event & evtToUnregister);

        /**
        * Returns the number of images that are currently in use.  Each of
        * the images that are currently in use must be cleaned up with
        * a call to image->Release() before calling system->ReleaseInstance().
        *
        * @return The number of images that needs to be cleaned up.
        *
        */
        unsigned int GetNumImagesInUse();

        /**
        * Returns the number of streams that a device supports.
        *
        *
        * @return The number of data streams
        *
        */
        unsigned int GetNumDataStreams();

        /**
        * Returns the largest packet size that can be safely used on the interface that device is connected to
        *
        *
        * @return The maximum packet size returned.
        *
        */
        unsigned int DiscoverMaxPacketSize();

    protected:

        /**
        * Default constructor.
        */
        CameraBase(void);

        /**
        * Copy constructor.
        */
        CameraBase(const CameraBase & /*cam*/);

        /**
        * Assignment operator.
        */
        CameraBase& operator=(const CameraBase& /*cam*/);

        friend class InterfaceImpl;
    };

    /*@}*/

    /*@}*/
}

#endif // FLIR_SPINNAKER_CAMERABASE_H
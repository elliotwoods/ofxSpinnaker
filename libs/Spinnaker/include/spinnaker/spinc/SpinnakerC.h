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

#ifndef FLIR_SPINNAKER_C_H
#define FLIR_SPINNAKER_C_H

/**
 * SpinnakerPlatform C Include
 */
#include "SpinnakerPlatformC.h"

/**
 * Spinnaker C Definition Includes
 */
#include "SpinnakerDefsC.h"
#include "CameraDefsC.h"
#include "ChunkDataDefC.h"

/**
 * Spinnaker GenICam C Wrapper Includes
 */
#include "SpinnakerGenApiDefsC.h"
#include "SpinnakerGenApiC.h"

/**
 * Spinnaker QuickSpin C Includes
 */
#include "QuickSpinC.h"

/**
 * @defgroup SpinnakerC Spinnaker C API
 */
/*@{*/

#ifdef __cplusplus
extern "C" {
#endif
    /**
    * @defgroup CErrorHandling Error Handling
    *
    * @brief The functions in this section provide access to additional
    * information related to error returns.
    */
    /*@{*/

    /**
    * Retrieves the error code of the last error
    * @see spinError
    *
    * @param pError The error enum pointer in which the error message is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinErrorGetLast(spinError* pError);

    /**
    * Retrieves the error message of the last error
    * @see spinError
    *
    * @param pBuf The c-string character buffer in which the error message is returned
    * @param pBufLen The unsigned integer pointer in which the length of the c-string is returned; the input value is the maximum length
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinErrorGetLastMessage(char* pBuf, size_t* pBufLen);

    /**
    * Retrieves the build date of the last error
    * @see spinError
    *
    * @param pBuf The c-string character buffer in which the build date is returned
    * @param pBufLen The unsigned integer pointer in which the length of the c-string is returned; the input value is the maximum length
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinErrorGetLastBuildDate(char* pBuf, size_t* pBufLen);

    /**
    * Retrieves the build time of the last error
    * @see spinError
    *
    * @param pBuf The c-string character buffer in which the build time is returned
    * @param pBufLen The unsigned integer pointer in which the length of the c-string is returned; the input value is the maximum length
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinErrorGetLastBuildTime(char* pBuf, size_t* pBufLen);

    /**
    * Retrieves the filename of the last error
    * @see spinError
    *
    * @param pBuf The c-string character buffer in which the file name is returned
    * @param pBufLen The unsigned integer pointer in which the length of the c-string is returned; the input value is the maximum length
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinErrorGetLastFileName(char* pBuf, size_t* pBufLen);

    /**
    * Retrieves the full error message of the last error
    * @see spinError
    *
    * @param pBuf The c-string character buffer in which the full error message is returned
    * @param pBufLen The unsigned integer pointer in which the length of the c-string is returned; the input value is the maximum length
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinErrorGetLastFullMessage(char* pBuf, size_t* pBufLen);

    /**
    * Retrieves the function name of the last error
    * @see spinError
    *
    * @param pBuf The c-string character buffer in which the function name is returned
    * @param pBufLen The unsigned integer pointer in which the length of the c-string is returned; the input value is the maximum length
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinErrorGetLastFunctionName(char* pBuf, size_t* pBufLen);

    /**
    * Retrieves the line number of the last error
    * @see spinError
    *
    * @param pBuf The c-string character buffer in which the line number is returned
    * @param pBufLen The unsigned integer pointer in which the length of the c-string is returned; the input value is the maximum length
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinErrorGetLastLineNumber(int64_t* pLineNum);
    /*@}*/

    /**
    * @defgroup CSystem System Access
    *
    * @brief The functions in this section provide access to information,
    * objects, and functionality of the system object. This includes the
    * system object, interface and camera lists, and interface and logging
    * events.
    */
    /*@{*/

    /**
    * Retrieves an instance of the system object; the system is a singleton, so there will only ever be one instance;
    * system instance must be destroyed by calling spinSystemReleaseInstance
    *
    * @see spinSystemReleaseInstance
    * @see spinError
    *
    * @param phSystem The system handle pointer in which the system instance is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinSystemGetInstance(spinSystem* phSystem);

    /**
    * Releases the system; make sure handle is cleaned up properly by setting it to NULL
    * after system is released; the handle can only be used again after calling
    * spinSystemGetInstance
    *
    * @see spinSystemGetInstance
    * @see spinError
    *
    * @param hSystem The system handle
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinSystemReleaseInstance(spinSystem hSystem);

    /**
    * Retrieves a list of detected (and enumerable) interfaces on the system;
    * interface lists must be created and destroyed
    *
    * @see spinInterfaceListCreateEmpty()
    * @see spinInterfaceListDestroy()
    * @see spinError
    *
    * @param hSystem The system, from which the interface list is retrieved
    * @param hInterfaceList The interface list to house the interfaces from the system
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinSystemGetInterfaces(spinSystem hSystem, spinInterfaceList hInterfaceList);

    /**
    * Retrieves a list of detected (and enumerable) cameras on the system;
    * camera lists must be created and destroyed
    *
    * @see spinCameraListCreateEmpty()
    * @see spinCameraListDestroy()
    * @see spinError
    *
    * @param hSystem The system, from which the camera list is retrieved
    * @param hCameraList The camera list to house the cameras from the system
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinSystemGetCameras(spinSystem hSystem, spinCameraList hCameraList);

    /**
    * Retrieves a list of detected (and enumerable) cameras on the system;
    * manually set whether to update the current interface and camera lists;
    * camera lists must be created and destroyed
    *
    * @see spinCameraListCreateEmpty()
    * @see spinCameraListDestroy()
    * @see spinError
    *
    * @param hSystem The system, from which the camera list is retrieved
    * @param bUpdateInterfaces The boolean of whether to update the interface list
    * @param bUpdateCameras The boolean of whether to update the camera list
    * @param hCameraList The camera list to house the cameras from the system
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinSystemGetCamerasEx(spinSystem hSystem, bool8_t bUpdateInterfaces, bool8_t bUpdateCameras, spinCameraList hCameraList);

    /**
    * Sets the logging level for all logging events on the system
    * @see spinError
    *
    * @param hSystem The system, on which the logging level is set
    * @param logLevel The logging level to set
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinSystemSetLoggingLevel(spinSystem hSystem, spinnakerLogLevel logLevel);

    /**
    * Retrieves the logging level for all logging events on the system
    * @see spinError
    *
    * @param hSystem The system, from which the logging level is retrieved
    * @param logLevel The logging level enum pointer in which the current logging level is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinSystemGetLoggingLevel(spinSystem hSystem, spinnakerLogLevel* pLogLevel);

    /**
    * Registers a logging event to the system (events registered in this way must be unregistered)
    * @see spinError
    *
    * @param hSystem The system, on which the logging event is registered
    * @param hLogEvent The logging event to register on the system
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinSystemRegisterLogEvent(spinSystem hSystem, spinLogEvent hLogEvent);

    /**
    * Unregisters a selected logging event from the system
    * @see spinError
    *
    * @param hSystem The system, from which the logging event is unregistered
    * @param hLogEvent The logging event to unregister from the system
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinSystemUnregisterLogEvent(spinSystem hSystem, spinLogEvent hLogEvent);

    /**
    * Unregisters all logging events from the system
    * @see spinError
    *
    * @param hSystem The system, from which all logging events are unregistered
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinSystemUnregisterAllLogEvents(spinSystem hSystem);

    /**
    * Checks whether a system is currently in use
    * @see spinError
    *
    * @param hSystem The system to check
    * @param pbIsInUse The boolean pointer to return whether the system is currently in use
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinSystemIsInUse(spinSystem hSystem, bool8_t* pbIsInUse);

    /**
    * Registers an arrival event to every interface on the system (events registered this way must be unregistered)
    * @see spinError
    *
    * @param hSystem The system, on which the arrival event is registered
    * @param hArrivalEvent The arrival event to register on the system
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinSystemRegisterArrivalEvent(spinSystem hSystem, spinArrivalEvent hArrivalEvent);

    /**
    * Registers a removal event to the system to every interface on the system (events registered this way must be unregistered)
    * @see spinError
    *
    * @param hSystem The system, on which the removal event is registered
    * @param hRemovalEvent The removal event to register on the system
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinSystemRegisterRemovalEvent(spinSystem hSystem, spinRemovalEvent hRemovalEvent);

    /**
    * Unregisters an arrival event from the system
    * @see spinError
    *
    * @param hSystem The system, from which the arrival event is unregistered
    * @param hArrivalEvent The arrival event to unregister from the system
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinSystemUnregisterArrivalEvent(spinSystem hSystem, spinArrivalEvent hArrivalEvent);

    /**
    * Unregisters a removal event from the system
    * @see spinError
    *
    * @param hSystem The system, from which the removal event is unregistered
    * @param hRemovalEvent The removal event to unregister from the system
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinSystemUnregisterRemovalEvent(spinSystem hSystem, spinRemovalEvent hRemovalEvent);

    /**
    * Registers an interface event (arrival and removal) to every interface on the system (interface events registered this way must be unregistered)
    * @see spinError
    *
    * @param hSystem The system, on which the interface event is registered
    * @param hInterfaceEvent The interface event (arrival and removal) to register on the system
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinSystemRegisterInterfaceEvent(spinSystem hSystem, spinInterfaceEvent hInterfaceEvent);

    /**
    * Unregisters an interface event from the system
    * @see spinError
    *
    * @param hSystem The system, from which the interface event is unregistered
    * @param hInterfaceEvent The interface event (arrival and removal) to unregister from the system
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinSystemUnregisterInterfaceEvent(spinSystem hSystem, spinInterfaceEvent hInterfaceEvent);

    /**
    * Updates the list of cameras on the system, informing whether there has been any changes
    * @see spinError
    *
    * @param hSystem The system, on which the list of attached cameras is updated
    * @param pbChanged The boolean pointer to return whether cameras have arrived on or been removed from the system
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinSystemUpdateCameras(spinSystem hSystem, bool8_t* pbChanged);

    /**
    * Updates the list of cameras on the system, informing whether there has been any changes; manually set whether to update the current interface lists
    * @see spinError
    *
    * @param hSystem The system, on which the list of attached cameras is updated
    * @param bUpdateInterfaces The boolean of whether to update the interface list
    * @param pbChanged The boolean pointer to return whether cameras have arrived or been removed from the system
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinSystemUpdateCamerasEx(spinSystem hSystem, bool8_t bUpdateInterfaces, bool8_t* pbChanged);

    /**
    * Broadcast an Action Command to all devices on system
    * @see spinError
    *
    * @param iDeviceKey The Action Command's device key
    * @param iGroupKey The Action Command's group key
    * @param iGroupMask The Action Command's group mask
    * @param iActionTime (Optional) Time when to assert a future action. Zero means immediate action.
    * @param piResultSize (Optional) The number of results in the results array. The value passed should be equal to the expected number of devices that acknowledge the command. Returns the number of received results.
    * @param results (Optional) An Array with *piResultSize elements to hold the action command result status. The buffer is filled starting from index 0. If received results are less than expected number of devices that acknowledge the command, remaining results are not changed. If received results are more than expected number of devices that acknowledge the command, extra results are ignored and not appended to array. This parameter is ignored if piResultSize is 0. Thus this parameter can be NULL if pResultSize is 0 or NULL.
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinSystemSendActionCommand(spinSystem hSystem, size_t iDeviceKey, size_t iGroupKey, size_t iGroupMask, size_t iActionTime, size_t* piResultSize, actionCommandResult results[]);

    /**
    * Get current library version of Spinnaker.
    *
    * @return A struct containing the current version of Spinnaker(major, minor, type, build).
    */
    SPINNAKERC_API spinSystemGetLibraryVersion(spinSystem hSystem, spinLibraryVersion* hLibraryVersion);
    /*@}*/

    /**
    * @defgroup CInterfaceList InterfaceList Access
    *
    * @brief The functions in this section provide access to information,
    * objects, and functionality of interface lists. This includes
    * updating, size and interface retrieval, and clearance.
    */
    /*@{*/

    /**
    * Creates an empty interface list (interface lists created this way must be destroyed)
    * @see spinError
    *
    * @param phInterfaceList The interface list handle pointer in which the empty interface list is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinInterfaceListCreateEmpty(spinInterfaceList* phInterfaceList);

    /**
    * Destroys an interface list
    * @see spinError
    *
    * @param hInterfaceList The interface list to destroy
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinInterfaceListDestroy(spinInterfaceList hInterfaceList);

    /**
    * Retrieves the number of interfaces in an interface list
    * @see spinError
    *
    * @param hInterfaceList The interface list where the interfaces to be counted are
    * @param pSize The unsigned integer pointer in which the number of interfaces is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    * @see spinError
    */
    SPINNAKERC_API spinInterfaceListGetSize(spinInterfaceList hInterfaceList, size_t* pSize);

    /**
    * Retrieves an interface from an interface list using an index (interfaces retrieved this way must be released)
    * @see spinError
    *
    * @param hInterfaceList The interface list of the interface to be retrieved
    * @param index The index of the interface
    * @param phInterface The interface handle pointer in which the interface is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinInterfaceListGet(spinInterfaceList hInterfaceList, size_t index, spinInterface* phInterface);

    /**
    * Clears an interface list
    * @see spinError
    *
    * @param hInterfaceList The interface list to clear
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinInterfaceListClear(spinInterfaceList hInterfaceList);
    /*@}*/

    /**
    * @defgroup CCameraList CameraList Access
    *
    * @brief The functions in this section provide access to information,
    * objects, and functionality of camera lists. This includes updating,
    * size and camera retrieval, and clearance.
    */
    /*@{*/

    /**
    * Creates an empty camera list (camera lists created this way must be destroyed)
    * @see spinError
    *
    * @param phCameraList The camera list handle pointer in which the empty camera list is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraListCreateEmpty(spinCameraList* phCameraList);

    /**
    * Destroys a camera list
    * @see spinError
    *
    * @param hCameraList The camera list to destroy
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraListDestroy(spinCameraList hCameraList);

    /**
    * Retrieves the number of cameras on a camera list
    * @see spinError
    *
    * @param hCameraList The camera list where the cameras to be counted are
    * @param pSize The unsigned integer pointer in which the number of cameras is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraListGetSize(spinCameraList hCameraList, size_t* pSize);

    /**
    * Retrieves a camera from a camera list using an index. This function will return
    * a SPINNAKER_ERR_INVALID_PARAMETER error if the input index is out of range.
    * @see spinError
    *
    * @param hCameraList The camera list of the camera to retrieve
    * @param index The index of the camera
    * @param phCamera The camera handle pointer in which the camera is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraListGet(spinCameraList hCameraList, size_t index, spinCamera* phCamera);

    /**
    * Clears a camera list
    * @see spinError
    *
    * @param hCameraList The camera list to clear
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraListClear(spinCameraList hCameraList);

    /**
    * Removes a camera from a camera list using its index
    * @see spinError
    *
    * @param hCameraList The camera list of the camera to remove
    * @param index The index of the camera to remove
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraListRemove(spinCameraList hCameraList, size_t index);

    /**
    * Appends all the cameras from one camera list to another
    * @see spinError
    *
    * @param hCameraListBase The camera list to receive the other
    * @param hCameraListToAppend The camera list to add to the other
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraListAppend(spinCameraList hCameraListBase, spinCameraList hCameraListToAppend);

    /**
    * Retrieves a camera from a camera list using its serial number. This
    * function will return a NULL spinCamera pointer if no matching camera
    * serial is found.
    * @see spinError
    *
    * @param hCameraList The camera list of the camera to retrieve
    * @param serial The serial number of the camera to retrieve
    * @param phCamera The camera handle pointer in which the camera is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraListGetBySerial(spinCameraList hCameraList, const char* pSerial, spinCamera* phCamera);

    /**
    * Removes a camera from a camera list using its serial number
    * @see spinError
    *
    * @param hCameraList The camera of the camera to remove
    * @param pSerial The serial number of the camera to remove
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraListRemoveBySerial(spinCameraList hCameraList, const char* pSerial);
    /*@}*/

    /**
    * @defgroup CInterface Interface Access
    *
    * @brief The functions in this section provide access to information,
    * objects, and functionality of interfaces. This includes camera list
    * and nodemap retrieval, event registration, and interface release.
    */
    /*@{*/

    /**
    * Checks whether any cameras have been connected or disconnected on an interface
    * @see spinError
    *
    * @param hInterface The interface of the list of attached cameras to update
    * @param pbChanged The boolean pointer to return whether or not the cameras have changed
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinInterfaceUpdateCameras(spinInterface hInterface, bool8_t* pbChanged);

    /**
    * Retrieves a camera list from an interface;
    * camera lists must be created and destroy
    *
    * @see spinCameraListCreateEmpty()
    * @see spinCameraListDestroy()
    * @see spinError
    *
    * @param hInterface The interface of the camera list to retrieve
    * @param hCameraList The camera list to house the cameras from the interface
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinInterfaceGetCameras(spinInterface hInterface, spinCameraList hCameraList);

    /**
    * Retrieves a camera list from an interface;
    * manually set whether to update the cameras;
    * camera lists must be created and destroyed
    *
    * @see spinCameraListCreateEmpty()
    * @see spinCameraListDestroy()
    * @see spinError
    *
    * @param hInterface The interface of the camera list to retrieve
    * @param bUpdateCameras The boolean of whether or not to update the cameras
    * @param hCameraList The camera list to house the cameras from the interface
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinInterfaceGetCamerasEx(spinInterface hInterface, bool8_t bUpdateCameras, spinCameraList hCameraList);

    /**
    * Retrieves the transport layer nodemap from an interface
    * @see spinError
    *
    * @param hInterface The interface of the nodemap to retrieve
    * @param phNodeMap The nodemap handle pointer in which the transport layer interface nodemap is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinInterfaceGetTLNodeMap(spinInterface hInterface, spinNodeMapHandle* phNodeMap);

    /**
    * Registers an arrival event on an interface (events registered in this way must be unregistered)
    * @see spinError
    *
    * @param hInterface The interface on which to register the arrival event
    * @param hArrivalEvent The arrival event to register
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinInterfaceRegisterArrivalEvent(spinInterface hInterface, spinArrivalEvent hArrivalEvent);

    /**
    * Registers a removal event on an interface (events registered in this way must be unregistered)
    * @see spinError
    *
    * @param hInterface the Interface on which to register the removal event
    * @param hRemovalEvent The removal event to register
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinInterfaceRegisterRemovalEvent(spinInterface hInterface, spinRemovalEvent hRemovalEvent);

    /**
    * Unregisters an arrival event from an interface
    * @see spinError
    *
    * @param hInterface The interface from which to unregister the arrival event
    * @param hArrivalEvent The arrival event to unregister
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinInterfaceUnregisterArrivalEvent(spinInterface hInterface, spinArrivalEvent hArrivalEvent);

    /**
    * Unregisters a removal event from an interface
    * @see spinError
    *
    * @param hInterface The interface from which to unregister the removal event
    * @param hRemovalEvent The removal event to unregister
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinInterfaceUnregisterRemovalEvent(spinInterface hInterface, spinRemovalEvent hRemovalEvent);

    /**
    * Registers an interface event (both arrival and removal) on an interface
    * @see spinError
    *
    * @param hInterface The interface on which to register the interface event
    * @param hInterfaceEvent The interface event to register
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinInterfaceRegisterInterfaceEvent(spinInterface hInterface, spinInterfaceEvent hInterfaceEvent);

    /**
    * Unregisters an interface event from an interface
    * @see spinError
    *
    * @param hInterface The interface from which to unregister the interface event
    * @param hInterfaceEvent The interface event to unregister
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinInterfaceUnregisterInterfaceEvent(spinInterface hInterface, spinInterfaceEvent hInterfaceEvent);

    /**
    * Releases an interface
    * @see spinError
    *
    * @param hInterface The interface to release
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinInterfaceRelease(spinInterface hInterface);

    /**
    * Checks whether an interface is in use
    * @see spinError
    *
    * @param hInterface The interface to check
    * @param pbIsInUse The boolean pointer to return whether or not the interface is in use
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinInterfaceIsInUse(spinInterface hInterface, bool8_t* pbIsInUse);

    /**
    * Broadcast an Action Command to all devices on interface
    * @see spinError
    *
    * @param iDeviceKey The Action Command's device key
    * @param iGroupKey The Action Command's group key
    * @param iGroupMask The Action Command's group mask
    * @param iActionTime (Optional) Time when to assert a future action. Zero means immediate action.
    * @param piResultSize (Optional) The number of results in the results array. The value passed should be equal to the expected number of devices that acknowledge the command. Returns the number of received results.
    * @param results (Optional) An Array with *piResultSize elements to hold the action command result status. The buffer is filled starting from index 0. If received results are less than expected number of devices that acknowledge the command, remaining results are not changed. If received results are more than expected number of devices that acknowledge the command, extra results are ignored and not appended to array. This parameter is ignored if piResultSize is 0. Thus this parameter can be NULL if pResultSize is 0 or NULL.
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinInterfaceSendActionCommand(spinInterface hInterface, size_t iDeviceKey, size_t iGroupKey, size_t iGroupMask, size_t iActionTime, size_t* piResultSize, actionCommandResult results[]);
    /*@}*/

    /**
    * @defgroup CCamera Camera Access
    *
    * @brief The functions in this section provide access to information,
    * objects, and functionality of cameras. This includes nodemap retrieval,
    * acquisition and init commands, event registration, and camera property
    * retrieval.
    */
    /*@{*/

    /**
    * Initializes a camera, allowing for much more interaction
    * @see spinError
    *
    * @param hCamera The camera to initialize
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraInit(spinCamera hCamera);

    /**
    * Deinitializes a camera, greatly reducing functionality
    * @see spinError
    *
    * @param hCamera The camera to deinitialize
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraDeInit(spinCamera hCamera);

    /**
    * Retrieves the GenICam nodemap from a camera
    * @see spinError
    *
    * @param hCamera The camera from which the nodemap is retrieved
    * @param phNodeMap The nodemap handle pointer in which the nodemap is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraGetNodeMap(spinCamera hCamera, spinNodeMapHandle* phNodeMap);

    /**
    * Retrieves the transport layer device nodemap from a camera
    * @see spinError
    *
    * @param hCamera The camera from which the transport layer device nodemap is retrieved
    * @param phNodeMap The nodemap handle pointer in which the nodemap is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraGetTLDeviceNodeMap(spinCamera hCamera, spinNodeMapHandle* phNodeMap);

    /**
    * Retrieves the transport layer stream nodemap from a camera
    * @see spinError
    *
    * @param hCamera The camera from which the transport layer streaming nodemap is retrieved
    * @param phNodeMap The nodemap handle pointer in which the nodemap is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraGetTLStreamNodeMap(spinCamera hCamera, spinNodeMapHandle* phNodeMap);

    /**
    * Retrieves the access mode of a camera (as an enum, spinAccessMode)
    * @see spinError
    * @see spinAccessMode
    *
    * @param hCamera The camera of the access mode to retrieve
    * @param pAccessMode The access mode enum pointer in which the access mode is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraGetAccessMode(spinCamera hCamera, spinAccessMode* pAccessMode);


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
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraReadPort(spinCamera hCamera, uint64_t iAddress, void *pBuffer, size_t iSize);

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
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraWritePort(spinCamera hCamera, uint64_t iAddress, void *pBuffer, size_t iSize);

    /**
    * Has a camera start acquiring images
    * @see spinError
    *
    * @param hCamera The camera to begin acquiring images
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraBeginAcquisition(spinCamera hCamera);

    /**
    * Has a camera stop acquiring images
    * @see spinError
    *
    * @param hCamera The camera to stop acquiring images
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraEndAcquisition(spinCamera hCamera);

    /**
    * Retrieves an image from a camera
    * @see spinError
    *
    * @param hCamera The camera of the image to retrieve
    * @param phImage The image handle pointer in which the image is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraGetNextImage(spinCamera hCamera, spinImage* phImage);

    /**
    * Retrieves an image from a camera; manually set the timeout in milliseconds
    * @see spinError
    *
    * @param hCamera The camera of the image to retrieve
    * @param grabTimeout The timeout value for returned an image
    * @param phImage The image handle pointer in which the image is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraGetNextImageEx(spinCamera hCamera, uint64_t grabTimeout, spinImage* phImage);

    /**
    * Retrieves a unique identifier for a camera
    * @see spinError
    *
    * @param hCamera The camera of the unique identifier
    * @param pBuf The c-string character buffer in which the unique identifier is returned
    * @param pBufLen The unsigned integer pointer in which the length of the c-string is returned; the input value is the maximum length
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraGetUniqueID(spinCamera hCamera, char* pBuf, size_t* pBufLen);

    /**
    * Checks whether a camera is currently acquiring images
    * @see spinError
    *
    * @param hCamera The camera to check
    * @param pbIsStreaming The boolean pointer to return whether or not the camera is currently streaming
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraIsStreaming(spinCamera hCamera, bool8_t* pbIsStreaming);

    /**
    * Retrieves the GUI XML from a camera
    * @see spinError
    *
    * @param hCamera The camera of the GUI XML to retrieve
    * @param pBuf The c-string character buffer in which the GUI XML is returned
    * @param pBufLen The unsigned integer pointer in which the length of the c-string is returned; the input value is the maximum length
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraGetGuiXml(spinCamera hCamera, char* pBuf, size_t* pBufLen);

    /**
    * Registers a universal device event (every device event type) to a camera
    * @see spinError
    *
    * @param hCamera The camera on which to register the universal device event
    * @param hDeviceEvent The device event to register
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraRegisterDeviceEvent(spinCamera hCamera, spinDeviceEvent hDeviceEvent);

    /**
    * Registers a specific device event (only one device event type) to a camera
    * @see spinError
    *
    * @param hCamera The camera on which to register the specific device event
    * @param hDeviceEvent The device event to register
    * @param pName The name of the device event to register
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraRegisterDeviceEventEx(spinCamera hCamera, spinDeviceEvent hDeviceEvent, const char* pName);

    /**
    * Unregisters a device event from a camera
    * @see spinError
    *
    * @param hCamera The camera from which to unregister the device event
    * @param hDeviceEvent The device event to unregister
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraUnregisterDeviceEvent(spinCamera hCamera, spinDeviceEvent hDeviceEvent);

    /**
    * Registers an image event to a camera
    * @see spinError
    *
    * @param hCamera The camera on which to register the image event
    * @param hImageEvent The image event to register
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraRegisterImageEvent(spinCamera hCamera, spinImageEvent hImageEvent);

    /**
    * Unregisters an image event from a camera
    * @see spinError
    *
    * @param hCamera The camera from which to unregister the image event
    * @param hImageEvent The image event to unregister
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraUnregisterImageEvent(spinCamera hCamera, spinImageEvent hImageEvent);

    /**
    * Releases a camera
    * @see spinError
    *
    * @param hCamera The camera to release
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraRelease(spinCamera hCamera);

    /**
    * Checks whether a camera is still valid for use
    * @see spinError
    *
    * @param hCamera The camera to check
    * @param pbValid The boolean pointer to return whether or not the camera is valid
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraIsValid(spinCamera hCamera, bool8_t* pbValid);

    /**
    * Checks whether a camera is currently initialized
    * @see spinError
    *
    * @param hCamera The camera to check
    * @param pbInit The boolean pointer to return whether or not the camera is initialized
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraIsInitialized(spinCamera hCamera, bool8_t* pbInit);
    /*@}*/

    /**
    * Returns the largest packet size that can be safely used on the interface that device is connected to.
    * @see spinError
    *
    * @param hCamera The camera to check
    * @param pMaxPacketSize The maximum packet size returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinCameraDiscoverMaxPacketSize(spinCamera hCamera, unsigned int* pMaxPacketSize);
    /*@}*/

    /**
    * @defgroup CImage Image Access
    *
    * @brief The functions in this section provide access to information
    * and functionality of images. This includes creation, destruction,
    * and saving as well as a wealth of information including things like
    * width, height, stride, and timestamp.
    */
    /*@{*/

    /**
    * Creates an empty image;
    * images created this way must be destroyed
    *
    * @see spinError
    *
    * @param phImage The image handle pointer in which the empty image is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageCreateEmpty(spinImage* phImage);

    /**
    * Creates an image from another;
    * images created this way must be destroyed
    *
    * @see spinError
    *
    * @param hSrcImage The image to be copied
    * @param phDestImage The image handle pointer of the image to be created
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageCreate(spinImage hSrcImage, spinImage* phDestImage);

    /**
    * Creates an image with some set properties;
    * images created this way must be destroyed
    *
    * @see spinError
    *
    * @param phImage The image handle pointer in which the image is returned
    * @param width The width to set
    * @param height The height to set
    * @param offsetX The offset along the X axis to set
    * @param offsetY The offset along the Y axis to set
    * @param pixelFormat The pixel format to set
    * @param pData The image data to set; can be set to null
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageCreateEx(spinImage* phImage, size_t width, size_t height, size_t offsetX, size_t offsetY, spinPixelFormatEnums pixelFormat, void* pData);

    /**
    * Destroys an image
    * @see spinError
    *
    * @param hImage The image to destroy
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageDestroy(spinImage hImage);

    /**
    * Sets the default color processing algorithm of all images (if not otherwise set)
    * @see spinError
    *
    * @param algorithm The color processing algorithm used by default
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageSetDefaultColorProcessing(spinColorProcessingAlgorithm algorithm);

    /**
    * Retrieves the default color processing algorithm
    * @see spinError
    *
    * @param pAlgorithm The color processing algorithm enum pointer in which the color processing algorithm is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageGetDefaultColorProcessing(spinColorProcessingAlgorithm* pAlgorithm);

    /**
    * Retrieves the color processing algorithm of a specific image
    * @see spinError
    *
    * @param hImage The image of the color processing algorithm to retrieve
    * @param pAlgorithm The color processing algorithm pointer in which the color processing algorithm is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageGetColorProcessing(spinImage hImage, spinColorProcessingAlgorithm* pAlgorithm);

    /**
    * Converts the pixel format of one image into a new image
    * @see spinError
    *
    * @param hSrcImage The image to be converted
    * @param pixelFormat The pixel format to be converted to
    * @param hDestImage The image handle pointer in which the converted image is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageConvert(spinImage hSrcImage, spinPixelFormatEnums pixelFormat, spinImage hDestImage);

    /**
    * Converts the pixel format and color processing algorithm of one image into a new image
    * @see spinError
    *
    * @param hSrcImage The image to be converted
    * @param pixelFormat The pixel format to be converted to
    * @param algorithm The color processing algorithm to use for conversion
    * @param hDestImage The image handle pointer in which the converted image is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageConvertEx(spinImage hSrcImage, spinPixelFormatEnums pixelFormat, spinColorProcessingAlgorithm algorithm, spinImage hDestImage);

    /**
    * Resets an image with some set properties
    * @see spinError
    *
    * @param hImage The image to be reset
    * @param width The width to be reset to
    * @param height The height to be reset to
    * @param offsetX The offset to be reset to along the X axis
    * @param offsetY The offset to be reset to along the Y axis
    * @param pixelFormat The pixel format to be reset to
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageReset(spinImage hImage, size_t width, size_t height, size_t offsetX, size_t offsetY, spinPixelFormatEnums pixelFormat);

    /**
    * Resets an image with some set properties and image data
    * @see spinError
    *
    * @param hImage The image to reset
    * @param width The width to be reset to
    * @param height The height to be reset to
    * @param offsetX The offset to be reset to along the X axis
    * @param offsetY The offset to be reset to along the Y axis
    * @param pixelFormat The pixel format to be reset to
    * @param pData The image data to reset to
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageResetEx(spinImage hImage, size_t width, size_t height, size_t offsetX, size_t offsetY, spinPixelFormatEnums pixelFormat, void* pData);

    /**
    * Retrieves the ID of an image
    * @see spinError
    *
    * @param hImage The image of the ID to retrieve
    * @param pId The unsigned integer pointer in which the ID is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageGetID(spinImage hImage, uint64_t* pId);

    /**
    * Retrieves the image data of an image
    * @see spinError
    *
    * @param hImage The image of the image data to retrieve
    * @param ppData The pointer to the void pointer in which the image data is retrieved
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageGetData(spinImage hImage, void** ppData);

    /**
    * Retrieves the private data of an image
    * @see spinError
    *
    * @param hImage The image of the private image data to retrieve
    * @param ppData The pointer to the void pointer in which the private image data is retrieved
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageGetPrivateData(spinImage hImage, void** ppData);

    /**
    * Retrieves the buffer size of an image
    * @see spinError
    *
    * @param hImage The image of image data buffer to retrieve
    * @param pSize The unsigned integer pointer in which the size of the image data if returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageGetBufferSize(spinImage hImage, size_t* pSize);

    /**
    * Creates a deep copy of an image (the destination image must be created as an empty image prior to the deep copy)
    * @see spinError
    *
    * @param hSrcImage The image to be copied
    * @param hDestImage The image handle in which the image is copied
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageDeepCopy(spinImage hSrcImage, spinImage hDestImage);

    /**
    * Retrieves the width of an image
    * @see spinError
    *
    * @param hImage The image of the width to retrieve
    * @param pWidth The unsigned integer pointer in which the width is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageGetWidth(spinImage hImage, size_t* pWidth);

    /**
    * Retrieves the height of an image
    * @see spinError
    *
    * @param hImage The image of the height to retrieve
    * @param pHeight The unsigned integer pointer in which the height is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageGetHeight(spinImage hImage, size_t* pHeight);

    /**
    * Retrieves the offset of an image along its X axis
    * @see spinError
    *
    * @param hImage The image of the offset along the X axis to retrieve
    * @param pOffsetX The unsigned integer pointer in which the offset along the X axis is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageGetOffsetX(spinImage hImage, size_t* pOffsetX);

    /**
    * Retrieves the offset of an image along its Y axis
    * @see spinError
    *
    * @param hImage The image of the offset along the Y axis to retrieve
    * @param pOffsetY The unsigned integer pointer in which the offset along the Y axis is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageGetOffsetY(spinImage hImage, size_t* pOffsetY);

    /**
    * Retrieves the padding of an image along its X axis
    * @see spinError
    *
    * @param hImage The image of the padding along the X axis to retrieve
    * @param pPaddingX The unsigned integer pointer in which the padding along the X axis is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageGetPaddingX(spinImage hImage, size_t* pPaddingX);

    /**
    * Retrieves the padding of an image along its Y axis
    * @see spinError
    *
    * @param hImage The image of the padding along the Y axis to retrieve
    * @param pPaddingY The unsigned integer pointer in which the padding along the Y axis is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageGetPaddingY(spinImage hImage, size_t* pPaddingY);

    /**
    * Retrieves the frame ID of an image
    * @see spinError
    *
    * @param hImage The image of the frame ID to retrieve
    * @param pFrameID The unsigned integer pointer in which the frame ID is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageGetFrameID(spinImage hImage, uint64_t* pFrameID);

    /**
    * Retrieves the timestamp of an image
    * @see spinError
    *
    * @param hImage The image of the timestamp to retrieve
    * @param pTimeStamp The unsigned integer pointer om which the timestamp is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageGetTimeStamp(spinImage hImage, uint64_t* pTimeStamp);

    /**
    * Retrieves the payload type of an image (as an enum, spinPayloadTypeInfoIds)
    * @see spinError
    * @see spinPayloadTypeInfoIds
    *
    * @param hImage The image of the payload type to retrieve
    * @param pPayloadType The payload type enum pointer in which the payload type is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageGetPayloadType(spinImage hImage, size_t* pPayloadType);

    /**
    * Retrieves the transport layer payload type of an image (as an enum, spinPayloadTypeInfoIds)
    * @see spinError
    * @see spinPayloadTypeInfoIds
    *
    * @param hImage The image of the TL payload type to retrieve
    * @param pPayloadType The payload type enum pointer in which the TL payload type is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageGetTLPayloadType(spinImage hImage, spinPayloadTypeInfoIDs* pPayloadType);

    /**
    * Retrieves the pixel format of an image (as an enum, spinPixelFormatEnums)
    * @see spinError
    * @see spinPixelFormatEnums
    *
    * @param hImage The image of the pixel format to retrieve
    * @param pPixelFormat The pixel format enum pointer in which the pixel format is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageGetPixelFormat(spinImage hImage, spinPixelFormatEnums* pPixelFormat);

    /**
    * Retrieves the transport layer pixel format of an image (as an unsigned integer)
    * @see spinError
    *
    * @param hImage The image of the TL pixel format to retrieve
    * @param pPixelFormat The unsigned integer pointer in which the TL pixel format is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageGetTLPixelFormat(spinImage hImage, uint64_t* pPixelFormat);

    /**
    * Retrieves the transport layer pixel format namespace of an image (as an enum, spinPixelFormatNamespaceID)
    * @see spinError
    * @see spinPixelFormatNamespaceID
    *
    * @param hImage The image of the TL pixel format namespace to retrieve
    * @param pPixelFormatNamespace The pixel format namespace pointer in which the pixel format namespace is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageGetTLPixelFormatNamespace(spinImage hImage, spinPixelFormatNamespaceID* pPixelFormatNamespace);

    /**
    * Retrieves the pixel format of an image (as a symbolic)
    * @see spinError
    *
    * @param hImage The image of the pixel format to retrieve
    * @param pBuf The c-string character buffer in which the pixel format symbolic is returned
    * @param pBufLen The unsigned integer pointer in which the length of the c-string is returned; the input value is the maximum length
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageGetPixelFormatName(spinImage hImage, char* pBuf, size_t* pBufLen);

    /**
    * Checks whether an image is incomplete
    * @see spinError
    *
    * @param hImage The image to check
    * @param pbIsIncomplete The boolean pointer to return whether or not the image is incomplete
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageIsIncomplete(spinImage hImage, bool8_t* pbIsIncomplete);

    /**
    * Retrieves the valid payload size of an image
    * @see spinError
    *
    * @param hImage The image of the payload size to retrieve
    * @param pSize The unsigned integer pointer in which the size of the valid payload is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageGetValidPayloadSize(spinImage hImage, size_t* pSize);

    /**
    * Saves an image using a specified file format (using an enum, spinImageFileFormat)
    * @see spinError
    * @see spinImageFileFormat
    *
    * @param hImage The image to be saved
    * @param pFilename The filename to use to save the image (with or without the appropriate file extension)
    * @Param format The file format to use to save the image
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageSave(spinImage hImage, const char* pFilename, spinImageFileFormat format);

    /**
    * Saves an image using a specified file format (using the extension of the filename)
    * @see spinError
    *
    * @param hImage The image to be saved
    * @param pFilename The filename to use to save the image (with or without the appropriate file extension)
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageSaveFromExt(spinImage hImage, const char* pFilename);

    /**
    * Saves an image as a PNG image
    * @see spinError
    *
    * @param hImage The image to be saved
    * @param pFilename The filename to use to save the image (with or without the appropriate file extension)
    * @param pOption The image options related to saving as PNG; includes compression level and whether to save as interlaced
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageSavePng(spinImage hImage, const char* pFilename, const spinPNGOption* pOption);

    /**
    * Saves an image as a PPM image
    * @see spinError
    *
    * @param hImage The image to be saved
    * @param pFilename The filename to use to save the image (with or without the appropriate file extension)
    * @param pOption The image options related to saving as PPM; includes whether to save as binary
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageSavePpm(spinImage hImage, const char* pFilename, const spinPPMOption* pOption);

    /**
    * Saves an image as an PGM image
    * @see spinError
    *
    * @param hImage The image to be saved
    * @param pFilename The filename to use to save the image (with or without the appropriate file extension)
    * @param pOption The image options related to saving as PGM; includes whether to save as binary
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageSavePgm(spinImage hImage, const char* pFilename, const spinPGMOption* pOption);

    /**
    * Saves an image as a TIFF image
    * @see spinError
    *
    * @param hImage The image to be saved
    * @param pFilename The filename to use to save the image (with or without the appropriate file extension)
    * @param pOption The image options related to saving as TIFF; includes compression method
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageSaveTiff(spinImage hImage, const char* pFilename, const spinTIFFOption* pOption);

    /**
    * Saves an image as a JPEG image
    * @see spinError
    *
    * @param hImage The image to be saved
    * @param pFilename The filename to use to save the image (with or without the appropriate file extension)
    * @param pOption The image options related to saving as JPEG; includes quality and whether to save as progressive
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageSaveJpeg(spinImage hImage, const char* pFilename, const spinJPEGOption* pOption);

    /**
    * Saves an image as a JPEG 2000 image
    * @see spinError
    *
    * @param hImage The image to be saved
    * @param pFilename The filename to use to save the image (with or without the appropriate file extension)
    * @param pOption The image options related to saving as JPEG 2000; includes quality
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageSaveJpg2(spinImage hImage, const char* pFilename, const spinJPG2Option* pOption);

    /**
    * Saves an image as a BMP image
    * @see spinError
    *
    * @param hImage The image to be saved
    * @param pFilename The filename to use to save the image (with or without the appropriate file extension)
    * @param pOption The image options related to saving as BMP; includes whether to save as indexed 8-bit
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageSaveBmp(spinImage hImage, const char* pFilename, const spinBMPOption* pOption);

    /**
    * Retrieves the chunk layout ID of an image
    * @see spinError
    *
    * @param hImage The image to be saved
    * @param pId The unsigned integer pointer in which the chunk layout ID is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageGetChunkLayoutID(spinImage hImage, uint64_t* pId);

    /**
    * Calculates the image statistics of an image
    * @see spinError
    *
    * @param hImage The image to be saved
    * @param hStatistics The image statistics context in which the calculated statistics are returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageCalculateStatistics(spinImage hImage, const spinImageStatistics hStatistics);

    /**
    * Retrieves the image status of an image
    * @see spinError
    *
    * @param hImage The image to be saved
    * @param pStatus The status enum pointer in which the image status is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageGetStatus(spinImage hImage, spinImageStatus* pStatus);

    /**
    * Retrieves the description of image status
    * @see spinError
    *
    * @param status The status enum
    * @param pBuf The c-string character buffer in which the explanation of image status enum is returned
    * @param pBufLen The unsigned integer pointer in which the length of the c-string is returned; the input value is the maximum length; if pBuf is NULL, minimum length of string buffer is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageGetStatusDescription(spinImageStatus status, char* pBuf, size_t* pBufLen);

    /**
    * Releases an image
    * @see spinError
    *
    * @param hImage The image to be saved
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageRelease(spinImage hImage);

    /**
    * Checks whether an image has CRC
    * @see spinError
    *
    * @param hImage The image to be saved
    * @param pbHasCRC The boolean pointer to return whether the image has CRC available
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageHasCRC(spinImage hImage, bool8_t* pbHasCRC);

    /**
    * Checks whether the CRC of an image is correct
    * @see spinError
    *
    * @param hImage The image to be saved
    * @param pbCheckCRC The boolean pointer to return whether the image CRC passes
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageCheckCRC(spinImage hImage, bool8_t* pbCheckCRC);

    /**
    * Retrieves the number of bits per pixel of an image
    * @see spinError
    *
    * @param hImage The image to be saved
    * @param pBitsPerPixel The unsigned integer pointer in which the number of bits per pixel is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageGetBitsPerPixel(spinImage hImage, size_t* pBitsPerPixel);

    /**
    * Retrieves the size of an image
    * @see spinError
    *
    * @param hImage The image to be saved
    * @param pImageSize The unsigned integer pointer in which the size of the image is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageGetSize(spinImage hImage, size_t* pImageSize);

    /**
    * Retrieves the stride of an image
    * @see spinError
    *
    * @param hImage The image to be saved
    * @param pStride The unsigned integer pointer in which the stride is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageGetStride(spinImage hImage, size_t* pStride);
    /*@}*/

    /**
    * @defgroup CEvent Event Access
    *
    * @brief The functions in this section allow for the creation and
    * destruction of events.
    */
    /*@{*/

    /**
    * Creates a device event
    * @see spinError
    *
    * @param phDeviceEvent The device event handle pointer in which the device event context is created
    * @param pFunction The function to be called at device event occurrences; signature to match: void(*spinDeviceEventFunction)(const spinDeviceEventData hEventData, const char* pEventName, void* pUserData)
    * @param pUserData Properties that can be passed into the event function
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinDeviceEventCreate(spinDeviceEvent* phDeviceEvent, spinDeviceEventFunction pFunction, void* pUserData);

    /**
    * Destroys a device event
    * @see spinError
    *
    * @param hDeviceEvent The device event to destroy
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinDeviceEventDestroy(spinDeviceEvent hDeviceEvent);

    /**
    * Creates an image event
    * @see spinError
    *
    * @param phImageEvent The image event handle pointer in which the image event context is created
    * @param pFunction The function to be called at image event occurrences; signature to match: void(*spinImageEventFunction)(const spinImage hImage, void* pUserData)
    * @param pUserData Properties that can be passed into the event function
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageEventCreate(spinImageEvent* phImageEvent, spinImageEventFunction pFunction, void* pUserData);

    /**
    * Destroys an image event
    * @see spinError
    *
    * @param hImageEvent The image event to destroy
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageEventDestroy(spinImageEvent hImageEvent);

    /**
    * Creates an arrival event
    * @see spinError
    *
    * @param phArrivalEvent The arrival event handle pointer in which the arrival event context is created
    * @param pFunction The function to be called at device event occurrences; signature to match: void(*spinArrivalEventFunction)(void* pUserData)
    * @param pUserData Properties that can be passed into the event function
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinArrivalEventCreate(spinArrivalEvent* phArrivalEvent, spinArrivalEventFunction pFunction, void* pUserData);

    /**
    * Destroys an arrival event
    * @see spinError
    *
    * @param hArrivalEvent The arrival event to destroy
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinArrivalEventDestroy(spinArrivalEvent hArrivalEvent);

    /**
    * Creates a removal event
    * @see spinError
    *
    * @param phRemovalEvent The removal event handle pointer in which the removal event context is created
    * @param pFunction The function to be called at device event occurrences; signature to match: void(*spinRemovalEventFunction)(uint64_t deviceSerialNumber, void* pUserData)

    * @param pUserData Properties that can be passed into the event function
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinRemovalEventCreate(spinRemovalEvent* phRemovalEvent, spinRemovalEventFunction pFunction, void* pUserData);

    /**
    * Destroys a removal event
    * @see spinError
    *
    * @param hRemovalEvent The removal event to destroy
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinRemovalEventDestroy(spinRemovalEvent hRemovalEvent);

    /**
    * Creates an interface event (both arrival and removal)
    * @see spinError
    *
    * @param phInterfaceEvent The interface event handle pointer in which the interface event context is created
    * @param pArrivalFunction The function to be called at arrival event occurrences; signature to match: void(*spinArrivalEventFunction)(void* pUserData)
    * @param hRemovalFunction The function to be called at removal event occurrences; signature to match: void(*spinRemovalEventFunction)(uint64_t deviceSerialNumber, void* pUserData)
    * @param pUserData Properties that can be passed into the event function
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinInterfaceEventCreate(spinInterfaceEvent* phInterfaceEvent, spinArrivalEventFunction pArrivalFunction, spinRemovalEventFunction pRemovalFunction, void* pUserData);

    /**
    * Destroys an interface event (both arrival and removal)
    * @see spinError
    *
    * @param hInterfaceEvent The interface event to destroy
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinInterfaceEventDestroy(spinInterfaceEvent hInterfaceEvent);

    /**
    * Creates a log event
    * @see spinError
    *
    * @param phLogEvent The log event handle pointer in which the log event context is created
    * @param pFunction The function to be called at device event occurrences; signature to match: void(*spinLogEventFunction)(const spinLogEventData hEventData, void* pUserData)
    * @param pUserData Properties that can be passed into the event function
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinLogEventCreate(spinLogEvent* phLogEvent, spinLogEventFunction pFunction, void* pUserData);

    /**
    * Destroys a log event
    * @see spinError
    *
    * @param hLogEvent The log event to destroy
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinLogEventDestroy(spinLogEvent hLogEvent);
    /*@}*/

    /**
    * @defgroup CImageStatistics ImageStatistics Access
    *
    * @brief The functions in this section provide access to information and
    * functionality related to image statistics. This includes context
    * creation and destruction, the enabling and disabling of channels, and
    * value retrieval.
    */
    /*@{*/

    /**
    * Creates an image statistics context
    *
    * @param phStatistics The statistics handle pointer in which the image statistics context is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageStatisticsCreate(spinImageStatistics* phStatistics);

    /**
    * Destroys an image statistics context
    * @see spinError
    *
    * @param hStatistics The image statistics context to destroy
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageStatisticsDestroy(spinImageStatistics hStatistics);

    /**
    * Enables all channels of an image statistics context
    * @see spinError
    *
    * @param hStatistics The image statistics context to enable all channels
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageStatisticsEnableAll(spinImageStatistics hStatistics);

    /**
    * Disables all channels of an image statistics context
    * @see spinError
    *
    * @param hStatistics The image statistics context to disable all channels
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageStatisticsDisableAll(spinImageStatistics hStatistics);

    /**
    * Disables all channels of an image statistics context except grey-scale
    * @see spinError
    *
    * @param hStatistics The image statistics context to enable only grey
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageStatisticsEnableGreyOnly(spinImageStatistics hStatistics);

    /**
    * Disables all channels of an image statistics context except red, blue, and green
    * @see spinError
    *
    * @param hStatistics The image statistics context to enable only RGB
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageStatisticsEnableRgbOnly(spinImageStatistics hStatistics);

    /**
    * Disables all channels of an image statistics context except hue, saturation, and lightness
    * @see spinError
    *
    * @param hStatistics The image statistics context to enable only HSL
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageStatisticsEnableHslOnly(spinImageStatistics hStatistics);

    /**
    * Checks whether an image statistics context is enabled
    * @see spinError
    *
    * @param hStatistics The image statistics context of the channel
    * @param channel The channel to check
    * @param pbEnabled The boolean pointer to return whether or not the channel is enabled
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageStatisticsGetChannelStatus(spinImageStatistics hStatistics, spinStatisticsChannel channel, bool8_t* pbEnabled);

    /**
    * Sets the status of an image statistics channel
    * @see spinError
    *
    * @param hStatistics The image statistics context of the channel
    * @param channel The channel to enable/disable
    * @param bEnable The boolean value to set; true enables, false disables
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageStatisticsSetChannelStatus(spinImageStatistics hStatistics, spinStatisticsChannel channel, bool8_t bEnable);

    /**
    * Retrieves the range of an image statistics channel
    * @see spinError
    *
    * @param hStatistics The image statistics context of the channel
    * @param channel The channel of the range to retrieve
    * @param pMin The unsigned integer pointer in which the minimum value of the range is returned
    * @param pMax The unsigned integer pointer in which the maximum value of the range is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageStatisticsGetRange(spinImageStatistics hStatistics, spinStatisticsChannel channel, unsigned int* pMin, unsigned int* pMax);

    /**
    * Retrieves the pixel value range of an image statistics channel
    * @see spinError
    *
    * @param hStatistics The image statistics context of the channel
    * @param channel The channel of the pixel value range to retrieve
    * @param pMin The unsigned integer pointer in which the minimum value of the pixel value range is returned
    * @param pMax The unsigned integer pointer in which the maximum value of the pixel value range is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageStatisticsGetPixelValueRange(spinImageStatistics hStatistics, spinStatisticsChannel channel, unsigned int* pMin, unsigned int* pMax);

    /**
    * Retrieves the number of pixel values of an image statistics channel
    * @see spinError
    *
    * @param hStatistics The image statistics context of the channel
    * @param channel The channel where the pixel values to be counted are
    * @param iNumValues The unsigned integer pointer in which the number of pixel values is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageStatisticsGetNumPixelValues(spinImageStatistics hStatistics, spinStatisticsChannel channel, unsigned int* pNumValues);

    /**
    * Retrieves the mean of pixel values of an image statistics channel
    * @see spinError
    *
    * @param hStatistics The image statistics context of the channel
    * @param channel The channel of the mean pixel value to be retrieved
    * @param pMean The float pointer in which the mean pixel value is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageStatisticsGetMean(spinImageStatistics hStatistics, spinStatisticsChannel channel, float* pMean);

    /**
    * Retrieves a histogram of an image statistics channel
    * @see spinError
    *
    * @param hStatistics The image statistics context of the channel
    * @param channel The channel of the histogram to be returned
    * @param pHistogram The pointer to the integer pointer in which the histogram data is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageStatisticsGetHistogram(spinImageStatistics hStatistics, spinStatisticsChannel channel, int** ppHistogram);

    /**
    * Retrieves all available information of an image statistics channel
    * @see spinError
    *
    * @param hStatistics The image statistics context of the channel
    * @param channel The channel of the information to retrieve
    * @param pRangeMin The unsigned integer pointer in which the minimum value of the range is returned
    * @param pRangeMax The unsigned integer pointer in which the maximum value of the range is returned
    * @param pPixelValueMin The unsigned integer pointer in which the minimum pixel value of the range is returned
    * @param pPixelValueMax The unsigned integer pointer in which the maximum pixel value of the range is returned
    * @param pNumPixelValues The unsigned integer pointer in which the number of pixel values is returned
    * @param pPixelValueMean The float pointer in which the mean pixel value is returned
    * @param ppiHistogram The pointer to the pointer in which the histogram data is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageStatisticsGetAll(
        spinImageStatistics hStatistics,
        spinStatisticsChannel channel,
        unsigned int* pRangeMin,
        unsigned int* pRangeMax,
        unsigned int* pPixelValueMin,
        unsigned int* pPixelValueMax,
        unsigned int* pNumPixelValues,
        float* pPixelValueMean,
        int** ppHistogram);
    /*@}*/

    /**
    * @defgroup CLoggingEventData Logging Event Data Access
    *
    * @brief The functions in this section allow for the retrieval
    * of logging event data.
    */
    /*@{*/

    /**
    * Retrieves the category name of a log event
    * @see spinError
    *
    * @param hLogEventData The log event data received from the log event
    * @param pBuf The c-string character buffer in which the category name of the log event is returned
    * @param pBufLen The unsigned integer pointer in which the length of the c-string is returned; the input value is the maximum length
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinLogDataGetCategoryName(spinLogEventData hLogEventData, char* pBuf, size_t* pBufLen);

    /**
    * Retrieves the priority of a log event
    * @see spinError
    *
    * @param hLogEventData The log event data received from the log event
    * @param pValue The integer pointer in which the priority value is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinLogDataGetPriority(spinLogEventData hLogEventData, int64_t* pValue);

    /**
    * Retrieves the priority name of a log event
    * @see spinError
    *
    * @param hLogEventData The log event data received from the log event
    * @param pBuf The c-string character buffer in which the priority name of the log event is returned
    * @param pBufLen The unsigned integer pointer in which the length of the c-string is returned; the input value is the maximum length
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinLogDataGetPriorityName(spinLogEventData hLogEventData, char* pBuf, size_t* pBufLen);

    /**
    * Retrieves the timestamp of a log event
    * @see spinError
    *
    * @param hLogEventData The log event data received from the log event
    * @param pBuf The c-string character buffer in which the timestamp of the log event is returned
    * @param pBufLen The unsigned integer pointer in which the length of the c-string is returned; the input value is the maximum length
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinLogDataGetTimestamp(spinLogEventData hLogEventData, char* pBuf, size_t* pBufLen);

    /**
    * Retrieves the NDC of a log event
    * @see spinError
    *
    * @param hLogEventData The log event data received from the log event
    * @param pBuf The c-string character buffer in which the NDC of the log event is returned
    * @param pBufLen The unsigned integer pointer in which the length of the c-string is returned; the input value is the maximum length
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinLogDataGetNDC(spinLogEventData hLogEventData, char* pBuf, size_t* pBufLen);

    /**
    * Retrieves the thread name of a log event
    * @see spinError
    *
    * @param hLogEventData The log event data received from the log event
    * @param pBuf The c-string character buffer in which the thread name of the log event is returned
    * @param pBufLen The unsigned integer pointer in which the length of the c-string is returned; the input value is the maximum length
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinLogDataGetThreadName(spinLogEventData hLogEventData, char* pBuf, size_t* pBufLen);

    /**
    * Retrieves the log message of a log event
    * @see spinError
    *
    * @param hLogEventData The log event data received from the log event
    * @param pBuf The c-string character buffer in which the log message of the log event is returned
    * @param pBufLen The unsigned integer pointer in which the length of the c-string is returned; the input value is the maximum length
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinLogDataGetLogMessage(spinLogEventData hLogEventData, char* pBuf, size_t* pBufLen);

    /*@}*/

    /**
    * @defgroup CDeviceEventData Device Event Data Access
    *
    * @brief The functions in this section allow for the retrieval
    * of device event data.
    */
    /*@{*/

    /**
    * Retrieves the event ID of a device event
    * @see spinError
    *
    * @param hDeviceEventData The log event data received from the log event
    * @param pEventId The unsigned integer pointer in which the event ID is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinDeviceEventGetId(spinDeviceEventData hDeviceEventData, uint64_t* pEventId);

    /**
    * Retrieves the payload data of a device event
    * @see spinError
    *
    * @param hDeviceEventData The log event data received from the log event
    * @param pBuf The unsigned integer pointer in which the event payload is returned
    * @param pBufSize The unsigned integer pointer in which the size of the payload is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinDeviceEventGetPayloadData(spinDeviceEventData hDeviceEventData, const uint8_t* pBuf, size_t* pBufSize);

    /**
    * Retrieves the payload data size of a device event
    * @see spinError
    *
    * @param hDeviceEventData The log event data received from the log event
    * @param pBufSize The unsigned integer pointer in which the size of the payload is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinDeviceEventGetPayloadDataSize(spinDeviceEventData hDeviceEventData, size_t* pBufSize);

    /**
    * Retrieves the event name of a device event
    * @see spinError
    *
    * @param hDeviceEventData The log event data received from the log event
    * @param pBuf The c-string character buffer in which the name of the device event is returned
    * @param pBufLen The unsigned integer pointer in which the length of the c-string is returned; the input value is the maximum length
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinDeviceEventGetName(spinDeviceEventData hDeviceEventData, char* pBuf, size_t* pBufLen);

    /*@}*/

    /**
    * @defgroup CAVIRecorder AVIRecorder Access
    *
    * @brief The functions in this section provide access to AVI recording
    * capabilities, which include opening, building, and closing video
    * files.
    *
    * NOTE: This class is deprecated and replaced by SpinVideo. Refer to SpinVideoC.h instead.
    */
    /*@{*/

    /*
    * Opens an empty video file to create an uncompressed AVI video
    * @see spinError
    *
    * @param phRecorder The AVI recorder handle pointer in which the AVI recorder is returned
    * @param pName The filename to save the video as; do not include the extension
    * @param option The video options related to saving as uncompressed AVI; includes frame rate
    *
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API_DEPRECATED("spinAVIRecorderOpenUncompressed is deprecated, use spinVideoOpenUncompressed instead.",
        spinAVIRecorderOpenUncompressed(spinAVIRecorder* phRecorder, const char* pName, spinAVIOption option));

    /*
    * Opens an empty video file to create an MJPG file
    * @see spinError
    *
    * @param phRecorder The AVI recorder handle pointer in which the AVI recorder is returned
    * @param pName The filename to save the video as; do not include the extension
    * @param option The video options related to saving as MJPG; includes frame rate and quality
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API_DEPRECATED("spinAVIRecorderOpenMJPG is deprecated, use spinVideoOpenMJPG instead.",
        spinAVIRecorderOpenMJPG(spinAVIRecorder* phRecorder, const char* pName, spinMJPGOption option));

    /*
    * Opens an empty video file to create an H264 MP4 file
    * @see spinError
    *
    * @param phRecorder The AVI recorder handle pointer in which the AVI recorder is returned
    * @param pName The filename to save the video as; do not include the extension
    * @param option The video options related to saving as MJPG; includes frame rate, bitrate, height, and width
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API_DEPRECATED("spinAVIRecorderOpenH264 is deprecated, use spinVideoOpenH264 instead.",
        spinAVIRecorderOpenH264(spinAVIRecorder* phRecorder, const char* pName, spinH264Option option));

    /*
    * Appends an image to the end of an open video file
    * @see spinError
    *
    * @param hRecorder The AVI recorder to append the image to
    * @param hImage The image to append
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API_DEPRECATED("spinAVIRecorderAppend is deprecated, use spinVideoAppend instead.",
        spinAVIRecorderAppend(spinAVIRecorder hRecorder, spinImage hImage));

    /**
    * Set the maximum file size (in megabytes) of a AVI/MP4 file. A new AVI/MP4 file
    * is created automatically when file size limit is reached. Setting
    * a maximum size of 0 indicates no limit on file size.
    *
    * @param spinAVIRecorder The AVI recorder to append the image to
    * @param size The maximum AVI file size in MB.
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API_DEPRECATED("spinAVISetMaximumSize is deprecated, use spinVideoSetMaximumFileSize instead.",
        spinAVISetMaximumSize(spinAVIRecorder hRecorder, unsigned int size));

    /*
    * Closes a video file to complete its creation
    * @see spinError
    *
    * @param hRecorder The AVI recorder to close
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API_DEPRECATED("spinAVIRecorderClose is deprecated, use spinVideoClose instead.",
        spinAVIRecorderClose(spinAVIRecorder hRecorder));

    /*@}*/

    // Chunk data from Image
    /**
    * @defgroup CChunkData Chunk data access
    *
    * @brief The functions in this section provide access to chunk data
    * stored on images.
    */
    /*@{*/

    /*
    * Retrieves an integer value of a piece of chunk data
    * @see spinError
    *
    * @param hImage The image of the chunk data to retrieve
    * @param pName The name of the piece of chunk data to retrieve
    * @param pValue The integer pointer in which the piece of chunk data is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageChunkDataGetIntValue(spinImage hImage, const char* pName, int64_t* pValue);

    /*
    * Retrieves a float value of a piece of chunk data
    * @see spinError
    *
    * @param hImage The image of the chunk data to retrieve
    * @param pName The name of the piece of chunk data to retrieve
    * @param pValue The float pointer in which the piece of chunk data is returned
    *
    * @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
    */
    SPINNAKERC_API spinImageChunkDataGetFloatValue(spinImage hImage, const char* pName, double* pValue);

    /*@}*/

#ifdef __cplusplus
}
#endif

/*@}*/

#endif /* FLIR_SPINNAKER_C_H */
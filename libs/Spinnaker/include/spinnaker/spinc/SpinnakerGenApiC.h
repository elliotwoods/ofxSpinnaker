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

#ifndef PGR_SPINNAKER_GENAPI_C_H_
#define PGR_SPINNAKER_GENAPI_C_H_

#include "SpinnakerGenApiDefsC.h"
#include "SpinnakerDefsC.h"
#include "SpinnakerPlatformC.h"

/**
* @defgroup SpinnakerCGenApi Spinnaker C GenICam API
*/
/*@{*/

#ifdef __cplusplus
extern "C" {
#endif

	/**
	* @defgroup CNodeMapAccess Node Map Access
	*
	* @brief The functions in this section provide access to information,
	* objects, and functionality related to nodemaps. This includes nodes,
	* node counts, and polling.
	*/
	/*@{*/

	/**
	* Retrieves a node from the nodemap by name
	* @see spinError
	*
	* @param hNodeMap The node map where the node is
	* @param pName The name of the node
	* @param phNode The node handle pointer in which the node is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinNodeMapGetNode(spinNodeMapHandle hNodeMap, const char* pName, spinNodeHandle* phNode);

	/**
	* Gets the number of nodes in the map
	* @see spinError
	*
	* @param hNodeMap The node map where the nodes to be counted are
	* @param pValue The unsigned integer pointer in which the number of nodes is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinNodeMapGetNumNodes(spinNodeMapHandle hNodeMap, size_t* pValue);

	/**
	* Retrieves a node from the nodemap by index
	* @see spinError
	*
	* @param hNodeMap The node map where the node is
	* @param index The index of the node
	* @param phNode The node handle pointer in which the node is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinNodeMapGetNodeByIndex(spinNodeMapHandle hNodeMap, size_t index, spinNodeHandle* phNode);

	/**
	* Fires nodes which have a polling time
	* @see spinError
	*
	* @param hNodeMap The nodemap to poll
	* @param timestamp The timestamp
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinNodeMapPoll(spinNodeMapHandle hNodeMap, int64_t timestamp);
	/*@}*/

	/**
	* @defgroup CNodeAccess Node Access
	*
	* @brief The functions in this section provide access to information and
	* objects retrieved from nodes. This includes node properties and callback
	* registration.
	*/
	/*@{*/

	/**
	* Checks whether a node is implemented
	* @see spinError
	*
	* @param hNode The node to check
	* @param pbResult The boolean pointer to return whether or not the node is implemented
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinNodeIsImplemented(spinNodeHandle hNode, bool8_t* pbResult);

	/**
	* Checks whether a node is readable
	* @see spinError
	*
	* @param hNode The node to check
	* @param pbResult The boolean pointer to return whether or not the node is readable
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinNodeIsReadable(spinNodeHandle hNode, bool8_t* pbResult);

	/**
	* Checks whether a node is writable
	* @see spinError
	*
	* @param hNode The node to check
	* @param pbResult The boolean pointer to return whether or not the node is writable
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinNodeIsWritable(spinNodeHandle hNode, bool8_t* pbResult);

	/**
	* Checks whether a node is available
	* @see spinError
	*
	* @param hNode The node to check
	* @param pbResult The boolean pointer to return whether or not the node is available
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinNodeIsAvailable(spinNodeHandle hNode, bool8_t* pbResult);

	/**
	* Checks whether two nodes are equal
	*
	* @see spinError
	*
	* @param hNodeFirst The first node to check
	* @param hNodeSecond The second node to check
	* @param pbResult The boolean pointer to return whether or not the two nodes are equal
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinNodeIsEqual(spinNodeHandle hNodeFirst, spinNodeHandle hNodeSecond, bool8_t* pbResult);

	/**
	* Retrieves the access mode of a node (as an enum, spinAccessMode)
	* @see spinError
	* @see spinAccessMode
	*
	* @param hNode The node of the access mode to retrieve
	* @param pAccessMode The access mode enum pointer in which the access mode is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinNodeGetAccessMode(spinNodeHandle hNode, spinAccessMode* pAccessMode);

	/**
	* Retrieves the name of a node (no whitespace)
	* @see spinError
	*
	* @param hNode The node of the name to retrieve
	* @param pBuf The c-string character buffer in which the name of the node is returned
	* @param pBufLen The unsigned integer pointer in which the length of the c-string is returned; the input value is the maximum length
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinNodeGetName(spinNodeHandle hNode, char* pBuf, size_t* pBufLen);

	/**
	* Retrieve the namespace of a node (as an enum, spinNameSpace)
	* @see spinError
	* @see spinNameSpace
	*
	* @param hNode The node of the namespace to retrieve
	* @param pNamespace The namespace enum pointer in which the namespace is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinNodeGetNameSpace(spinNodeHandle hNode, spinNameSpace* pNamespace);

	/**
	* Retrieves the recommended visibility of a node (as an enum, spinVisibility)
	* @see spinError
	* @see spinVisibility
	*
	* @param hNode The node of the visibility to retrieve
	* @param pVisibility The visibility enum pointer in which the visibility is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinNodeGetVisibility(spinNodeHandle hNode, spinVisibility* pVisibility);

	/**
	* Invalidates a node in case its values may have changed, rendering it no longer valid
	* @see spinError
	*
	* @param hNode The node whose values may have changed
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinNodeInvalidateNode(spinNodeHandle hNode);

	/**
	* Retrieves the caching mode of a node (as an enum, spinCachingMode)
	* @see spinError
	* @see spinCachingMode
	*
	* @param hNode The node of the caching mode to retrieve
	* @param pCachingMode The caching mode enum pointer in which the caching mode is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinNodeGetCachingMode(spinNodeHandle hNode, spinCachingMode* pCachingMode);

	/**
	* Retrieves a short description of a node
	* @see spinError
	*
	* @param hNode The node of the tooltip to retrieve
	* @param pBuf The c-string character buffer in which the short description of the node is returned
	* @param pBufLen The unsigned integer pointer in which the length of the c-string is returned; the input value is the maximum length
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinNodeGetToolTip(spinNodeHandle hNode, char* pBuf, size_t* pBufLen);

	/**
	* Retrieves a longer description of a node
	* @see spinError
	*
	* @param hNode The node of the description to retrieve
	* @param pBuf The c-string character buffer in which the longer descrition of the node is returned
	* @param pBufLen The unsigned integer pointer in which the length of the c-string is returned; the input value is the maximum length
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinNodeGetDescription(spinNodeHandle hNode, char* pBuf, size_t* pBufLen);

	/**
	* Retrieves the display name of a node (whitespace possible)
	* @see spinError
	*
	* @param hNode The node of the display name to retrieve
	* @param pBuf The c-string character buffer in which the display name of the node is returned
	* @param pBufLen The unsigned integer pointer in which the length of the c-string is returned; the input value is the maximum length
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinNodeGetDisplayName(spinNodeHandle hNode, char* pBuf, size_t* pBufLen);

	/**
	* Retrieves the type of a node (as an enum, spinNodeType)
	* @see spinError
	* @see spinNodeType
	*
	* @param hNode The node of the node type to retrieve
	* @param pType The node type enum pointer in which the type of node is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinNodeGetType(spinNodeHandle hNode, spinNodeType* pType);

	/**
	* Retrieve the polling time of a node
	* @see spinError
	*
	* @param hNode The node of the polling time to retrieve
	* @param pPollingTime The integer pointer in which the polling time is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinNodeGetPollingTime(spinNodeHandle hNode, int64_t* pPollingTime);

	/**
	* Registers a callback to a node
	* @see spinError
	*
	* @param hNode The node on which to register the callback
	* @param pCbFunction The function pointer of the function that will execute when the callback is triggered; must match signature "void spinNodeCallbackFunction(spinNodeHandle hNode)"
	* @param phCb The callback handle pointer in which the callback is returned; used to unregister callbacks
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinNodeRegisterCallback(spinNodeHandle hNode, spinNodeCallbackFunction pCbFunction, spinNodeCallbackHandle* phCb);

	/**
	* Unregisters a callback from a node
	* @see spinError
	*
	* @param hNode The node from which to unregister the callback
	* @param hCb The callback handle to unregister
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinNodeDeregisterCallback(spinNodeHandle hNode, spinNodeCallbackHandle hCb);

	/**
	* Retrieves the imposed access mode of a node
	* @see spinError
	*
	* @param hNode The node of the imposed access mode to retrieve
	* @param imposedAccessMode The access mode enum pointer in which the imposed access mode is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinNodeGetImposedAccessMode(spinNodeHandle hNode, spinAccessMode imposedAccessMode);

	/**
	* Retrieves the imposed visibility of a node
	* @see spinError
	*
	* @param hNode The node of the visibility to impose
	* @param imposedVisibility The visibility enum pointer in which the imposed visibility is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinNodeGetImposedVisibility(spinNodeHandle hNode, spinVisibility imposedVisibility);
	/*@}*/

	/**
	* @defgroup CIValueAccess IValue Access
	*
	* @brief The functions in this section provide access to nodes as
	* value nodes. As value nodes are not an actual node type, the functions
	* are named as regular nodes. Functions include reading from and writing to
	* any node with a string.
	*/
	/*@{*/

	/**
	* Retrieves the value of any node type as a c-string
	* @see spinError
	*
	* @param hNode The node of the value to read
	* @param pBuf The c-string character buffer in which the value of the node is returned
	* @param pBufLen The unsigned integer pointer in which the length of the c-string is returned; the input value is the maximum length
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinNodeToString(spinNodeHandle hNode, char* pBuf, size_t* pBufLen);

	/**
	* Retrieves the value of any node type as a c-string; manually set whether to verify the node
	* @see spinError
	*
	* @param hNode The node of the value to read
	* @param bVerify The boolean of whether to verify the node
	* @param pBuf The c-string character buffer in which the value of the node is returned
	* @param pBufLen The unsigned integer pointer in which the length of the c-string is returned; the input value is the maximum length
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinNodeToStringEx(spinNodeHandle hNode, bool8_t bVerify, char* pBuf, size_t* pBufLen);

	/**
	* Sets the value of any node type from a c-string; it is important to ensure that the value of the c-string is appropriate to the node type
	* @see spinError
	*
	* @param hNode The node having its value changed
	* @param pBuf The c-string of the value to set
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinNodeFromString(spinNodeHandle hNode, const char* pBuf);

	/**
	* Sets the value of any node type from a c-string; manually set whether to verify the node; ensure the value of the c-string is appropriate to the node type
	* @see spinError
	*
	* @param hNode The node having its value changed
	* @param bVerify The boolean of whether to verify the node
	* @param pBuf The c-string of the value to set
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinNodeFromStringEx(spinNodeHandle hNode, bool8_t bVerify, const char* pBuf);
	/*@}*/

	/**
	* @defgroup StringAccess String Access
	*
	* @brief The functions in this section provide access to string nodes using
	* character pointers and arrays. This includes getters and setters of
	* values and value lengths.
	*/
	/*@{*/

	/**
	* Sets the value of a string node
	* @see spinError
	*
	* @param hNode The string node having its value changed
	* @param pBuf The c-string of the value to set
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinStringSetValue(spinNodeHandle hNode, const char* pBuf);

	/**
	* Sets the value of a string node; manually set whether to verify the node
	* @see spinError
	*
	* @param hNode The string node having its value changed
	* @param bVerify The boolean of whether to verify the node
	* @param pBuf The c-string of the value to set
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinStringSetValueEx(spinNodeHandle hNode, bool8_t bVerify, const char* pBuf);

	/**
	* Retrieves the value of a string node as a c-string
	* @see spinError
	*
	* @param hNode The string node of the value to read
	* @param pBuf The c-string character buffer in which the value of the node is returned
	* @param pBufLen The unsigned integer pointer in which the length of the c-string is returned; the input value is the maximum length
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinStringGetValue(spinNodeHandle hNode, char* pBuf, size_t* pBufLen);

	/**
	* Retrieves the value of a string node as a cstring; manually set whether to verify the node
	* @see spinError
	*
	* @param hNode The string node of the value to read
	* @param bVerify The boolean of whether to verify the node
	* @param pBuf The c-string character buffer in which the value of the node is returned
	* @param pBufLen The unsigned integer pointer in which the length of the c-string is returned; the input value is the maximum length
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinStringGetValueEx(spinNodeHandle hNode, bool8_t bVerify, char* pBuf, size_t* pBufLen);

	/**
	* Retrieves the maximum length of the c-string to be returned
	* @see spinError
	*
	* @param hNode The string node of the length to retrieve
	* @param pValue The integer pointer in which the maximum length of the c-string is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinStringGetMaxLength(spinNodeHandle hNode, int64_t* pValue);
	/*@}*/

	/**
	* @defgroup CIIntegerAccess IInteger Access
	*
	* @brief The functions in this section provide access to integer nodes
	* using the int64_t data type. This includes value getters and setters,
	* min, max, and increment functions, and node representation.
	*/
	/*@{*/

	/**
	* Sets the value of an integer node
	* @see spinError
	*
	* @param hNode The integer node having its value changed
	* @param value The integer value to set
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinIntegerSetValue(spinNodeHandle hNode, int64_t value);

	/**
	* Sets the value of an integer node; manually set whether to verify the node
	* @see spinError
	*
	* @param hNode The integer node having its value changed
	* @param bVerify The boolean of whether to verify the node
	* @param value The integer value to set
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinIntegerSetValueEx(spinNodeHandle hNode, bool8_t bVerify, int64_t value);

	/**
	* Retrieves the value of an integer node
	* @see spinError
	*
	* @param hNode The integer node of the value to read
	* @param pValue The integer pointer in which the value is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinIntegerGetValue(spinNodeHandle hNode, int64_t* pValue);

	/**
	* Retrieves the value of an integer node; manually set whether to verify the node
	* @see spinError
	*
	* @param hNode The integer node of the value to read
	* @param bVerify The boolean of whether to verify the node
	* @param pValue The integer pointer in which the value is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinIntegerGetValueEx(spinNodeHandle hNode, bool8_t bVerify, int64_t* pValue);

	/**
	* Retrieves the minimum value of an integer node; all potential values must be greater than or equal to the minimum
	* @see spinError
	*
	* @param hNode The integer node of the minimum value to retrieve
	* @param pValue The integer pointer in which the minimum value is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinIntegerGetMin(spinNodeHandle hNode, int64_t* pValue);

	/**
	* Retrieves the maximum value of an integer node; all potential values must be lesser than or equal to the maximum
	* @see spinError
	*
	* @param hNode The integer node of the maximum value to retrieve
	* @param pValue The integer pointer in which the maximum value is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinIntegerGetMax(spinNodeHandle hNode, int64_t* pValue);

	/**
	* Retrieves the increment of an integer node; all possible values must be divisible by the increment
	* @see spinError
	*
	* @param hNode The integer node of the increment to retrieve
	* @param pValue The integer pointer in which the increment is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinIntegerGetInc(spinNodeHandle hNode, int64_t* pValue);

	/**
	* Retrieves the numerical representation of the value of a node; i.e. linear, logarithmic, hexidecimal, MAC address, etc.
	* @see spinError
	*
	* @param hNode The integer node of the numerical representation to retrieve
	* @param pValue The representation enum pointer in which the type of numerical representation is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinIntegerGetRepresentation(spinNodeHandle hNode, spinRepresentation* pValue);
	/*@}*/

	/**
	* @defgroup CIFloat IFloat Access
	*
	* @brief The functions in this section provide access to float nodes
	* using double as the data type. This includes value getters and setters,
	* min and max functions, and node representation.
	*/
	/*@{*/

	/**
	* Sets the value of a float node
	* @see spinError
	*
	* @param hNode The float node having its value changed
	* @param value The float value to set
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinFloatSetValue(spinNodeHandle hNode, double value);

	/**
	* Sets the value of a float node; manually set whether to verify the node
	* @see spinError
	*
	* @param hNode The float node having its value changed
	* @param bVerify The boolean of whether to verify the node
	* @param value The float value to set
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinFloatSetValueEx(spinNodeHandle hNode, bool8_t bVerify, double value);

	/**
	* Retrieves the value of a float node
	* @see spinError
	*
	* @param hNode The float node of the value to read
	* @param pValue The double pointer in which the value is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinFloatGetValue(spinNodeHandle hNode, double* pValue);

	/**
	* Retrieves the value of a float node; manually set whether to verify the node
	* @see spinError
	*
	* @param hNode The float node of the value to read
	* @param pValue The double pointer in which the value is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinFloatGetValueEx(spinNodeHandle hNode, bool8_t bVerify, double* pValue);

	/**
	* Retrieves the minimum value of a float node; all potential values must be greater than or equal to the minimum
	* @see spinError
	*
	* @param hNode The float node of the minimum value to retrieve
	* @param pValue The double pointer in which the minimum value is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinFloatGetMin(spinNodeHandle hNode, double* pValue);

	/**
	* Retrieves the maximum value of a float node; all potential values must be lesser than or equal to the maximum
	* @see spinError
	*
	* @param hNode The float node of the maximum value to retrieve
	* @param pValue The double pointer in which the maximum value is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinFloatGetMax(spinNodeHandle hNode, double* pValue);

	/**
	* Retrieves the numerical representation of the value of a node; i.e. linear, logarithmic, hexidecimal, MAC address, etc.
	* @see spinError
	*
	* @param hNode The float node of the numerical representation to retrieve
	* @param pValue The representation enum pointer in which the type of numerical representation is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinFloatGetRepresentation(spinNodeHandle hNode, spinRepresentation* pValue);

	/**
	* Retrieves the units of the float node value
	* @see spinError
	*
	* @param hNode The float node of the units to retrieve
	* @param pBuf The c-string character buffer in which the value units are returned
	* @param pBufLen The unsigned integer pointer in which the length of the c-string is returned; the input value is the maximum length
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinFloatGetUnit(spinNodeHandle hNode, char* pBuf, size_t* pBufLen);
	/*@}*/

	/**
	* @defgroup CIEnumeration IEnumeration Access
	*
	* @brief The functions in this section provide access to enum nodes.
	* This includes retrieving the number of entries, an entry by index or name,
	* retrieving the current entry node, or setting the node using an integer.
	*/
	/*@{*/

	/**
	* Retrieves the number of entries of an enum node
	* @see spinError
	*
	* @param hNode The enum node where the entries to be counted are
	* @param pValue The unsigned integer pointer in which the number of entries is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinEnumerationGetNumEntries(spinNodeHandle hNode, size_t* pValue);

	/**
	* Retrieves an entry node from an enum node using an index
	* @see spinError
	*
	* @param hNode The enum node from which the entry node is retrieved
	* @param index The index of the entry node
	* @param phEntry The node handle pointer in which the entry node is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinEnumerationGetEntryByIndex(spinNodeHandle hNode, size_t index, spinNodeHandle* phEntry);

	/**
	* Retrieves an entry node from an enum node using the entry's symbolic
	* @see spinError
	*
	* @param hNode The enum node from which the entry node is retrieved
	* @param pName The name of the entry node
	* @param phEntry The node handle pointer in which the entry node is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinEnumerationGetEntryByName(spinNodeHandle hNode, const char* pName, spinNodeHandle* phEntry);

	/**
	* Retrieves the currently selected entry node from an enum node
	* @see spinError
	*
	* @param hNode The enum node from which the current entry node is retrieved
	* @param phEntry The node handle pointer in which the current entry node is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinEnumerationGetCurrentEntry(spinNodeHandle hNode, spinNodeHandle* phEntry);

	/**
	* Sets a new entry using its integer value retrieved from a call to spinEnumerationEntryGetIntValue(); note that enumeration entry int and enum values are different - int values defined on camera, enum values found in SpinnakerDefsC.h
	* @see spinEnumerationEntryGetIntValue()
	* @see spinError
	*
	* @param hNode The enum node having its entry changed
	* @param value The integer value of the entry node to set; this corresponds to the integer value internal to the camera
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinEnumerationSetIntValue(spinNodeHandle hNode, int64_t value);

	/**
	* Sets a new entry using its enum; note that enumeration entry int and enum values are different - int values defined on camera, enum values found in SpinnakerDefsC.h
	* @see spinEnumerationEntryGetEnumValue()
	* @see spinError
	*
	* @param hNode The enum node have its entry changed
	* @param value The enum value of the entry node to set; this corresponds to its integer value created in the library
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinEnumerationSetEnumValue(spinNodeHandle hNode, size_t value);
	/*@}*/

	/**
	* @defgroup CIEnumEntry IEnumEntry Access
	*
	* @brief The functions in this section provide access to entry nodes
	* This includes retrieving the integer value or the symbolic of an entry.
	*/
	/*@{*/

	/**
	* Retrieves the integer value of an entry node; note that enumeration entry int and enum values are different - int values defined on camera, enum values found in SpinnakerDefsC.h
	* @see spinEnumerationSetIntValue()
	* @see spinError
	*
	* @param hNode The entry node of the integer value to retrieve
	* @param pValue The integer pointer in which the integer value of the entry is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinEnumerationEntryGetIntValue(spinNodeHandle hNode, int64_t* pValue);

	/**
	* Retrieves the enum value (as an integer) of an entry node; note that enumeraiton entry int and enum values are different - int values defined on camera, enum values found in SpinnakerDefsC.h
	* @see spinEnumerationSetEnumValue()
	* @see spinError
	*
	* @param hNode The entry node of the enum value to retrieve
	* @param pValue The unsigned integer pointer in which the enum value of the entry is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinEnumerationEntryGetEnumValue(spinNodeHandle hNode, size_t* pValue);

	/**
	* Retrieves the symbolic of an entry node as a c-string
	* @see spinError
	*
	* @param hNode The entry node of the symbolic to retrieve
	* @param pBuf The c-string character buffer in which the symbolic of the entry node is returned
	* @param pBufLen The unsigned integer pointer in which the length of the c-string is returned; the input value is the maximum length
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinEnumerationEntryGetSymbolic(spinNodeHandle hNode, char* pBuf, size_t* pBufLen);
	/*@}*/

	/**
	* @defgroup CIBoolean IBoolean Access
	*
	* @brief The functions in this section provide access to boolean nodes
	* using the bool8_t data type, values represented with 'True' and 'False'.
	* This includes value getters and setters.
	*/
	/*@{*/

	/**
	* Sets the value of a boolean node; boolean values are represented by 'True' (which equals '0') and 'False' (which equals '1')
	* @see spinError
	*
	* @param hNode The boolean node having its value changed
	* @param value The boolean value to set
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinBooleanSetValue(spinNodeHandle hNode, bool8_t value);

	/**
	* Retrieves the value of a boolean node; boolean values are represented by 'True' (which equals '0') and 'False' (which equals '1')
	* @see spinError
	*
	* @param hNode The boolean node of the value to read
	* @param pValue The boolean pointer in which the value is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinBooleanGetValue(spinNodeHandle hNode, bool8_t* pbValue);
	/*@}*/

	/**
	* @defgroup CICommand ICommand Access
	*
	* @brief The functions in this section all provide access to
	* information and objects retrieved from nodes. This includes node
	* properties and callbacks.
	*/
	/*@{*/

	/**
	* Executes the action associated to a command node
	* @see spinError
	*
	* @param hNode The command node to execute
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinCommandExecute(spinNodeHandle hNode);

	/**
	* Retrieves whether or not the action of a command node has completed
	* @see spinError
	*
	* @param hNode The command node to check
	* @param pValue The boolean pointer to return whether or not the command has completed
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinCommandIsDone(spinNodeHandle hNode, bool8_t* pbValue);
	/*@}*/

	/**
	* @defgroup CICategory ICategory Access
	*
	* @brief The functions in this section all provide access to
	* information and objects retrieved from nodes. This includes node
	* properties and callbacks.
	*/
	/*@{*/

	/**
	* Retrieves the number of a features (or child nodes) or a category node
	* @see spinError
	*
	* @param hNode The category node where the features to be counted are
	* @param pValue The unsigned integer pointer in which the number of features is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinCategoryGetNumFeatures(spinNodeHandle hNode, size_t* pValue);

	/**
	* Retrieves a node from a category node using an index
	* @see spinError
	*
	* @param hNode The category node of the node to retrieve
	* @param index The index of the feature node
	* @param phFeature The node handle pointer in which the feature node is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinCategoryGetFeatureByIndex(spinNodeHandle hNode, size_t index, spinNodeHandle* phFeature);
	/*@}*/

	/**
	* @defgroup CIRegister IRegister Access
	*
	* @brief The functions in this section provide access to register nodes.
	* This includes access to the node, its address and length, and reference.
	*/
	/*@{*/

	/**
	* Retrieves the value of a register node
	* @see spinError
	*
	* @param hNode The register node of the value to retrieve
	* @param pBuf The unsigned integer buffer in which the value is returned
	* @param length The integer pointer in which the length of the register array is returned; the input value is the maximum length
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinRegisterGet(spinNodeHandle hNode, uint8_t* pBuf, int64_t length);

	/**
	* Retrieves the value of a register node; manually set whether to verify the node and whether to ignore the cache
	* @see spinError
	*
	* @param hNode The register node of the value to retrieve
	* @param bVerify The boolean of whether to verify the node
	* @param IgnoreCache The boolean of whether to ignore the cache
	* @param pBuf The unsigned integer buffer in which the value is returned
	* @param length The integer pointer in which the length of the register array is returned; the input value is the maximum length
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinRegisterGetEx(spinNodeHandle hNode, bool8_t bVerify, bool8_t bIgnoreCache, uint8_t* pBuf, int64_t length);

	/**
	* Retrieves the address of a register node
	* @see spinError
	*
	* @param hNode The register node of the address to retrieve
	* @param pAddress The integer pointer in which the address is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinRegisterGetAddress(spinNodeHandle hNode, int64_t* pAddress);

	/**
	* Retrieves the length (in bytes) of the value of a register node
	* @see spinError
	*
	* @param hNode The register node of the length to retrieve
	* @param plength The integer in which the number of bytes is returned
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinRegisterGetLength(spinNodeHandle hNode, int64_t* pLength);

	/**
	* Sets the value of a register node
	* @see spinError
	*
	* @param hNode The register node of the value to set
	* @param pBuf The unsigned integer buffer of the value to set
	* @param length The number of bytes of the value to set
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinRegisterSet(spinNodeHandle hNode, const uint8_t* pBuf, int64_t length);

	/**
	* Sets the value of a register node; manually set whether to verify the node
	* @see spinError
	*
	* @param hNode The register node of the value to set
	* @param bVerify The boolean of whether to verify the node
	* @param pBuf The unsigned integer buffer of the value to set
	* @param length The number of bytes of the value to set
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinRegisterSetEx(spinNodeHandle hNode, bool8_t bVerify, const uint8_t* pBuf, int64_t length);

	/**
	* Uses a second node as a reference for a register node
	* @see spinError
	*
	* @param hNode The register node that houses the reference
	* @param hRef The reference node
	*
	* @return spinError The error code; returns SPINNAKER_ERR_SUCCESS (or 0) for no error
	*/
	SPINNAKERC_API spinRegisterSetReference(spinNodeHandle hNode, spinNodeHandle hRef);
	/*@}*/

#ifdef __cplusplus
}
#endif

/*@}*/

#endif /* PGR_SPINNAKER_GENAPI_C_H_ */
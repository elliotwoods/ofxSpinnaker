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

#ifndef FLIR_SPINNAKER_GENAPIDEFS_C_H
#define FLIR_SPINNAKER_GENAPIDEFS_C_H

/**
* @defgroup SpinnakerCGenApi Spinnaker C GenICam API
*/
/*@{*/

/**
* @defgroup SpinnakerGenApiHandlesC Spinnaker C GenICam Handles
*
* Handle definitions for Spinnaker C GenICam API
*/
/*@{*/

/**
* Handle for nodemap functionality. Created by calling
* spinCameraGetNodemap(), spinCameraGetTLDeviceNodeMap(),
* spinCameraGetTLStreamNodeMap() or spinInterfaceGetTLNodeMap(). No need
* to release, clear, or destroy.
*/
typedef void* spinNodeMapHandle;

/**
* Handle for node functionality. Created by calling
* spinNodeMapGetNode(). No need to release, clear, or destroy.
*/
typedef void* spinNodeHandle;

/**
* Handle for callback functionality. Created by calling
* spinNodeRegisterCallback(), which requires a call to
* spinNodeUnregisterCallback() destroy.
*/
typedef void* spinNodeCallbackHandle;

// Functions signatures

/**
* Function signatures are used to create and trigger callbacks and
* events.
*/
typedef void(*spinNodeCallbackFunction)(spinNodeHandle hNode);

/*@}*/

/**
 * @defgroup SpinnakerGenApiEnumsC Spinnaker C GenICam Enumerations
 *
 * Enumeration definitions for Spinnaker C GenICam API
 */
/*@{*/

typedef enum _spinNodeType
{
	ValueNode,
	BaseNode,
	IntegerNode,
	BooleanNode,
	FloatNode,
	CommandNode,
	StringNode,
	RegisterNode,
	EnumerationNode,
	EnumEntryNode,
	CategoryNode,
	PortNode,
	UnknownNode = -1
} spinNodeType;

// signed or unsigned integers
typedef enum _spinSign
{
	Signed,
	Unsigned,
	_UndefinedSign
} spinSign;


// access mode of a node
typedef enum _spinAccessMode
{
	NI,
	NA,
	WO,
	RO,
	RW,
	_UndefinedAccesMode,
	_CycleDetectAccesMode

} spinAccessMode;

// recommended visibility of a node
typedef enum _spinVisibility
{
	Beginner = 0,
	Expert = 1,
	Guru = 2,
	Invisible = 3,
	_UndefinedVisibility  = 99
} spinVisibility;

// caching mode of a register
typedef enum _spinCachingMode
{
	NoCache,              // Do not use cache
	WriteThrough,         // Write to cache and register
	WriteAround,          // Write to register, write to cache on read
	_UndefinedCachingMode // Not yet initialized
} spinCachingMode;

//! recommended representation of a node value
typedef enum _spinRepresentation
{
	Linear,                     //!< Slider with linear behavior
	Logarithmic,                //!< Slider with logarithmic behaviour
	Boolean,                    //!< Check box
	PureNumber,                 //!< Decimal number in an edit control
	HexNumber,                  //!< Hex number in an edit control
	IPV4Address,                //!< IP-Address
	MACAddress,                 //!< MAC-Address
	_UndefinedRepresentation
} spinRepresentation;


//! Endianess of a value in a register
typedef enum _spinEndianess
{
	BigEndian,        //!< Register is big endian
	LittleEndian,     //!< Register is little endian
	_UndefinedEndian  //!< Object is not yet initialized
} spinEndianess;


//! Defines if a node name is standard or custom
typedef enum _spinNameSpace
{
	Custom,             //!< name resides in custom namespace
	Standard,           //!< name resides in one of the standard namespaces
	_UndefinedNameSpace //!< Object is not yet initialized
} spinNameSpace;


//! Defines from which standard namespace a node name comes from
typedef enum _spinStandardNameSpace
{
	None,            //!< name resides in custom namespace
	GEV,             //!< name resides in GigE Vision namespace
	IIDC,            //!< name resides in 1394 IIDC namespace
	CL,              //!< name resides in camera link namespace
	USB,             //!< name resides in USB namespace
	_UndefinedStandardNameSpace  //!< Object is not yet initialized
} spinStandardNameSpace;


//! Defines the chices of a Yes/No alternaitve
typedef enum _spinYesNo
{
	Yes = 1,                //!< yes
	No = 0,                 //!< no
	_UndefinedYesNo = 2     //!< Object is not yet initialized
} spinYesNo;

//! typedef for fomula type
typedef enum _spinSlope
{
	Increasing,      //!> strictly monotonous increasing
	Decreasing,      //!> strictly monotonous decreasing
	Varying,         //!> slope changes, e.g. at run-time
	Automatic,       //!> slope is determined automatically by probing the function
	_UndefinedESlope //!< Object is not yet initialized
} spinSlope;

//! typedef describing the different validity checks which can be performed on an XML file
/*! The enum values for a bitfield of lenght uint32_t */
typedef enum _spinXMLValidation
{
	xvLoad      = 0x00000001L,    //!> Creates a dummy node map
	xvCycles    = 0x00000002L,    //!> checks for write and dependency cycles (implies xvLoad)
	xvSFNC      = 0x00000004L,    //!> checks for conformance with the standard feature naming convention (SFNC)
	xvDefault   = 0x00000000L,    //!> checks performed if nothing else is said
	xvAll       = 0xffffffffL,    //!> all possible checks
	_UndefinedEXMLValidation = 0x8000000L //!< Object is not yet initialized
} spinXMLValidation;

//! typedef for float notation
typedef enum _spinDisplayNotation
{
	fnAutomatic, //!> the notation if either scientific or fixed depending on what is shorter
	fnFixed,    //!> the notation is fixed, e.g. 123.4
	fnScientific,    //!> the notation is scientific, e.g. 1.234e2
	_UndefinedEDisplayNotation  //!< Object is not yet initialized
} spinDisplayNotation;

//! typedef for interface type
typedef enum _spinInterfaceType
{
	intfIValue,       //!> IValue interface
	intfIBase,        //!> IBase interface
	intfIInteger,     //!> IInteger interface
	intfIBoolean,     //!> IBoolean interface
	intfICommand,     //!> ICommand interface
	intfIFloat,       //!> IFloat interface
	intfIString,      //!> IString interface
	intfIRegister,    //!> IRegister interface
	intfICategory,    //!> ICategory interface
	intfIEnumeration, //!> IEnumeration interface
	intfIEnumEntry,   //!> IEnumEntry interface
	intfIPort         //!> IPort interface
} spinInterfaceType;

//! typedef for link type
typedef enum _spinLinkType
{
	ctAllDependingNodes, //!> All nodes which will be invalidated if this node becomes invalid
	ctAllTerminalNodes,  //!> All terminal nodes which may be written to by this node
	ctInvalidators,      //!> List of references to nodes which may invalidate this node
	ctReadingChildren,   //!> All child nodes which influence this node's AccessMode
	ctWritingChildren,   //!> All child nodes which may be written to
	ctDependingChildren  //!> All child nodes which will cause this node to be invalidated
} spinLinkType;

//! typedef for increment mode
typedef enum _spinIncMode
{
	noIncrement, // !> The feature has no increment
	fixedIncrement, // !> The feature has a fix increment
	listIncrement, // !> The feature has a list of valid value
} spinIncMode;

//! typedef for link type
typedef enum _spinInputDirection
{
	idFrom, //!> Indicates a swiss knife that it is used as worker for a converter computing FROM
	idTo,   //!> Indicates a swiss knife that it is used as worker for a converter computing TO
	idNone  //!> SwissKnife is not used within a converter
} spinInputDirection;

/*@}*/

/*@}*/

#endif // FLIR_SPINNAKER_GENAPIDEFS_C_H
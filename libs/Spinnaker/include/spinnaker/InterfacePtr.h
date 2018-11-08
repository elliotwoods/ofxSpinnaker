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

#ifndef FLIR_SPINNAKER_INTERFACE_PTR_H
#define FLIR_SPINNAKER_INTERFACE_PTR_H

#include "SpinnakerPlatform.h"
#include "BasePtr.h"
#include "Interface.h"

namespace Spinnaker
{
    /**
    *  @defgroup SpinnakerClasses Spinnaker Classes
    */

    /*@{*/

    /**
    *  @defgroup InterfacePtr_h InterfacePtr Class
    */

    /*@{*/

    /**
    * @brief A reference tracked pointer to the interface object.
    */

    class SPINNAKER_API InterfacePtr : public BasePtr<IInterface>
    {
    public:
        /**
        * Default Constructor
        */
        InterfacePtr() throw()
            : BasePtr<IInterface>()
        {
        }

        /**
        * Default Constructor
        */
        InterfacePtr(const int /*nMustBeNull*/) throw()
            : BasePtr<IInterface>()
        {
        }

        /**
        * Virtual Destructor
        */
        virtual ~InterfacePtr(void){};

        /**
        * Copy Constructor
        */
        virtual InterfacePtr& operator=(const int nMustBeNull)
        {
            return dynamic_cast<InterfacePtr&>(BasePtr<IInterface>::operator=(nMustBeNull));
        };
        
    };

    /*@}*/

    /*@}*/
}

#endif //FLIR_SPINNAKER_INTERFACE_PTR_H
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

#ifndef SPINNAKER_GENAPI_IPORTCONSTRUCT_H
#define SPINNAKER_GENAPI_IPORTCONSTRUCT_H

#include "SpinnakerPlatform.h"
#include "Types.h"
#include "IPort.h"

#pragma warning ( push )
#pragma warning ( disable : 4251 ) // XXX needs to have dll-interface to be used by clients of class YYY

namespace Spinnaker
{
    namespace GenApi
    {
        /**
        *  @defgroup SpinnakerGenApiInterfaces Spinnaker GenApi Interfaces
        */
        /*@{*/

        /**
        *  @defgroup IPortConstruct_h IPortConstruct Interface
        */
        /*@{*/

        //*************************************************************
        // IPortContruct interface
        //*************************************************************

        /**
        * @brief Interface for ports
        */
        interface SPINNAKER_API IPortConstruct : virtual public IPort
        {
            /**
            * Sets pointer the real port implementation; this function may called only once
            */
            virtual void SetPortImpl(IPort* pPort) = 0;

            /**
            * Determines if the port adapter must perform an endianess swap
            */
            virtual EYesNo GetSwapEndianess() = 0;
        };

        /*@}*/
        /*@}*/
    }
}

#pragma warning ( pop )

#endif // ifndef SPINNAKER_GENAPI_IPORTCONSTRUCT_H

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

#ifndef SPINNAKER_GENAPI_ICATEGORY_H
#define SPINNAKER_GENAPI_ICATEGORY_H

#include "SpinnakerPlatform.h"
#include "Types.h"
#include "IValue.h"
#include "GCString.h"
#include "Container.h"

//#include <assert.h>

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
        *  @defgroup ICategory_h ICategory Interfaces
        */
        /*@{*/

        interface INode;

        //*************************************************************
        // ICategory interface
        //*************************************************************

        /**
        * @brief Gives access to a category node
        */
        interface SPINNAKER_API_ABSTRACT ICategory : virtual public IValue
        {
            /**
            * @brief Get all features of the category (including sub-categories)
            */
            virtual void GetFeatures(FeatureList_t &Features) const = 0;
        };

        /*@}*/
        /*@}*/
    }
}
#pragma warning ( pop )

#endif // ifndef SPINNAKER_GENAPI_ICATEGORY_H

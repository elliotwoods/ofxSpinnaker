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

#ifndef SPINNAKER_GENAPI_INTREGNODE_H
#define SPINNAKER_GENAPI_INTREGNODE_H

#include "SpinnakerPlatform.h"
#include "Types.h"
#include "Base.h"
#include "GCString.h"
#include "ISelector.h"
#include "INode.h"
#include "ValueNode.h"
#include "IntegerNode.h"
#include "RegisterNode.h"

#pragma warning ( push )
#pragma warning( disable : 4250 ) // C4250 - 'class1' : inherits 'class2::member' via dominance
#pragma warning ( disable : 4251 ) // XXX needs to have dll-interface to be used by clients of class YYY
#pragma warning( disable: 4275 ) // non dll-interface structXXX used as base

namespace Spinnaker
{
    namespace GenApi
    {
        /**
        *  @defgroup SpinnakerGenApiClasses Spinnaker GenApi Classes
        */
        /*@{*/

        /**
        *  @defgroup IntRegNode_h IntRegNode Class
        */
        /*@{*/

        /**
        * @brief Interface for string properties
        */
        class SPINNAKER_API IntRegNode : virtual public IntegerNode, virtual public RegisterNode
        {
        public:
            struct NodeImpl;
            IntRegNode();

            IntRegNode(std::shared_ptr<Node::NodeImpl> pInteger);

            virtual ~IntRegNode();

            /**
            * overload SetReference for Value
            */
            virtual void SetReference(INode* pBase);
        };

        /*@}*/
        /*@}*/
    }
}

#endif // SPINNAKER_GENAPI_INTREGNODE_H
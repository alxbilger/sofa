﻿/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#pragma once

#include <sofa/core/config.h>
#include <sofa/core/MatricesFactors.h>
#include <sofa/core/MultiVecId.h>
#include <sofa/core/objectmodel/BaseObject.h>

namespace sofa::core::behavior
{

/**
 * The vectors required to be able to compute the right-hand side
 */
struct SOFA_CORE_API RHSInput
{
    MultiVecDerivId intermediateVelocity;
    MultiVecCoordId intermediatePosition;
};

class SOFA_CORE_API BaseIntegrationMethod : public sofa::core::objectmodel::BaseObject
{
public:
    SOFA_CLASS(BaseIntegrationMethod, sofa::core::objectmodel::BaseObject);

    using Factors = std::tuple<MatricesFactors::M, MatricesFactors::B, MatricesFactors::K>;

    virtual void initializeVectors(const core::ExecParams* params, ConstMultiVecCoordId x, ConstMultiVecDerivId v) {}

    virtual Factors getMatricesFactors(SReal dt) const = 0;

    virtual void computeRightHandSide(const core::ExecParams* params,
        RHSInput input,
        MultiVecDerivId force,
        MultiVecDerivId rightHandSide,
        SReal dt) = 0;

    virtual void updateStates(const core::ExecParams* params, SReal dt,
        MultiVecCoordId x,
        MultiVecDerivId v,
        MultiVecCoordId newX,
        MultiVecDerivId newV,
        MultiVecDerivId linearSystemSolution) = 0;
};

}
/******************************************************************************
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

#include <sofa/component/constraint/lagrangian/solver/config.h>
#include <sofa/core/ConstraintParams.h>


namespace sofa::component::constraint::lagrangian::solver
{

class AssembleConstraints
{
public:

    virtual ~AssembleConstraints() = default;

    /// Calls the method resetConstraint on all the mechanical states and BaseConstraintSet
    /// In the case of a MechanicalObject, it clears the constraint jacobian matrix
    void resetConstraints(const core::ConstraintParams* cParams);

    /// Call the method buildConstraintMatrix on all the BaseConstraintSet
    void buildLocalConstraintMatrix(const core::ConstraintParams* cparams, unsigned int &constraintId);

    /// Calls the method applyJT on all the mappings to project the mapped
    /// constraint matrices on the main constraint matrix
    void accumulateMatrixDeriv(const core::ConstraintParams* cparams);

    void applyProjectiveConstraintOnConstraintMatrix(const core::ConstraintParams* cparams);

    virtual const core::objectmodel::BaseContext* getLagrangianConstraintsContext() const = 0;
    virtual core::objectmodel::BaseContext* getLagrangianConstraintsContext() = 0;
};

}

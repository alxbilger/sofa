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

#include <sofa/core/config.h>
#include <sofa/type/fwd.h>
#include <sofa/core/MatrixAccumulator.h>
#include <sofa/core/behavior/BaseForceField.h>
#include <sofa/core/BaseMatrixAccumulatorComponent.h>

namespace sofa::core::behavior
{

class SOFA_CORE_API StiffnessMatrixAccumulator : public virtual MatrixAccumulatorInterface {};
class SOFA_CORE_API ListStiffnessMatrixAccumulator : public ListMatrixAccumulator<StiffnessMatrixAccumulator>{};

} //namespace sofa::core::behavior

namespace sofa::core::matrixaccumulator
{
template<>
struct get_abstract_strong<behavior::BaseForceField>
{
    using type = behavior::StiffnessMatrixAccumulator;
    using ComponentType = behavior::BaseForceField;
};

using BaseForceFieldMatrixAccumulator = BaseMatrixAccumulatorComponent<behavior::BaseForceField>;

template<>
struct get_base_object_strong<behavior::BaseForceField>
{
    using type = BaseForceFieldMatrixAccumulator;
    using ComponentType = behavior::BaseForceField;
};

template<>
struct get_list_abstract_strong<behavior::BaseForceField>
{
    using type = behavior::ListStiffnessMatrixAccumulator;
    using ComponentType = behavior::BaseForceField;
};

} //namespace sofa::core::matrixaccumulator

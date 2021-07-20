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

#include <sofa/component/linearsystem/config.h>
#include <sofa/core/behavior/BaseLocalMassMatrix.h>
#include <sofa/core/behavior/BaseLocalForceFieldMatrix.h>

namespace sofa::component::linearsystem
{

/// Base class for local matrices
/// This type of local matrix stores the contributions of its associated force field or mass into an array.
/// Then, the global linear system visits all the local matrices and read the contributions stored
/// in the array.
/// Storing the contributions into an intermediate array (indirect) can be useful for debugging.
template<class TComponentType>
class BaseIndirectAssemblingLocalMatrix : public sofa::core::matrixaccumulator::get_base_object_strong_type<TComponentType>
{
public:
    SOFA_ABSTRACT_CLASS(BaseIndirectAssemblingLocalMatrix, sofa::core::matrixaccumulator::get_base_object_strong_type<TComponentType>)

    void setMatrixSize(const sofa::type::Vec2u& matrixSize);

    virtual void addContributionsToMatrix(sofa::linearalgebra::BaseMatrix* /*matrix*/, SReal /*factor*/, const sofa::type::Vec2u& /*positionInMatrix*/);

protected:

    BaseIndirectAssemblingLocalMatrix();
    ~BaseIndirectAssemblingLocalMatrix() override = default;

    /// Size of the local matrix. It corresponds to the number of degrees of freedom of the associated mechanical state
    Data< sofa::type::Vec2u > d_matrixSize;
    /// When the local matrix is added into the global matrix, it is added starting this location (row index and column index)
    std::map<sofa::core::behavior::BaseMechanicalState*, sofa::type::Vec2u> d_positionInGlobalMatrix;
};


} //namespace sofa::component::linearsystem

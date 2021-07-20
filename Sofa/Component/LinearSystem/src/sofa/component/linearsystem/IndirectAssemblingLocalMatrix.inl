/*********************************************************************************
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
#include <sofa/component/linearsystem/IndirectAssemblingLocalMatrix.h>

namespace sofa::component::linearsystem
{

template <class TMatrixAccumulator>
void BaseIndirectAssemblingLocalMatrix<TMatrixAccumulator>::setMatrixSize(const sofa::type::Vec2u& matrixSize)
{
    d_matrixSize.setValue(matrixSize);
}

template <class TMatrixAccumulator>
void BaseIndirectAssemblingLocalMatrix<TMatrixAccumulator>::addContributionsToMatrix(sofa::linearalgebra::BaseMatrix*,
    SReal, const sofa::type::Vec2u&)
{}

template <class TMatrixAccumulator>
BaseIndirectAssemblingLocalMatrix<TMatrixAccumulator>::BaseIndirectAssemblingLocalMatrix(): Inherit1()
    , d_matrixSize(initData(&d_matrixSize, "matrixSize", "Size of the local matrix"))
    // , d_positionInGlobalMatrix(initData(&d_positionInGlobalMatrix, "positionInGlobalMatrix", "Position of the local matrix in the global matrix"))
{
    d_matrixSize.setReadOnly(true);
    // d_positionInGlobalMatrix.setReadOnly(true);
}

} //namespace sofa::component::linearsystem

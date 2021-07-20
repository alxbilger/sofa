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

#include <sofa/linearalgebra/BaseMatrix.h>
#include <sofa/core/behavior/DefaultMultiMatrixAccessor.h>

namespace sofa::core::behavior
{
/// This class exists only for compatibility reasons. To be removed once the deprecated API
/// addKToMatrix and addMToMatrix is removed
class MatrixAccessorCompat : public sofa::core::behavior::DefaultMultiMatrixAccessor
{
public:
    MatrixRef getMatrix(const sofa::core::behavior::BaseMechanicalState* mstate) const override
    {
        MatrixRef r;
        r.matrix = this->globalMatrix;
        r.offset = 0;
        return r;
    }
};

/// A fake BaseMatrix redirecting its add methods to the MatrixAccumulator API
/// This class exists only for compatibility reasons. To be removed once the deprecated API
/// addKToMatrix and addMToMatrix is removed
template<typename TComponentType>
class AddToMatrixCompatMatrix : public sofa::linearalgebra::BaseMatrix
{
public:
    static constexpr const char* compatibilityMessage = "This message appears only for compatibility"
        " of the deprecated API addKToMatrix. Update your code with the new API buildStiffnessMatrix "
        " to remove this warning. ";

    ~AddToMatrixCompatMatrix() override = default;
    Index rowSize() const override
    {
        msg_error(component) << compatibilityMessage << "rowSize is not a supported operation in the compatibility";
        return {};
    }
    Index colSize() const override
    {
        msg_error(component) << compatibilityMessage << "colSize is not a supported operation in the compatibility";
        return {};
    }
    SReal element(Index i, Index j) const override
    {
        msg_error(component) << compatibilityMessage << "element is not a supported operation in the compatibility";
        return {};
    }
    void resize(Index nbRow, Index nbCol) override
    {
        msg_error(component) << compatibilityMessage << "resize is not a supported operation in the compatibility";
    }
    void clear() override
    {
        msg_error(component) << compatibilityMessage << "clear is not a supported operation in the compatibility";
    }
    void set(Index i, Index j, double v) override
    {
        msg_error(component) << compatibilityMessage << "set is not a supported operation in the compatibility";
    }
    void add(Index row, Index col, double v) override
    {
        matrices->add(row, col, v);
    }
    void add(Index row, Index col, const type::Mat3x3d& _M) override
    {
        matrices->add(row, col, _M);
    }
    void add(Index row, Index col, const type::Mat3x3f& _M) override
    {
        matrices->add(row, col, _M);
    }

    TComponentType* component { nullptr };
    typename sofa::core::matrixaccumulator::get_abstract_strong<TComponentType>::type* matrices;
};
}

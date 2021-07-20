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

#include <sofa/core/behavior/BlockType.h>
#include <sofa/component/linearsystem/IndirectAssemblingLocalMatrix.h>

namespace sofa::component::linearsystem
{

class BaseBlockTypeDeducerLocalMatrix
{
public:
    virtual ~BaseBlockTypeDeducerLocalMatrix() = default;
    bool hasInstantiated() const;

protected:
    bool m_hasInstanciated { false };
};


/// A local matrix that uses the type of the values provided by TMatrixAccumulator to instantiate another type of matrix
/// accumulator if not created yet.
/// This object is used only to instantiate. It is destroyed in at the end of a simulation time step.
///
/// The interest is to instantiate a matrix accumulator with a type appropriate to the type of values that a component
/// accumulates. For example, if a force field accumulates doubles (respectively Mat<3,3,double>), then a matrix
/// accumulator specialized (or optimized) with doubles (respectively Mat<3,3,double>) is instantiated.
///
/// @tparam TReassignedMatrixType Type of matrix that is instantiated by BlockTypeDeducerLocalMatrix. This is usually
/// a base class without template parameters and derived classes from TReassignedMatrixType can be specialized based on
/// a type of blocks.
template<class ComponentType, class TReassignedMatrixType >
class BlockTypeDeducerLocalMatrix :
    public BaseIndirectAssemblingLocalMatrix<ComponentType>,
    public BaseBlockTypeDeducerLocalMatrix
{
public:
    using MatrixAccumulator = sofa::core::matrixaccumulator::get_base_object_strong_type<ComponentType>;
    SOFA_CLASS(BlockTypeDeducerLocalMatrix, BaseIndirectAssemblingLocalMatrix<ComponentType>);
    using ReassignedMatrixType = TReassignedMatrixType;
    using BaseIndirectAssemblingLocalMatrix<ComponentType>::add;

    virtual void add(sofa::SignedIndex row, sofa::SignedIndex col, float value) override;
    virtual void add(sofa::SignedIndex row, sofa::SignedIndex col, double value) override;
    virtual void add(sofa::SignedIndex row, sofa::SignedIndex col, const sofa::type::Mat<3, 3, float>& value) override;
    virtual void add(sofa::SignedIndex row, sofa::SignedIndex col, const sofa::type::Mat<3, 3, double>& value) override;

    virtual void clear() override;

    void addContributionsToMatrix(sofa::linearalgebra::BaseMatrix* globalMatrix, SReal factor, const sofa::type::Vec2u& positionInMatrix) override;

    ~BlockTypeDeducerLocalMatrix() override = default;
    auto getCreatedLocalMatrix() const -> typename TReassignedMatrixType::SPtr;

protected:

    BlockTypeDeducerLocalMatrix();

    typename TReassignedMatrixType::SPtr m_createdLocalMatrix { nullptr };

    void reassignLocalMatrix(core::behavior::BlockType blocType);
    virtual typename TReassignedMatrixType::SPtr instantiateLocalMatrix(core::behavior::BlockType blocType) = 0;
};

}

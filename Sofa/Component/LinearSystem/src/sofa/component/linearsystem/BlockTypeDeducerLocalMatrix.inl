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
#include <sofa/component/linearsystem/BlockTypeDeducerLocalMatrix.h>

#include <sofa/simulation/AnimateEndEvent.h>

namespace sofa::component::linearsystem
{
template <class TMatrixAccumulator, class TReassignedMatrixType>
auto BlockTypeDeducerLocalMatrix<TMatrixAccumulator, TReassignedMatrixType>::
getCreatedLocalMatrix() const -> typename TReassignedMatrixType::SPtr
{
    return m_createdLocalMatrix;
}

template <class TMatrixAccumulator, class TReassignedMatrixType>
BlockTypeDeducerLocalMatrix<TMatrixAccumulator, TReassignedMatrixType>::BlockTypeDeducerLocalMatrix()
{}

template <class TMatrixAccumulator, class TReassignedMatrixType>
void BlockTypeDeducerLocalMatrix<TMatrixAccumulator, TReassignedMatrixType>::add(sofa::SignedIndex row, sofa::SignedIndex col, float value)
{
    reassignLocalMatrix(sofa::core::behavior::getScalarBlocType<float>());
    if (m_createdLocalMatrix)
    {
        m_createdLocalMatrix->add(row, col, value);
    }
}

template <class TMatrixAccumulator, class TReassignedMatrixType>
void BlockTypeDeducerLocalMatrix<TMatrixAccumulator, TReassignedMatrixType>::add(sofa::SignedIndex row, sofa::SignedIndex col, double value)
{
    reassignLocalMatrix(sofa::core::behavior::getScalarBlocType<double>());
    if (m_createdLocalMatrix)
    {
        m_createdLocalMatrix->add(row, col, value);
    }
}

template <class TMatrixAccumulator, class TReassignedMatrixType>
void BlockTypeDeducerLocalMatrix<TMatrixAccumulator, TReassignedMatrixType>::add(sofa::SignedIndex row, sofa::SignedIndex col, const sofa::type::Mat<3, 3, float>& value)
{
    reassignLocalMatrix(sofa::core::behavior::getMat33BlocType<float>());
    if (m_createdLocalMatrix)
    {
        m_createdLocalMatrix->add(row, col, value);
    }
}

template <class TMatrixAccumulator, class TReassignedMatrixType>
void BlockTypeDeducerLocalMatrix<TMatrixAccumulator, TReassignedMatrixType>::add(sofa::SignedIndex row, sofa::SignedIndex col, const sofa::type::Mat<3, 3, double>& value)
{
    reassignLocalMatrix(sofa::core::behavior::getMat33BlocType<double>());
    if (m_createdLocalMatrix)
    {
        m_createdLocalMatrix->add(row, col, value);
    }
}

template <class TMatrixAccumulator, class TReassignedMatrixType>
void BlockTypeDeducerLocalMatrix<TMatrixAccumulator, TReassignedMatrixType>::clear()
{
}

template <class TMatrixAccumulator, class TReassignedMatrixType>
void BlockTypeDeducerLocalMatrix<TMatrixAccumulator, TReassignedMatrixType>::addContributionsToMatrix(
    sofa::linearalgebra::BaseMatrix* global_matrix, SReal factor, const sofa::type::Vec2u& positionInMatrix)
{
    if (m_createdLocalMatrix)
    {
        m_createdLocalMatrix->addContributionsToMatrix(global_matrix, factor, positionInMatrix);
    }
}

template <class TMatrixAccumulator, class TReassignedMatrixType>
void BlockTypeDeducerLocalMatrix<TMatrixAccumulator, TReassignedMatrixType>::reassignLocalMatrix(core::behavior::BlockType blocType)
{
    if (!m_createdLocalMatrix)
    {
        m_createdLocalMatrix = instantiateLocalMatrix(blocType);

        if (m_createdLocalMatrix == nullptr)
        {
            msg_error() << "Could not instantiate a new local matrix for block type " << blocType;
            this->d_componentState.setValue(core::objectmodel::ComponentState::Invalid);
            return;
        }

        m_hasInstanciated = true;
        m_createdLocalMatrix->setName(this->getName());

        if (this->getMaster())
        {
            this->getMaster()->addSlave(m_createdLocalMatrix);
        }
        else
        {
            msg_error() << "This component must be added as a child of another component";
            this->d_componentState.setValue(core::objectmodel::ComponentState::Invalid);
            return;
        }
        m_createdLocalMatrix->f_printLog.setValue(this->f_printLog.getValue());
        m_createdLocalMatrix->setMatrixSize(this->d_matrixSize.getValue());
    }
}

inline bool BaseBlockTypeDeducerLocalMatrix::hasInstantiated() const
{
    return m_hasInstanciated;
}
} //namespace sofa::component::linearsystem

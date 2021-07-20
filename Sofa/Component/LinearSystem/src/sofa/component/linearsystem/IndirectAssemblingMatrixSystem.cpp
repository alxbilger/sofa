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
#include <sofa/component/linearsystem/IndirectAssemblingMatrixSystem.inl>

#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/Factory.inl>

#include <sofa/core/behavior/BaseLocalForceFieldMatrix.h>
#include <sofa/linearalgebra/CompressedRowSparseMatrix.h>
#include <sofa/linearalgebra/SparseMatrix.h>
#include <sofa/linearalgebra/DiagonalMatrix.h>
#include <sofa/linearalgebra/RotationMatrix.h>
#include <sofa/linearalgebra/FullMatrix.h>
#include <sofa/linearalgebra/BlockDiagonalMatrix.h>

namespace sofa::component::linearsystem
{

using sofa::linearalgebra::CompressedRowSparseMatrix;
using sofa::linearalgebra::SparseMatrix;
using sofa::linearalgebra::DiagonalMatrix;
using sofa::linearalgebra::BlockDiagonalMatrix;
using sofa::linearalgebra::RotationMatrix;
using sofa::linearalgebra::FullMatrix;
using sofa::linearalgebra::FullVector;

template<>
void IndirectAssemblingLocalMatrix<core::matrixaccumulator::Contribution::STIFFNESS, sofa::type::Mat3x3d>::addContributionsToMatrix(sofa::linearalgebra::BaseMatrix* globalMatrix, SReal factor, const sofa::type::Vec2u& positionInMatrix)
{
    if (globalMatrix)
    {
        if (auto* crsmat_d = dynamic_cast<sofa::linearalgebra::CompressedRowSparseMatrix<type::Mat<3,3,double> > * >(globalMatrix))
        {
            for (const auto& matrixEntry : sofa::helper::getReadAccessor(d_entries))
            {
                *crsmat_d->wbloc((matrixEntry.row + positionInMatrix[0]) / 3, (matrixEntry.col + positionInMatrix[1]) / 3, true) += factor * matrixEntry.value;
            }
        }
        else
        {
            for (const auto& matrixEntry : sofa::helper::getReadAccessor(d_entries))
            {
                globalMatrix->add(matrixEntry.row + positionInMatrix[0], matrixEntry.col + positionInMatrix[1], factor * matrixEntry.value);
            }
        }
    }
}

template<>
void IndirectAssemblingLocalMatrix<core::matrixaccumulator::Contribution::STIFFNESS, sofa::type::Mat3x3f>::addContributionsToMatrix(sofa::linearalgebra::BaseMatrix* globalMatrix, SReal factor, const sofa::type::Vec2u& positionInMatrix)
{
    if (globalMatrix)
    {
        if (auto* crsmat_f = dynamic_cast<sofa::linearalgebra::CompressedRowSparseMatrix<type::Mat<3,3,float> > * >(globalMatrix))
        {
            for (const auto& matrixEntry : sofa::helper::getReadAccessor(d_entries))
            {
                *crsmat_f->wbloc((matrixEntry.row + positionInMatrix[0]) / 3, (matrixEntry.col + positionInMatrix[1]) / 3, true) += matrixEntry.value * factor;
            }
        }
        else
        {
            for (const auto& [row, col, value] : sofa::helper::getReadAccessor(d_entries))
            {
                globalMatrix->add(row + positionInMatrix[0], col + positionInMatrix[1], value * factor);
            }
        }
    }
}

template class SOFA_COMPONENT_LINEARSYSTEM_API IndirectAssemblingLocalMatrix<core::matrixaccumulator::Contribution::STIFFNESS, float>;
template class SOFA_COMPONENT_LINEARSYSTEM_API IndirectAssemblingLocalMatrix<core::matrixaccumulator::Contribution::STIFFNESS, double>;
template class SOFA_COMPONENT_LINEARSYSTEM_API IndirectAssemblingLocalMatrix<core::matrixaccumulator::Contribution::STIFFNESS, sofa::type::Mat3x3f>;
template class SOFA_COMPONENT_LINEARSYSTEM_API IndirectAssemblingLocalMatrix<core::matrixaccumulator::Contribution::STIFFNESS, sofa::type::Mat3x3d>;

template class SOFA_COMPONENT_LINEARSYSTEM_API IndirectAssemblingLocalMatrix<core::matrixaccumulator::Contribution::STIFFNESS, float>;
template class SOFA_COMPONENT_LINEARSYSTEM_API IndirectAssemblingLocalMatrix<core::matrixaccumulator::Contribution::STIFFNESS, double>;
template class SOFA_COMPONENT_LINEARSYSTEM_API IndirectAssemblingLocalMatrix<core::matrixaccumulator::Contribution::STIFFNESS, sofa::type::Mat3x3f>;
template class SOFA_COMPONENT_LINEARSYSTEM_API IndirectAssemblingLocalMatrix<core::matrixaccumulator::Contribution::STIFFNESS, sofa::type::Mat3x3d>;

IndirectProjectiveConstraintLocalMatrix::IndirectProjectiveConstraintLocalMatrix()
: Inherit1()
, d_positionInGlobalMatrix(initData(&d_positionInGlobalMatrix,"positionInGlobalMatrix","Position of the local matrix in the global matrix"))
, d_discardedRowCol(initData(&d_discardedRowCol, "discardedRowCol", "Rows and columns discarded in the local matrix."))
{
    d_positionInGlobalMatrix.setReadOnly(true);
    d_discardedRowCol.setReadOnly(true);
}

void IndirectProjectiveConstraintLocalMatrix::setPositionInGlobalMatrix(const sofa::type::Vec2u& pos, sofa::core::behavior::BaseMechanicalState* topmostParent)
{
    d_positionInGlobalMatrix.setValue(pos);
}

template class SOFA_COMPONENT_LINEARSYSTEM_API IndirectAssemblingMatrixSystem< FullMatrix<double>, FullVector<double> >;
template class SOFA_COMPONENT_LINEARSYSTEM_API IndirectAssemblingMatrixSystem< FullMatrix<float>, FullVector<float> >;
template class SOFA_COMPONENT_LINEARSYSTEM_API IndirectAssemblingMatrixSystem< SparseMatrix<double>, FullVector<double> >;
template class SOFA_COMPONENT_LINEARSYSTEM_API IndirectAssemblingMatrixSystem< SparseMatrix<float>, FullVector<float> >;
template class SOFA_COMPONENT_LINEARSYSTEM_API IndirectAssemblingMatrixSystem< CompressedRowSparseMatrix<double>, FullVector<double> >;
template class SOFA_COMPONENT_LINEARSYSTEM_API IndirectAssemblingMatrixSystem< CompressedRowSparseMatrix<float>, FullVector<float> >;
template class SOFA_COMPONENT_LINEARSYSTEM_API IndirectAssemblingMatrixSystem< CompressedRowSparseMatrix<type::Mat<2,2,double> >, FullVector<double> >;
template class SOFA_COMPONENT_LINEARSYSTEM_API IndirectAssemblingMatrixSystem< CompressedRowSparseMatrix<type::Mat<2,2,float> >, FullVector<float> >;
template class SOFA_COMPONENT_LINEARSYSTEM_API IndirectAssemblingMatrixSystem< CompressedRowSparseMatrix<type::Mat<3,3,double> >, FullVector<double> >;
template class SOFA_COMPONENT_LINEARSYSTEM_API IndirectAssemblingMatrixSystem< CompressedRowSparseMatrix<type::Mat<3,3,float> >, FullVector<float> >;
template class SOFA_COMPONENT_LINEARSYSTEM_API IndirectAssemblingMatrixSystem< CompressedRowSparseMatrix<type::Mat<4,4,double> >, FullVector<double> >;
template class SOFA_COMPONENT_LINEARSYSTEM_API IndirectAssemblingMatrixSystem< CompressedRowSparseMatrix<type::Mat<4,4,float> >, FullVector<float> >;
template class SOFA_COMPONENT_LINEARSYSTEM_API IndirectAssemblingMatrixSystem< CompressedRowSparseMatrix<type::Mat<6,6,double> >, FullVector<double> >;
template class SOFA_COMPONENT_LINEARSYSTEM_API IndirectAssemblingMatrixSystem< CompressedRowSparseMatrix<type::Mat<6,6,float> >, FullVector<float> >;
template class SOFA_COMPONENT_LINEARSYSTEM_API IndirectAssemblingMatrixSystem< CompressedRowSparseMatrix<type::Mat<8,8,double> >, FullVector<double> >;
template class SOFA_COMPONENT_LINEARSYSTEM_API IndirectAssemblingMatrixSystem< CompressedRowSparseMatrix<type::Mat<8,8,float> >, FullVector<float> >;
template class SOFA_COMPONENT_LINEARSYSTEM_API IndirectAssemblingMatrixSystem< DiagonalMatrix<double>, FullVector<double> >;
template class SOFA_COMPONENT_LINEARSYSTEM_API IndirectAssemblingMatrixSystem< DiagonalMatrix<float>, FullVector<float> >;
template class SOFA_COMPONENT_LINEARSYSTEM_API IndirectAssemblingMatrixSystem< BlockDiagonalMatrix<3,double>, FullVector<double> >;
template class SOFA_COMPONENT_LINEARSYSTEM_API IndirectAssemblingMatrixSystem< RotationMatrix<double>, FullVector<double> >;
template class SOFA_COMPONENT_LINEARSYSTEM_API IndirectAssemblingMatrixSystem< RotationMatrix<float>, FullVector<float> >;

int IndirectAssemblingMatrixLinearSystemClass = core::RegisterObject("Linear system")
.add<IndirectAssemblingMatrixSystem< FullMatrix<double>, FullVector<double> > >()
.add<IndirectAssemblingMatrixSystem< FullMatrix<float>, FullVector<float> > >()
.add<IndirectAssemblingMatrixSystem< SparseMatrix<double>, FullVector<double> > >()
.add<IndirectAssemblingMatrixSystem< CompressedRowSparseMatrix<double>, FullVector<double> > >(true)
.add<IndirectAssemblingMatrixSystem< CompressedRowSparseMatrix<float>, FullVector<float> > >()
.add<IndirectAssemblingMatrixSystem< CompressedRowSparseMatrix<type::Mat<2,2,double> >, FullVector<double> > >()
.add<IndirectAssemblingMatrixSystem< CompressedRowSparseMatrix<type::Mat<2,2,float> >, FullVector<float> > >()
.add<IndirectAssemblingMatrixSystem< CompressedRowSparseMatrix<type::Mat<3,3,double> >, FullVector<double> > >()
.add<IndirectAssemblingMatrixSystem< CompressedRowSparseMatrix<type::Mat<3,3,float> >, FullVector<float> > >()
.add<IndirectAssemblingMatrixSystem< CompressedRowSparseMatrix<type::Mat<4,4,double> >, FullVector<double> > >()
.add<IndirectAssemblingMatrixSystem< CompressedRowSparseMatrix<type::Mat<4,4,float> >, FullVector<float> > >()
.add<IndirectAssemblingMatrixSystem< CompressedRowSparseMatrix<type::Mat<6,6,double> >, FullVector<double> > >()
.add<IndirectAssemblingMatrixSystem< CompressedRowSparseMatrix<type::Mat<6,6,float> >, FullVector<float> > >()
.add<IndirectAssemblingMatrixSystem< CompressedRowSparseMatrix<type::Mat<8,8,double> >, FullVector<double> > >()
.add<IndirectAssemblingMatrixSystem< CompressedRowSparseMatrix<type::Mat<8,8,float> >, FullVector<float> > >()
;

} //namespace sofa::component::linearsystem

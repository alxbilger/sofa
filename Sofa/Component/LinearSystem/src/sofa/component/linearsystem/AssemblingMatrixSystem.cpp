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
#include <sofa/component/linearsystem/AssemblingMatrixSystem.inl>

#include <sofa/core/ObjectFactory.h>

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

template class SOFA_COMPONENT_LINEARSYSTEM_API AssemblingMatrixSystem< FullMatrix<SReal>, FullVector<SReal> >;
template class SOFA_COMPONENT_LINEARSYSTEM_API AssemblingMatrixSystem< SparseMatrix<SReal>, FullVector<SReal> >;
template class SOFA_COMPONENT_LINEARSYSTEM_API AssemblingMatrixSystem< CompressedRowSparseMatrix<SReal>, FullVector<SReal> >;
template class SOFA_COMPONENT_LINEARSYSTEM_API AssemblingMatrixSystem< CompressedRowSparseMatrix<type::Mat<2,2,SReal> >, FullVector<SReal> >;
template class SOFA_COMPONENT_LINEARSYSTEM_API AssemblingMatrixSystem< CompressedRowSparseMatrix<type::Mat<3,3,SReal> >, FullVector<SReal> >;
template class SOFA_COMPONENT_LINEARSYSTEM_API AssemblingMatrixSystem< CompressedRowSparseMatrix<type::Mat<4,4,SReal> >, FullVector<SReal> >;
template class SOFA_COMPONENT_LINEARSYSTEM_API AssemblingMatrixSystem< CompressedRowSparseMatrix<type::Mat<6,6,SReal> >, FullVector<SReal> >;
template class SOFA_COMPONENT_LINEARSYSTEM_API AssemblingMatrixSystem< CompressedRowSparseMatrix<type::Mat<8,8,SReal> >, FullVector<SReal> >;
template class SOFA_COMPONENT_LINEARSYSTEM_API AssemblingMatrixSystem< DiagonalMatrix<SReal>, FullVector<SReal> >;
template class SOFA_COMPONENT_LINEARSYSTEM_API AssemblingMatrixSystem< BlockDiagonalMatrix<3,SReal>, FullVector<SReal> >;
template class SOFA_COMPONENT_LINEARSYSTEM_API AssemblingMatrixSystem< RotationMatrix<SReal>, FullVector<SReal> >;

int AssemblingMatrixLinearSystemClass = core::RegisterObject("Linear system")
        .add<AssemblingMatrixSystem< FullMatrix<SReal>, FullVector<SReal> > >()
        .add<AssemblingMatrixSystem< SparseMatrix<SReal>, FullVector<SReal> > >()
        .add<AssemblingMatrixSystem< CompressedRowSparseMatrix<SReal>, FullVector<SReal> > >(true)
        .add<AssemblingMatrixSystem< CompressedRowSparseMatrix<type::Mat<2,2,SReal> >, FullVector<SReal> > >()
        .add<AssemblingMatrixSystem< CompressedRowSparseMatrix<type::Mat<3,3,SReal> >, FullVector<SReal> > >()
        .add<AssemblingMatrixSystem< CompressedRowSparseMatrix<type::Mat<4,4,SReal> >, FullVector<SReal> > >()
        .add<AssemblingMatrixSystem< CompressedRowSparseMatrix<type::Mat<6,6,SReal> >, FullVector<SReal> > >()
        .add<AssemblingMatrixSystem< CompressedRowSparseMatrix<type::Mat<8,8,SReal> >, FullVector<SReal> > >()
        ;

} //namespace sofa::component::linearsystem

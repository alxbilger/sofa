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
#include <SofaMatrix/LocalMassMatrixExporter.h>
#include <sofa/core/ObjectFactory.h>

#include <sofa/linearalgebra/FullMatrix.h>
#include <sofa/linearalgebra/CompressedRowSparseMatrix.h>
#include <sofa/linearalgebra/SparseMatrix.h>
#include <sofa/linearalgebra/DiagonalMatrix.h>
// #include <sofa/linearalgebra/RotationMatrix.h>
#include <sofa/linearalgebra/BlockDiagonalMatrix.h>

namespace sofa::component::linearsystem
{

using namespace sofa::linearalgebra;

template class SOFA_SOFAMATRIX_API LocalMassMatrixExporter< FullMatrix<SReal> >;
template class SOFA_SOFAMATRIX_API LocalMassMatrixExporter< SparseMatrix<SReal> >;
template class SOFA_SOFAMATRIX_API LocalMassMatrixExporter< CompressedRowSparseMatrix<SReal> >;
template class SOFA_SOFAMATRIX_API LocalMassMatrixExporter< CompressedRowSparseMatrix<type::Mat<2,2,SReal> > >;
template class SOFA_SOFAMATRIX_API LocalMassMatrixExporter< CompressedRowSparseMatrix<type::Mat<3,3,SReal> > >;
template class SOFA_SOFAMATRIX_API LocalMassMatrixExporter< CompressedRowSparseMatrix<type::Mat<4,4,SReal> > >;
template class SOFA_SOFAMATRIX_API LocalMassMatrixExporter< CompressedRowSparseMatrix<type::Mat<6,6,SReal> > >;
template class SOFA_SOFAMATRIX_API LocalMassMatrixExporter< CompressedRowSparseMatrix<type::Mat<8,8,SReal> > >;
template class SOFA_SOFAMATRIX_API LocalMassMatrixExporter< DiagonalMatrix<SReal> >;
template class SOFA_SOFAMATRIX_API LocalMassMatrixExporter< BlockDiagonalMatrix<3,SReal> >;
//template class SOFA_SOFAMATRIX_API LocalMassMatrixExporter< RotationMatrix<SReal> >;

int LocalMassMatrixExporterClass = core::RegisterObject("Local mass matrix exporter")
    .add<LocalMassMatrixExporter< FullMatrix<SReal> > >()
    .add<LocalMassMatrixExporter< SparseMatrix<SReal> > >()
    .add<LocalMassMatrixExporter< CompressedRowSparseMatrix<SReal> > >()
    .add<LocalMassMatrixExporter< CompressedRowSparseMatrix<type::Mat<2,2,SReal> > > >()
    .add<LocalMassMatrixExporter< CompressedRowSparseMatrix<type::Mat<3,3,SReal> > > >()
    .add<LocalMassMatrixExporter< CompressedRowSparseMatrix<type::Mat<4,4,SReal> > > >()
    .add<LocalMassMatrixExporter< CompressedRowSparseMatrix<type::Mat<6,6,SReal> > > >()
    .add<LocalMassMatrixExporter< CompressedRowSparseMatrix<type::Mat<8,8,SReal> > > >()
    ;

} //namespace sofa::component::linearsystem

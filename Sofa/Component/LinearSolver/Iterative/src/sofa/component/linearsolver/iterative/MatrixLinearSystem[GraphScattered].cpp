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
#define SOFA_COMPONENT_LINEARSOLVER_MATRIXLINEARSYSTEM_GRAPHSCATTERED_CPP
#include <sofa/component/linearsolver/iterative/MatrixLinearSystem[GraphScattered].h>
#include <sofa/component/linearsystem/MatrixLinearSystem.inl>

namespace sofa::component::linearsystem
{

using sofa::component::linearsolver::GraphScatteredMatrix;
using sofa::component::linearsolver::GraphScatteredVector;

template<> SOFA_COMPONENT_LINEARSOLVER_ITERATIVE_API
void MatrixLinearSystem<GraphScatteredMatrix, GraphScatteredVector>::setRHS(core::MultiVecDerivId v)
{
    if (m_linearSystem.getRHS())
    {
        m_linearSystem.getRHS()->set(v);
    }
}

template<> SOFA_COMPONENT_LINEARSOLVER_ITERATIVE_API
void MatrixLinearSystem<GraphScatteredMatrix, GraphScatteredVector>::setSystemSolution(core::MultiVecDerivId v)
{
    if (m_linearSystem.getSolution())
    {
        m_linearSystem.getSolution()->set(v);
    }
}

template<> SOFA_COMPONENT_LINEARSOLVER_ITERATIVE_API
void MatrixLinearSystem<GraphScatteredMatrix, GraphScatteredVector>::copyLocalVectorToGlobalVector(core::MultiVecDerivId v, GraphScatteredVector* globalVector)
{
    if (globalVector)
    {
        globalVector->set(v);
    }
}

template<> SOFA_COMPONENT_LINEARSOLVER_ITERATIVE_API
void MatrixLinearSystem<GraphScatteredMatrix, GraphScatteredVector>::dispatchSystemSolution(core::MultiVecDerivId v)
{}

template<> SOFA_COMPONENT_LINEARSOLVER_ITERATIVE_API
void MatrixLinearSystem<GraphScatteredMatrix, GraphScatteredVector>::dispatchSystemRHS(core::MultiVecDerivId v)
{}

template class SOFA_COMPONENT_LINEARSOLVER_ITERATIVE_API MatrixLinearSystem< GraphScatteredMatrix, GraphScatteredVector >;

} //namespace sofa::component::linearsystem

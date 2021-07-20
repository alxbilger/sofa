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
#include <sofa/component/linearsystem/MatrixLinearSystem.h>
#include <sofa/core/ObjectFactory.h>

#include <sofa/core/behavior/BaseLocalMassMatrix.h>
#include <sofa/core/behavior/LinearSolver.h>

#include <sofa/component/linearsystem/IndirectAssemblingLocalMatrix.h>

#include <sofa/component/linearsystem/MappingGraph.h>

#include <sofa/core/behavior/BaseProjectiveConstraintSet.h>

namespace sofa::component::linearsystem
{

class IndirectProjectiveConstraintLocalMatrix;

/// This component assembles and stores the linear matrix system.
/// It adds the contributions of all the elements of the Node where the component is, as well as all the subnodes.
/// All the contributions are added in a global system matrix. This matrix is then solved using the linear solver.
///
/// This assembling component works in two steps:
/// 1) Get the contributions of all the elements, and store them in an array of triplets (row, col, value)
/// 2) Gather all the arrays into the matrix
///
/// The first step of getting the contributions of all the elements is performed using intermediate components.
/// IndirectAssemblingMatrixSystem creates and associates a local matrix component to each element (force field, mass,
/// projective constraint). The component acting as a local matrix is defined as a child of the element. When the
/// element is asked to add its contributions, it traverses all its children and calls a virtual function. In the case
/// of the local matrix, this function adds the triplet (row, column, value) into an array stored in the local matrix
/// component.
template<class TMatrix, class TVector>
class IndirectAssemblingMatrixSystem : public MatrixLinearSystem<TMatrix, TVector>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE2(IndirectAssemblingMatrixSystem, TMatrix, TVector), SOFA_TEMPLATE2(MatrixLinearSystem, TMatrix, TVector));

    void assembleSystem(const core::MechanicalParams* mparams) override;

    void associateLocalMatrixToComponents(const core::MechanicalParams* mparams) override;

private:

    using Inherit1::m_mappingGraph;

    /**
     * Generic function to create a local matrix and associate it to a component
     */
    template<class ComponentType>
    BaseIndirectAssemblingLocalMatrix<ComponentType>* createLocalMatrix(ComponentType* object) const;

    /// Associate a local matrix to the provided component.
    template<class ComponentType>
    void associateLocalMatrixTo(ComponentType* component);

    /// Associate a local matrix to the provided projective constraint
    void associateLocalMatrixTo(sofa::core::behavior::BaseProjectiveConstraintSet* c);

    template<class ComponentType>
    void assembleComponent(ComponentType* component, const sofa::core::MechanicalParams *mparams);

    void assembleComponent(sofa::core::behavior::BaseProjectiveConstraintSet* c);

    struct Dirichlet : public sofa::core::behavior::ZeroDirichletCondition
    {
        IndirectProjectiveConstraintLocalMatrix* m_localProjectiveConstraintMatrix { nullptr };

        void discardRowCol(sofa::Index row, sofa::Index col) override;
    } m_discarder;

    void applyZeroDirichletCondition();

};

/// Data structure stored in local matrices
/// It is a triplet representing a location in a matrix (row index and column index) and the value at this location
template<class TBlockType>
struct MatrixEntry
{
    sofa::SignedIndex row{}, col{};
    TBlockType value;
};

/// Operator required for serialization
template<class TBlockType>
std::istream& operator>>( std::istream& in, MatrixEntry<TBlockType>& s);
/// Operator required for serialization
template<class TBlockType>
std::ostream& operator<<( std::ostream& out, const MatrixEntry<TBlockType>& s);

template<class ComponentType>
class BaseIndirectAssemblingLocalMatrix_IndirectAssemblingMatrixSystem : public BaseIndirectAssemblingLocalMatrix<ComponentType>
{
public:
    SOFA_ABSTRACT_CLASS(BaseIndirectAssemblingLocalMatrix_IndirectAssemblingMatrixSystem, BaseIndirectAssemblingLocalMatrix<ComponentType>);
};

using core::matrixaccumulator::no_check_policy;

/// A local matrix storing the contributions of its associated force field or mass into a Data
template<class ComponentType, class BlocType, class TStrategy = sofa::core::matrixaccumulator::NoIndexVerification>
class IndirectAssemblingLocalMatrix:
public virtual sofa::core::MatrixAccumulatorIndexChecker< BaseIndirectAssemblingLocalMatrix_IndirectAssemblingMatrixSystem<ComponentType>, TStrategy>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE3(IndirectAssemblingLocalMatrix, ComponentType, BlocType, TStrategy),
        SOFA_TEMPLATE2(sofa::core::MatrixAccumulatorIndexChecker, BaseIndirectAssemblingLocalMatrix_IndirectAssemblingMatrixSystem<ComponentType>, TStrategy));
    using Inherit1::add;

    void clear() override;

    void addContributionsToMatrix(sofa::linearalgebra::BaseMatrix* globalMatrix, SReal factor, const sofa::type::Vec2u& positionInMatrix) override;

protected:

    /// Add a matrix entry to the Data only if BlocType is of type float
    void add(const no_check_policy&, sofa::SignedIndex /*row*/, sofa::SignedIndex /*col*/, float /*value*/) override;
    /// Add a matrix entry to the Data only if BlocType is of type double
    void add(const no_check_policy&, sofa::SignedIndex /*row*/, sofa::SignedIndex /*col*/, double /*value*/) override;
    /// Add a matrix entry to the Data only if BlocType is of type Mat<3, 3, float>
    void add(const no_check_policy&, sofa::SignedIndex /*row*/, sofa::SignedIndex /*col*/, const sofa::type::Mat<3, 3, float>& /*value*/) override;
    /// Add a matrix entry to the Data only if BlocType is of type Mat<3, 3, double>
    void add(const no_check_policy&, sofa::SignedIndex /*row*/, sofa::SignedIndex /*col*/, const sofa::type::Mat<3, 3, double>& /*value*/) override;

    /// Contributions to be added to the global system matrix. The indices are expressed in the local matrix.
    Data<sofa::type::vector<MatrixEntry<BlocType> > > d_entries;

    IndirectAssemblingLocalMatrix();
    ~IndirectAssemblingLocalMatrix() override = default;
};

class SOFA_COMPONENT_LINEARSYSTEM_API IndirectProjectiveConstraintLocalMatrix : public sofa::core::objectmodel::BaseObject
{
public:
    SOFA_CLASS(IndirectProjectiveConstraintLocalMatrix, sofa::core::objectmodel::BaseObject);

    void setPositionInGlobalMatrix(const sofa::type::Vec2u&, sofa::core::behavior::BaseMechanicalState* topmostParent);

    Data< sofa::type::Vec2u > d_positionInGlobalMatrix;
    Data< sofa::type::vector< sofa::type::Vec2u > > d_discardedRowCol;

protected:
    IndirectProjectiveConstraintLocalMatrix();

};

} //namespace sofa::component::linearsystem

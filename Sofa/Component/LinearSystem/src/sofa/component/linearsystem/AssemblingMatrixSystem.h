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
#include <sofa/core/behavior/BaseLocalForceFieldMatrix.h>
#include <sofa/core/behavior/BaseLocalMassMatrix.h>
#include <sofa/core/BaseLocalMappingMatrix.h>
#include <sofa/core/behavior/LinearSolver.h>
#include <sofa/linearalgebra/CompressedRowSparseMatrix.h>
#include <Eigen/Sparse>
#include <sofa/component/linearsystem/MappingGraph.h>
#include <sofa/core/behavior/BaseProjectiveConstraintSet.h>
#include <sofa/component/linearsystem/matrixaccumulators/AssemblingMappedMatrixAccumulator.h>

namespace sofa::component::linearsystem
{

using sofa::core::behavior::BaseForceField;
using sofa::core::behavior::BaseMass;
using sofa::core::BaseMapping;

template<class TComponentType, class Real>
struct LocalMatrixMaps
{
    using ListMatrixType = sofa::core::matrixaccumulator::get_list_abstract_strong_type<TComponentType>;

    /// List of local matrices that components will use to add their contributions
    std::map< TComponentType*, ListMatrixType > accumulators;
    /// The local matrix (value) that has been created and associated to a non-mapped component (key)
    std::map< TComponentType*, BaseAssemblingMatrixAccumulator<TComponentType>* > localMatrix;
    /// The local matrix (value) that has been created and associated to a mapped component (key)
    std::map< TComponentType*, AssemblingMappedMatrixAccumulator<TComponentType, Real>* > mappedLocalMatrix;
    /// A verification strategy allowing to verify that the matrix indices provided are valid
    std::map< TComponentType*, std::shared_ptr<core::matrixaccumulator::RangeVerification> > indexVerificationStrategy;
};

template<class Real>
struct LocalMatrixMaps<BaseMapping, Real>
{
    using ListMatrixType = sofa::core::matrixaccumulator::get_list_abstract_strong_type<BaseMapping>;
    /// List of local matrices that components will use to add their contributions
    std::map< BaseMapping*, ListMatrixType > accumulators;
    /// The local matrix (value) that has been created and associated to a non-mapped component (key)
    std::map< BaseMapping*, BaseAssemblingMatrixAccumulator<BaseMapping>* > localMatrix;
};

/**
 * Assemble the global matrix using local matrix components
 *
 * Components add their contributions directly to the global matrix, through their local matrices.
 * Local matrices act as a proxy (they don't really store a local matrix). They have a link to the global matrix and
 * an offset parameter to add the contribution into the right entry into the global matrix.
 *
 * @tparam TMatrix The type of the data structure used to represent the global matrix. In the general cases, this type
 * derives from sofa::linearalgebra::BaseMatrix.
 * @tparam TVector The type of the data structure used to represent the vectors of the linear system: the right-hand
 * side and the solution. In the general cases, this type derives from sofa::linearalgebra::BaseVector.
 */
template<class TMatrix, class TVector>
class AssemblingMatrixSystem : public MatrixLinearSystem<TMatrix, TVector>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE2(AssemblingMatrixSystem, TMatrix, TVector), SOFA_TEMPLATE2(MatrixLinearSystem, TMatrix, TVector));

    using Real = typename TMatrix::Real;

    void assembleSystem(const core::MechanicalParams* mparams) override;
    void makeLocalMatrixGroups(const core::MechanicalParams* mparams);
    void associateLocalMatrixToComponents(const core::MechanicalParams* mparams) override;

    const MappingGraph& getMappingGraph() const;

protected:

    Data< bool > d_assembleStiffness; ///< If true, the stiffness is added to the global matrix
    Data< bool > d_assembleMass; ///< If true, the mass is added to the global matrix
    Data< bool > d_assembleMappings; ///< If true, the geometric stiffness of mappings is added to the global matrix
    Data< bool > d_applyProjectiveConstraints; ///< If true, projective constraints are applied on the global matrix
    Data< bool > d_applyMappedComponents; ///< If true, mapped components contribute to the global matrix
    Data< bool > d_checkIndices; ///< If true, indices are verified before being added in to the global matrix, favoring security over speed

    AssemblingMatrixSystem();

    using Inherit1::m_mappingGraph;

    LocalMatrixMaps<BaseForceField, Real> stiffness;
    LocalMatrixMaps<BaseMass, Real> mass;
    LocalMatrixMaps<BaseMapping, Real> mapping;

    /// List of shared local matrices under mappings
    sofa::type::vector< std::pair<
        core::behavior::BaseMechanicalState*,
        std::shared_ptr<LocalMappedMatrixType<Real> >
    > > m_localMappedMatrices;


    /// Associate a local matrix to the provided component. The type of the local matrix depends on the situtation of
    /// the component: type of the component, mapped vs non-mapped
    template<class ComponentType>
    void associateLocalMatrixTo(ComponentType* component, const core::MechanicalParams* mparams, LocalMatrixMaps<ComponentType, Real>& matrixMaps);

    /// Associate a local matrix to the provided mapping
    void associateLocalMatrixTo(BaseMapping* mapping, const core::MechanicalParams* mparams);

    /**
     * Generic function to create a local matrix and associate it to a component
     */
    template<class TLocalMatrix>
    TLocalMatrix* createLocalMatrixT(typename TLocalMatrix::ComponentType* object, SReal factor) const;

    virtual BaseAssemblingMatrixAccumulator<BaseForceField>* createLocalMatrix(BaseForceField* object, SReal factor) const;
    virtual BaseAssemblingMatrixAccumulator<BaseMass>* createLocalMatrix(BaseMass* object, SReal factor) const;
    virtual BaseAssemblingMatrixAccumulator<BaseMapping>* createLocalMatrix(BaseMapping* object, SReal factor) const;

    virtual AssemblingMappedMatrixAccumulator<BaseForceField, Real>* createLocalMappedMatrix(BaseForceField* object, SReal factor) const;
    virtual AssemblingMappedMatrixAccumulator<BaseMass, Real>* createLocalMappedMatrix(BaseMass* object, SReal factor) const;

    virtual void projectMappedMatrices();

    /**
     * Assemble the matrices under mappings into the global matrix
     */
    virtual void assembleMappedMatrices(const core::MechanicalParams* mparams);

    virtual void applyProjectiveConstraints(const core::MechanicalParams* mparams);

    /**
     * Define how zero Dirichlet boundary conditions are applied on the global matrix
     */
    struct Dirichlet final : public sofa::core::behavior::ZeroDirichletCondition
    {
        ~Dirichlet() override = default;
        void discardRowCol(sofa::Index row, sofa::Index col) override;

        sofa::type::Vec2u m_offset;

        /// The matrix to apply a zero Dirichlet boundary condition
        TMatrix* m_globalMatrix { nullptr };
    } m_discarder;
};


} //namespace sofa::component::linearsystem

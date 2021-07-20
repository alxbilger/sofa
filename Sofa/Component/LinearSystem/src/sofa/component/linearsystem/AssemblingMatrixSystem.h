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
#include <optional>

namespace sofa::component::linearsystem
{

using sofa::core::behavior::BaseForceField;
using sofa::core::behavior::BaseMass;
using sofa::core::BaseMapping;
using sofa::core::matrixaccumulator::Contribution;

/**
 * Data structure storing local matrix components created during the matrix assembly and associated
 * to each component contributing to the global matrix
 */
template<Contribution c, class Real>
struct LocalMatrixMaps
{
    using ListMatrixType = sofa::core::matrixaccumulator::get_list_abstract_strong_type<c>;
    using ComponentType = sofa::core::matrixaccumulator::get_component_type<c>;
    using PairMechanicalStates = sofa::type::fixed_array<core::behavior::BaseMechanicalState*, 2>;

    /// List of local matrices that components will use to add their contributions
    std::map< ComponentType*, ListMatrixType > accumulators;
    /// The local matrix (value) that has been created and associated to a non-mapped component (key)
    std::map< ComponentType*, BaseAssemblingMatrixAccumulator<c>* > localMatrix;
    /// The local matrix (value) that has been created and associated to a mapped component (key)
    std::map< ComponentType*, std::map<PairMechanicalStates, AssemblingMappedMatrixAccumulator<c, Real>*> > mappedLocalMatrix;
    /// A verification strategy allowing to verify that the matrix indices provided are valid
    std::map< ComponentType*, std::shared_ptr<core::matrixaccumulator::RangeVerification> > indexVerificationStrategy;


    std::map< ComponentType*, std::map<PairMechanicalStates, BaseAssemblingMatrixAccumulator<c>* > > componentLocalMatrix;

    void clear()
    {
        for (const auto [component, matrix] : localMatrix)
        {
            if (component)
            {
                component->removeSlave(matrix);
            }
        }

        for (const auto& [component, matrixMap] : mappedLocalMatrix)
        {
            for (const auto& [pair, matrix] : matrixMap)
            {
                component->removeSlave(matrix);
                matrix->reset();
            }
        }

        accumulators.clear();
        localMatrix.clear();
        mappedLocalMatrix.clear();
        indexVerificationStrategy.clear();
        componentLocalMatrix.clear();
    }
};

template<class Real>
struct LocalMatrixMaps<Contribution::GEOMETRIC_STIFFNESS, Real>
{
    using ListMatrixType = sofa::core::matrixaccumulator::get_list_abstract_strong_type<Contribution::GEOMETRIC_STIFFNESS>;
    /// List of local matrices that components will use to add their contributions
    std::map< BaseMapping*, ListMatrixType > accumulators;
    /// The local matrix (value) that has been created and associated to a non-mapped component (key)
    std::map< BaseMapping*, BaseAssemblingMatrixAccumulator<core::matrixaccumulator::Contribution::GEOMETRIC_STIFFNESS>* > localMatrix;

    void clear()
    {
        for (const auto& [component, matrix] : localMatrix)
        {
            component->removeSlave(matrix);
            matrix->reset();
        }

        accumulators.clear();
        localMatrix.clear();
    }
};

struct GroupOfComponentsAssociatedToAPairOfMechanicalStates;

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
    using Contribution = core::matrixaccumulator::Contribution;
    using PairMechanicalStates = sofa::type::fixed_array<core::behavior::BaseMechanicalState*, 2>;

    [[nodiscard]] const MappingGraph& getMappingGraph() const;

    Data< bool > d_assembleStiffness; ///< If true, the stiffness is added to the global matrix
    Data< bool > d_assembleMass; ///< If true, the mass is added to the global matrix
    Data< bool > d_assembleDamping; ///< If true, the damping is added to the global matrix
    Data< bool > d_assembleGeometricStiffness; ///< If true, the geometric stiffness of mappings is added to the global matrix
    Data< bool > d_applyProjectiveConstraints; ///< If true, projective constraints are applied on the global matrix
    Data< bool > d_applyMappedComponents; ///< If true, mapped components contribute to the global matrix
    Data< bool > d_checkIndices; ///< If true, indices are verified before being added in to the global matrix, favoring security over speed

protected:

    AssemblingMatrixSystem();

    using Inherit1::m_mappingGraph;

    template<Contribution c>
    void contribute(const core::MechanicalParams* mparams);

    void assembleSystem(const core::MechanicalParams* mparams) override;

    void makeLocalMatrixGroups(const core::MechanicalParams* mparams);
    void associateLocalMatrixToComponents(const core::MechanicalParams* mparams) override;

    void cleanLocalMatrices();

    std::tuple<
        LocalMatrixMaps<Contribution::STIFFNESS          , Real>,
        LocalMatrixMaps<Contribution::MASS               , Real>,
        LocalMatrixMaps<Contribution::DAMPING            , Real>,
        LocalMatrixMaps<Contribution::GEOMETRIC_STIFFNESS, Real>
    > m_localMatrixMaps;


    std::map<BaseForceField*, core::behavior::StiffnessMatrix> m_stiffness;
    std::map<BaseForceField*, core::behavior::DampingMatrix> m_damping;
    std::map<BaseMass*,       std::shared_ptr<core::behavior::MassMatrix> > m_mass;

    /**
     * Return the element of the tuple corresponding to @c
     * Example: getLocalMatrixMap<Contribution::STIFFNESS>()
     */
    template<Contribution c>
    LocalMatrixMaps<c, Real>& getLocalMatrixMap();

    /**
     * Return the element of the tuple corresponding to @c
     * Example: getLocalMatrixMap<Contribution::STIFFNESS>()
     */
    template<Contribution c>
    const LocalMatrixMaps<c, Real>& getLocalMatrixMap() const;

    /// List of shared local matrices under mappings
    sofa::type::vector< std::pair<
        PairMechanicalStates,
        std::shared_ptr<LocalMappedMatrixType<Real> >
    > > m_localMappedMatrices;


    /// Associate a local matrix to the provided component. The type of the local matrix depends on
    /// the situtation of the component: type of the component, mapped vs non-mapped
    template<Contribution c>
    void associateLocalMatrixTo(sofa::core::matrixaccumulator::get_component_type<c>* component,
                                const core::MechanicalParams* mparams);

    /// Associate a local matrix to the provided mapping
    void associateLocalMatrixTo(BaseMapping* mapping, const core::MechanicalParams* mparams);

    /**
     * Generic function to create a local matrix and associate it to a component
     */
    template<class TLocalMatrix>
    TLocalMatrix* createLocalMatrixComponent(typename TLocalMatrix::ComponentType* object, SReal factor) const;


    template<Contribution c>
    BaseAssemblingMatrixAccumulator<c>* createLocalMatrixT(
        sofa::core::matrixaccumulator::get_component_type<c>* object,
        SReal factor);

    template<Contribution c>
    AssemblingMappedMatrixAccumulator<c, Real>* createLocalMappedMatrixT(sofa::core::matrixaccumulator::get_component_type<c>* object, SReal factor);


    virtual BaseAssemblingMatrixAccumulator<Contribution::STIFFNESS>*           createLocalStiffnessMatrix         (BaseForceField* object, SReal factor) const;
    virtual BaseAssemblingMatrixAccumulator<Contribution::MASS>*                createLocalMassMatrix              (BaseMass* object      , SReal factor) const;
    virtual BaseAssemblingMatrixAccumulator<Contribution::DAMPING>*             createLocalDampingMatrix           (BaseForceField* object, SReal factor) const;
    virtual BaseAssemblingMatrixAccumulator<Contribution::GEOMETRIC_STIFFNESS>* createLocalGeometricStiffnessMatrix(BaseMapping* object   , SReal factor) const;

    template<Contribution c>
    BaseAssemblingMatrixAccumulator<c>* createLocalMatrixImpl(sofa::core::matrixaccumulator::get_component_type<c>* object, SReal factor, const LocalMatrixMaps<c, Real>& matrixMaps) const;

    virtual AssemblingMappedMatrixAccumulator<Contribution::STIFFNESS, Real>* createLocalMappedStiffnessMatrix(BaseForceField* object, SReal factor) const;
    virtual AssemblingMappedMatrixAccumulator<Contribution::MASS, Real>* createLocalMappedMassMatrix(BaseMass* object, SReal factor) const;
    virtual AssemblingMappedMatrixAccumulator<Contribution::DAMPING, Real>* createLocalMappedDampingMatrix(BaseForceField* object, SReal factor) const;

    virtual void projectMappedMatrices(const core::MechanicalParams* mparams);

    using JacobianMatrixType = LocalMappedMatrixType<Real>;
    std::map< core::behavior::BaseMechanicalState*, std::shared_ptr<JacobianMatrixType > > computeJacobianFrom(BaseMechanicalState* mstate, const core::MechanicalParams* mparams);

    /**
     * Assemble the matrices under mappings into the global matrix
     */
    virtual void assembleMappedMatrices(const core::MechanicalParams* mparams);

    virtual void applyProjectiveConstraints(const core::MechanicalParams* mparams);

    /// Return the mechanical state associated to a component
    static sofa::type::vector<core::behavior::BaseMechanicalState*> retrieveAssociatedMechanicalState(const sofa::core::behavior::StateAccessor* component);

    /// Generate all possible pairs of Mechanical States from a list of Mechanical States
    static sofa::type::vector<PairMechanicalStates> generatePairs(
        const sofa::type::vector<core::behavior::BaseMechanicalState*>& mstates);

    template <Contribution c>
    std::shared_ptr<LocalMappedMatrixType<Real> > getSharedMatrix(
        sofa::core::matrixaccumulator::get_component_type<c>* object,
        const PairMechanicalStates& pair);

    template <Contribution c>
    std::optional<type::Vec2u> getSharedMatrixSize(
        sofa::core::matrixaccumulator::get_component_type<c>* object,
        const PairMechanicalStates& pair);

    template <Contribution c>
    void setSharedMatrix(sofa::core::matrixaccumulator::get_component_type<c>* object, const PairMechanicalStates& pair, std::shared_ptr<LocalMappedMatrixType<Real> > matrix);

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

    Data<bool> m_needClearLocalMatrices { false };

    /// Get the list of components contributing to the global matrix through the contribution type @c
    template<Contribution c>
    sofa::type::vector<sofa::core::matrixaccumulator::get_component_type<c>*> getContributors() const;

    void buildGroupsOfComponentAssociatedToMechanicalStates(
        std::map< PairMechanicalStates, GroupOfComponentsAssociatedToAPairOfMechanicalStates>& groups);
};

struct GroupOfComponentsAssociatedToAPairOfMechanicalStates
{
    std::set<BaseForceField*> forcefieds;
    std::set<BaseMass*> masses;

    friend std::ostream& operator<<(std::ostream& os,
        const GroupOfComponentsAssociatedToAPairOfMechanicalStates& group);
};

inline std::ostream& operator<<(std::ostream& os,
    const GroupOfComponentsAssociatedToAPairOfMechanicalStates& group)
{
    for (const auto component : group.masses)
    {
        os << component->getPathName() << " ";
    }
    for (const auto component : group.forcefieds)
    {
        os << component->getPathName() << " ";
    }
    return os;
}

} //namespace sofa::component::linearsystem

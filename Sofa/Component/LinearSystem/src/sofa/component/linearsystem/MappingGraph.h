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

#include <sofa/simulation/Node.h>

namespace sofa::component::linearsystem
{

/**
 * Connexions betweeen objects through mappings
 *
 * Graph must be built with the build() function.
 */
class SOFA_COMPONENT_LINEARSYSTEM_API MappingGraph
{

public:
    using MappingInputs = type::vector<core::behavior::BaseMechanicalState*>;

    /// Return the node used to start the exploration of the scene graph in order to build the mapping graph
    core::objectmodel::BaseContext* getRootNode() const;
    /// Return the list of all mechanical states which are not mapped
    const sofa::type::vector<core::behavior::BaseMechanicalState*>& getMainMechanicalStates() const;

    /// Return the list of mechanical states which are:
    /// 1) non-mapped
    /// 2) input of a mapping involving the provided mechanical state as an output.
    /// The search is recursive (more than one level of mapping) and is done during mapping graph construction.
    MappingInputs getTopMostMechanicalStates(core::behavior::BaseMechanicalState*) const;

    /// Return the list of mechanical states which are:
    /// 1) non-mapped
    /// 2) input of a mapping involving the mechanical states associated to the provided force field as an output.
    /// The search is recursive (more than one level of mapping) and is done during mapping graph construction.
    MappingInputs getTopMostMechanicalStates(core::behavior::BaseForceField*) const;

    /// Return the list of mechanical states which are:
    /// 1) non-mapped
    /// 2) input of a mapping involving the mechanical states associated to the provided mass as an output.
    /// The search is recursive (more than one level of mapping) and is done during mapping graph construction.
    MappingInputs getTopMostMechanicalStates(core::behavior::BaseMass*) const;

    struct SameGroupComponents
    {
        sofa::type::vector<core::behavior::BaseForceField*> forceFields;
        sofa::type::vector<core::behavior::BaseMass*> masses;
    };

    using ComponentGroups = std::map<core::behavior::BaseMechanicalState*, SameGroupComponents>;

    /// Create groups of components associated to the same mechanical state
    ComponentGroups makeComponentGroups(const sofa::core::ExecParams* params) const;

    [[nodiscard]]
    bool hasAnyMapping() const;

    /// Return true if the provided mechanical state is an output of a mapping
    bool hasAnyMappingInput(core::behavior::BaseMechanicalState*) const;
    /// Return true if the mechanical states associated to the provided force field is an output of a mapping
    bool hasAnyMappingInput(core::behavior::BaseForceField*) const;
    /// Return true if the mechanical states associated to the provided mass is an output of a mapping
    bool hasAnyMappingInput(core::behavior::BaseMass*) const;

    /// Return the sum of the degrees of freedom of all main mechanical states
    [[nodiscard]]
    sofa::Size getTotalNbMainDofs() const { return m_totalNbMainDofs; }

    /// Return where in the global matrix the provided mechanical state writes its contribution
    type::Vec2u getPositionInGlobalMatrix(core::behavior::BaseMechanicalState*) const;
    /// Return where in the global matrix the provided mechanical states writes its contribution
    type::Vec2u getPositionInGlobalMatrix(core::behavior::BaseMechanicalState* a, core::behavior::BaseMechanicalState* b) const;

    MappingGraph() = default;

    bool isBuilt() const;

    /// Build the graph: mandatory to get valid data from the functions that use the graph
    void build(const sofa::core::ExecParams* params, core::objectmodel::BaseContext* rootNode);

private:

    /// node used to start the exploration of the scene graph in order to build the mapping graph
    core::objectmodel::BaseContext* m_rootNode { nullptr };

    /// List of mechanical states that are non-mapped. They can be involved as a mapping input, but not as an output.
    sofa::type::vector<core::behavior::BaseMechanicalState*> m_mainMechanicalStates;

    /// Association between a mechanical state (the key) and a list of mapping input which are non-mapped. In this list,
    /// the mechanical states are involved as an input, but not as an output. The mechanical state in the key is an
    /// output of mappings (even over multiple levels).
    std::map< core::behavior::BaseMechanicalState*, MappingInputs> m_topMostInputsMechanicalStates;

    /// for each main mechanical states, gives the position of its contribution in the global matrix
    std::map< core::behavior::BaseMechanicalState*, type::Vec2u > m_positionInGlobalMatrix;

    sofa::Size m_totalNbMainDofs {};
    bool m_hasAnyMapping = false;

    friend class MappingGraphBuilderVisitor;
};

} //namespace sofa::component::linearsolver

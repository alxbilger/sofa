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
#include <sofa/component/linearsystem/MappingGraph.h>

#include <sofa/core/BaseMapping.h>
#include <sofa/core/behavior/BaseForceField.h>
#include <sofa/core/behavior/BaseMass.h>
#include <sofa/core/behavior/BaseMechanicalState.h>
#include <sofa/core/behavior/BaseProjectiveConstraintSet.h>
#include <sofa/simulation/BaseMechanicalVisitor.h>

namespace sofa::component::linearsystem
{

core::objectmodel::BaseContext* MappingGraph::getRootNode() const
{
    return m_rootNode;
}

const sofa::type::vector<core::behavior::BaseMechanicalState*>& MappingGraph::getMainMechanicalStates() const
{
    return m_mainMechanicalStates;
}

auto MappingGraph::getTopMostMechanicalStates(core::behavior::BaseMechanicalState* mstate) const -> MappingInputs
{
    if (m_rootNode == nullptr)
    {
        msg_error("MappingGraph") << "Graph is not built yet";
    }

    if (mstate == nullptr)
    {
        dmsg_error("MappingGraph") << "Requested mechanical state is invalid";
        return {};
    }

    if (const auto it = m_topMostInputsMechanicalStates.find(mstate); it != m_topMostInputsMechanicalStates.end())
        return it->second;

    return {};
}

auto MappingGraph::getTopMostMechanicalStates(core::behavior::BaseForceField* forceField) const -> MappingInputs
{
    const auto& associatedMechanicalStates = forceField->getMechanicalStates();
    MappingInputs topMostMechanicalStates;
    for (auto* mstate : associatedMechanicalStates)
    {
        const auto mstates = getTopMostMechanicalStates(mstate);
        topMostMechanicalStates.insert(topMostMechanicalStates.end(), mstates.begin(), mstates.end());
    }
    return topMostMechanicalStates;
}

auto MappingGraph::getTopMostMechanicalStates(core::behavior::BaseMass* mass) const -> MappingInputs
{
    const auto& associatedMechanicalStates = mass->getMechanicalStates();
    MappingInputs topMostMechanicalStates;
    for (auto* mstate : associatedMechanicalStates)
    {
        const auto mstates = getTopMostMechanicalStates(mstate);
        topMostMechanicalStates.insert(topMostMechanicalStates.end(), mstates.begin(), mstates.end());
    }
    return topMostMechanicalStates;
}

class ComponentGroupsVisitor final : public simulation::BaseMechanicalVisitor
{
public:
    ComponentGroupsVisitor(const sofa::core::ExecParams* params, MappingGraph::ComponentGroups& groups)
    : simulation::BaseMechanicalVisitor(params)
    , m_groups(groups)
    {}

    Result fwdMass(simulation::Node*, sofa::core::behavior::BaseMass* mass) override
    {
        if (mass)
        {
            for (auto mstate : mass->getMechanicalStates())
            {
                if (mstate)
                {
                    m_groups[mstate].masses.push_back(mass);
                }
            }
        }
        return Result::RESULT_CONTINUE;
    }
    Result fwdForceField(simulation::Node*, sofa::core::behavior::BaseForceField* ff) override
    {
        if (ff)
        {
            for (auto mstate : ff->getMechanicalStates())
            {
                if (mstate)
                {
                    m_groups[mstate].forceFields.push_back(ff);
                }
            }
        }
        return Result::RESULT_CONTINUE;
    }

private:
    MappingGraph::ComponentGroups& m_groups;
};

MappingGraph::ComponentGroups MappingGraph::makeComponentGroups(const sofa::core::ExecParams* params) const
{
    ComponentGroups groups;
    ComponentGroupsVisitor(params, groups).execute(getRootNode());
    return groups;
}

bool MappingGraph::hasAnyMapping() const
{
    return m_hasAnyMapping;
}

bool MappingGraph::hasAnyMappingInput(core::behavior::BaseMechanicalState* mstate) const
{
    if (m_rootNode == nullptr)
    {
        msg_error("MappingGraph") << "Graph is not built yet";
        return false;
    }

    if (mstate == nullptr)
    {
        msg_error("MappingGraph") << "Requested mechanical state is not valid : cannot get its position in the global matrix";
        return false;
    }

    //only main (non mapped) mechanical states are in this map
    return m_positionInGlobalMatrix.find(mstate) == m_positionInGlobalMatrix.end();
}

bool MappingGraph::hasAnyMappingInput(core::behavior::BaseForceField* forceField) const
{
    for (auto* mstate : forceField->getMechanicalStates())
    {
        if (mstate)
        {
            if (hasAnyMappingInput(mstate))
            {
                return true;
            }
        }
    }
    return false;
}

bool MappingGraph::hasAnyMappingInput(core::behavior::BaseMass* mass) const
{
    for (auto* mstate : mass->getMechanicalStates())
    {
        if (mstate)
        {
            if (hasAnyMappingInput(mstate))
            {
                return true;
            }
        }
    }
    return false;
}

type::Vec2u MappingGraph::getPositionInGlobalMatrix(core::behavior::BaseMechanicalState* mstate) const
{
    if (m_rootNode == nullptr)
    {
        msg_error("MappingGraph") << "Graph is not built yet";
        return type::Vec2u{};
    }

    if (mstate == nullptr)
    {
        msg_error("MappingGraph") << "Requested mechanical state is not valid : cannot get its position in the global matrix";
        return type::Vec2u{};
    }

    if (const auto it = m_positionInGlobalMatrix.find(mstate); it != m_positionInGlobalMatrix.end())
        return it->second;

    msg_error("MappingGraph") << "Requested mechanical state (" << mstate->getPathName() <<
        ") is probably mapped or unknown from the graph: only main mechanical states have an associated submatrix in the global matrix";
    return type::Vec2u{};
}

type::Vec2u MappingGraph::getPositionInGlobalMatrix(core::behavior::BaseMechanicalState* a,
    core::behavior::BaseMechanicalState* b) const
{
    const auto pos_a = getPositionInGlobalMatrix(a);
    const auto pos_b = getPositionInGlobalMatrix(b);
    return {pos_a[0], pos_b[1]};
}

class MappingGraphBuilderVisitor final : public simulation::BaseMechanicalVisitor
{
public:
    MappingGraphBuilderVisitor(const sofa::core::ExecParams* params, MappingGraph& mappingGraph)
    : simulation::BaseMechanicalVisitor(params)
    , m_mappingGraph(mappingGraph)
    {}

    bool stopAtMechanicalMapping(simulation::Node* /*node*/, sofa::core::BaseMapping* /*map*/) override { return false; }

    Result fwdMechanicalMapping(simulation::Node* node, sofa::core::BaseMapping* baseMapping) override
    {
        if (baseMapping == nullptr)
        {
            return Result::RESULT_CONTINUE;
        }

        m_mappingGraph.m_hasAnyMapping = true;

        // The mechanical states which are parents of another mechanical state through a mapping are stored in a map for later use
        for (auto* child : baseMapping->getMechTo())
        {
            if (child != nullptr)
            {
                for (auto* parent : baseMapping->getMechFrom())
                {
                    if (parent != nullptr)
                    {
                        m_mstatesParents[child].push_back(parent);
                    }
                }
            }
        }
        return Result::RESULT_CONTINUE;
    }

    Result fwdMechanicalState(simulation::Node* /*node*/,sofa::core::behavior::BaseMechanicalState* mstate) override
    {
        visitMechanicalState(mstate);
        return Result::RESULT_CONTINUE;
    }

    Result fwdMappedMechanicalState(simulation::Node* /*node*/,sofa::core::behavior::BaseMechanicalState* mstate) override
    {
        visitMechanicalState(mstate);
        return Result::RESULT_CONTINUE;
    }

    void visitMechanicalState(sofa::core::behavior::BaseMechanicalState* mstate)
    {
        if (mstate == nullptr)
        {
            return;
        }

        auto it = m_mstatesParents.find(mstate);

        if (it == m_mstatesParents.end())
        {
            //mstate has not been found in the map: it's not an output of any mapping
            const auto matrixSize = mstate->getMatrixSize();

            m_mappingGraph.m_mainMechanicalStates.push_back(mstate);
            m_mappingGraph.m_positionInGlobalMatrix[mstate] = sofa::type::Vec2u(m_mappingGraph.m_totalNbMainDofs, m_mappingGraph.m_totalNbMainDofs);

            m_mappingGraph.m_totalNbMainDofs += matrixSize;

            m_mappingGraph.m_topMostInputsMechanicalStates[mstate].push_back(mstate);
        }
        else
        {
            //mstate is the output of at least one mapping and has at least one mechanical state as an input
            MappingGraph::MappingInputs inputs = it->second;
            if (inputs.empty())
            {
                msg_error("MappingGraph") << "Mechanical state " << mstate->getPathName() << " is involved in a mapping, but does not have any valid input mechanical states";
            }
            else
            {
                while(!inputs.empty())
                {
                    auto* visitedMState = inputs.back();
                    inputs.pop_back();
                    it = m_mstatesParents.find(visitedMState);
                    if (it != m_mstatesParents.end())
                    {
                        for (auto* p : it->second)
                            inputs.push_back(p);
                    }
                    else
                    {
                        m_mappingGraph.m_topMostInputsMechanicalStates[mstate].push_back(visitedMState);
                    }
                }
            }

        }
    }

private:

    MappingGraph& m_mappingGraph;

    std::unordered_map<core::behavior::BaseMechanicalState*, MappingGraph::MappingInputs > m_mstatesParents {};
};

bool MappingGraph::isBuilt() const
{
    return m_rootNode != nullptr;
}

void MappingGraph::build(const sofa::core::ExecParams* params, core::objectmodel::BaseContext* rootNode)
{
    m_rootNode = rootNode;

    m_mainMechanicalStates.clear();
    m_topMostInputsMechanicalStates.clear();
    m_positionInGlobalMatrix.clear();

    m_totalNbMainDofs = 0;
    m_hasAnyMapping = false;

    MappingGraphBuilderVisitor(params, *this).execute(rootNode);
}
}

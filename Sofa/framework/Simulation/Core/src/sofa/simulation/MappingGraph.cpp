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
#define SOFA_SIMULATION_MAPPINGGRAPH_CPP
#include <sofa/simulation/MappingGraph.inl>

#include <sofa/helper/taskflow.h>

#include <sofa/simulation/MechanicalVisitor.h>
#include <sofa/simulation/Node.h>


namespace sofa::simulation
{

template struct SOFA_SIMULATION_CORE_API details::TasksContainer<mapping_graph::VisitorDirection::FORWARD>;
template struct SOFA_SIMULATION_CORE_API details::TasksContainer<mapping_graph::VisitorDirection::BACKWARD>;

template<>
std::string MappingGraph::category<MappingGraph::ListMass>() { return "Mass"; };
template<>
std::string MappingGraph::category<MappingGraph::ListForceFields>() { return "ForceField"; }
template<>
std::string MappingGraph::category<MappingGraph::ListProjectiveConstraints>() { return "ProjConstraint"; }

size_t details::SetHash::operator()(
    const std::set<sofa::core::behavior::BaseMechanicalState*>& objSet) const
{
    size_t hashValue = 0;
    for (sofa::core::behavior::BaseMechanicalState* obj : objSet)
    {
        hashValue ^= std::hash<sofa::core::behavior::BaseMechanicalState*>{}(obj) + 0x9e3779b9 +
                     (hashValue << 6) + (hashValue >> 2);
    }
    return hashValue;
}

bool details::SetEqual::operator()(
    const std::set<sofa::core::behavior::BaseMechanicalState*>& lhs,
    const std::set<sofa::core::behavior::BaseMechanicalState*>& rhs) const
{
    return lhs == rhs;  // Directly compare sets
}

class CreateGraphVisitor : public sofa::simulation::Visitor
{
   public:
    CreateGraphVisitor(const core::ExecParams* p, MappingGraph& graph)
        : sofa::simulation::Visitor(p), mappingGraph(graph)
    {}

    template<class ListGraphVertex>
    void processStateAccessors(const auto& links)
    {
        for (auto* object : links)
        {
            const auto states = object->getMechanicalStates();
            const auto stateSet = MappingGraphVertex::StateGroup{states.begin(), states.end()};

            m_states.emplace_back(stateSet);
            std::get<ListGraphVertex>(mappingGraph.m_stateAccessors).emplace_back(stateSet, object);
        }
    }

    Result processNodeTopDown(simulation::Node* node) override
    {
        if (auto* state = node->mechanicalState.get())
        {
            m_states.emplace_back(MappingGraphVertex::StateGroup{state});
        }

        if (auto* mapping = node->mechanicalMapping.get())
        {
            if (mapping->isMechanical())
            {
                mappingGraph.m_mappings.emplace_back(mapping);

                {
                    if (const auto from = mapping->getMechFrom(); !from.empty())
                    {
                        m_states.emplace_back(MappingGraphVertex::StateGroup{from.begin(), from.end()});
                    }
                }
                {
                    if (const auto to = mapping->getMechTo(); !to.empty())
                    {
                        m_states.emplace_back(MappingGraphVertex::StateGroup{to.begin(), to.end()});
                    }
                }
            }
        }

        processStateAccessors<MappingGraph::ListMass>(node->mass);
        processStateAccessors<MappingGraph::ListForceFields>(node->forceField);
        processStateAccessors<MappingGraph::ListForceFields>(node->interactionForceField);
        processStateAccessors<MappingGraph::ListProjectiveConstraints>(node->projectiveConstraintSet);

        return Result::RESULT_CONTINUE;
    }
    MappingGraph& mappingGraph;

    sofa::type::vector<MappingGraphVertex::StateGroup> m_states;
};

void MappingGraph::buildGraph()
{
    CreateGraphVisitor visitor(m_mparams, *this);
    m_context->executeVisitor(&visitor);

    // remove duplicates
    std::sort(visitor.m_states.begin(), visitor.m_states.end());
    visitor.m_states.erase(std::unique(visitor.m_states.begin(), visitor.m_states.end()),
                           visitor.m_states.end());

    for (const auto& group : visitor.m_states)
    {
        m_states.emplace_back(group);
    }

    std::unordered_map<MappingGraphVertex::StateGroup, StateVertex*, details::SetHash, details::SetEqual>
        stateVertexMap;
    for (auto& v : m_states)
    {
        stateVertexMap[v.m_states] = &v;
    }

    for (auto& mapping : m_mappings)
    {
        {
            const auto from = mapping.m_mapping->getMechFrom();
            const auto group = MappingGraphVertex::StateGroup{from.begin(), from.end()};

            for (auto& state : m_states)
            {
                if (sofa::helper::share_element(group, state.m_states, std::less()))
                {
                    mapping.m_parents.push_back(&state);
                    state.m_mappingChildren.push_back(&mapping);
                }
            }
        }
        {
            const auto to = mapping.m_mapping->getMechTo();
            const auto group = MappingGraphVertex::StateGroup{to.begin(), to.end()};

            for (auto& state : m_states)
            {
                if (sofa::helper::share_element(group, state.m_states, std::less()))
                {
                    mapping.m_children.push_back(&state);
                    state.m_parents.push_back(&mapping);
                }
            }
        }
        mapping.m_task = m_taskflow.emplace([m = &mapping]() { m->task(); }).name("fwdMapping" + mapping.m_mapping->getPathName());
        setupSemaphore(mapping.m_task);
    }

    const auto stateAccessorTasks = [&]<class T>(T& stateAccessors)
    {
        for (auto& accessor : stateAccessors)
        {
            if (auto* stateVertex = stateVertexMap[accessor.m_states])
            {
                accessor.m_parent = stateVertex;
                stateVertex->m_accessorChildren.emplace_back(&accessor);
            }
            accessor.m_task = m_taskflow.emplace([a = &accessor] { a->task(); })
                .name("fwd" + category<T>() + accessor.m_stateAccessor->getPathName());
            setupSemaphore(accessor.m_task);
        }
    };

    stateAccessorTasks(std::get<ListMass>(m_stateAccessors));
    stateAccessorTasks(std::get<ListForceFields>(m_stateAccessors));
    stateAccessorTasks(std::get<ListProjectiveConstraints>(m_stateAccessors));

    for (auto& state : m_states)
    {
        state.m_task = m_taskflow.emplace([s = &state]() { s->task(); })
            .name("fwdState" + sofa::helper::join(state.m_states.begin(), state.m_states.end(),
                [](core::behavior::BaseMechanicalState* s){return s->getPathName(); }, '-'));
        setupSemaphore(state.m_task);

        if (!state.m_accessorChildren.empty())
        {
            state.m_exitTask = std::make_unique<tf::Task>(m_taskflow.emplace([](){}).name("exit"));
            setupSemaphore(*state.m_exitTask);
        }
    }


    const auto setupVisitor = [&]<class T>(sofa::type::vector<T>& vertices)
    {
        for (auto& vertex : vertices)
        {
            vertex.m_visitor = &m_visitor;
        }
    };
    setupVisitor(m_states);
    setupVisitor(m_mappings);
    setupVisitor(std::get<ListMass>(m_stateAccessors));
    setupVisitor(std::get<ListForceFields>(m_stateAccessors));
    setupVisitor(std::get<ListProjectiveConstraints>(m_stateAccessors));
}

void MappingGraph::buildTaskDependencies()
{
    const auto stateAccessorDependencies = [&](auto& stateAccessors)
    {
        for (auto& accessor : stateAccessors)
        {
            if (accessor.m_parent)
            {
                accessor.m_task.precede(*accessor.m_parent->exitPoint());
                accessor.m_task.succeed(accessor.m_parent->m_task);
            }
        }
    };

    stateAccessorDependencies(std::get<ListMass>(m_stateAccessors));
    stateAccessorDependencies(std::get<ListForceFields>(m_stateAccessors));
    stateAccessorDependencies(std::get<ListProjectiveConstraints>(m_stateAccessors));


    for (const auto& mapping : m_mappings)
    {
        for (auto* parent : mapping.m_parents)
        {
            parent->exitPoint()->precede(mapping.m_task);
        }
        for (auto* parent : mapping.m_children)
        {
            parent->m_task.succeed(mapping.m_task);
        }
    }

    for (auto it1 = m_states.begin(); it1 != m_states.end(); ++it1)
    {
        for (auto it2 = std::next(it1); it2 != m_states.end(); ++it2)
        {
            if (sofa::helper::share_element(it1->m_states, it2->m_states, std::less()))
            {
                if (it1->m_states.size() > it2->m_states.size())
                {
                    it1->m_task.succeed(*it2->exitPoint());
                }
                else
                {
                    it1->m_task.precede(it2->m_task);
                }
            }
        }
    }
}

void MappingGraph::dumpTasksGraph(std::ostream& ostream) const { m_taskflow.dump(ostream); }

void MappingGraph::accept(mapping_graph::BaseMappingGraphVisitor& visitor,
                          MappingGraphVisitParameters params)
{
    this->m_visitor = &visitor;

    static tf::Executor executor;
    m_globalSemaphore.reset(params.forceSingleThreadAllTasks ? 1 : params.numberParallelTasks);

    if (params.forward.dumpTaskGraph && m_context->notMuted())
    {
        std::stringstream ss;
        m_taskflow.dump(ss);
        msg_info(m_context) << ss.str();
    }

    executor.run(m_taskflow).wait();
}

void MappingGraph::setupSemaphore(tf::Task& task)
{
    task.acquire(m_globalSemaphore);
    task.release(m_globalSemaphore);
}

}  // namespace sofa::simulation

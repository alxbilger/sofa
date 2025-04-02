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

#include <sofa/core/BaseMapping.h>
#include <sofa/core/MechanicalParams.h>
#include <sofa/helper/taskflow.h>
#include <sofa/simulation/MappingGraphVisitor.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/Visitor.h>
#include <sofa/simulation/config.h>
#include <sofa/helper/ScopedAdvancedTimer.h>

namespace sofa::simulation
{

struct MappingGraphVisitParameters
{
    bool forceSingleThreadAllTasks = false;
    std::size_t numberParallelTasks = std::thread::hardware_concurrency();

    struct Direction
    {
        bool dumpTaskGraph = false;

        bool stateAccessorTasksPrecedeMappingTasks = true;
        bool stateAccessorTasksSucceedStateTasks = true;
        bool sortMappingTasks = true;
    } forward, backward;
};

struct StateVertex;
struct MappingVertex;
struct StateAccessorVertex;

struct MappingGraphVertex
{
    virtual ~MappingGraphVertex() = default;
    using StateGroup = std::set<sofa::core::behavior::BaseMechanicalState*>;

    mapping_graph::BaseMappingGraphVisitor** m_visitor { nullptr };

    tf::Task m_forwardTask;
    tf::Task m_backwardTask;
};

struct StateAccessorVertex : MappingGraphVertex
{
    explicit StateAccessorVertex(const StateGroup& states) : m_states({states}) {}

    StateGroup m_states;

    StateVertex* m_parent { nullptr };
};

template<class T>
struct TStateAccessorVertex : StateAccessorVertex
{
    explicit TStateAccessorVertex(const StateGroup& states, T* object)
        : StateAccessorVertex({states}), m_stateAccessor(object) {}
    T* m_stateAccessor { nullptr };

    void forwardTask()
    {
        if (m_visitor && *m_visitor && m_stateAccessor)
        {
            (*m_visitor)->forwardVisit(m_stateAccessor);
        }
    }

    void backwardTask()
    {
        if (m_visitor && *m_visitor && m_stateAccessor)
        {
            (*m_visitor)->backwardVisit(m_stateAccessor);
        }
    }
};

using ForceFieldVertex = TStateAccessorVertex<core::behavior::BaseForceField>;
using MassVertex = TStateAccessorVertex<core::behavior::BaseMass>;
using ProjectiveConstraintVertex = TStateAccessorVertex<core::behavior::BaseProjectiveConstraintSet>;

struct StateVertex : MappingGraphVertex
{
    explicit StateVertex(sofa::core::behavior::BaseMechanicalState* state) : m_states({state}) {}
    explicit StateVertex(const StateGroup& states) : m_states({states}) {}

    StateGroup m_states;
    sofa::type::vector<MappingVertex*> m_parents;
    sofa::type::vector<MappingVertex*> m_mappingChildren;
    sofa::type::vector<StateAccessorVertex*> m_accessorChildren;

    std::unique_ptr<tf::Task> m_forwardExitTask { nullptr };
    std::unique_ptr<tf::Task> m_backwardEntryTask { nullptr };

    void forwardTask()
    {
        if (m_visitor && *m_visitor && m_states.size() == 1)
        {
            (*m_visitor)->forwardVisit(*m_states.begin());
        }
    }

    void backwardTask()
    {
        if (m_visitor && *m_visitor && m_states.size() == 1)
        {
            (*m_visitor)->backwardVisit(*m_states.begin());
        }
    }

    tf::Task* forwardExitPoint()
    {
        if (m_forwardExitTask)
            return m_forwardExitTask.get();
        return &m_forwardTask;
    }

    tf::Task* backwardEntryPoint()
    {
        if (m_backwardEntryTask)
            return m_backwardEntryTask.get();
        return &m_backwardTask;
    }
};


struct MappingVertex : MappingGraphVertex
{
    explicit MappingVertex(sofa::core::BaseMapping* mapping) : m_mapping(mapping) {}

    sofa::core::BaseMapping* m_mapping { nullptr };
    sofa::type::vector<StateVertex*> m_parents;
    sofa::type::vector<StateVertex*> m_children;

    void forwardTask()
    {
        if (m_visitor && *m_visitor && m_mapping)
        {
            (*m_visitor)->forwardVisit(m_mapping);
        }
    }

    void backwardTask()
    {
        if (m_visitor && *m_visitor && m_mapping)
        {
            (*m_visitor)->backwardVisit(m_mapping);
        }
    }
};

class SOFA_SIMULATION_CORE_API MappingGraph
{
public:
    MappingGraph(const sofa::core::MechanicalParams* mparams, core::objectmodel::BaseContext* context)
        : m_mparams(mparams), m_context(context)
    {
        buildGraph();
    }

    void buildGraph();
    void buildTaskDependencies();
    void dumpTasksGraph(std::ostream& ostream) const;

    void accept(mapping_graph::BaseMappingGraphVisitor& visitor, MappingGraphVisitParameters = {});

private:
    const sofa::core::MechanicalParams* m_mparams{nullptr};
    core::objectmodel::BaseContext* m_context{nullptr};

    sofa::type::vector<MappingVertex> m_mappings;
    sofa::type::vector<StateVertex> m_states;

    using ListForceFields = sofa::type::vector<ForceFieldVertex>;
    using ListMass = sofa::type::vector<MassVertex>;
    using ListProjectiveConstraints = sofa::type::vector<ProjectiveConstraintVertex>;

    std::tuple<
        ListForceFields,
        ListMass,
        ListProjectiveConstraints
    > m_stateAccessors;

    tf::Taskflow m_taskflow;
    tf::Semaphore m_globalSemaphore;

    template<class T>
    static std::string category() { return ""; }

    mapping_graph::BaseMappingGraphVisitor* m_visitor { nullptr };

    void setupSemaphore(tf::Task& task);

    friend class CreateGraphVisitor;
};

namespace details
{
// Custom hash function for std::set<BaseMechanicalState*>
struct SOFA_SIMULATION_CORE_API SetHash
{
    size_t operator()(const std::set<sofa::core::behavior::BaseMechanicalState*>& objSet) const;
};

// Custom equality function for std::set<BaseMechanicalState*>
struct SOFA_SIMULATION_CORE_API SetEqual
{
    bool operator()(const std::set<sofa::core::behavior::BaseMechanicalState*>& lhs,
                    const std::set<sofa::core::behavior::BaseMechanicalState*>& rhs) const;
};

template<mapping_graph::VisitorDirection D>
struct TasksContainer
{
    static const std::string prefix;

    tf::Taskflow* taskflow { nullptr };

    using StateGroup = std::set<sofa::core::behavior::BaseMechanicalState*>;

    std::unordered_map<StateGroup, std::vector<tf::Task>, SetHash, SetEqual > stateAccessorTasks;

    std::unordered_map<core::behavior::BaseMechanicalState*, tf::Task> stateTasks;
    std::unordered_map<core::BaseMapping*, tf::Task> startMappingTasks;
    std::unordered_map<core::BaseMapping*, tf::Task> mappingTasks;

    void sortAllTasks(bool stateAccessorTasksSucceedStateTasks, bool stateAccessorTasksPrecedeMappingTasks, bool sortMappingTasks);
    void applyGlobalSemaphore(tf::Semaphore& s);

private:
    void _makeStateAccessorTasksSequential();
    void _stateAccessorTasksSucceedStateTasks();
    void _stateAccessorTasksPrecedeMappingTasks();
    void _findDependenciesInStateAccessorTasks();
    void _sortMappingTasks();
};
}

#if !defined(SOFA_SIMULATION_MAPPINGGRAPH_CPP)
extern template struct SOFA_SIMULATION_CORE_API details::TasksContainer<mapping_graph::VisitorDirection::FORWARD>;
extern template struct SOFA_SIMULATION_CORE_API details::TasksContainer<mapping_graph::VisitorDirection::BACKWARD>;
#endif

}  // namespace sofa::simulation

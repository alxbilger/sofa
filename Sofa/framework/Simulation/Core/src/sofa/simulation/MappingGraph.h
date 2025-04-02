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
};

struct StateAccessorVertex : MappingGraphVertex
{
    explicit StateAccessorVertex(const StateGroup& states) : m_states({states}) {}

    StateGroup m_states;
    tf::Task m_task;

    StateVertex* m_parent { nullptr };
};

template<class T>
struct TStateAccessorVertex : StateAccessorVertex
{
    explicit TStateAccessorVertex(const StateGroup& states, T* object)
        : StateAccessorVertex({states}), m_stateAccessor(object) {}
    T* m_stateAccessor { nullptr };

    void task()
    {
        if (m_visitor && *m_visitor && m_stateAccessor)
        {
            (*m_visitor)->forwardVisit(m_stateAccessor);
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

    tf::Task m_task;
    std::unique_ptr<tf::Task> m_exitTask { nullptr };

    void task()
    {
        if (m_visitor && *m_visitor && m_states.size() == 1)
        {
            (*m_visitor)->forwardVisit(*m_states.begin());
        }
    }

    tf::Task* entryPoint() const { return nullptr; }
    tf::Task* exitPoint()
    {
        if (m_exitTask)
            return m_exitTask.get();
        return &m_task;
    }
};


struct MappingVertex : MappingGraphVertex
{
    explicit MappingVertex(sofa::core::BaseMapping* mapping) : m_mapping(mapping) {}

    sofa::core::BaseMapping* m_mapping { nullptr };
    sofa::type::vector<StateVertex*> m_parents;
    sofa::type::vector<StateVertex*> m_children;

    tf::Task m_task;
    void task()
    {
        if (m_visitor && *m_visitor && m_mapping)
        {
            (*m_visitor)->forwardVisit(m_mapping);
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

    // template<mapping_graph::IsVisitor Visitor>
    // void accept(Visitor& visitor, MappingGraphVisitParameters = {}) const;

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

#if !defined(SOFA_SIMULATION_MAPPINGGRAPH_CPP)
extern template struct SOFA_SIMULATION_CORE_API details::TasksContainer<mapping_graph::VisitorDirection::FORWARD>;
extern template struct SOFA_SIMULATION_CORE_API details::TasksContainer<mapping_graph::VisitorDirection::BACKWARD>;
#endif

template<mapping_graph::IsVisitor MappingGraphVisitor>
class CreateTasksVisitor : public sofa::simulation::Visitor
{
public:
    explicit CreateTasksVisitor(const sofa::core::MechanicalParams *p, MappingGraphVisitor* v)
        : Visitor(p), mappingGraphVisitor(v)
    {}

    Result processNodeTopDown(simulation::Node* node) override
    {
        /**
         * The forward and backward tasks are created for each state, even if the visitor does not visit states
         */
        addTasks<mapping_graph::VisitorDirection::FORWARD>(node->mechanicalState, forward.stateTasks, "State");
        addTasks<mapping_graph::VisitorDirection::BACKWARD>(node->mechanicalState, backward.stateTasks, "State");

        /**
         * The forward and backward tasks are created for each mapping, even if the visitor does not visit mappings
         */
        addMappingTasks<mapping_graph::VisitorDirection::FORWARD>(node->mechanicalMapping, forward.mappingTasks, forward.startMappingTasks, "Mapping");
        addMappingTasks<mapping_graph::VisitorDirection::BACKWARD>(node->mechanicalMapping, backward.mappingTasks, backward.startMappingTasks, "Mapping");

        /**
         * The forward and backward tasks are created for each component (mass, forcefield, projective constraint)
         */
        addStateAccessorTaskToGroup<core::behavior::BaseMass>(node->mass, "Mass");
        addStateAccessorTaskToGroup<core::behavior::BaseForceField>(node->forceField, "ForceField");
        addStateAccessorTaskToGroup<core::behavior::BaseForceField>(node->interactionForceField, "ForceField");
        addStateAccessorTaskToGroup<core::behavior::BaseProjectiveConstraintSet>(node->projectiveConstraintSet, "ProjectiveConstraint");

        return Result::RESULT_CONTINUE;
    }

private:
    MappingGraphVisitor* mappingGraphVisitor { nullptr };

    template<mapping_graph::VisitorDirection D>
    TasksContainer<D>& getTaskContainer()
    {
        if constexpr (D == mapping_graph::VisitorDirection::FORWARD)
        {
            return forward;
        }
        else
        {
            return backward;
        }
    }

    template<mapping_graph::VisitorDirection D>
    void addDirectionTaskToGroup(auto& links, const std::string& category)
    {
        for (auto* object : links)
        {
            if (object)
            {
                const auto& states = object->getMechanicalStates();
                typename TasksContainer<D>::StateGroup group { states.begin(), states.end() };
                tf::Task task = getTaskContainer<D>().taskflow->emplace([v = mappingGraphVisitor, object]()
                {
                    if constexpr (D == mapping_graph::VisitorDirection::FORWARD)
                    {
                        v->forwardVisit(object);
                    }
                    else
                    {
                        v->backwardVisit(object);
                    }
                }).name(TasksContainer<D>::prefix + category + object->getPathName());
                getTaskContainer<D>().stateAccessorTasks[group].push_back(task);
            }
        }
    }

    template <class T, class NodeLinks>
    void addStateAccessorTaskToGroup(NodeLinks& links, const std::string& category)
    {
        SCOPED_TIMER_TR("addStateAccessorTaskToGroup");
        if constexpr (mapping_graph::CanForwardVisit<MappingGraphVisitor, T>)
        {
            addDirectionTaskToGroup<mapping_graph::VisitorDirection::FORWARD>(links, category);
        }
        if constexpr (mapping_graph::CanBackwardVisit<MappingGraphVisitor, T>)
        {
            addDirectionTaskToGroup<mapping_graph::VisitorDirection::BACKWARD>(links, category);
        }
    }

    template<mapping_graph::VisitorDirection D, mapping_graph::IsTypeVisitable T>
    void addTasks(auto& links, std::unordered_map<T*, tf::Task>& tasks, const std::string& category)
    {
        SCOPED_TIMER_TR("addTasks");
        for (auto* object : links)
        {
            if (object)
            {
                tasks[object] = getTaskContainer<D>().taskflow->emplace([v = mappingGraphVisitor, object]()
                {
                    if constexpr (D == mapping_graph::VisitorDirection::FORWARD
                        && mapping_graph::CanForwardVisit<MappingGraphVisitor, T>)
                    {
                        v->forwardVisit(object);
                    }
                    if constexpr (D == mapping_graph::VisitorDirection::BACKWARD
                        && mapping_graph::CanBackwardVisit<MappingGraphVisitor, T>)
                    {
                        v->backwardVisit(object);
                    }
                }).name(TasksContainer<D>::prefix + category + object->getPathName());
            }
        }
    }

    template<mapping_graph::VisitorDirection D, mapping_graph::IsTypeVisitable T>
    void addMappingTasks(auto& links,
        std::unordered_map<T*, tf::Task>& tasks,
        std::unordered_map<T*, tf::Task>& startTasks,
        const std::string& category)
    {
        SCOPED_TIMER_TR("addMappingTasks");
        for (auto* object : links)
        {
            if (object)
            {
                tf::Task mappingTask = getTaskContainer<D>().taskflow->emplace([v = mappingGraphVisitor, object]()
                {
                    if constexpr (D == mapping_graph::VisitorDirection::FORWARD
                        && mapping_graph::CanForwardVisit<MappingGraphVisitor, T>)
                    {
                        v->forwardVisit(object);
                    }
                    if constexpr (D == mapping_graph::VisitorDirection::BACKWARD
                        && mapping_graph::CanBackwardVisit<MappingGraphVisitor, T>)
                    {
                        v->backwardVisit(object);
                    }
                }).name(TasksContainer<D>::prefix + category + object->getPathName());

                tf::Task pruneTask = getTaskContainer<D>().taskflow->emplace([v = mappingGraphVisitor, object]() -> int
                {
                    if constexpr (D == mapping_graph::VisitorDirection::FORWARD
                        && mapping_graph::CanForwardVisit<MappingGraphVisitor, T>)
                    {
                        return v->forwardPrune(object);
                    }
                    if constexpr (D == mapping_graph::VisitorDirection::BACKWARD
                        && mapping_graph::CanBackwardVisit<MappingGraphVisitor, T>)
                    {
                        return v->backwardPrune(object);
                    }
                    return false;
                }).name("prune");

                tf::Task initTask = getTaskContainer<D>().taskflow->emplace([](){}).name("start");
                tf::Task stopTask = getTaskContainer<D>().taskflow->emplace([](){}).name("stop");

                pruneTask.succeed(initTask).precede(mappingTask, stopTask);

                tasks[object] = mappingTask;
                startTasks[object] = initTask;
            }
        }
    }

public:

    TasksContainer<mapping_graph::VisitorDirection::FORWARD> forward;
    TasksContainer<mapping_graph::VisitorDirection::BACKWARD> backward;
};
}
//
// template <mapping_graph::IsVisitor Visitor>
// void MappingGraph::accept(Visitor& visitor, MappingGraphVisitParameters params) const
// {
//     if (m_mparams && m_context)
//     {
//         SCOPED_TIMER_TR("acceptVisitor");
//
//         tf::Taskflow forwardTaskFlow("forward");
//         tf::Taskflow backwardTaskFlow("backward");
//
//         static tf::Executor executor;
//         tf::Semaphore semaphore(params.forceSingleThreadAllTasks ? 1 : params.numberParallelTasks);
//
//         details::CreateTasksVisitor<Visitor> v(m_mparams, &visitor);
//         v.forward.taskflow = &forwardTaskFlow;
//         v.backward.taskflow = &backwardTaskFlow;
//         {
//             SCOPED_TIMER_TR("createTasks");
//             m_context->executeVisitor(&v);
//         }
//
//         if constexpr(mapping_graph::IsForwardVisitor<Visitor>)
//         {
//             {
//                 SCOPED_TIMER_TR("sortForwardTasks");
//                 v.forward.sortAllTasks(
//                    params.forward.stateAccessorTasksSucceedStateTasks,
//                    params.forward.stateAccessorTasksPrecedeMappingTasks,
//                    params.forward.sortMappingTasks);
//                 v.forward.applyGlobalSemaphore(semaphore);
//             }
//
//             if (params.forward.dumpTaskGraph && m_context->notMuted())
//             {
//                 std::stringstream ss;
//                 forwardTaskFlow.dump(ss);
//                 msg_info(m_context) << ss.str();
//             }
//
//             SCOPED_TIMER_TR("executeForward");
//             executor
//                .run(forwardTaskFlow)
//                .wait();
//         }
//
//         if constexpr(mapping_graph::IsBackwardVisitor<Visitor>)
//         {
//             {
//                 SCOPED_TIMER_TR("sortBackwardTasks");
//                 v.backward.sortAllTasks(
//                    params.backward.stateAccessorTasksSucceedStateTasks,
//                    params.backward.stateAccessorTasksPrecedeMappingTasks,
//                    params.backward.sortMappingTasks);
//                 v.backward.applyGlobalSemaphore(semaphore);
//             }
//
//             if (params.backward.dumpTaskGraph && m_context->notMuted())
//             {
//                 std::stringstream ss;
//                 backwardTaskFlow.dump(ss);
//                 msg_info(m_context) << ss.str();
//             }
//
//             SCOPED_TIMER_TR("executeBackward");
//             executor
//                .run(backwardTaskFlow)
//                .wait();
//         }
//     }
// }
}  // namespace sofa::simulation

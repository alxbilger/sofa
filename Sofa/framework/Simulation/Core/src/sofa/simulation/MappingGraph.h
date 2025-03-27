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

namespace sofa::simulation
{

class SOFA_SIMULATION_CORE_API MappingGraph
{
public:
    MappingGraph(const sofa::core::MechanicalParams* mparams, core::objectmodel::BaseContext* context)
        : m_mparams(mparams), m_context(context)
    {}

    template<mapping_graph::IsVisitor Visitor>
    void accept(Visitor& visitor, bool executeConcurrently = true) const;

private:
    const sofa::core::MechanicalParams* m_mparams{nullptr};
    core::objectmodel::BaseContext* m_context{nullptr};
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
    std::unordered_map<core::BaseMapping*, tf::Task> mappingTasks;

    void sortAllTasks();
    void applyGlobalSemaphore(tf::Semaphore& s);

private:
    void makeStateAccessorTasksSequential();
    void stateAccessorTasksSucceedStateTasks();
    void stateAccessorTasksPrecedeMappingTasks();
    void findDependenciesInStateAccessorTasks();
    void sortMappingTasks();
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
        addTasks<mapping_graph::VisitorDirection::FORWARD>(node->mechanicalMapping, forward.mappingTasks, "Mapping");
        addTasks<mapping_graph::VisitorDirection::BACKWARD>(node->mechanicalMapping, backward.mappingTasks, "Mapping");

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

public:

    TasksContainer<mapping_graph::VisitorDirection::FORWARD> forward;
    TasksContainer<mapping_graph::VisitorDirection::BACKWARD> backward;
};
}

template <mapping_graph::IsVisitor Visitor>
void MappingGraph::accept(Visitor& visitor, bool executeConcurrently) const
{
    if (m_mparams && m_context)
    {
        tf::Taskflow forwardTaskFlow, backwardTaskFlow;

        static tf::Executor executor;
        tf::Semaphore semaphore(executeConcurrently ? executor.num_workers() : 1);

        details::CreateTasksVisitor<Visitor> v(m_mparams, &visitor);
        v.forward.taskflow = &forwardTaskFlow;
        v.backward.taskflow = &backwardTaskFlow;
        m_context->executeVisitor(&v);

        v.forward.sortAllTasks();
        v.forward.applyGlobalSemaphore(semaphore);

        v.backward.sortAllTasks();
        v.backward.applyGlobalSemaphore(semaphore);

        executor
            .run(forwardTaskFlow, [&backwardTaskFlow]() { executor.run(backwardTaskFlow).wait(); })
            .wait();
    }
}
}  // namespace sofa::simulation

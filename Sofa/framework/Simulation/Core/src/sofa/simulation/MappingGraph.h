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

#include <sofa/simulation/config.h>
#include <sofa/simulation/MappingGraphVisitor.h>
#include <sofa/simulation/Visitor.h>
#include <sofa/simulation/Node.h>
#include <sofa/core/MechanicalParams.h>
#include <sofa/helper/taskflow.h>

#include <map>



namespace sofa::simulation
{
class Node;

class SOFA_SIMULATION_CORE_API MappingGraph
{
public:
    MappingGraph(const sofa::core::MechanicalParams* mparams, simulation::Node* node)
        : m_mparams(mparams), m_node(node)
    {}

    template<mapping_graph::IsVisitor Visitor>
    void accept(Visitor& visitor, bool executeConcurrently = true) const;

private:
    const sofa::core::MechanicalParams* m_mparams{nullptr};
    simulation::Node* m_node{nullptr};

    static void sortMappingTasks(std::map<core::BaseMapping*, tf::Task>& mappingTasks,
      std::map<core::behavior::BaseMechanicalState*, tf::Task>& stateTasks,
      bool isForward);

    template<class Component>
    static void sortComponentTasks(std::map<Component*, tf::Task>& componentTasks,
                            std::map<core::behavior::BaseMechanicalState*, tf::Task>& stateTasks,
                            bool isForward);
};


namespace details
{

template<mapping_graph::IsVisitor MappingGraphVisitor>
class CreateTasksVisitor : public sofa::simulation::Visitor
{
public:
    explicit CreateTasksVisitor(const sofa::core::MechanicalParams *p, MappingGraphVisitor* v)
        : Visitor(p), mappingGraphVisitor(v)
    {}

    Result processNodeTopDown(simulation::Node* node) override
    {
        const auto addForwardTasks = [this]<mapping_graph::IsTypeVisitable T0>(
            auto& links,
            tf::FlowBuilder* forwardFlow,
            std::map<T0*, tf::Task>& forwardTasks,
            const std::string& category)
        {
            if constexpr (mapping_graph::CanForwardVisit<MappingGraphVisitor, T0>)
            {
                for (auto* object : links)
                {
                    if (object)
                    {
                        auto* ptr = static_cast<T0*>(object);

                        forwardTasks[ptr] = forwardFlow->emplace([v = mappingGraphVisitor, ptr]()
                        {
                            v->forwardVisit(ptr);
                        }).name("fwd" + category + ptr->getPathName());
                    }
                }
            }
        };

        const auto addBackwardTasks = [this]<mapping_graph::IsTypeVisitable T0>(
            auto& links,
            tf::FlowBuilder* backwardFlow,
            std::map<T0*, tf::Task>& backwardTasks,
            const std::string& category)
        {
            if constexpr (mapping_graph::CanBackwardVisit<MappingGraphVisitor, T0>)
            {
                for (auto* object : links)
                {
                    if (object)
                    {
                        auto* ptr = static_cast<T0*>(object);

                        backwardTasks[ptr] = backwardFlow->emplace([v = mappingGraphVisitor, ptr]()
                        {
                            v->backwardVisit(ptr);
                        }).name("bwd" + category + ptr->getPathName());
                    }
                }
            }
        };

        addForwardTasks(node->mechanicalState, forward.taskflow, forward.stateTasks, "State");
        addBackwardTasks(node->mechanicalState, backward.taskflow, backward.stateTasks, "State");

        addForwardTasks(node->mechanicalMapping, forward.taskflow, forward.mappingTasks, "Mapping");
        addBackwardTasks(node->mechanicalMapping, backward.taskflow, backward.mappingTasks, "Mapping");

        addForwardTasks(node->mass, forward.taskflow, forward.massTasks, "Mass");
        addBackwardTasks(node->mass, backward.taskflow, backward.massTasks, "Mass");

        addForwardTasks(node->forceField, forward.taskflow, forward.forceFieldTasks, "ForceField");
        addBackwardTasks(node->forceField, backward.taskflow, backward.forceFieldTasks, "ForceField");

        addForwardTasks(node->interactionForceField, forward.taskflow, forward.forceFieldTasks, "InteractionForceField");
        addBackwardTasks(node->interactionForceField, backward.taskflow, backward.forceFieldTasks, "InteractionForceField");

        addForwardTasks(node->projectiveConstraintSet, forward.taskflow, forward.projectiveConstraintTasks, "ProjectiveConstraint");
        addBackwardTasks(node->projectiveConstraintSet, backward.taskflow, backward.projectiveConstraintTasks, "ProjectiveConstraint");

        return Result::RESULT_CONTINUE;
    }

private:
    MappingGraphVisitor* mappingGraphVisitor { nullptr };

public:
    struct Tasks
    {
        tf::Taskflow* taskflow { nullptr };
        std::map<core::behavior::BaseMechanicalState*, tf::Task> stateTasks;
        std::map<core::BaseMapping*, tf::Task> mappingTasks;
        std::map<sofa::core::behavior::BaseMass*, tf::Task> massTasks;
        std::map<sofa::core::behavior::BaseForceField*, tf::Task> forceFieldTasks;
        std::map<sofa::core::behavior::BaseProjectiveConstraintSet*, tf::Task> projectiveConstraintTasks;
    };

    Tasks forward;
    Tasks backward;
};
}



template <mapping_graph::IsVisitor Visitor>
void MappingGraph::accept(Visitor& visitor, bool executeConcurrently) const
{
    if (m_mparams && m_node)
    {
        tf::Taskflow forwardTaskFlow, backwardTaskFlow;

        static tf::Executor executor;
        tf::Semaphore semaphore(executeConcurrently ? executor.num_workers() : 1);

        details::CreateTasksVisitor<Visitor> v(m_mparams, &visitor);
        v.forward.taskflow = &forwardTaskFlow;
        v.backward.taskflow = &backwardTaskFlow;
        m_node->executeVisitor(&v);

        sortMappingTasks(v.forward.mappingTasks, v.forward.stateTasks, true);
        sortMappingTasks(v.backward.mappingTasks, v.backward.stateTasks, false);

        sortComponentTasks(v.forward.massTasks, v.forward.stateTasks, true);
        sortComponentTasks(v.backward.massTasks, v.backward.stateTasks, false);

        sortComponentTasks(v.forward.forceFieldTasks, v.forward.stateTasks, true);
        sortComponentTasks(v.backward.forceFieldTasks, v.backward.stateTasks, false);

        sortComponentTasks(v.forward.projectiveConstraintTasks, v.forward.stateTasks, true);
        sortComponentTasks(v.backward.projectiveConstraintTasks, v.backward.stateTasks, false);

        const auto handleSemaphore = [&semaphore](auto& tasks)
        {
            for (auto& [_, task] : tasks)
            {
                task.acquire(semaphore);
                task.release(semaphore);
            }
        };

        handleSemaphore(v.forward.stateTasks);
        handleSemaphore(v.backward.stateTasks);

        handleSemaphore(v.forward.mappingTasks);
        handleSemaphore(v.backward.mappingTasks);

        handleSemaphore(v.forward.massTasks);
        handleSemaphore(v.backward.massTasks);

        handleSemaphore(v.forward.forceFieldTasks);
        handleSemaphore(v.backward.forceFieldTasks);

        handleSemaphore(v.forward.projectiveConstraintTasks);
        handleSemaphore(v.backward.projectiveConstraintTasks);

        forwardTaskFlow.dump(std::cout);
        std::cout << std::endl;

        executor
            .run(forwardTaskFlow, [&backwardTaskFlow]() { executor.run(backwardTaskFlow).wait(); })
            .wait();
    }
}
template <class Component>
void MappingGraph::sortComponentTasks(
    std::map<Component*, tf::Task>& componentTasks,
    std::map<core::behavior::BaseMechanicalState*, tf::Task>& stateTasks, bool isForward)
{
    for (auto& [component, task] : componentTasks)
    {
        if (component)
        {
            for (auto* state : component->getMechanicalStates())
            {
                if (state)
                {
                    const auto it = stateTasks.find(state);
                    if (it != stateTasks.end())
                    {
                        if (isForward)
                        {
                            it->second.precede(task);
                        }
                        else
                        {
                            it->second.succeed(task);
                        }
                    }
                }
            }
        }
    }
}

}  // namespace sofa::simulation

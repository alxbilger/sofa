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
#include <sofa/simulation/MappingGraph.h>
#include <sofa/simulation/MechanicalVisitor.h>
#include <sofa/simulation/Node.h>

#include <sofa/helper/taskflow.h>

namespace sofa::simulation
{

class CreateTasksVisitor : public Visitor
{
public:
    explicit CreateTasksVisitor(const sofa::core::MechanicalParams *p, MappingGraphVisitor* v)
        : Visitor(p), mappingGraphVisitor(v)
    {}

    Result processNodeTopDown(simulation::Node* node) override
    {
        for (auto* state : node->mechanicalState)
        {
            if (state)
            {
                forward.stateTasks[state] = forward.taskflow->emplace([v = mappingGraphVisitor, state]()
                {
                    v->forwardVisit(state);
                }).name("fwdState" + state->getPathName());

                backward.stateTasks[state] = backward.taskflow->emplace([v = mappingGraphVisitor, state]()
                {
                    v->backwardVisit(state);
                }).name("bwdState" + state->getPathName());
            }
        }

        for (auto* mapping : node->mechanicalMapping)
        {
            if (mapping)
            {
                forward.mappingTasks[mapping] = forward.taskflow->emplace([v = mappingGraphVisitor, mapping]()
                {
                    v->forwardVisit(mapping);
                }).name("fwdMapping" + mapping->getPathName());

                backward.mappingTasks[mapping] = backward.taskflow->emplace([v = mappingGraphVisitor, mapping]()
                {
                    v->backwardVisit(mapping);
                }).name("bwdMapping" + mapping->getPathName());
            }
        }

        return Result::RESULT_CONTINUE;
    }

private:
    MappingGraphVisitor* mappingGraphVisitor;

public:
    struct Tasks
    {
        tf::Taskflow* taskflow { nullptr };
        std::map<core::behavior::BaseMechanicalState*, tf::Task> stateTasks;
        std::map<core::BaseMapping*, tf::Task> mappingTasks;
    };

    Tasks forward;
    Tasks backward;
};

void MappingGraph::accept(MappingGraphVisitor& visitor, bool executeConcurrently)
{
    if (m_mparams && m_node)
    {
        tf::Taskflow forwardTaskFlow, backwardTaskFlow;

        static tf::Executor executor;
        tf::Semaphore semaphore(executeConcurrently ? executor.num_workers() : 1);

        CreateTasksVisitor v(m_mparams, &visitor);
        v.forward.taskflow = &forwardTaskFlow;
        v.backward.taskflow = &backwardTaskFlow;
        m_node->executeVisitor(&v);

        for (auto& [mapping, task] : v.forward.mappingTasks)
        {
            for (auto* state : mapping->getMechFrom())
            {
                if (state)
                {
                    const auto it = v.forward.stateTasks.find(state);
                    if (it != v.forward.stateTasks.end())
                    {
                        it->second.precede(task);
                    }
                }
            }
            for (auto* state : mapping->getMechTo())
            {
                if (state)
                {
                    const auto it = v.forward.stateTasks.find(state);
                    if (it != v.forward.stateTasks.end())
                    {
                        it->second.succeed(task);
                    }
                }
            }
        }


        for (auto& [mapping, task] : v.backward.mappingTasks)
        {
            for (auto* state : mapping->getMechFrom())
            {
                if (state)
                {
                    const auto it = v.backward.stateTasks.find(state);
                    if (it != v.backward.stateTasks.end())
                    {
                        it->second.succeed(task);
                    }
                }
            }
            for (auto* state : mapping->getMechTo())
            {
                if (state)
                {
                    const auto it = v.backward.stateTasks.find(state);
                    if (it != v.backward.stateTasks.end())
                    {
                        it->second.precede(task);
                    }
                }
            }
        }

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

        executor.run(forwardTaskFlow, [&backwardTaskFlow]()
        {
            executor.run(backwardTaskFlow).wait();
        }).wait();

    }
}

}  // namespace sofa::simulation

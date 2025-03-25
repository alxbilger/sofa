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

#include <sofa/helper/taskflow.h>

#include <sofa/simulation/MechanicalVisitor.h>
#include <sofa/simulation/Node.h>


namespace sofa::simulation
{



void sortMappingTasks(
    std::map<core::BaseMapping*, tf::Task>& mappingTasks,
    std::map<core::behavior::BaseMechanicalState*, tf::Task>& stateTasks, bool isForward)
{
    for (auto& [mapping, task] : mappingTasks)
    {
        for (auto* state : mapping->getMechFrom())
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
        for (auto* state : mapping->getMechTo())
        {
            if (state)
            {
                const auto it = stateTasks.find(state);
                if (it != stateTasks.end())
                {
                    if (isForward)
                    {
                        it->second.succeed(task);
                    }
                    else
                    {
                        it->second.precede(task);
                    }
                }
            }
        }
    }
}

template<class Component>
void sortComponentTasks(std::map<Component*, tf::Task>& componentTasks,
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
//
// void MappingGraph::accept(MappingGraphVisitor& visitor, bool executeConcurrently) const
// {
//     if (m_mparams && m_node)
//     {
//         tf::Taskflow forwardTaskFlow, backwardTaskFlow;
//
//         static tf::Executor executor;
//         tf::Semaphore semaphore(executeConcurrently ? executor.num_workers() : 1);
//
//         CreateTasksVisitor v(m_mparams, &visitor);
//         v.forward.taskflow = &forwardTaskFlow;
//         v.backward.taskflow = &backwardTaskFlow;
//         m_node->executeVisitor(&v);
//
//         sortMappingTasks(v.forward.mappingTasks, v.forward.stateTasks, true);
//         sortMappingTasks(v.backward.mappingTasks, v.backward.stateTasks, false);
//
//         sortComponentTasks(v.forward.massTasks, v.forward.stateTasks, true);
//         sortComponentTasks(v.backward.massTasks, v.backward.stateTasks, false);
//         sortComponentTasks(v.forward.forceFieldTasks, v.forward.stateTasks, true);
//         sortComponentTasks(v.backward.forceFieldTasks, v.backward.stateTasks, false);
//         sortComponentTasks(v.forward.projectiveConstraintTasks, v.forward.stateTasks, true);
//         sortComponentTasks(v.backward.projectiveConstraintTasks, v.backward.stateTasks, false);
//
//         const auto handleSemaphore = [&semaphore](auto& tasks)
//         {
//             for (auto& [_, task] : tasks)
//             {
//                 task.acquire(semaphore);
//                 task.release(semaphore);
//             }
//         };
//
//         handleSemaphore(v.forward.stateTasks);
//         handleSemaphore(v.backward.stateTasks);
//
//         handleSemaphore(v.forward.mappingTasks);
//         handleSemaphore(v.backward.mappingTasks);
//
//         handleSemaphore(v.forward.massTasks);
//         handleSemaphore(v.backward.massTasks);
//
//         handleSemaphore(v.forward.forceFieldTasks);
//         handleSemaphore(v.backward.forceFieldTasks);
//
//         handleSemaphore(v.forward.projectiveConstraintTasks);
//         handleSemaphore(v.backward.projectiveConstraintTasks);
//
//         forwardTaskFlow.dump(std::cout);
//
//         executor.run(forwardTaskFlow, [&backwardTaskFlow]()
//         {
//             executor.run(backwardTaskFlow).wait();
//         }).wait();
//
//     }
// }

void MappingGraph::sortMappingTasks(
    std::map<core::BaseMapping*, tf::Task>& mappingTasks,
    std::map<core::behavior::BaseMechanicalState*, tf::Task>& stateTasks, bool isForward)
{
    for (auto& [mapping, task] : mappingTasks)
    {
        for (auto* state : mapping->getMechFrom())
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
        for (auto* state : mapping->getMechTo())
        {
            if (state)
            {
                const auto it = stateTasks.find(state);
                if (it != stateTasks.end())
                {
                    if (isForward)
                    {
                        it->second.succeed(task);
                    }
                    else
                    {
                        it->second.precede(task);
                    }
                }
            }
        }
    }
}
}  // namespace sofa::simulation

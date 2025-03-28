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

#include <sofa/simulation/MappingGraph.h>

namespace sofa::simulation
{

template<mapping_graph::VisitorDirection D>
const std::string details::TasksContainer<D>::prefix = mapping_graph::VisitorDirection::FORWARD == D ? "fwd" : "bwd";

template<mapping_graph::VisitorDirection D>
void details::TasksContainer<D>::_makeStateAccessorTasksSequential()
{
    for (auto& [_, tasks] : this->stateAccessorTasks)
    {
        taskflow->linearize(tasks);
    }
}

template<mapping_graph::VisitorDirection D>
void details::TasksContainer<D>::_stateAccessorTasksSucceedStateTasks()
{
    for (auto& [states, tasks] : stateAccessorTasks)
    {
        if (!tasks.empty())
        {
            for (auto* state : states)
            {
                if (state)
                {
                    const auto it = stateTasks.find(state);
                    if (it != stateTasks.end())
                    {
                        if constexpr (D == mapping_graph::VisitorDirection::FORWARD)
                        {
                            tasks.front().succeed(it->second);
                        }
                        else
                        {
                            tasks.front().precede(it->second);
                        }
                    }
                }
            }
        }
    }
}

template<mapping_graph::VisitorDirection D>
void details::TasksContainer<D>::_stateAccessorTasksPrecedeMappingTasks()
{
    for (auto& [states, tasks] : stateAccessorTasks)
    {
        if (!tasks.empty())
        {
            for (auto* state : states)
            {
                if (state)
                {
                    for (const auto& [mapping, mappingTask] : mappingTasks)
                    {
                        const auto& mappingInput = mapping->getMechFrom();
                        if (std::find(mappingInput.begin(), mappingInput.end(), state) !=
                            mappingInput.end())
                        {
                            if constexpr (D == mapping_graph::VisitorDirection::FORWARD)
                            {
                                tasks.back().precede(mappingTask);
                            }
                            else
                            {
                                tasks.back().succeed(mappingTask);
                            }
                        }
                    }
                }
            }
        }
    }
}

template<mapping_graph::VisitorDirection D>
void details::TasksContainer<D>::_findDependenciesInStateAccessorTasks()
{
    for (auto it1 = stateAccessorTasks.begin(); it1 != stateAccessorTasks.end(); ++it1)
    {
        for (auto it2 = std::next(it1, 1); it2 != stateAccessorTasks.end(); ++it2)
        {
            if (sofa::helper::share_element(it1->first, it2->first, std::less<>{}))
            {
                if constexpr (D == mapping_graph::VisitorDirection::FORWARD)
                {
                    it1->second.back().precede(it2->second.front());
                }
                else
                {
                    it1->second.back().succeed(it2->second.front());
                }
            }
        }
    }
}

template <mapping_graph::VisitorDirection D>
void details::TasksContainer<D>::_sortMappingTasks()
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
                    if constexpr (D == mapping_graph::VisitorDirection::FORWARD)
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
                    if constexpr (D == mapping_graph::VisitorDirection::FORWARD)
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

template <mapping_graph::VisitorDirection D>
void details::TasksContainer<D>::sortAllTasks(bool stateAccessorTasksSucceedStateTasks, bool stateAccessorTasksPrecedeMappingTasks, bool sortMappingTasks)
{
    _makeStateAccessorTasksSequential();

    if (stateAccessorTasksSucceedStateTasks)
        _stateAccessorTasksSucceedStateTasks();

    if (stateAccessorTasksPrecedeMappingTasks)
        _stateAccessorTasksPrecedeMappingTasks();

    _findDependenciesInStateAccessorTasks();

    if (sortMappingTasks)
        _sortMappingTasks();
}

template <mapping_graph::VisitorDirection D>
void details::TasksContainer<D>::applyGlobalSemaphore(tf::Semaphore& s)
{
    for (auto& [_, task] : stateTasks)
    {
        task.acquire(s);
        task.release(s);
    }

    for (auto& [_, task] : mappingTasks)
    {
        task.acquire(s);
        task.release(s);
    }

    for (auto& [_, tasks] : this->stateAccessorTasks)
    {
        for (auto& task : tasks)
        {
            task.acquire(s);
            task.release(s);
        }
    }
}

}

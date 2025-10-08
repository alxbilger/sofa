#pragma once
#include <sofa/helper/taskflow.h>
#include <sofa/simulation/config.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/taskflow/detail.h>

namespace sofa::simulation::taskflow
{

enum class TaskflowType : char
{
    TrivialParallelism,
    StateSemaphoreParallelism,
};

template<TaskflowType type>
struct TaskflowContainer;

template<>
struct TaskflowContainer<TaskflowType::TrivialParallelism>
{
    explicit TaskflowContainer(tf::FlowBuilder* _builder) : builder(_builder) {}
    tf::FlowBuilder* builder { nullptr };
};

template<class Callable>
void processComponents(TaskflowContainer<TaskflowType::TrivialParallelism>& taskflow, const Node* node, Callable callable)
{
    const auto& componentList = detail::ComponentInNode<detail::callable_argument_t<Callable>>::get(node);
    if constexpr (detail::is_iterable<decltype(componentList)>)
    {
        for (const auto& component : componentList)
        {
            if (component)
            {
                taskflow.builder->emplace([component, callable]()
                {
                    callable(component);
                });
            }
        }
    }
    else
    {
        if (componentList)
        {
            taskflow.builder->emplace([component = componentList, callable]()
            {
                callable(component);
            });
        }
    }
}

template<class... Callable>
void processNode(TaskflowContainer<TaskflowType::TrivialParallelism>& taskflow, const Node* node, Callable... callable)
{
    (processComponents<Callable>(taskflow, node, callable), ...);

    for (auto& child : node->child)
    {
        processNode(taskflow, child.get(), callable...);
    }
}

template<class... Callable>
void createTaskflow(
    TaskflowContainer<TaskflowType::TrivialParallelism>& taskflow,
    const sofa::simulation::Node* node,
    Callable... callable)
{
    processNode(taskflow, node, callable...);
}



using StateSemaphore = std::unordered_map<core::behavior::BaseMechanicalState*, tf::Semaphore>;

template<>
struct TaskflowContainer<TaskflowType::StateSemaphoreParallelism>
{
    explicit TaskflowContainer(tf::FlowBuilder* _builder) : builder(_builder) {}
    tf::FlowBuilder* builder;
    StateSemaphore semaphores;
};

template<class Callable>
void processComponentsStateSemaphore(
    TaskflowContainer<TaskflowType::StateSemaphoreParallelism>& taskflowContainer,
    const Node* node, Callable callable)
{
    const auto& componentList = detail::ComponentInNode<detail::callable_argument_t<Callable>>::get(node);
    if constexpr (detail::is_iterable<decltype(componentList)>)
    {
        for (const auto& component : componentList)
        {
            if (component)
            {
                tf::Task task = taskflowContainer.builder->emplace([component, callable]()
                {
                    callable(component);
                });

                const auto& mstates = component->getMechanicalStates();
                for (auto* mstate : mstates)
                {
                    auto [it, success] = taskflowContainer.semaphores.emplace(mstate, 1);
                    task.acquire(it->second).release(it->second);
                }
            }
        }
    }
    else
    {
        if (componentList)
        {
            tf::Task task = taskflowContainer.builder->emplace([component = componentList, callable]()
            {
                callable(component);
            });

            const auto& mstates = componentList->getMechanicalStates();
            for (auto* mstate : mstates)
            {
                auto [it, success] = taskflowContainer.semaphores.emplace(mstate, 1);
                task.acquire(it->second).release(it->second);
            }
        }
    }
}

template<class... Callable>
void processNodeStateSemaphore(
    TaskflowContainer<TaskflowType::StateSemaphoreParallelism>& taskflowContainer,
    const Node* node, Callable... callable)
{
    (processComponentsStateSemaphore<Callable>(taskflowContainer, node, callable), ...);

    for (auto& child : node->child)
    {
        processNodeStateSemaphore(taskflowContainer, child.get(), callable...);
    }
}


template<class... Callable>
void createTaskflow(
    TaskflowContainer<TaskflowType::StateSemaphoreParallelism>& taskflowContainer,
    const sofa::simulation::Node* node,
    Callable... callable)
{
    processNodeStateSemaphore(taskflowContainer, node, callable...);
}

}  // namespace sofa::simulation::taskflow

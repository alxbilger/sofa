#pragma once

#include <sofa/simulation/taskflow/TaskflowVisitor.h>
#include <sofa/simulation/taskflow/detail.h>
#include <sofa/helper/ScopedAdvancedTimer.h>

namespace sofa::simulation::taskflow
{

template<class Component>
struct ComponentFunction
{
    virtual ~ComponentFunction() = default;
    virtual void apply(Component* /*component*/) = 0;
};

template<class... Components>
struct StateGroupVisitor : public TaskflowVisitor, public ComponentFunction<Components>...
{
    using TaskflowVisitor::TaskflowVisitor;
    using TaskflowVisitor::s_executor;
    using ComponentFunction<Components>::apply...;

    void run(Node* node) override
    {
        SCOPED_TIMER_TR("StateGroupVisitor");
        processNode(node);
        s_executor.run(m_taskflow).wait();
    }

private:

    void processNode(Node* node)
    {
        (processComponents<Components>(node), ...);

        for (auto& child : node->child)
        {
            this->processNode(child.get());
        }
    }

    template<class Component>
    void processComponents(Node* node)
    {
        auto componentList = detail::ComponentInNode<Component>::get(node);
        for (const auto component : componentList)
        {
            if (component)
            {
                tf::Task task = m_taskflow.emplace([this, component]()
                {
                    this->apply(component);
                });

                const auto& mstates = component->getMechanicalStates();
                for (auto* mstate : mstates)
                {
                    auto [it, success] = m_semaphores.emplace(mstate, 1);
                    task.acquire(it->second).release(it->second);
                }
            }
        }
    }

    std::unordered_map<core::behavior::BaseMechanicalState*, tf::Semaphore> m_semaphores;
    tf::Taskflow m_taskflow;
};

#if !defined(SOFA_SIMULATION_TASKFLOW_STATEGROUPVISITOR_CPP)
extern template SOFA_SIMULATION_CORE_API struct ComponentFunction<core::behavior::BaseForceField>;
extern template SOFA_SIMULATION_CORE_API struct ComponentFunction<core::behavior::BaseInteractionForceField>;
#endif

}

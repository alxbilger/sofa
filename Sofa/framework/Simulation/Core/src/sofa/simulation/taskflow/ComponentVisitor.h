#pragma once
#include <sofa/simulation/taskflow/TaskflowVisitor.h>
#include <sofa/simulation/taskflow/detail.h>
#include <sofa/helper/ScopedAdvancedTimer.h>

namespace sofa::simulation::taskflow
{

template<class... Components>
struct ComponentVisitor : public TaskflowVisitor, public detail::ComponentFunction<Components>...
{
    using TaskflowVisitor::TaskflowVisitor;
    using TaskflowVisitor::s_executor;
    using detail::ComponentFunction<Components>::apply...;

    void run(Node* node) override
    {
        SCOPED_TIMER_TR("ComponentVisitor");
        s_executor.silent_async([this, node]()
        {
            processNode(node);
        });
        s_executor.wait_for_all();
    }

private:
    void processNode(Node* node)
    {
        (processComponents<Components>(node), ...);

        for (auto& child : node->child)
        {
            s_executor.silent_async([this, c = child.get()]()
            {
                this->processNode(c);
            });
        }
    }

    template<class Component>
    void processComponents(Node* node)
    {
        const auto& componentList = detail::ComponentInNode<Component>::get(node);
        if constexpr (detail::is_iterable<decltype(componentList)>)
        {
            for (const auto& component : componentList)
            {
                if (component)
                {
                    s_executor.silent_async([this, component]()
                    {
                        apply(component);
                    });
                }
            }
        }
        else
        {
            if (componentList)
            {
                s_executor.silent_async([this, componentList]()
                {
                    apply(componentList);
                });
            }
        }
    }
};


}

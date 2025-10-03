#pragma once
#include <sofa/simulation/taskflow/TaskflowVisitor.h>
#include <sofa/simulation/taskflow/detail.h>
#include <sofa/helper/ScopedAdvancedTimer.h>

namespace sofa::simulation::taskflow
{

template<class Component>
struct ComponentVisitor : public TaskflowVisitor
{
    using TaskflowVisitor::TaskflowVisitor;
    using TaskflowVisitor::s_executor;

    void run(Node* node) override
    {
        SCOPED_TIMER_TR("ComponentVisitor");
        s_executor.silent_async([this, node]()
        {
            processNode(node);
        });
        s_executor.wait_for_all();
    }

    virtual void apply(Component* /*component*/) = 0;

private:
    void processNode(Node* node)
    {
        auto componentList = detail::ComponentInNode<Component>::get(node);
        if constexpr (detail::is_iterable<decltype(componentList)>)
        {
            for (const auto component : componentList)
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

        for (auto& child : node->child)
        {
            s_executor.silent_async([this, c = child.get()]()
            {
                this->processNode(c);
            });
        }
    }
};


}

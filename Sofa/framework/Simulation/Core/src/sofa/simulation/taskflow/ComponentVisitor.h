#pragma once
#include <sofa/simulation/taskflow/TaskflowVisitor.h>
#include <sofa/simulation/taskflow/detail.h>
#include <sofa/simulation/taskflow/GlobalExecutor.h>
#include <sofa/helper/ScopedAdvancedTimer.h>

namespace sofa::simulation::taskflow
{

template<class... Components>
struct ComponentVisitor : public TaskflowVisitor, public detail::ComponentFunction<Components>...
{
    using TaskflowVisitor::TaskflowVisitor;
    using detail::ComponentFunction<Components>::apply...;

    void run(Node* node) override
    {
        SCOPED_TIMER_TR("ComponentVisitor");
        getExecutor().silent_async([this, node]()
        {
            processNode(node);
        });
        getExecutor().wait_for_all();
    }

private:
    void processNode(Node* node)
    {
        (processComponents<Components>(node), ...);

        for (auto& child : node->child)
        {
            getExecutor().silent_async([this, c = child.get()]()
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
                    getExecutor().silent_async([this, component]()
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
                getExecutor().silent_async([this, componentList]()
                {
                    apply(componentList);
                });
            }
        }
    }
};

template<class Callable>
struct CallableComponentVisitor : public ComponentVisitor<detail::callable_argument_t<Callable>>
{
    using Component = detail::callable_argument_t<Callable>;
    CallableComponentVisitor(const sofa::core::ExecParams* params, Callable callable)
        : ComponentVisitor<Component>(params), m_callable(callable) {}

    void apply(Component* component) override
    {
        this->m_callable(component);
    }
protected:
    Callable m_callable;
};


template<class Callable>
void executeVisitor(core::objectmodel::BaseContext* context, Callable callable)
{
    CallableComponentVisitor<Callable> v(nullptr, callable);
    context->executeTaskflowVisitor(&v);
}

}

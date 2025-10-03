#pragma once
#include <sofa/simulation/taskflow/TaskflowVisitor.h>
#include <sofa/simulation/Node.h>

namespace sofa::simulation::taskflow
{

namespace detail
{
template<class Component>
struct ComponentInNode
{};

template<> struct ComponentInNode<core::behavior::BaseMechanicalState>
{
    static auto get(const Node* node)
    {
        return node->getMechanicalState();
    }
};

template <typename T>
concept is_iterable_impl = requires(T& t)
{
    begin(t) != end(t); // begin/end and operator !=
    ++std::declval<decltype(begin(t))&>(); // operator ++
    *begin(t); // operator*
};

template <typename T>
concept is_iterable = detail::is_iterable_impl<T>;

}

template<class Component>
struct ComponentVisitor : public TaskflowVisitor
{
    using TaskflowVisitor::TaskflowVisitor;
    using TaskflowVisitor::s_executor;

    void run(Node* node) override
    {
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

    tf::Taskflow m_taskflow;
};


}

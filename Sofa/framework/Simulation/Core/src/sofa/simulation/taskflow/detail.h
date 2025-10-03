#pragma once
#include <sofa/simulation/Node.h>

namespace sofa::simulation::taskflow::detail
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

template<> struct ComponentInNode<core::behavior::BaseForceField>
{
    static const auto& get(const Node* node)
    {
        return node->forceField;
    }
};

template<> struct ComponentInNode<core::behavior::BaseInteractionForceField>
{
    static const auto& get(const Node* node)
    {
        return node->interactionForceField;
    }
};

template <typename T>
concept is_iterable_impl = requires(T& t)
{
    t.begin();
    t.end();
    ++std::declval<decltype(t.begin())&>(); // operator ++
    *t.begin(); // operator*
};

template <typename T>
concept is_iterable = detail::is_iterable_impl<T>;


static_assert(is_iterable<NodeSequence<core::behavior::BaseForceField>>);
}

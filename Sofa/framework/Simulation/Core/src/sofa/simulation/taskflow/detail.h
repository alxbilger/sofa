#pragma once
#include <sofa/simulation/Node.h>

namespace sofa::simulation::taskflow::detail
{

template<class Component>
struct ComponentInNode
{};

#define ADD_COMPONENTINNODE_ENTRY(Component, fieldName) \
    template<> struct ComponentInNode<Component> \
    { \
        static const auto& get(const Node* node) \
        { \
            return node->fieldName; \
        } \
    };

ADD_COMPONENTINNODE_ENTRY(core::BehaviorModel, behaviorModel)
// ADD_COMPONENTINNODE_ENTRY(core::BaseMapping, mapping) //There is a conflict with mechanicalMapping preventing the template specialization
ADD_COMPONENTINNODE_ENTRY(core::behavior::OdeSolver, solver)
ADD_COMPONENTINNODE_ENTRY(core::behavior::ConstraintSolver, constraintSolver)
ADD_COMPONENTINNODE_ENTRY(core::behavior::BaseLinearSolver, linearSolver)
ADD_COMPONENTINNODE_ENTRY(core::topology::BaseTopologyObject, topologyObject)
ADD_COMPONENTINNODE_ENTRY(core::behavior::BaseForceField, forceField)
ADD_COMPONENTINNODE_ENTRY(core::behavior::BaseInteractionForceField, interactionForceField)
ADD_COMPONENTINNODE_ENTRY(core::behavior::BaseProjectiveConstraintSet, projectiveConstraintSet)
ADD_COMPONENTINNODE_ENTRY(core::behavior::BaseConstraintSet, constraintSet)
ADD_COMPONENTINNODE_ENTRY(core::objectmodel::ContextObject, contextObject)
ADD_COMPONENTINNODE_ENTRY(core::objectmodel::ConfigurationSetting, configurationSetting)
ADD_COMPONENTINNODE_ENTRY(core::visual::Shader, shaders)
ADD_COMPONENTINNODE_ENTRY(core::visual::VisualModel, visualModel)
ADD_COMPONENTINNODE_ENTRY(core::visual::VisualManager, visualManager)
ADD_COMPONENTINNODE_ENTRY(core::CollisionModel, collisionModel)

ADD_COMPONENTINNODE_ENTRY(sofa::core::behavior::BaseAnimationLoop, animationManager)
ADD_COMPONENTINNODE_ENTRY(sofa::core::visual::VisualLoop, visualLoop)
ADD_COMPONENTINNODE_ENTRY(sofa::core::visual::BaseVisualStyle, visualStyle)
ADD_COMPONENTINNODE_ENTRY(sofa::core::topology::Topology, topology)
ADD_COMPONENTINNODE_ENTRY(sofa::core::topology::BaseMeshTopology, meshTopology)
ADD_COMPONENTINNODE_ENTRY(sofa::core::BaseState, state)
ADD_COMPONENTINNODE_ENTRY(sofa::core::behavior::BaseMechanicalState, mechanicalState)
ADD_COMPONENTINNODE_ENTRY(sofa::core::BaseMapping, mechanicalMapping)
ADD_COMPONENTINNODE_ENTRY(sofa::core::behavior::BaseMass, mass)
ADD_COMPONENTINNODE_ENTRY(sofa::core::collision::Pipeline, collisionPipeline)

#undef ADD_COMPONENTINNODE_ENTRY

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




template<class Component>
struct ComponentFunction
{
    virtual ~ComponentFunction() = default;
    virtual void apply(Component* /*component*/) = 0;
};

#if !defined(SOFA_SIMULATION_CORE_TASKFLOW_DETAIL_CPP)
extern template SOFA_SIMULATION_CORE_API struct ComponentFunction<core::behavior::BaseForceField>;
extern template SOFA_SIMULATION_CORE_API struct ComponentFunction<core::behavior::BaseInteractionForceField>;
#endif





// Helper to extract the argument type from a callable
template <typename T>
struct callable_argument;

// Specialization for function pointers
template <typename T>
struct callable_argument<void(*)(T*)>
{
    using argument_type = T;
};

// Specialization for std::function
template <typename T>
struct callable_argument<std::function<void(T*)>>
{
    using argument_type = T;
};

// Specialization for member function pointers
template <typename C, typename T>
struct callable_argument<void(C::*)(T*)>
{
    using argument_type = T;
};

// Specialization for const member function pointers
template <typename C, typename T>
struct callable_argument<void(C::*)(T*) const>
{
    using argument_type = T;
};

// Specialization for functors and lambdas
template <typename F>
struct callable_argument : public callable_argument<decltype(&F::operator())> {};

template <typename T>
using callable_argument_t = typename callable_argument<T>::argument_type;



}

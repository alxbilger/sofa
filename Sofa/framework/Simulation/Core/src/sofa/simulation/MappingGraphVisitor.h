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

#include <sofa/core/behavior/BaseForceField.h>
#include <sofa/core/behavior/BaseInteractionForceField.h>
#include <sofa/core/behavior/BaseMass.h>
#include <sofa/core/behavior/BaseProjectiveConstraintSet.h>
#include <sofa/simulation/config.h>

#include <sofa/type/trait/callable_argument.h>
#include <type_traits>

namespace sofa::simulation::mapping_graph
{

enum class VisitorDirection : bool
{
    FORWARD,
    BACKWARD
};

template<VisitorDirection Direction>
constexpr inline VisitorDirection OtherDirection =
    Direction == VisitorDirection::FORWARD ? VisitorDirection::BACKWARD : VisitorDirection::FORWARD;

template<class T>
struct TForwardVisitor
{
    virtual ~TForwardVisitor() {}
    virtual void forwardVisit(T*) {}
};

template<>
struct TForwardVisitor<core::BaseMapping>
{
    virtual ~TForwardVisitor() {}
    virtual void forwardVisit(core::BaseMapping*) {}
    virtual bool forwardPrune(core::BaseMapping*) { return false; }
};

template<class T>
struct TBackwardVisitor
{
    virtual ~TBackwardVisitor() {}
    virtual void backwardVisit(T*) {}
};

template<>
struct TBackwardVisitor<core::BaseMapping>
{
    virtual ~TBackwardVisitor() {}
    virtual void backwardVisit(core::BaseMapping*) {}
    virtual bool backwardPrune(core::BaseMapping*) { return false; }
};

template<class T, VisitorDirection Direction>
using TVisitor = std::conditional_t<Direction == VisitorDirection::FORWARD, TForwardVisitor<T>, TBackwardVisitor<T>>;

namespace details
{
    /**
     * An empty structure to contain a list of types
     */
    template <class... Types>
    struct ListTypes{};

    /**
     * A trait to check if there is at least one type from a given list of types which is a base
     * class of a given type.
     */
    template <typename T, typename List>
    struct is_base_of_any;

    template<typename T, template<class...> class List, typename... Types>
    struct is_base_of_any<T, List<Types...>>
    {
        static constexpr bool value = (std::is_base_of_v<Types, T> || ...);
    };
}

using VisitableTypes = details::ListTypes<
    sofa::core::behavior::BaseMechanicalState,
    sofa::core::BaseMapping,
    sofa::core::behavior::BaseMass,
    sofa::core::behavior::BaseForceField,
    sofa::core::behavior::BaseProjectiveConstraintSet
>;

template<class T>
concept IsTypeVisitable = details::is_base_of_any<T, VisitableTypes>::value;

template<class VisitorType, class T>
concept CanForwardVisit = IsTypeVisitable<T> && requires(VisitorType& v, T* t)
{
    v.forwardVisit(t);
};

template<class VisitorType, class T>
concept CanBackwardVisit = IsTypeVisitable<T> && requires(VisitorType& v, T* t)
{
    v.backwardVisit(t);
};

template<class VisitorType, VisitorDirection Direction, class T>
concept CanVisit =
    Direction == VisitorDirection::FORWARD && CanForwardVisit<VisitorType, T>
    || Direction == VisitorDirection::BACKWARD && CanBackwardVisit<VisitorType, T>;

namespace details
{
    template<typename T, VisitorDirection Direction, typename List>
    struct IsVisitorOfAny;

    template<typename T, VisitorDirection Direction, template<class...> class List, typename... Types>
    struct IsVisitorOfAny<T, Direction, List<Types...>>
    {
        static constexpr bool value = (CanVisit<T, Direction, Types> || ...);
    };
}


template<class T>
concept IsForwardVisitor = details::IsVisitorOfAny<T, VisitorDirection::FORWARD, VisitableTypes>::value;
template<class T>
concept IsBackwardVisitor = details::IsVisitorOfAny<T, VisitorDirection::BACKWARD, VisitableTypes>::value;

template<class T>
concept IsVisitor = IsForwardVisitor<T> || IsBackwardVisitor<T>;

template<VisitorDirection Direction>
using StateVisitor = TVisitor<sofa::core::behavior::BaseMechanicalState, Direction>;
using StateForwardVisitor = StateVisitor<VisitorDirection::FORWARD>;
using StateBackwardVisitor = StateVisitor<VisitorDirection::BACKWARD>;

template<VisitorDirection Direction>
using MappingVisitor = TVisitor<sofa::core::BaseMapping, Direction>;
using MappingForwardVisitor = MappingVisitor<VisitorDirection::FORWARD>;
using MappingBackwardVisitor = MappingVisitor<VisitorDirection::BACKWARD>;

template<VisitorDirection Direction>
using MassVisitor = TVisitor<sofa::core::behavior::BaseMass, Direction>;
using MassForwardVisitor = MassVisitor<VisitorDirection::FORWARD>;
using MassBackwardVisitor = MassVisitor<VisitorDirection::BACKWARD>;

template<VisitorDirection Direction>
using ForceFieldVisitor = TVisitor<sofa::core::behavior::BaseForceField, Direction>;
using ForceFieldForwardVisitor = ForceFieldVisitor<VisitorDirection::FORWARD>;
using ForceFieldBackwardVisitor = ForceFieldVisitor<VisitorDirection::BACKWARD>;

template<VisitorDirection Direction>
using ProjectiveConstraintVisitor = TVisitor<sofa::core::behavior::BaseProjectiveConstraintSet, Direction>;
using ProjectiveConstraintForwardVisitor = ProjectiveConstraintVisitor<VisitorDirection::FORWARD>;
using ProjectiveConstraintBackwardVisitor = ProjectiveConstraintVisitor<VisitorDirection::BACKWARD>;

template<class T1, class T2>
struct CompositeVisitor : T1, T2
{
    using ForwardVisitor = std::conditional_t<IsForwardVisitor<T1>, T1, T2>;
    using BackwardVisitor = std::conditional_t<IsBackwardVisitor<T1>, T1, T2>;
    static_assert(IsForwardVisitor<T1> && IsBackwardVisitor<T2> || IsBackwardVisitor<T1> && IsForwardVisitor<T2>);

    using BackwardVisitor::backwardVisit;
    using ForwardVisitor::forwardVisit;
    CompositeVisitor(const T1& a, const T2& b)
        : T1(a), T2(b){}
};

template <VisitorDirection Direction, typename... T>
struct CallableVisitor;

template <VisitorDirection Direction, class Derived>
struct ComposableVisitor
{
    template<typename... Other>
    auto operator+(CallableVisitor<OtherDirection<Direction>, Other...> other)
    {
        return CompositeVisitor(*static_cast<Derived*>(this), other);
    }
};

template <typename Callable>
requires IsTypeVisitable<type::trait::callable_argument_t<Callable>>
struct CallableVisitor<VisitorDirection::FORWARD, Callable> :
    TVisitor<sofa::type::trait::callable_argument_t<Callable>, VisitorDirection::FORWARD>,
    ComposableVisitor<VisitorDirection::FORWARD, CallableVisitor<VisitorDirection::FORWARD, Callable>>
{
    explicit CallableVisitor(const Callable& callable)
        : m_function(callable) {}
    using ComposableVisitor<VisitorDirection::FORWARD, CallableVisitor>::operator+;

    void forwardVisit(sofa::type::trait::callable_argument_t<Callable>* object) override
    {
        m_function(object);
    }

private:
    Callable m_function;
};

template <typename Callable>
requires IsTypeVisitable<sofa::type::trait::callable_argument_t<Callable>>
struct CallableVisitor<VisitorDirection::BACKWARD, Callable> :
    TVisitor<sofa::type::trait::callable_argument_t<Callable>, VisitorDirection::BACKWARD>,
    ComposableVisitor<VisitorDirection::BACKWARD, CallableVisitor<VisitorDirection::BACKWARD, Callable>>
{
    explicit CallableVisitor(const Callable& callable)
        : m_function(callable) {}
    using ComposableVisitor<VisitorDirection::BACKWARD, CallableVisitor>::operator+;

    void backwardVisit(sofa::type::trait::callable_argument_t<Callable>* object) override
    {
        m_function(object);
    }

private:
    Callable m_function;
};

template <typename Callable, typename... Rest>
struct CallableVisitor<VisitorDirection::FORWARD, Callable, Rest...> :
    CallableVisitor<VisitorDirection::FORWARD, Callable>,
    CallableVisitor<VisitorDirection::FORWARD, Rest...>,
    ComposableVisitor<VisitorDirection::FORWARD, CallableVisitor<VisitorDirection::FORWARD, Callable, Rest...>>
{
    explicit CallableVisitor(const Callable& callable, Rest... rest)
        : CallableVisitor<VisitorDirection::FORWARD, Rest...>(rest...)
        , CallableVisitor<VisitorDirection::FORWARD, Callable>(callable) {}
    using ComposableVisitor<VisitorDirection::FORWARD, CallableVisitor>::operator+;
    using CallableVisitor<VisitorDirection::FORWARD, Rest...>::forwardVisit;
    using CallableVisitor<VisitorDirection::FORWARD, Callable>::forwardVisit;
};

template <typename Callable, typename... Rest>
struct CallableVisitor<VisitorDirection::BACKWARD, Callable, Rest...> :
    CallableVisitor<VisitorDirection::BACKWARD, Callable>,
    CallableVisitor<VisitorDirection::BACKWARD, Rest...>,
    ComposableVisitor<VisitorDirection::BACKWARD, CallableVisitor<VisitorDirection::BACKWARD, Callable, Rest...>>
{
    explicit CallableVisitor(const Callable& callable, Rest... rest)
        : CallableVisitor<VisitorDirection::BACKWARD, Rest...>(rest...)
        , CallableVisitor<VisitorDirection::BACKWARD, Callable>(callable) {}
    using ComposableVisitor<VisitorDirection::BACKWARD, CallableVisitor>::operator+;
    using CallableVisitor<VisitorDirection::BACKWARD, Rest...>::backwardVisit;
    using CallableVisitor<VisitorDirection::BACKWARD, Callable>::backwardVisit;
};

template <typename... Callable>
auto makeForwardVisitor(Callable... callable)
{
    return CallableVisitor<VisitorDirection::FORWARD, Callable...>(callable...);
}

template <typename... Callable>
auto makeBackwardVisitor(Callable... callable)
{
    return CallableVisitor<VisitorDirection::BACKWARD, Callable...>(callable...);
}

}

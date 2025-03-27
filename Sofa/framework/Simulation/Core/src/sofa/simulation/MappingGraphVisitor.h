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

#include <sofa/simulation/config.h>

#include <type_traits>

#include <sofa/core/behavior/BaseMass.h>
#include <sofa/core/behavior/BaseForceField.h>
#include <sofa/core/behavior/BaseProjectiveConstraintSet.h>
#include <sofa/core/behavior/BaseInteractionForceField.h>

namespace sofa::simulation::mapping_graph
{

enum class VisitorDirection : bool
{
    FORWARD,
    BACKWARD
};

template<class T>
struct TForwardVisitor
{
    virtual ~TForwardVisitor() {}
    virtual void forwardVisit(T*) {}
};

template<class T>
struct TBackwardVisitor
{
    virtual ~TBackwardVisitor() {}
    virtual void backwardVisit(T*) {}
};

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

namespace details
{
    template<typename T, typename List>
    struct IsForwardVisitorOfAny;

    template<typename T, template<class...> class List, typename... Types>
    struct IsForwardVisitorOfAny<T, List<Types...>>
    {
        static constexpr bool value = (CanForwardVisit<T, Types> || ...);
    };

    template<typename T, typename List>
    struct IsBackwardVisitorOfAny;

    template<typename T, template<class...> class List, typename... Types>
    struct IsBackwardVisitorOfAny<T, List<Types...>>
    {
        static constexpr bool value = (CanBackwardVisit<T, Types> || ...);
    };
}


template<class T>
concept IsForwardVisitor = details::IsForwardVisitorOfAny<T, VisitableTypes>::value;
template<class T>
concept IsBackwardVisitor = details::IsBackwardVisitorOfAny<T, VisitableTypes>::value;

template<class T>
concept IsVisitor = IsForwardVisitor<T> || IsBackwardVisitor<T>;

using StateForwardVisitor = TForwardVisitor<sofa::core::behavior::BaseMechanicalState>;
using StateBackwardVisitor = TBackwardVisitor<sofa::core::behavior::BaseMechanicalState>;

using MappingForwardVisitor = TForwardVisitor<sofa::core::BaseMapping>;
using MappingBackwardVisitor = TBackwardVisitor<sofa::core::BaseMapping>;

using MassForwardVisitor = TForwardVisitor<sofa::core::behavior::BaseMass>;
using MassBackwardVisitor = TBackwardVisitor<sofa::core::behavior::BaseMass>;

using ForceFieldForwardVisitor = TForwardVisitor<sofa::core::behavior::BaseForceField>;
using ForceFieldBackwardVisitor = TBackwardVisitor<sofa::core::behavior::BaseForceField>;

using ProjectiveConstraintForwardVisitor = TForwardVisitor<sofa::core::behavior::BaseProjectiveConstraintSet>;
using ProjectiveConstraintBackwardVisitor = TBackwardVisitor<sofa::core::behavior::BaseProjectiveConstraintSet>;

}

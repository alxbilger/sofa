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

#include <sofa/core/config.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/core/objectmodel/BaseComponent.h>
#include <sofa/core/topology/BaseMeshTopology.h>

namespace sofa::core::behavior
{

template<class TBaseClass>
class TopologyAccessorWrapper : public TBaseClass
{
public:
    SOFA_CLASS(TopologyAccessorWrapper, TBaseClass);
    using BaseClass = TBaseClass;

    sofa::SingleLink<TopologyAccessorWrapper, sofa::core::topology::BaseMeshTopology,
        sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK> l_topology;

protected:
    TopologyAccessorWrapper() : l_topology(initLink("topology", "Link to a topology")) {}

    /**
     * Ensure a link to a valid topology
     */
    void validateTopology()
    {
        if (l_topology.empty())
        {
            msg_info() << "This component requires a defined topology link. As no link was provided,"
                          " it will automatically default to using the first available topology in "
                          "this context. For reliable behavior, please explicitly supply the "
                          "required topology link.";
            l_topology.set(this->getContext()->getMeshTopologyLink());
        }

        if (l_topology == nullptr)
        {
            const auto allTopologyComponents = sofa::core::ObjectFactory::getInstance()->listClassesDerivedFrom<sofa::core::topology::BaseMeshTopology>();
            msg_error() << "Cannot initialize component. This component requires a valid link to a "
                           "topology that could not be found at path `" << this->l_topology.getLinkedPath()
                        << "` or within the current context `" << this->getContext()->name << "`. "
                           "Please explicitly supply the required topology link. Refer to this list "
                           "for available topology components:" << allTopologyComponents;

            this->d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        }
    }

    void wrapperInit()
    {
        BaseClass::init();

        if (!this->isComponentStateInvalid())
        {
            validateTopology();
        }
    }
};

template<class TBaseClass> using AddTopologyAccessTo = TopologyAccessorWrapper<TBaseClass>;

struct TopologyAccess{};

template<class TBaseClass, class TComposer> struct Compose;
template<class TBaseClass, class TComposer> using Compose_t = typename Compose<TBaseClass, TComposer>::type;

template<class TBaseClass, TopologyAccess> struct Compose
{
    using type = TopologyAccessorWrapper<TBaseClass>;
};

class SOFA_CORE_API TopologyAccessor : public TopologyAccessorWrapper<sofa::core::objectmodel::BaseComponent>
{
public:
    SOFA_CLASS(TopologyAccessor, TopologyAccessorWrapper<sofa::core::objectmodel::BaseComponent>);

    void init() override;
};

}

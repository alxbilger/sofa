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
#define SOFA_COMPONENT_MAPPING_DistanceFromTargetMapping_CPP

#include <sofa/component/mapping/nonlinear/DistanceFromTargetMapping.inl>
#include <sofa/core/ObjectFactory.h>

namespace sofa::component::mapping::nonlinear
{

using namespace defaulttype;

void registerDistanceFromTargetMapping(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(core::ObjectRegistrationData("Mapping point positions to distances from target points.")
        .add< DistanceFromTargetMapping< Vec3Types, Vec1Types > >()
        .add< DistanceFromTargetMapping< Vec1Types, Vec1Types > >()
        .add< DistanceFromTargetMapping< Rigid3Types, Vec1Types > >());
}

template class SOFA_COMPONENT_MAPPING_NONLINEAR_API DistanceFromTargetMapping< Vec3Types, Vec1Types >;
template class SOFA_COMPONENT_MAPPING_NONLINEAR_API DistanceFromTargetMapping< Vec1Types, Vec1Types >;
template class SOFA_COMPONENT_MAPPING_NONLINEAR_API DistanceFromTargetMapping< Rigid3Types, Vec1Types >;

} // namespace sofa::component::mapping::nonlinear

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

#include <sofa/component/mapping/mappedmatrix/config.h>
#include <sofa/core/objectmodel/BaseObject.h>

namespace sofa::component::mapping::mappedmatrix
{

template<typename TDataTypes1, typename TDataTypes2>
class
// SOFA_ATTRIBUTE_DEPRECATED("v23.06", "v23.12", "Matrix mapping is now supported automatically. Therefore, MechanicalMatrixMapper is no longer necessary.")
MechanicalMatrixMapper : public sofa::core::objectmodel::BaseObject
{
public:
    SOFA_CLASS(MechanicalMatrixMapper, BaseObject);

    static bool canCreate(MechanicalMatrixMapper<TDataTypes1, TDataTypes2>* obj, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg)
    {
        SOFA_UNUSED(obj);
        SOFA_UNUSED(context);
        arg->logError("Matrix mapping is now supported automatically. Therefore, MechanicalMatrixMapper is no longer necessary. Remove it from your scene.");
        return false;
    }
};

#if !defined(SOFA_COMPONENT_ANIMATIONLOOP_MECHANICALMATRIXMAPPER_CPP)
extern template class SOFA_COMPONENT_MAPPING_MAPPEDMATRIX_API MechanicalMatrixMapper<defaulttype::Rigid3Types, defaulttype::Rigid3Types>;
extern template class SOFA_COMPONENT_MAPPING_MAPPEDMATRIX_API MechanicalMatrixMapper<defaulttype::Vec3Types, defaulttype::Rigid3Types>;
extern template class SOFA_COMPONENT_MAPPING_MAPPEDMATRIX_API MechanicalMatrixMapper<defaulttype::Vec3Types, defaulttype::Vec3Types>;
extern template class SOFA_COMPONENT_MAPPING_MAPPEDMATRIX_API MechanicalMatrixMapper<defaulttype::Vec1Types, defaulttype::Rigid3Types>;
extern template class SOFA_COMPONENT_MAPPING_MAPPEDMATRIX_API MechanicalMatrixMapper<defaulttype::Vec1Types, defaulttype::Vec3Types>;
extern template class SOFA_COMPONENT_MAPPING_MAPPEDMATRIX_API MechanicalMatrixMapper<defaulttype::Vec1Types, defaulttype::Vec1Types>;
extern template class SOFA_COMPONENT_MAPPING_MAPPEDMATRIX_API MechanicalMatrixMapper<defaulttype::Rigid3Types, defaulttype::Vec1Types>;
#endif

} // namespace sofa::component::mapping::mappedmatrix

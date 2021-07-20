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
#include <sofa/core/behavior/BaseForceField.h>
#include <sofa/core/objectmodel/BaseNode.h>
#include <sofa/core/behavior/BaseLocalForceFieldMatrix.h>
#include <sofa/core/MechanicalParams.h>
#include <sofa/core/behavior/MatrixAPICompatibility.h>

namespace sofa::core::behavior
{

BaseForceField::BaseForceField()
    : objectmodel::BaseObject()
    , isCompliance( initData(&isCompliance, false, "isCompliance", "Consider the component as a compliance, else as a stiffness"))
    , rayleighStiffness( initData(&rayleighStiffness, SReal(0), "rayleighStiffness", "Rayleigh damping - stiffness matrix coefficient"))
{
}

void BaseForceField::addMBKdx(const MechanicalParams* mparams, MultiVecDerivId dfId)
{
    if (sofa::core::mechanicalparams::kFactorIncludingRayleighDamping(mparams,rayleighStiffness.getValue()) != 0.0 || sofa::core::mechanicalparams::bFactor(mparams) != 0.0)
        addDForce(mparams, dfId);
}

void BaseForceField::addBToMatrix(const MechanicalParams* /*mparams*/, const sofa::core::behavior::MultiMatrixAccessor* /*matrix*/)
{
}

void BaseForceField::addMBKToMatrix(const MechanicalParams* mparams, const sofa::core::behavior::MultiMatrixAccessor* matrix)
{
    if (sofa::core::mechanicalparams::kFactorIncludingRayleighDamping(mparams,rayleighStiffness.getValue()) != 0.0 )
        addKToMatrix(mparams, matrix);
    if (sofa::core::mechanicalparams::bFactor(mparams) != 0.0)
        addBToMatrix(mparams, matrix);
}

void core::behavior::BaseForceField::buildStiffnessMatrix()
{
    const auto& slaves = getSlaves();

    ListStiffnessMatrixAccumulator matrices;

    for (const auto& slave : slaves )
    {
        if (auto matrix = sofa::core::objectmodel::SPtr_dynamic_cast<matrixaccumulator::BaseForceFieldMatrixAccumulator>(slave))
        {
            matrices.push_back(matrix.get());
        }
    }

    if (!matrices.empty())
    {
        matrices.clear();
        buildStiffnessMatrix(&matrices);
    }
}

void BaseForceField::buildStiffnessMatrix(StiffnessMatrixAccumulator* matrices)
{
    static std::set<BaseForceField*> hasEmittedWarning;
    if (hasEmittedWarning.insert(this).second)
    {
        dmsg_warning() << "buildStiffnessMatrix not implemented: for compatibility reason, the "
            "deprecated API (addKToMatrix) will be used. This compatibility will disapear in the "
            "future, and will cause issues in simulations. Please update the code of " <<
            this->getClassName() << " to ensure right behavior: the function addKToMatrix "
            "has been replaced by buildStiffnessMatrix";
    }

    MatrixAccessorCompat accessor;
    accessor.setDoPrintInfo(true);

    AddToMatrixCompatMatrix<BaseForceField> mat;
    mat.component = this;
    mat.matrices = matrices;

    accessor.setGlobalMatrix(&mat);

    MechanicalParams params;
    params.setKFactor(1.);
    params.setMFactor(1.);

    addKToMatrix(&params, &accessor);
}

bool BaseForceField::insertInNode( objectmodel::BaseNode* node )
{
    node->addForceField(this);
    Inherit1::insertInNode(node);
    return true;
}

bool BaseForceField::removeInNode( objectmodel::BaseNode* node )
{
    node->removeForceField(this);
    Inherit1::removeInNode(node);
    return true;
}

} // namespace sofa::core::behavior

#pragma once
#include <sofa/core/behavior/BaseForceField.h>
#include <sofa/core/behavior/BaseInteractionForceField.h>
#include <sofa/simulation/taskflow/StateGroupVisitor.h>

namespace sofa::simulation::taskflow
{

struct SOFA_SIMULATION_CORE_API AddForceVisitor final : public StateGroupVisitor<core::behavior::BaseForceField, core::behavior::BaseInteractionForceField>
{
    AddForceVisitor(const sofa::core::MechanicalParams *mparams, sofa::core::MultiVecDerivId res) :
        StateGroupVisitor(mparams),
        m_res(res), m_mparams(mparams)
    {}

    void apply(core::behavior::BaseForceField* component) override
    {
        component->addForce(this->m_mparams, this->m_res);
    }

    void apply(core::behavior::BaseInteractionForceField* component) override
    {
        component->addForce(this->m_mparams, this->m_res);
    }

private:
    sofa::core::MultiVecDerivId m_res;
    const sofa::core::MechanicalParams* m_mparams { nullptr };
};


}

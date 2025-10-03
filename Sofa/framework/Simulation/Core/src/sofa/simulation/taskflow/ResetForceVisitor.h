#pragma once

#include <sofa/core/behavior/BaseMechanicalState.h>
#include <sofa/simulation/config.h>
#include <sofa/simulation/taskflow/ComponentVisitor.h>

namespace sofa::simulation::taskflow
{

struct SOFA_SIMULATION_CORE_API ResetForceVisitor final : public ComponentVisitor<core::behavior::BaseMechanicalState>
{
    explicit ResetForceVisitor(const sofa::core::ExecParams* params, sofa::core::MultiVecDerivId res)
        : ComponentVisitor(params)
        , m_res(res)
    {}

    void apply(core::behavior::BaseMechanicalState* state) override
    {
        state->resetForce(this->m_params, m_res.getId(state));
    }

private:
    sofa::core::MultiVecDerivId m_res;
};

}

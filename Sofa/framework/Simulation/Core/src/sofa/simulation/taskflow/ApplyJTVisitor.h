#pragma once

#include <sofa/simulation/taskflow/MappingVisitor.h>

namespace sofa::simulation::taskflow
{

struct SOFA_SIMULATION_CORE_API ApplyJTVisitor : public MappingVisitor<VisitDirection::Backward>
{
    using MappingVisitor::MappingVisitor;

    ApplyJTVisitor(const sofa::core::MechanicalParams *mparams, sofa::core::MultiVecDerivId res) :
        MappingVisitor(mparams),
        m_res(res), m_mparams(mparams)
    {}

    void apply(core::BaseMapping* mapping) override
    {
        mapping->applyJT(m_mparams, m_res, m_res);
    }

private:
    sofa::core::MultiVecDerivId m_res;
    const sofa::core::MechanicalParams* m_mparams { nullptr };
};

}

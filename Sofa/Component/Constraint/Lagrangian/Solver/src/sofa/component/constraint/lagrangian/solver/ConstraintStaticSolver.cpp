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
#include <sofa/component/constraint/lagrangian/solver/ConstraintStaticSolver.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/ScopedAdvancedTimer.h>
#include <sofa/simulation/MechanicalOperations.h>
#include <sofa/simulation/VectorOperations.h>

namespace sofa::component::constraint::lagrangian::solver
{

void registerConstraintStaticSolver(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(
        core::ObjectRegistrationData("Static solver with lagrangian constraints")
            .add<ConstraintStaticSolver>());
}

ConstraintStaticSolver::ConstraintStaticSolver()
    : l_newtonSolver(initLink("newtonSolver", "Link to a NewtonRaphsonSolver"))
    , l_linearSystem(initLink("linearSystem", "Link to a LinearSystem"))
{}

struct ConstraintStaticResidualFunction : odesolver::backward::newton_raphson::BaseNonLinearFunction
{
    void evaluateCurrentGuess() override
    {
        SCOPED_TIMER("ComputeForce");
        static constexpr bool clearForcesBeforeComputingThem = true;
        static constexpr bool applyBottomUpMappings = true;

        mop.computeForce(force, clearForcesBeforeComputingThem, applyBottomUpMappings);
        force.peq(lambda); // F + H^T.lambda
        mop.projectResponse(force);
    }

    SReal squaredNormLastEvaluation() override { return force.dot(force); }

    void computeGradientFromCurrentGuess() override
    {
        SCOPED_TIMER("MBKBuild");

        static constexpr core::MatricesFactors::M m(0);
        static constexpr core::MatricesFactors::B b(0);
        static constexpr core::MatricesFactors::K k(-1);

        //matrix assembly
        linearSystem->setConstraintParams(&mop.cparams);
        mop.setSystemMBKMatrix(m, b, k, linearSolver);
    }

    void updateGuessFromLinearSolution(SReal alpha) override
    {
        // x += alpha dx
        x.peq(dx, alpha);
        mop.solveConstraint(x, sofa::core::ConstraintOrder::POS);
        mop.propagateX(x);

        //compute H^T.lambda
    }

    void solveLinearEquation() override
    {
        SCOPED_TIMER("MBKSolve");

        linearSolver->setSystemLHVector(dx);
        linearSolver->setSystemRHVector(force);
        linearSolver->solveSystem();
    }

    SReal squaredNormDx() override { return dx.dot(dx); }

    SReal squaredLastEvaluation() override { return x.dot(x); }

    sofa::simulation::common::MechanicalOperations& mop;
    core::behavior::MultiVecCoord& x;
    core::behavior::MultiVecDeriv& force;
    core::behavior::MultiVecDeriv& dx;
    core::behavior::MultiVecDeriv& lambda; // H^T.lambda
    BaseLagrangianConstraintLinearSystem* linearSystem{nullptr};
    core::behavior::LinearSolver* linearSolver{nullptr};

    ConstraintStaticResidualFunction(sofa::simulation::common::MechanicalOperations& mop,
                                     core::behavior::MultiVecCoord& x,
                                     core::behavior::MultiVecDeriv& force,
                                     core::behavior::MultiVecDeriv& dx,
                                     core::behavior::MultiVecDeriv& lambda,
                                     BaseLagrangianConstraintLinearSystem* linearSystem,
                                     core::behavior::LinearSolver* linearSolver)
        : mop(mop),
          x(x),
          force(force),
          dx(dx),
          lambda(lambda),
          linearSystem(linearSystem),
          linearSolver(linearSolver)
    {}
};

void ConstraintStaticSolver::init()
{
    OdeSolver::init();
    LinearSolverAccessor::init();

    if (!l_newtonSolver.get())
    {
        l_newtonSolver.set(getContext()->get<odesolver::backward::NewtonRaphsonSolver>(getContext()->getTags(), core::objectmodel::BaseContext::SearchDown));

        if (!l_newtonSolver)
        {
            msg_warning() << "A Newton-Raphson solver is required by this component but has not been found. It will be created automatically";
            auto newtonRaphsonSolver = core::objectmodel::New<odesolver::backward::NewtonRaphsonSolver>();
            newtonRaphsonSolver->setName(this->getContext()->getNameHelper().resolveName(newtonRaphsonSolver->getClassName(), core::ComponentNameHelper::Convention::xml));
            this->getContext()->addObject(newtonRaphsonSolver);
            l_newtonSolver.set(newtonRaphsonSolver);
        }
    }

    if (!l_linearSystem.get())
    {
        l_linearSystem.set(getContext()->get<BaseLagrangianConstraintLinearSystem>(getContext()->getTags(), core::objectmodel::BaseContext::SearchDown));

        if (!l_linearSystem)
        {
            const std::string compatibleComponents = core::ObjectFactory::getInstance()->listClassesDerivedFrom<BaseLagrangianConstraintLinearSystem>();
            msg_error() << "A compatible linear system has not been found. Here is the list compatible components: " << compatibleComponents;
            this->d_componentState.setValue(core::objectmodel::ComponentState::Invalid);
            return;
        }
    }

    //TODO: make sure the linear system and the linear solver are using each other

    simulation::common::VectorOperations vop(sofa::core::execparams::defaultInstance(), this->getContext());

    // allocate a VecId to store H^T.lambda in each state
    sofa::core::behavior::MultiVecDeriv lambda(&vop, m_lambdaId);
    lambda.realloc(&vop, false, true, core::VecIdProperties{"H^T.lambda", GetClass()->className});
    m_lambdaId = lambda.id();

    if (!this->isComponentStateInvalid())
    {
        d_componentState.setValue(core::objectmodel::ComponentState::Valid);
    }
}
void ConstraintStaticSolver::solve(const core::ExecParams* params, SReal dt,
                                   core::MultiVecCoordId x, core::MultiVecDerivId v)
{
    if (!isComponentStateValid())
    {
        return;
    }

    SOFA_UNUSED(dt);
    SOFA_UNUSED(v);

    // Create the vector and mechanical operations tools. These are used to execute special
    // operations (multiplication, additions, etc.) on multi-vectors (a vector that is stored
    // in different buffers inside the mechanical objects)
    sofa::simulation::common::VectorOperations vop(params, this->getContext());
    sofa::simulation::common::MechanicalOperations mop(params, this->getContext());
    mop->setImplicit(true);

    core::behavior::MultiVecCoord pos(&vop, x);
    core::behavior::MultiVecDeriv force(&vop, sofa::core::vec_id::write_access::force);

    core::behavior::MultiVecDeriv dx(&vop, core::vec_id::write_access::dx);
    dx.realloc(&vop, true, true);

    core::behavior::MultiVecDeriv lambda(&vop, m_lambdaId);
    lambda.realloc(&vop, false, true);

    mop.cparams.setLambda(lambda);

    SCOPED_TIMER("StaticSolver::Solve");

    ConstraintStaticResidualFunction staticResidualFunction(mop, pos, force, dx, lambda, l_linearSystem.get(), l_linearSolver.get());
    l_newtonSolver->solve(staticResidualFunction);
}


}  // namespace sofa::component::constraint::lagrangian::solver

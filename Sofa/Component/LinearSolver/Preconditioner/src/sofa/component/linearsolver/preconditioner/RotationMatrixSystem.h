#pragma once
#include <sofa/component/linearsolver/preconditioner/config.h>
#include <sofa/component/linearsystem/MatrixLinearSystem.h>
namespace sofa::component::linearsolver::preconditioner
{

template<class TMatrix, class TVector>
class RotationMatrixSystem : public linearsystem::MatrixLinearSystem<TMatrix, TVector>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE2(RotationMatrixSystem, TMatrix, TVector), SOFA_TEMPLATE2(linearsystem::MatrixLinearSystem, TMatrix, TVector));
};
}

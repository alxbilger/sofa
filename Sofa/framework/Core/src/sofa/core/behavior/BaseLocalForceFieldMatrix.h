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
#include <sofa/type/fwd.h>
#include <sofa/core/MatrixAccumulator.h>
#include <sofa/core/behavior/BaseForceField.h>
#include <sofa/core/MechanicalStatesMatrixAccumulators.h>

namespace sofa::core::behavior
{
class SOFA_CORE_API StiffnessMatrixAccumulator : public virtual MatrixAccumulatorInterface {};
class SOFA_CORE_API ListStiffnessMatrixAccumulator : public ListMatrixAccumulator<StiffnessMatrixAccumulator>{};

class SOFA_CORE_API DampingMatrixAccumulator : public virtual MatrixAccumulatorInterface {};
class SOFA_CORE_API ListDampingMatrixAccumulator : public ListMatrixAccumulator<DampingMatrixAccumulator>{};

} //namespace sofa::core::behavior

namespace sofa::core::matrixaccumulator
{
template<>
struct get_abstract_strong<Contribution::STIFFNESS>
{
    using type = behavior::StiffnessMatrixAccumulator;
    using ComponentType = behavior::BaseForceField;
};

template<>
struct get_abstract_strong<Contribution::DAMPING>
{
    using type = behavior::DampingMatrixAccumulator;
    using ComponentType = behavior::BaseForceField;
};

template<>
struct get_list_abstract_strong<Contribution::STIFFNESS>
{
    using type = behavior::ListStiffnessMatrixAccumulator;
    using ComponentType = behavior::BaseForceField;
};

template<>
struct get_list_abstract_strong<Contribution::DAMPING>
{
    using type = behavior::ListDampingMatrixAccumulator;
    using ComponentType = behavior::BaseForceField;
};

} //namespace sofa::core::matrixaccumulator


namespace sofa::core::behavior
{

template<matrixaccumulator::Contribution c>
class ForceDerivativeMatrix : public MechanicalStatesMatrixAccumulators<c>
{
public:
    using MatrixAccumulator = typename MechanicalStatesMatrixAccumulators<c>::MatrixAccumulator;

    struct ForceDerivativeElement
    {
        ForceDerivativeElement(sofa::SignedIndex _row, sofa::SignedIndex _col, MatrixAccumulator* _mat)
            : row(_row), col(_col), mat(_mat)
        {}
        void operator+=(const float value) const { mat->add(row, col, value); }
        void operator+=(const double value) const { mat->add(row, col, value); }

        template<sofa::Size L, sofa::Size C, class real>
        void operator+=(const sofa::type::Mat<L, C, real> & value) const { mat->matAdd(row, col, value); }

        void operator+=(const sofa::type::Mat<1, 1, float> & value) const { mat->add(row, col, value); }
        void operator+=(const sofa::type::Mat<1, 1, double>& value) const { mat->add(row, col, value); }
        void operator+=(const sofa::type::Mat<2, 2, float> & value) const { mat->add(row, col, value); }
        void operator+=(const sofa::type::Mat<2, 2, double>& value) const { mat->add(row, col, value); }
        void operator+=(const sofa::type::Mat<3, 3, float> & value) const { mat->add(row, col, value); }
        void operator+=(const sofa::type::Mat<3, 3, double>& value) const { mat->add(row, col, value); }

        [[nodiscard]] bool isValid() const { return mat != nullptr; }
        operator bool() const { return isValid(); }

    private:
        sofa::SignedIndex row;
        sofa::SignedIndex col;
        MatrixAccumulator* mat { nullptr };
    };

    struct ForceDerivative
    {
        ForceDerivativeElement operator()(sofa::SignedIndex row, sofa::SignedIndex col) const
        {
            return ForceDerivativeElement{row, col, mat};
        }

        ForceDerivative(BaseMechanicalState* _mstate1,
             BaseMechanicalState* _mstate2,
             ForceDerivativeMatrix* _mat)
        : mstate1(_mstate1)
        , mstate2(_mstate2)
        , mat(_mat->m_submatrix[{_mstate1, _mstate2}])
        {}

        [[nodiscard]] bool isValid() const { return mat != nullptr; }
        operator bool() const { return isValid(); }

        void checkValidity(const objectmodel::BaseObject* object) const
        {
            msg_error_when(!isValid() || !mstate1 || !mstate2, object)
                << "The force derivative in mechanical state '"
                << (mstate1 ? mstate1->getPathName() : "null")
                << "' with respect to state variable in mechanical state '"
                << (mstate2 ? mstate2->getPathName() : "null")
                << "' is invalid";
        }

    private:
        BaseMechanicalState* mstate1 { nullptr };
        BaseMechanicalState* mstate2 { nullptr };
        MatrixAccumulator* mat { nullptr };
    };

};


class SOFA_CORE_API StiffnessMatrix
    : public ForceDerivativeMatrix<matrixaccumulator::Contribution::STIFFNESS>
{
public:

    struct DF
    {
        DF(BaseMechanicalState* _mstate1, StiffnessMatrix* _mat)
            : mstate1(_mstate1), mat(_mat) {}

        ForceDerivative withRespectToPositionsIn(BaseMechanicalState* mstate2) const
        {
            return ForceDerivative{this->mstate1, mstate2, this->mat};
        }

    private:
        BaseMechanicalState* mstate1 { nullptr };
        StiffnessMatrix* mat { nullptr };
    };

    DF getForceDerivativeIn(BaseMechanicalState* mstate)
    {
        return DF{mstate, this};
    }
};

class SOFA_CORE_API DampingMatrix
    : public ForceDerivativeMatrix<matrixaccumulator::Contribution::DAMPING>
{
public:

    struct DF
    {
        DF(BaseMechanicalState* _mstate1, DampingMatrix* _mat)
            : mstate1(_mstate1), mat(_mat) {}

        ForceDerivative withRespectToVelocityIn(BaseMechanicalState* mstate2) const
        {
            return ForceDerivative{this->mstate1, mstate2, this->mat};
        }

    private:
        BaseMechanicalState* mstate1 { nullptr };
        DampingMatrix* mat { nullptr };
    };

    DF getForceDerivativeIn(BaseMechanicalState* mstate)
    {
        return DF{mstate, this};
    }
};

}

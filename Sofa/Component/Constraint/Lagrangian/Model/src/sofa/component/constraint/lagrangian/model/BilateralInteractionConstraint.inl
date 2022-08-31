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
#include <sofa/component/constraint/lagrangian/model/BilateralInteractionConstraint.h>
#include <sofa/type/Vec.h>
#include <sofa/helper/accessor.h>

#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>

#include <sofa/core/visual/VisualParams.h>

namespace sofa::component::constraint::lagrangian::model::bilateralinteractionconstraint
{
using sofa::type::Vec;
using sofa::helper::WriteAccessor ;
using sofa::core::objectmodel::KeypressedEvent ;
using sofa::core::objectmodel::Event ;


template<class DataTypes>
void BilateralInteractionConstraint<DataTypes>::reset(){
    init();
}


template<class DataTypes>
void BilateralInteractionConstraint<DataTypes>::reinit()
{
    prevForces.clear();
    activated = (activateAtIteration.getValue() >= 0 && activateAtIteration.getValue() <= iteration);
}

template<class DataTypes>
BilateralInteractionConstraint<DataTypes>::BilateralInteractionConstraint(MechanicalState* object1, MechanicalState* object2)
    : Inherit(object1, object2)
    , m1(initData(&m1, "first_point","index of the constraint on the first model"))
    , m2(initData(&m2, "second_point","index of the constraint on the second model"))
    , restVector(initData(&restVector, "rest_vector","Relative position to maintain between attached points (optional)"))

    , d_numericalTolerance(initData(&d_numericalTolerance, static_cast<Real>(0.0001), "numericalTolerance",
                                    "a real value specifying the tolerance during the constraint solving. (optional, default=0.0001)") )

    //TODO(dmarchal): Such kind of behavior shouldn't be implemented in the component but externalized in a second component or in a python script controlling the scene.
    , activateAtIteration( initData(&activateAtIteration, 0, "activateAtIteration", "activate constraint at specified interation (0 = always enabled, -1=disabled)"))

    //TODO(dmarchal): what do TEST means in the following ? should it be renamed (EXPERIMENTAL FEATURE) and when those Experimental feature will become official feature ?
    , merge(initData(&merge,false, "merge", "TEST: merge the bilateral constraints in a unique constraint"))
    , derivative(initData(&derivative,false, "derivative", "TEST: derivative"))
    , keepOrientDiff(initData(&keepOrientDiff,false, "keepOrientationDifference", "keep the initial difference in orientation (only for rigids)"))
{
    this->f_listening.setValue(true);
}

template<class DataTypes>
BilateralInteractionConstraint<DataTypes>::BilateralInteractionConstraint(MechanicalState* object)
    : BilateralInteractionConstraint(object, object)
{
}

template<class DataTypes>
BilateralInteractionConstraint<DataTypes>::BilateralInteractionConstraint()
    : BilateralInteractionConstraint(nullptr, nullptr)
{
}

template<class DataTypes>
void BilateralInteractionConstraint<DataTypes>::init()
{
    /// Do general check of validity for inputs
    Inherit1::init();

    /// Using assert means that the previous lines have check that there is two valid mechanical state.
    assert(this->mstate1);
    assert(this->mstate2);

    prevForces.clear();
    iteration = 0;
    activated = (activateAtIteration.getValue() >= 0 && activateAtIteration.getValue() <= iteration);
}


template<class DataTypes>
void BilateralInteractionConstraint<DataTypes>::clear(int reserve)
{
    WriteAccessor<Data<type::vector<int> > > wm1 = this->m1;
    WriteAccessor<Data<type::vector<int> > > wm2 = this->m2;
    WriteAccessor<Data<VecDeriv > > wrest = this->restVector;
    wm1.clear();
    wm2.clear();
    wrest.clear();
    if (reserve)
    {
        wm1.reserve(reserve);
        wm2.reserve(reserve);
        wrest.reserve(reserve);
    }
}

template<class DataTypes>
void BilateralInteractionConstraint<DataTypes>::getConstraintResolution(const ConstraintParams* cParams,
                                                                        std::vector<ConstraintResolution*>& resTab,
                                                                        unsigned int& offset)
{
    SOFA_UNUSED(cParams);
    unsigned minp=std::min(m1.getValue().size(),m2.getValue().size());

    if (!merge.getValue())
    {
        prevForces.resize(minp);
        for (unsigned pid=0; pid<minp; pid++)
        {
            resTab[offset] = new BilateralConstraintResolution3Dof(&prevForces[pid]);
            offset += 3;
        }
    }
    else
    {
        prevForces.resize(1);
        resTab[offset] = new BilateralConstraintResolution3Dof(&prevForces[0]);
        offset +=3;
    }
}

template<class DataTypes>
void BilateralInteractionConstraint<DataTypes>::buildConstraintMatrix(const ConstraintParams*, DataMatrixDeriv &c1_d, DataMatrixDeriv &c2_d, unsigned int &constraintId
                                                                      , const DataVecCoord &/*x1*/, const DataVecCoord &/*x2*/)
{
    if (!activated)
        return;

    unsigned minp = std::min(m1.getValue().size(), m2.getValue().size());
    if (minp == 0)
        return;

    const type::vector<int> &m1Indices = m1.getValue();
    const type::vector<int> &m2Indices = m2.getValue();

    MatrixDeriv &c1 = *c1_d.beginEdit();
    MatrixDeriv &c2 = *c2_d.beginEdit();

    cid.resize(minp);

    const VecDeriv& restVector = this->restVector.getValue();

    if (!merge.getValue())
    {
        for (unsigned pid=0; pid<minp; pid++)
        {
            int tm1 = m1Indices[pid];
            int tm2 = m2Indices[pid];

            constexpr type::Vec<3, Real> cx(1,0,0), cy(0,1,0), cz(0,0,1);

            cid[pid] = constraintId;
            constraintId += 3;

            MatrixDerivRowIterator c1_it = c1.writeLine(cid[pid]);
            c1_it.addCol(tm1, -cx);

            MatrixDerivRowIterator c2_it = c2.writeLine(cid[pid]);
            c2_it.addCol(tm2, cx);

            c1_it = c1.writeLine(cid[pid] + 1);
            c1_it.setCol(tm1, -cy);

            c2_it = c2.writeLine(cid[pid] + 1);
            c2_it.setCol(tm2, cy);

            c1_it = c1.writeLine(cid[pid] + 2);
            c1_it.setCol(tm1, -cz);

            c2_it = c2.writeLine(cid[pid] + 2);
            c2_it.setCol(tm2, cz);
        }
    }
    else
    {
        this->m_constraintIndex.setValue(constraintId);


        ///////////////// grouped constraints ///////////////
        dfree_square_total.clear();

        const DataVecCoord &d_x1 = *this->mstate1->read(ConstVecCoordId::position());
        const DataVecCoord &d_x2 = *this->mstate2->read(ConstVecCoordId::position());

        const VecCoord &x1 = d_x1.getValue();
        const VecCoord &x2 = d_x2.getValue();

        for (unsigned pid=0; pid<minp; pid++)
        {
            int tm1 = m1Indices[pid];
            int tm2 = m2Indices[pid];

            Deriv dfree_loc = x2[tm2] - x1[tm1];

            if (pid < restVector.size())
                dfree_loc -= restVector[pid];

            dfree_square_total[0]+= dfree_loc[0]*dfree_loc[0];
            dfree_square_total[1]+= dfree_loc[1]*dfree_loc[1];
            dfree_square_total[2]+= dfree_loc[2]*dfree_loc[2];
        }

        for (unsigned int i=0; i<3; i++)
        {
            if (dfree_square_total[i]>1.0e-15)
            {
                dfree_square_total[i] = sqrt(dfree_square_total[i]);
                squareXYZ[i]=derivative.getValue();
            }
            else
                squareXYZ[i]=false;
        }

        dfree.resize(minp);

        const DataVecCoord &d_x1free = *this->mstate1->read(ConstVecCoordId::freePosition());
        const DataVecCoord &d_x2free = *this->mstate2->read(ConstVecCoordId::freePosition());

        const VecCoord &x1free = d_x1free.getValue();
        const VecCoord &x2free = d_x2free.getValue();

        for (unsigned pid=0; pid<minp; pid++)
        {
            int tm1 = m1Indices[pid];
            int tm2 = m2Indices[pid];

            Deriv d_loc = x2[tm2] - x1[tm1];
            Deriv dfree_loc = x2free[tm2] - x1free[tm1];

            if (pid < restVector.size())
            {
                d_loc -= restVector[pid];
                dfree_loc -= restVector[pid];
            }
            dfree[pid] = dfree_loc;

            constexpr type::Vec<3, Real> cx(1.0,0,0), cy(0,1.0,0), cz(0,0,1.0);

            cid[pid] = constraintId;


            // if not grouped constraint
            // constraintId += 3;

            // contribution along x axis
            MatrixDerivRowIterator c1_it = c1.writeLine(cid[pid]);
            MatrixDerivRowIterator c2_it = c2.writeLine(cid[pid]);
            if(squareXYZ[0])
            {
                c1_it.addCol(tm1, -cx*dfree_loc[0]*2.0);
                c2_it.addCol(tm2, cx*dfree_loc[0]*2.0);
            }
            else
            {
                c1_it.addCol(tm1, -cx*sofa::helper::sign(dfree_loc[0]) );
                c2_it.addCol(tm2, cx*sofa::helper::sign(dfree_loc[0]));
            }


            // contribution along y axis
            c1_it = c1.writeLine(cid[pid] + 1);
            c2_it = c2.writeLine(cid[pid] + 1);
            if(squareXYZ[1])
            {

                c1_it.addCol(tm1, -cy*dfree_loc[1]*2.0);
                c2_it.addCol(tm2, cy*dfree_loc[1]*2.0);
            }
            else
            {
                c1_it.addCol(tm1, -cy*sofa::helper::sign(dfree_loc[1]));
                c2_it.addCol(tm2, cy*sofa::helper::sign(dfree_loc[1]));
            }

            // contribution along z axis
            c1_it = c1.writeLine(cid[pid] + 2);
            c2_it = c2.writeLine(cid[pid] + 2);
            if(squareXYZ[2])
            {
                c1_it.addCol(tm1, -cz*dfree_loc[2]*2.0);
                c2_it.addCol(tm2, cz*dfree_loc[2]*2.0);
            }
            else
            {
                c1_it.addCol(tm1, -cz*sofa::helper::sign(dfree_loc[2]));
                c2_it.addCol(tm2, cz*sofa::helper::sign(dfree_loc[2]));
            }
        }

        // if grouped constraint
        constraintId += 3;
    }

    c1_d.endEdit();
    c2_d.endEdit();
}




//TODO(dmarchal): implementing keyboard interaction behavior directly in a component is not a valid
//design for a component. Interaction should be defered to an independent Component implemented in the SofaInteraction
//a second possibility is to implement this behavir using script.
template<class DataTypes>
void BilateralInteractionConstraint<DataTypes>::handleEvent(Event *event)
{
    if (KeypressedEvent::checkEventType(event))
    {
        KeypressedEvent *ev = static_cast<KeypressedEvent *>(event);
        switch(ev->getKey())
        {

        case 'A':
        case 'a':
            msg_info() << "Activating constraint" ;
            activated = true;
            break;
        }
    }


    if (simulation::AnimateEndEvent::checkEventType(event) )
    {
        ++iteration;
        if (!activated && activateAtIteration.getValue() >= 0 && activateAtIteration.getValue() <= iteration)
        {
            msg_info() << "Activating constraint" ;
            activated = true;
        }
    }
}

template<class DataTypes>
void BilateralInteractionConstraint<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    if (!vparams->displayFlags().getShowInteractionForceFields()) return;

    vparams->drawTool()->saveLastState();
    vparams->drawTool()->disableLighting();

    constexpr sofa::type::RGBAColor colorActive = sofa::type::RGBAColor::magenta();
    constexpr sofa::type::RGBAColor colorNotActive = sofa::type::RGBAColor::green();
    std::vector< sofa::type::Vector3 > vertices;

    unsigned minp = std::min(m1.getValue().size(),m2.getValue().size());
    auto positionsM1 = sofa::helper::getReadAccessor(*this->mstate1->read(ConstVecCoordId::position()));
    auto positionsM2 = sofa::helper::getReadAccessor(*this->mstate2->read(ConstVecCoordId::position()));
    auto indicesM1 = sofa::helper::getReadAccessor(m1);
    auto indicesM2 = sofa::helper::getReadAccessor(m2);

    for (unsigned i=0; i<minp; i++)
    {
        vertices.push_back(DataTypes::getCPos(positionsM1[indicesM1[i]]));
        vertices.push_back(DataTypes::getCPos(positionsM2[indicesM2[i]]));
    }

    vparams->drawTool()->drawPoints(vertices, 10, (activated) ? colorActive : colorNotActive);

    vparams->drawTool()->restoreLastState();
}


}

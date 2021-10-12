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

#include "MechanicalMatrixMapper.h"
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/rmath.h>
#include <sofa/helper/AdvancedTimer.h>
#include <sofa/helper/ScopedAdvancedTimer.h>
#include <sofa/linearalgebra/SparseMatrixProduct[EigenSparseMatrix].h>

// accumulate jacobian

#include <sofa/core/objectmodel/BaseContext.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/behavior/BaseMass.h>
#include <sofa/defaulttype/MapMapSparseMatrix.h>
#include <sofa/linearalgebra/TripletMatrix.h>

#include <sofa/simulation/mechanicalvisitor/MechanicalResetConstraintVisitor.h>
using sofa::simulation::mechanicalvisitor::MechanicalResetConstraintVisitor;

// verify timing
#include <sofa/helper/system/thread/CTime.h>

//  Eigen Sparse Matrix
#include <Eigen/Sparse>

#include <sofa/simulation/Node.h>


namespace sofa::component::interactionforcefield
{
template <typename TDataTypes1, typename TDataTypes2>
void MechanicalMatrixMapper<TDataTypes1, TDataTypes2>::computeMatrixProduct(
    const bool fastProduct,
    linearalgebra::SparseMatrixProduct<Eigen::SparseMatrix<double> >& product_1,
    linearalgebra::SparseMatrixProduct<Eigen::SparseMatrix<double> >& product_2,
    const Eigen::SparseMatrix<double>* J1, const Eigen::SparseMatrix<double>* J2,
    const Eigen::SparseMatrix<double>* K,
    Eigen::SparseMatrix<double>*& output)
{
    const Eigen::SparseMatrix<double> Jt = J1->transpose();

    if (fastProduct)
    {
        product_1.matrixA = &Jt;
        product_1.matrixB = K;
        product_1.computeProduct();

        product_2.matrixA = &product_1.getProductResult();
        product_2.matrixB = J2;
        product_2.computeProduct();

        output = const_cast<Eigen::SparseMatrix<double>*>(&product_2.getProductResult());
    }
    else
    {
        output->resize(Jt.rows(), J2->cols());
        *output = Jt * (*K) * (*J2);
    }
}

template<class DataTypes1, class DataTypes2>
MechanicalMatrixMapper<DataTypes1, DataTypes2>::MechanicalMatrixMapper()
    :
      d_forceFieldList(initData(&d_forceFieldList,"forceFieldList","List of ForceField Names to work on (by default will take all)")),
      l_nodeToParse(initLink("nodeToParse","link to the node on which the component will work, from this link the mechanicalState/mass/forceField links will be made")),
      d_stopAtNodeToParse(initData(&d_stopAtNodeToParse,false,"stopAtNodeToParse","Boolean to choose whether forceFields in children Nodes of NodeToParse should be considered.")),
      d_skipJ1tKJ1(initData(&d_skipJ1tKJ1,false,"skipJ1tKJ1","Boolean to choose whether to skip J1tKJ1 to avoid 2 contributions, in case 2 MechanicalMatrixMapper are used")),
      d_skipJ2tKJ2(initData(&d_skipJ2tKJ2,false,"skipJ2tKJ2","Boolean to choose whether to skip J2tKJ2 to avoid 2 contributions, in case 2 MechanicalMatrixMapper are used")),
      d_fastMatrixProduct(initData(&d_fastMatrixProduct, true, "fastMatrixProduct", "If true, an accelerated method to compute matrix products based on the pre-computation of the matrices intersection is used. Regular matrix product otherwise.")),
      l_mechanicalState(initLink("mechanicalState","The mechanicalState with which the component will work on (filled automatically during init)")),
      l_mappedMass(initLink("mass","mass with which the component will work on (filled automatically during init)")),
      l_forceField(initLink("forceField","The ForceField(s) attached to this node (filled automatically during init)"))
{
}

template<class DataTypes1, class DataTypes2>
void MechanicalMatrixMapper<DataTypes1, DataTypes2>::init()
{
    if(this->d_componentState.getValue() == ComponentState::Valid){
        msg_warning() << "Calling an already fully initialized component. You should use reinit instead." ;
    }

    if(l_nodeToParse.get() == nullptr)
    {
        msg_error() << " failed to initialized -> missing/wrong link " << l_nodeToParse.getName() << " : " << l_nodeToParse.getLinkedPath();
        this->d_componentState.setValue(ComponentState::Invalid) ;
        return;
    }

    sofa::core::behavior::BaseInteractionForceField::init();

    if (mstate1.get() == nullptr || mstate2.get() == nullptr)
    {
        msg_error() << " failed to initialized -> missing/wrong link " << mstate1.getName() << " or " << mstate2.getName();
        this->d_componentState.setValue(ComponentState::Invalid) ;
        return;
    }


    // Add link to mass & and get mass component name to rm it from forcefields
    std::string massName;
    if (l_nodeToParse.get()->mass)
    {
        l_mappedMass.add(l_nodeToParse.get()->mass,l_nodeToParse.get()->mass->getPathName());
        massName.append(l_nodeToParse.get()->mass->getName());
    }

    // Add link to  mechanical
    if (l_nodeToParse.get()->mechanicalState)
    {
        l_mechanicalState.add(l_nodeToParse.get()->mechanicalState,l_nodeToParse.get()->mechanicalState->getPathName());
    }
    else
    {
        msg_error() << ": no mechanical object to link to for this node path: " << l_nodeToParse.getPath();
        this->d_componentState.setValue(ComponentState::Invalid) ;
        return;
    }

    // Parse l_nodeToParse to find & link with the forcefields
    parseNode(l_nodeToParse.get(),massName);
    m_nbInteractionForceFields = l_nodeToParse.get()->interactionForceField.size();

    if (l_forceField.size() == 0)
    {
        msg_warning() << ": no forcefield to link to for this node path: " << l_nodeToParse.getPath();
    }

    auto ms1 = this->getMState1();
    auto ms2 = this->getMState2();
    m_nbColsJ1 = ms1->getSize()*DerivSize1;
    m_nbColsJ2 = ms2->getSize()*DerivSize2;

    this->d_componentState.setValue(ComponentState::Valid) ;
}

template<class DataTypes1, class DataTypes2>
void MechanicalMatrixMapper<DataTypes1, DataTypes2>::bwdInit()
{
    m_fullMatrixSize = l_mechanicalState.get()->getMatrixSize();
    m_J1eig.resize(m_fullMatrixSize, m_nbColsJ1);
    m_J2eig.resize(m_fullMatrixSize, m_nbColsJ2);
}

template<class DataTypes1, class DataTypes2>
void MechanicalMatrixMapper<DataTypes1, DataTypes2>::parseNode(sofa::simulation::Node *node,std::string massName)
{
    bool empty = d_forceFieldList.getValue().empty();
    msg_info() << "parsing node:";
    for(BaseForceField* forcefield : node->forceField)
    {
        if (forcefield->name.getValue() != massName)
        {
            bool found = true;
            if (!empty)
                found = (std::find(d_forceFieldList.getValue().begin(),
                                   d_forceFieldList.getValue().end(), forcefield->getName()) != d_forceFieldList.getValue().end());

            if(found)
            {
                l_forceField.add(forcefield,forcefield->getPathName());
            }
        }
    }

    for(BaseForceField* iforcefield : node->interactionForceField)
    {
        bool found = true;
        if (!empty)
            found = (std::find(d_forceFieldList.getValue().begin(),
                               d_forceFieldList.getValue().end(),
                               iforcefield->getName()) != d_forceFieldList.getValue().end());

        if(found)
        {
            l_forceField.add(iforcefield,iforcefield->getPathName());
        }

    }
    if (!d_stopAtNodeToParse.getValue())
        for(auto& child : node->child){
            parseNode(child.get(), massName);
        }
    return;
}

template<class DataTypes1, class DataTypes2>
void MechanicalMatrixMapper<DataTypes1, DataTypes2>::buildIdentityBlocksInJacobian(core::behavior::BaseMechanicalState* mstate, sofa::core::MatrixDerivId Id)
{
    sofa::type::vector<unsigned int> list;
    for (unsigned int i=0; i<mstate->getSize(); i++)
        list.push_back(i);
    mstate->buildIdentityBlocksInJacobian(list, Id);
}

template<class DataTypes1, class DataTypes2>
void MechanicalMatrixMapper<DataTypes1, DataTypes2>::accumulateJacobiansOptimized(const MechanicalParams* mparams)
{
    this->accumulateJacobians(mparams);
}

template<class DataTypes1, class DataTypes2>
void MechanicalMatrixMapper<DataTypes1, DataTypes2>::accumulateJacobians(const MechanicalParams* mparams)
{
    const core::ExecParams* eparams = dynamic_cast<const core::ExecParams *>( mparams );
    core::ConstraintParams cparams = core::ConstraintParams(*eparams);

    sofa::core::MatrixDerivId Id= sofa::core::MatrixDerivId::mappingJacobian();
    core::objectmodel::BaseContext* context = this->getContext();
    simulation::Node* gnode = dynamic_cast<simulation::Node*>(context);
    MechanicalResetConstraintVisitor(&cparams).execute(context);
    buildIdentityBlocksInJacobian(l_mechanicalState,Id);

    MechanicalAccumulateJacobian(&cparams, core::MatrixDerivId::mappingJacobian()).execute(gnode);
}

template<class T>
void copyKToEigenFormat(linearalgebra::TripletMatrix< T >* K, Eigen::SparseMatrix<double,Eigen::ColMajor>& Keig)
{
    Keig.setFromTriplets(K->getTripletList().begin(), K->getTripletList().end());
}

template<class InputFormat>
static void copyMappingJacobianToEigenFormat(const typename InputFormat::MatrixDeriv& J, Eigen::SparseMatrix<double>& Jeig)
{
    typedef typename InputFormat::MatrixDeriv::RowConstIterator RowConstIterator;
    typedef typename InputFormat::MatrixDeriv::ColConstIterator ColConstIterator;
    typedef typename InputFormat::Deriv Deriv;
    int DerivSize = InputFormat::Deriv::total_size;
    int nbRowsJ = Jeig.rows();
    int maxRowIndex = 0, maxColIndex = 0;
    std::vector< Eigen::Triplet<double> > tripletListJ;

    for (RowConstIterator rowIt = J.begin(); rowIt !=  J.end(); ++rowIt)
    {
        int rowIndex = rowIt.index();
        if (rowIndex>maxRowIndex)
            maxRowIndex = rowIndex;
        for (ColConstIterator colIt = rowIt.begin(); colIt !=  rowIt.end(); ++colIt)
        {
            int colIndex = colIt.index();
            Deriv elemVal = colIt.val();
            for (int i=0;i<DerivSize;i++)
            {
                tripletListJ.push_back(Eigen::Triplet<double>(rowIndex,DerivSize*colIndex + i,elemVal[i]));
                if (colIndex>maxColIndex)
                        maxColIndex = colIndex;
            }
        }
    }
    Jeig.resize(nbRowsJ,DerivSize*(maxColIndex+1));
    Jeig.reserve(J.size());
    Jeig.setFromTriplets(tripletListJ.begin(), tripletListJ.end());
}
template<class DataTypes1, class DataTypes2>
void MechanicalMatrixMapper<DataTypes1, DataTypes2>::optimizeAndCopyMappingJacobianToEigenFormat1(const typename DataTypes1::MatrixDeriv& J, Eigen::SparseMatrix<double>& Jeig)
{
    copyMappingJacobianToEigenFormat<DataTypes1>(J, Jeig);
}

template<class DataTypes1, class DataTypes2>
void MechanicalMatrixMapper<DataTypes1, DataTypes2>::optimizeAndCopyMappingJacobianToEigenFormat2(const typename DataTypes2::MatrixDeriv& J, Eigen::SparseMatrix<double>& Jeig)
{
    copyMappingJacobianToEigenFormat<DataTypes2>(J, Jeig);
}

template<class DataTypes1, class DataTypes2>
void MechanicalMatrixMapper<DataTypes1, DataTypes2>::addMassToSystem(const MechanicalParams* mparams, const DefaultMultiMatrixAccessor* KAccessor)
{
    if (l_mappedMass != nullptr)
    {
        l_mappedMass->addMToMatrix(mparams, KAccessor);
    }
    else
    {
        msg_info() << "There is no mappedMass";
    }
}

template<class DataTypes1, class DataTypes2>
void MechanicalMatrixMapper<DataTypes1, DataTypes2>::addPrecomputedMassToSystem(const MechanicalParams* mparams, const unsigned int mstateSize,const Eigen::SparseMatrix<double> &Jeig, Eigen::SparseMatrix<double> &JtKJeig)
{
    SOFA_UNUSED(mparams);
    SOFA_UNUSED(mstateSize);
    SOFA_UNUSED(Jeig);
    SOFA_UNUSED(JtKJeig);
}

template<class DataTypes1, class DataTypes2>
void MechanicalMatrixMapper<DataTypes1, DataTypes2>::addKToMatrix(const MechanicalParams* mparams,
                                                                        const MultiMatrixAccessor* matrix)
{
    if(this->d_componentState.getValue() != ComponentState::Valid)
        return ;

    sofa::helper::ScopedAdvancedTimer addKToMatrixTimer("MMM-addKToMatrix");

    sofa::core::behavior::MechanicalState<DataTypes1>* ms1 = this->getMState1();
    sofa::core::behavior::MechanicalState<DataTypes2>* ms2 = this->getMState2();

    sofa::core::behavior::BaseMechanicalState*  bms1 = this->getMechModel1();
    sofa::core::behavior::BaseMechanicalState*  bms2 = this->getMechModel2();

    MultiMatrixAccessor::MatrixRef mat11 = matrix->getMatrix(mstate1);
    MultiMatrixAccessor::MatrixRef mat22 = matrix->getMatrix(mstate2);
    MultiMatrixAccessor::InteractionMatrixRef mat12 = matrix->getMatrix(mstate1, mstate2);
    MultiMatrixAccessor::InteractionMatrixRef mat21 = matrix->getMatrix(mstate2, mstate1);

    ///////////////////////////     STEP 1      ////////////////////////////////////
    /* -------------------------------------------------------------------------- */
    /*              compute jacobians using generic implementation                */
    /* -------------------------------------------------------------------------- */
    sofa::helper::AdvancedTimer::stepBegin("jacobian" );
    accumulateJacobiansOptimized(mparams);
    sofa::helper::AdvancedTimer::stepEnd("jacobian" );

    ///////////////////////////     STEP 2      ////////////////////////////////////
    /* -------------------------------------------------------------------------- */
    /*  compute the stiffness K of the forcefield and put it in a rowsparseMatrix */
    /*          get the stiffness matrix from the mapped ForceField               */
    /* TODO: use the template of the FF for Real                                  */
    /* -------------------------------------------------------------------------- */
    sofa::helper::AdvancedTimer::stepBegin("stiffness" );

    ///////////////////////     GET K       ////////////////////////////////////////
    // CompressedRowSparseMatrix< Real1 >* K = new CompressedRowSparseMatrix< Real1 > ( );
    linearalgebra::TripletMatrix<Real1> K;
    K.resize( m_fullMatrixSize ,  m_fullMatrixSize );
    DefaultMultiMatrixAccessor* KAccessor;
    KAccessor = new DefaultMultiMatrixAccessor;
    KAccessor->addMechanicalState( l_mechanicalState );
    KAccessor->setGlobalMatrix(&K);
    KAccessor->setupMatrices();

    sofa::simulation::Node *node = l_nodeToParse.get();

    if (node->meshTopology)
    {
        if (const auto currentRevision = node->meshTopology->getRevision(); currentRevision != m_topologyRevision)
        {
            //the topology has been modified: intersection is no longer valid
            m_product_J1tK.invalidateIntersection();
            m_product_J2tK.invalidateIntersection();
            m_product_J1tKJ1.invalidateIntersection();
            m_product_J2tKJ2.invalidateIntersection();
            m_product_J1tKJ2.invalidateIntersection();
            m_product_J2tKJ1.invalidateIntersection();
            m_topologyRevision = currentRevision;
        }
    }

    size_t currentNbInteractionFFs = node->interactionForceField.size();
    msg_info() << "nb m_nbInteractionForceFields :" << m_nbInteractionForceFields << msgendl << "nb currentNbInteractionFFs :" << currentNbInteractionFFs;
    if (m_nbInteractionForceFields != currentNbInteractionFFs)
    {
        bool emptyForceFieldList = l_forceField.empty();
        if (!emptyForceFieldList)
        {
            while(l_forceField.size()>0)
            {
                l_forceField.removeAt(0);
            }

        }
        std::string massName;
        if (l_nodeToParse.get()->mass)
            massName.append(l_nodeToParse.get()->mass->getName());
        parseNode(l_nodeToParse.get(),massName);
        m_nbInteractionForceFields = currentNbInteractionFFs;
    }

    for(unsigned int i=0; i<l_forceField.size(); i++)
    {
        l_forceField[i]->addKToMatrix(mparams, KAccessor);
    }

    addMassToSystem(mparams,KAccessor);

    sofa::helper::AdvancedTimer::stepEnd("stiffness" );

    //------------------------------------------------------------------------------

    sofa::helper::AdvancedTimer::stepBegin("copyKToEigen" );
    Eigen::SparseMatrix<double,Eigen::ColMajor> Keig;
    Keig.resize(m_fullMatrixSize,m_fullMatrixSize);
    copyKToEigenFormat(&K,Keig);
    sofa::helper::AdvancedTimer::stepEnd("copyKToEigen" );


    ///////////////////////    COPY J1 AND J2 IN EIGEN FORMAT //////////////////////////////////////
    sofa::helper::AdvancedTimer::stepBegin("copyJ1J2ToEigen" );
    sofa::core::MultiMatrixDerivId c = sofa::core::MatrixDerivId::mappingJacobian();
    const MatrixDeriv1 &J1 = c[ms1].read()->getValue();
    const MatrixDeriv2 &J2 = c[ms2].read()->getValue();

    optimizeAndCopyMappingJacobianToEigenFormat1(J1, m_J1eig);
    if (bms1 != bms2)
    {
        sofa::helper::ScopedAdvancedTimer copyJ2Timer("copyJ2ToEigen" );
        optimizeAndCopyMappingJacobianToEigenFormat2(J2, m_J2eig);
    }
    sofa::helper::AdvancedTimer::stepEnd("copyJ1J2ToEigen" );

    ///////////////////////////     STEP 4      ////////////////////////////////////
    /* -------------------------------------------------------------------------- */
    /*          perform the multiplication with [J1t J2t] * K * [J1 J2]           */
    /* -------------------------------------------------------------------------- */
    sofa::helper::AdvancedTimer::stepBegin("Multiplication" );
    const auto fastProduct = d_fastMatrixProduct.getValue();

    m_nbColsJ1 = m_J1eig.cols();
    if (bms1 != bms2)
    {
        m_nbColsJ2 = m_J2eig.cols();
    }
    Eigen::SparseMatrix<double>* J1tKJ1eigen{ &m_J1tKJ1eigen };

    if (!d_skipJ1tKJ1.getValue())
    {
        sofa::helper::ScopedAdvancedTimer J1tKJ1Timer("J1tKJ1" );
        computeMatrixProduct(fastProduct, m_product_J1tK, m_product_J1tKJ1, &m_J1eig, &m_J1eig, &Keig, J1tKJ1eigen);
    }

    Eigen::SparseMatrix<double>* J2tKJ2eigen{ &m_J2tKJ2eigen };
    Eigen::SparseMatrix<double>* J1tKJ2eigen{ &m_J1tKJ2eigen };
    Eigen::SparseMatrix<double>* J2tKJ1eigen{ &m_J2tKJ1eigen };

    if (bms1 != bms2)
    {
        if (!d_skipJ2tKJ2.getValue())
        {
            sofa::helper::ScopedAdvancedTimer J2tKJ2Timer("J2tKJ2" );
            computeMatrixProduct(fastProduct, m_product_J2tK, m_product_J2tKJ2, &m_J2eig, &m_J2eig, &Keig, J2tKJ2eigen);
        }
        {
            sofa::helper::ScopedAdvancedTimer J1tKJ2Timer("J1tKJ2" );
            computeMatrixProduct(fastProduct, m_product_J1tK, m_product_J1tKJ2, &m_J1eig, &m_J2eig, &Keig, J1tKJ2eigen);
        }
        {
            sofa::helper::ScopedAdvancedTimer J2tKJ1Timer("J2tKJ1" );
            computeMatrixProduct(fastProduct, m_product_J2tK, m_product_J2tKJ1, &m_J2eig, &m_J1eig, &Keig, J2tKJ1eigen);
        }

    }

    sofa::helper::AdvancedTimer::stepEnd("Multiplication" );
    //--------------------------------------------------------------------------------------------------------------------

    unsigned int mstateSize = l_mechanicalState->getSize();
    addPrecomputedMassToSystem(mparams,mstateSize,m_J1eig,*J1tKJ1eigen);

    const auto copyMatrixProduct = [](Eigen::SparseMatrix<double>* src, BaseMatrix* dst, const int offrow, const int offcol, const std::string stepName)
    {
        if (src)
        {
            sofa::helper::ScopedAdvancedTimer copyTimer(stepName );
            for (int k = 0; k < src->outerSize(); ++k)
            {
                for (Eigen::SparseMatrix<double>::InnerIterator it(*src, k); it; ++it)
                {
                    dst->add(offrow + it.row(),offcol + it.col(), it.value());
                }
            }
        }
    };

    copyMatrixProduct(J1tKJ1eigen, mat11.matrix, mat11.offset, mat11.offset, "J1tKJ1-copy");

    if (bms1 != bms2)
    {
        copyMatrixProduct(J2tKJ2eigen, mat22.matrix, mat22.offset, mat22.offset, "J2tKJ2-copy");
        copyMatrixProduct(J1tKJ2eigen, mat12.matrix, mat12.offRow, mat12.offCol, "J1tKJ2-copy");
        copyMatrixProduct(J2tKJ1eigen, mat21.matrix, mat21.offRow, mat21.offCol, "J2tKJ1-copy");
    }

    delete KAccessor;

    const core::ExecParams* eparams = dynamic_cast<const core::ExecParams *>( mparams );
    core::ConstraintParams cparams = core::ConstraintParams(*eparams);

    core::objectmodel::BaseContext* context = this->getContext();
    MechanicalResetConstraintVisitor(&cparams).execute(context);

}

// Even though it does nothing, this method has to be implemented
// since it's a pure virtual in parent class
template<class DataTypes1, class DataTypes2>
void MechanicalMatrixMapper<DataTypes1, DataTypes2>::addForce(const MechanicalParams* mparams,
                                                              DataVecDeriv1& f1,
                                                              DataVecDeriv2& f2,
                                                              const DataVecCoord1& x1,
                                                              const DataVecCoord2& x2,
                                                              const DataVecDeriv1& v1,
                                                              const DataVecDeriv2& v2)
{
    SOFA_UNUSED(mparams);
    SOFA_UNUSED(f1);
    SOFA_UNUSED(f2);
    SOFA_UNUSED(x1);
    SOFA_UNUSED(x2);
    SOFA_UNUSED(v1);
    SOFA_UNUSED(v2);
}

// Even though it does nothing, this method has to be implemented
// since it's a pure virtual in parent class
template<class DataTypes1, class DataTypes2>
void MechanicalMatrixMapper<DataTypes1, DataTypes2>::addDForce(const MechanicalParams* mparams,
                                                               DataVecDeriv1& df1,
                                                               DataVecDeriv2& df2,
                                                               const DataVecDeriv1& dx1,
                                                               const DataVecDeriv2& dx2)
{
    SOFA_UNUSED(mparams);
    SOFA_UNUSED(df1);
    SOFA_UNUSED(df2);
    SOFA_UNUSED(dx1);
    SOFA_UNUSED(dx2);
}

// Even though it does nothing, this method has to be implemented
// since it's a pure virtual in parent class
template<class DataTypes1, class DataTypes2>
double MechanicalMatrixMapper<DataTypes1, DataTypes2>::getPotentialEnergy(const MechanicalParams* mparams,
                                                                          const DataVecCoord1& x1,
                                                                          const DataVecCoord2& x2) const
{
    SOFA_UNUSED(mparams);
    SOFA_UNUSED(x1);
    SOFA_UNUSED(x2);

    return 0.0;
}

} // namespace sofa::component::interactionforcefield
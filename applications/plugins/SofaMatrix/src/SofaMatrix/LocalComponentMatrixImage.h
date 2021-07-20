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
#include <SofaMatrix/config.h>

#include <sofa/core/objectmodel/BaseObject.h>
#include <SofaMatrix/BaseMatrixImageProxy.h>
#include <sofa/core/BaseMatrixAccumulatorComponent.h>
#include <sofa/simulation/AnimateEndEvent.h>

namespace sofa::component::linearsystem
{

template<class ComponentType, class TMatrix>
class SOFA_SOFAMATRIX_API LocalComponentMatrixImage : public core::BaseMatrixAccumulatorComponent<ComponentType>
{
public:
    SOFA_CLASS(LocalComponentMatrixImage, core::BaseMatrixAccumulatorComponent<ComponentType>);

protected:

    /// Explicit instantiation of the local matrix.
    TMatrix m_localMatrix;

    LocalComponentMatrixImage();
    ~LocalComponentMatrixImage() override;

    void init() override;
    void handleEvent(core::objectmodel::Event *event) override;

    Data< sofa::type::BaseMatrixImageProxy > d_bitmap; ///< A proxy to visualize the produced image in the GUI through a DataWidget

public:

    void add(sofa::SignedIndex row, sofa::SignedIndex col, float value) override
    {
        m_localMatrix.add(row, col, value);
    }
    void add(sofa::SignedIndex row, sofa::SignedIndex col, double value) override
    {
        m_localMatrix.add(row, col, value);
    }

    void add(sofa::SignedIndex row, sofa::SignedIndex col, const sofa::type::Mat<3, 3, float>& value) override
    {
        m_localMatrix.add(row, col, value);
    }
    void add(sofa::SignedIndex row, sofa::SignedIndex col, const sofa::type::Mat<3, 3, double>& value) override
    {
        m_localMatrix.add(row, col, value);
    }

    void clear() override
    {
        m_localMatrix.clear();
    }
};

template <class ComponentType, class TMatrix>
LocalComponentMatrixImage<ComponentType, TMatrix>::LocalComponentMatrixImage() : Inherit1()
    , d_bitmap(initData(&d_bitmap, type::BaseMatrixImageProxy(), "bitmap", "Visualization of the representation of the matrix as a binary image. White pixels are zeros, black pixels are non-zeros."))
{
    d_bitmap.setGroup("Image");
    d_bitmap.setWidget("matrixbitmap"); //the widget used to display the image is registered in a factory with the key 'matrixbitmap'
    d_bitmap.setReadOnly(true);

    this->f_listening.setValue(true);
}

template <class ComponentType, class TMatrix>
LocalComponentMatrixImage<ComponentType, TMatrix>::~LocalComponentMatrixImage()
{
}

template <class ComponentType, class TMatrix>
void LocalComponentMatrixImage<ComponentType, TMatrix>::init()
{
    core::BaseMatrixAccumulatorComponent<ComponentType>::init();

    if (this->l_associatedComponent)
    {
        const auto matrixSize = this->l_associatedComponent->getContext()->getMechanicalState()->getMatrixSize();
        m_localMatrix.resize(matrixSize, matrixSize);

        //added as a slave of the component. The component will call the MatrixAccumulator
        //interface in order to fill the matrix.
        this->l_associatedComponent->addSlave(this);
        this->d_componentState.setValue(sofa::core::objectmodel::ComponentState::Valid);
    }
    else
    {
        msg_error() << "The attribute '" << this->l_associatedComponent.getName() << "' must be set to a valid component";
        this->d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
    }
}

template <class ComponentType, class TMatrix>
void LocalComponentMatrixImage<ComponentType, TMatrix>::handleEvent(core::objectmodel::Event* event)
{
    core::BaseMatrixAccumulatorComponent<ComponentType>::handleEvent(event);

    if (simulation::AnimateEndEvent::checkEventType(event))
    {
        // even if the pointer to the matrix stays the same, the write accessor leads to an update of the widget
        auto& bitmap = *helper::getWriteOnlyAccessor(d_bitmap);
        bitmap.setMatrix(&m_localMatrix);
    }
}
} //namespace sofa::component::linearsystem

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
#include <sofa/core/BaseMatrixAccumulatorComponent.h>
#include <sofa/simulation/BaseSimulationExporter.h>
#include <sofa/core/behavior/LinearSolver.h>
#include <sofa/defaulttype/MatrixExporter.h>
#include <sofa/helper/OptionsGroup.h>

namespace sofa::component::linearsystem
{

template<core::matrixaccumulator::Contribution c, class TMatrix>
class LocalComponentMatrixExporter :
    public sofa::core::BaseMatrixAccumulatorComponent<c>,
    public sofa::simulation::BaseSimulationExporter
{
public:
    SOFA_CLASS2(LocalComponentMatrixExporter, sofa::core::BaseMatrixAccumulatorComponent<c>, sofa::simulation::BaseSimulationExporter);

    bool write() override
    {
        if (this->l_associatedComponent)
        {
            const std::string basename = getOrCreateTargetPath(d_filename.getValue(),
                                                               d_exportEveryNbSteps.getValue());

            const auto selectedExporter = d_fileFormat.getValue().getSelectedItem();
            const auto exporter = sofa::defaulttype::matrixExporterMap.find(selectedExporter);
            if (exporter != sofa::defaulttype::matrixExporterMap.end())
            {
                const std::string filename = basename + "." + exporter->first;
                msg_info() << "Writing local system matrix from component '" << this->l_associatedComponent->getName() << "' in " << filename;
                return exporter->second(filename, &m_localMatrix, d_precision.getValue());
            }
        }
        return false;
    }

    void doInit() override
    {
        if (this->l_associatedComponent)
        {
            const auto matrixSize = this->l_associatedComponent->getContext()->getMechanicalState()->getMatrixSize();
            m_localMatrix.resize(matrixSize, matrixSize);

            //the exporter is added as a slave of the component. The component will call the MatrixAccumulator
            //interface in order to fill the matrix.
            this->l_associatedComponent->addSlave(this);
        }
        else
        {
            msg_error() << "The attribute '" << this->l_associatedComponent.getName() << "' must be set to a valid component";
        }
    }

    static bool canCreate(LocalComponentMatrixExporter<c, TMatrix>* obj, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg)
    {
        if (auto* linearSolver = context->get<sofa::core::behavior::LinearSolver >())
        {
            //checking that the matrix type is the same for the linear system and the exporter
            if (linearSolver->getTemplateName() == LocalComponentMatrixExporter<c, TMatrix>::GetClass()->templateName)
            {
                return sofa::core::BaseMatrixAccumulatorComponent<c>::canCreate(obj, context, arg);
            }
        }
        return false;
    }

protected:
    Data<sofa::helper::OptionsGroup> d_fileFormat;
    Data<int> d_precision; ///< Number of digits used to write an entry of the matrix, default is 6

    LocalComponentMatrixExporter()
    : Inherit1(), Inherit2()
    , d_fileFormat(initData(&d_fileFormat, sofa::defaulttype::matrixExporterOptionsGroup, "format", "File format"))
    , d_precision(initData(&d_precision, 6, "precision", "Number of digits used to write an entry of the matrix, default is 6"))
    {
        d_exportAtBegin.setReadOnly(true);
        d_exportAtEnd.setReadOnly(true);

        d_exportAtBegin.setDisplayed(false);
        d_exportAtEnd.setDisplayed(false);
    }

    /// Explicit instantiation of the local matrix.
    TMatrix m_localMatrix;

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
}

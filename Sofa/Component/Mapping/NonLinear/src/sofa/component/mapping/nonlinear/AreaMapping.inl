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
#include <sofa/component/mapping/nonlinear/AreaMapping.h>
#include <sofa/linearalgebra/CompressedRowSparseMatrixConstraintEigenUtils.h>

namespace sofa::component::mapping::nonlinear
{

using sofa::type::dyad;
using sofa::type::dot;

template <class TIn, class TOut>
AreaMapping<TIn, TOut>::AreaMapping()
    : d_computeRestArea(initData(&d_computeRestArea, false, "computeRestArea", "if 'computeRestArea = true', then rest area of each element equal 0, otherwise rest area is the initial area of each of them"))
    , d_restArea(initData(&d_restArea, "restArea", "Rest area of the surface primitives"))
    , l_topology(initLink("topology", "link to the topology container"))
{}

template <class TIn, class TOut>
auto AreaMapping<TIn, TOut>::computeSecondDerivativeArea(
    const sofa::type::fixed_array<sofa::type::Vec3, 3>& triangleVertices) ->
    sofa::type::Mat<3, 3, sofa::type::Mat<3, 3, Real> >
{
    const auto& v = triangleVertices;

    const auto N = sofa::type::cross(v[1] - v[0], v[2] - v[0]);
    const auto n2 = dot(N, N);

    sofa::type::MatNoInit<3, 3, sofa::type::MatNoInit<3, 3, Real>> d2A;

    const auto ka = 1 / (2 * std::sqrt(std::pow(n2, 3)));

    static constexpr auto skewSign = type::crossProductMatrix(sofa::type::Vec<3, Real>{1,1,1});

    for (unsigned int i = 0; i < 3; ++i)
    {
        for (unsigned int j = 0; j < 3; ++j)
        {
            auto& entry = d2A[i][j];

            const auto i1 = (i + 1) % 3;
            const auto j1 = (j + 1) % 3;
            const auto i2 = (i + 2) % 3;
            const auto j2 = (j + 2) % 3;

            const auto N_cross_Pi1Pi2 = N.cross(v[i1] - v[i2]);
            const auto N_cross_Pj1Pj2 = N.cross(v[j1] - v[j2]);

            const auto outer_a = dyad(N_cross_Pi1Pi2, N_cross_Pj1Pj2);
            static const auto& id = sofa::type::Mat<3, 3, SReal>::Identity();

            const auto dot_product = dot(v[i1] - v[i2], v[j1] - v[j2]);
            const auto outer_b = dyad(v[j1] - v[j2], v[i1] - v[i2]);

            entry = - outer_a + n2 * (dot_product * id - outer_b);

            if (i != j) // diagonal blocks are skipped because skewSign[i][j] == 0
            {
                const auto sign = skewSign[i][j];
                entry += sign * n2 * type::crossProductMatrix(N);
            }

            entry *= ka;
        }
    }

    return d2A;
}

template <class TIn, class TOut>
void AreaMapping<TIn, TOut>::init()
{
    if (l_topology.empty())
    {
        msg_warning() << "link to Topology container should be set to ensure right behavior. First Topology found in current context will be used.";
        l_topology.set(this->getContext()->getMeshTopologyLink());
    }

    msg_info() << "Topology path used: '" << l_topology.getLinkedPath() << "'";

    const auto nbSurfacePrimitives = l_topology->getNbQuads() + l_topology->getNbTriangles();

    if (nbSurfacePrimitives == 0)
    {
        msg_error() << "No topology component containing surface primitives found at path: " << l_topology.getLinkedPath() << ", nor in current context: " << this->getContext()->name;
        this->d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return;
    }

    typename core::behavior::MechanicalState<In>::ReadVecCoord pos = this->getFromModel()->readPositions();

    this->getToModel()->resize( nbSurfacePrimitives );
    jacobian.resizeBlocks(nbSurfacePrimitives, pos.size());

    baseMatrices.resize( 1 );
    baseMatrices[0] = &jacobian;

    Inherit1::init();
}

template <class TIn, class TOut>
void AreaMapping<TIn, TOut>::apply(const core::MechanicalParams* mparams,
    DataVecCoord_t<Out>& out, const DataVecCoord_t<In>& in)
{
    helper::WriteOnlyAccessor< Data<VecCoord_t<Out>> > _out = out;
    helper::ReadAccessor< Data<VecCoord_t<In> > > _in = in;

    m_vertices = _in.operator->();

    const auto& triangles = l_topology->getTriangles();
    const auto& quads = l_topology->getQuads();

    jacobian.clear();

    for (unsigned int triangleId = 0; triangleId < triangles.size(); ++triangleId)
    {
        const auto& triangle = triangles[triangleId];

        const auto& n0 = TIn::getCPos(_in[triangle[0]]);
        const auto& n1 = TIn::getCPos(_in[triangle[1]]);
        const auto& n2 = TIn::getCPos(_in[triangle[2]]);

        const auto n01 = n1 - n0;
        const auto n02 = n2 - n0;
        const auto N = sofa::type::cross(n01, n02);
        const auto norm = N.norm();

        const auto area = static_cast<typename In::Real>(0.5) * norm;

        _out[triangleId] = area;

        const auto k = 1 / (2 * norm);

        sofa::type::fixed_array<JacobianEntry, 3> jacobianEntries {
            JacobianEntry{triangle[0], k * sofa::type::cross(n1-n2, N)}, // dArea_dn0
            JacobianEntry{triangle[1], k * sofa::type::cross(n02, N)}, // dArea_dn1
            JacobianEntry{triangle[2],-k * sofa::type::cross(n01, N)}, // dArea_dn2
        };

        //insertion in increasing column order
        std::sort(jacobianEntries.begin(), jacobianEntries.end());

        jacobian.beginRow(triangleId);
        for (const auto& [vertexId, jacobianValue] : jacobianEntries)
        {
            for (unsigned d = 0; d < In::spatial_dimensions; ++d)
            {
                jacobian.insertBack(triangleId, vertexId * Nin + d, jacobianValue[d]);
            }
        }
    }

    if (!quads.empty())
    {
        msg_warning() << "Quads are not supported in this Mapping. Convert them to triangles first.";
    }

    jacobian.compress();
}

template <class TIn, class TOut>
void AreaMapping<TIn, TOut>::applyJ(const core::MechanicalParams* mparams,
    DataVecDeriv_t<Out>& out, const DataVecDeriv_t<In>& in)
{
    if( jacobian.rowSize() )
    {
        auto dOutWa = sofa::helper::getWriteOnlyAccessor(out);
        auto dInRa = sofa::helper::getReadAccessor(in);
        jacobian.mult(dOutWa.wref(),dInRa.ref());
    }
}

template <class TIn, class TOut>
void AreaMapping<TIn, TOut>::applyJT(const core::MechanicalParams* mparams,
    DataVecDeriv_t<In>& out, const DataVecDeriv_t<Out>& in)
{
    if( jacobian.rowSize() )
    {
        auto dOutRa = sofa::helper::getReadAccessor(in);
        auto dInWa = sofa::helper::getWriteOnlyAccessor(out);
        jacobian.addMultTranspose(dInWa.wref(),dOutRa.ref());
    }
}

template <class TIn, class TOut>
void AreaMapping<TIn, TOut>::applyJT(const core::ConstraintParams* cparams,
    DataMatrixDeriv_t<In>& out, const DataMatrixDeriv_t<Out>& in)
{
    SOFA_UNUSED(cparams);
    auto childMatRa  = sofa::helper::getReadAccessor(in);
    auto parentMatWa = sofa::helper::getWriteAccessor(out);
    addMultTransposeEigen(parentMatWa.wref(), jacobian.compressedMatrix, childMatRa.ref());
}

template <class TIn, class TOut>
void AreaMapping<TIn, TOut>::applyDJT(const core::MechanicalParams* mparams,
    core::MultiVecDerivId parentForceId, core::ConstMultiVecDerivId childForceId)
{
    if (!m_vertices)
    {
        return;
    }

    const unsigned geometricStiffness = d_geometricStiffness.getValue().getSelectedId();
    if( !geometricStiffness )
    {
        return;
    }

    helper::WriteAccessor<Data<VecDeriv_t<In> > > parentForceAccessor(*parentForceId[this->fromModel.get()].write());
    helper::ReadAccessor<Data<VecDeriv_t<In> > > parentDisplacementAccessor(*mparams->readDx(this->fromModel.get()));
    const SReal kFactor = mparams->kFactor();
    helper::ReadAccessor<Data<VecDeriv_t<Out> > > childForceAccessor(mparams->readF(this->toModel.get()));

    if( K.compressedMatrix.nonZeros() )
    {
        K.addMult( parentForceAccessor.wref(), parentDisplacementAccessor.ref(), (typename In::Real)kFactor );
    }
    else
    {
        const auto& triangles = l_topology->getTriangles();
        for (unsigned int triangleId = 0; triangleId < triangles.size(); ++triangleId)
        {
            const auto& triangle = triangles[triangleId];

            const sofa::type::fixed_array<Coord_t<In>, 3> v{
                (*m_vertices)[triangle[0]],
                (*m_vertices)[triangle[1]],
                (*m_vertices)[triangle[2]]
            };

            //it's a 9x9 matrix, where each entry is a 3x3 matrix
            const auto d2Area_d2x = computeSecondDerivativeArea(v);

            for (unsigned int i = 0; i < 3; ++i)
            {
                for (unsigned int j = 0; j < 3; ++j)
                {
                    parentForceAccessor[triangle[i]] +=
                        kFactor
                        * d2Area_d2x[i][j]
                        * parentDisplacementAccessor[triangle[j]]
                        * childForceAccessor[triangleId][0];
                }
            }
        }
    }
}

template <class TIn, class TOut>
void AreaMapping<TIn, TOut>::updateK(const core::MechanicalParams* mparams,
    core::ConstMultiVecDerivId childForceId)
{
    SOFA_UNUSED(mparams);
    const unsigned geometricStiffness = d_geometricStiffness.getValue().getSelectedId();
    if( !geometricStiffness ) { K.resize(0,0); return; }

    helper::ReadAccessor<Data<VecDeriv_t<Out> > > childForce( *childForceId[this->toModel.get()].read() );

    {
        unsigned int kSize = this->fromModel->getSize();
        K.resizeBlocks(kSize, kSize);
    }

    const auto& triangles = l_topology->getTriangles();
    for (unsigned int triangleId = 0; triangleId < triangles.size(); ++triangleId)
    {
        const auto& triangle = triangles[triangleId];

        const sofa::type::fixed_array<Coord_t<In>, 3> v{
            (*m_vertices)[triangle[0]],
            (*m_vertices)[triangle[1]],
            (*m_vertices)[triangle[2]]
        };

        //it's a 9x9 matrix, where each entry is a 3x3 matrix
        const auto d2Area_d2x = computeSecondDerivativeArea(v);

        for (unsigned int i = 0; i < 3; ++i)
        {
            for (unsigned int j = 0; j < 3; ++j)
            {
                K.addBlock(triangle[i], triangle[j], d2Area_d2x[i][j] * childForce[triangleId][0]);
            }
        }
    }

    K.compress();
}

template <class TIn, class TOut>
const linearalgebra::BaseMatrix* AreaMapping<TIn, TOut>::getK()
{
    return &K;
}

template <class TIn, class TOut>
const type::vector<sofa::linearalgebra::BaseMatrix*>* AreaMapping<TIn, TOut>::
getJs()
{
    return &baseMatrices;
}

}
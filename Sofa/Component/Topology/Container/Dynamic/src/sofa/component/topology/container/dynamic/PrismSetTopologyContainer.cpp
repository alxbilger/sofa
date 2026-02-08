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
#include <sofa/component/topology/container/dynamic/PrismSetTopologyContainer.h>
#include <sofa/core/topology/TopologyHandler.h>
#include <sofa/core/ObjectFactory.h>
#include <algorithm>
#include <map>
#include <set>

namespace sofa::component::topology::container::dynamic
{

/**
 * Convention for Prism:
 * Triangular bases: (0,1,2) and (3,4,5)
 * Quadrilateral sides: (0,1,4,3), (1,2,5,4), (2,0,3,5)
 *
 * Nodes:
 *   5-------4
 *  / \     /|
 * 2---1   / |
 * |   |  3--|
 * |   | /   /
 * 0---|----/
 *     |   /
 *     |  /
 *     | /
 *
 * Actually, standard numbering:
 * Bottom: 0, 1, 2
 * Top: 3, 4, 5
 *
 * Edges:
 * 0: (0,1), 1: (1,2), 2: (2,0)  -- bottom
 * 3: (3,4), 4: (4,5), 5: (5,3)  -- top
 * 6: (0,3), 7: (1,4), 8: (2,5)  -- vertical
 *
 * Triangles:
 * 0: (0,2,1) -- bottom (interior orientation?) Tetra uses {1,2,3}, {0,3,2}, {1,3,0}, {0,2,1}
 * Let's use:
 * 0: (0,1,2)
 * 1: (3,5,4)
 *
 * Quads:
 * 0: (0,1,4,3)
 * 1: (1,2,5,4)
 * 2: (2,0,3,5)
 */

static const unsigned int edgesInPrismArray[9][2] = {
    {0,1}, {1,2}, {2,0}, // bottom
    {3,4}, {4,5}, {5,3}, // top
    {0,3}, {1,4}, {2,5}  // vertical
};

static const unsigned int trianglesOrientationInPrismArray[2][3] = {
    {0,2,1}, // bottom
    {3,4,5}  // top
};

static const unsigned int quadsOrientationInPrismArray[3][4] = {
    {0,1,4,3},
    {1,2,5,4},
    {2,0,3,5}
};

void registerPrismSetTopologyContainer(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(core::ObjectRegistrationData("Topology container dedicated to a prism topology.")
        .add< PrismSetTopologyContainer >());
}

PrismSetTopologyContainer::PrismSetTopologyContainer()
    : QuadSetTopologyContainer()
    , d_createTriangleArray(initData(&d_createTriangleArray, bool(false), "createTriangleArray", "Force the creation of a set of triangles associated with each prism"))
    , d_createQuadArray(initData(&d_createQuadArray, bool(false), "createQuadArray", "Force the creation of a set of quads associated with each prism"))
    , d_prism(initData(&d_prism, "prisms", "List of prism indices"))
{
    addAlias(&d_prism, "pentahedra");
}

void PrismSetTopologyContainer::addPrism(Index a, Index b, Index c, Index d, Index e, Index f)
{
    helper::WriteAccessor<Data<sofa::type::vector<Prism>>> m_prism = d_prism;
    m_prism.push_back(Prism(a, b, c, d, e, f));
    if (a >= getNbPoints()) setNbPoints(a + 1);
    if (b >= getNbPoints()) setNbPoints(b + 1);
    if (c >= getNbPoints()) setNbPoints(c + 1);
    if (d >= getNbPoints()) setNbPoints(d + 1);
    if (e >= getNbPoints()) setNbPoints(e + 1);
    if (f >= getNbPoints()) setNbPoints(f + 1);
}

void PrismSetTopologyContainer::init()
{
    core::topology::TopologyContainer::init();

    const helper::ReadAccessor<Data<sofa::type::vector<Prism>>> m_prism = d_prism;

    if (d_initPoints.isSet())
    {
        setNbPoints(Size(d_initPoints.getValue().size()));
    }
    else if (!m_prism.empty())
    {
        for (size_t i = 0; i < m_prism.size(); ++i)
        {
            for (PointID j = 0; j < 6; ++j)
            {
                const Index a = m_prism[i][j];
                if (a >= getNbPoints()) setNbPoints(a + 1);
            }
        }
    }

    if (!m_prism.empty())
        computeCrossElementBuffers();
}

void PrismSetTopologyContainer::computeCrossElementBuffers()
{
    QuadSetTopologyContainer::computeCrossElementBuffers();

    createQuadsInPrismArray();
    createTrianglesInPrismArray();
    createEdgesInPrismArray();

    createPrismsAroundQuadArray();
    createPrismsAroundTriangleArray();
    createPrismsAroundEdgeArray();
    createPrismsAroundVertexArray();
}

void PrismSetTopologyContainer::createPrismSetArray()
{
    msg_error() << "createPrismSetArray method must be implemented by a child topology.";
}

void PrismSetTopologyContainer::createEdgeSetArray()
{
    if (!hasPrisms())
        createPrismSetArray();

    if (hasEdges())
    {
        EdgeSetTopologyContainer::clear();
        m_edgesInTriangle.clear();
        m_trianglesAroundEdge.clear();
        m_edgesInQuad.clear();
        m_quadsAroundEdge.clear();
        clearEdgesInPrism();
        clearPrismsAroundEdge();
    }

    std::map<Edge, EdgeID> edgeMap;
    helper::WriteAccessor<Data<sofa::type::vector<Edge>>> m_edge = d_edge;
    const helper::ReadAccessor<Data<sofa::type::vector<Prism>>> m_prism = d_prism;

    for (size_t i = 0; i < m_prism.size(); ++i)
    {
        const Prism &p = m_prism[i];
        for (EdgeID j = 0; j < 9; ++j)
        {
            const PointID v1 = p[edgesInPrismArray[j][0]];
            const PointID v2 = p[edgesInPrismArray[j][1]];
            const Edge e((v1 < v2) ? Edge(v1, v2) : Edge(v2, v1));

            if (!edgeMap.contains(e))
            {
                const size_t edgeIndex = edgeMap.size();
                edgeMap[e] = (EdgeID)edgeIndex;
                m_edge.push_back(e);
            }
        }
    }
}

void PrismSetTopologyContainer::createEdgesInPrismArray()
{
    clearEdgesInPrism();

    if (!hasPrisms())
        createPrismSetArray();

    if (hasEdgesInPrism())
        return;

    const helper::ReadAccessor<Data<sofa::type::vector<Prism>>> m_prism = d_prism;
    const size_t numPrisms = getNumberOfPrisms();
    m_edgesInPrism.resize(numPrisms);

    if (hasEdges())
    {
        const helper::ReadAccessor<Data<sofa::type::vector<Edge>>> m_edge = d_edge;
        std::multimap<PointID, EdgeID> edgesAroundVertexMap;
        for (EdgeID edge = 0; edge < (EdgeID)m_edge.size(); ++edge)
        {
            edgesAroundVertexMap.insert(std::pair<PointID, EdgeID>(m_edge[edge][0], edge));
            edgesAroundVertexMap.insert(std::pair<PointID, EdgeID>(m_edge[edge][1], edge));
        }

        for (size_t i = 0; i < numPrisms; ++i)
        {
            const Prism &p = m_prism[i];
            for (EdgeID j = 0; j < 9; ++j)
            {
                std::pair<std::multimap<PointID, EdgeID>::iterator, std::multimap<PointID, EdgeID>::iterator> itPair = edgesAroundVertexMap.equal_range(p[edgesInPrismArray[j][0]]);
                bool foundEdge = false;
                for (auto it = itPair.first; it != itPair.second; ++it)
                {
                    const EdgeID edge = it->second;
                    if ((m_edge[edge][0] == p[edgesInPrismArray[j][0]] && m_edge[edge][1] == p[edgesInPrismArray[j][1]]) || (m_edge[edge][0] == p[edgesInPrismArray[j][1]] && m_edge[edge][1] == p[edgesInPrismArray[j][0]]))
                    {
                        m_edgesInPrism[i][j] = edge;
                        foundEdge = true;
                        break;
                    }
                }
                if (!foundEdge)
                    msg_warning() << "Cannot find edge for prism " << i << " and edge " << j;
            }
        }
    }
    else
    {
        std::map<Edge, EdgeID> edgeMap;
        helper::WriteAccessor<Data<sofa::type::vector<Edge>>> m_edge = d_edge;
        for (size_t i = 0; i < m_prism.size(); ++i)
        {
            const Prism &p = m_prism[i];
            for (EdgeID j = 0; j < 9; ++j)
            {
                const PointID v1 = p[edgesInPrismArray[j][0]];
                const PointID v2 = p[edgesInPrismArray[j][1]];
                const Edge e((v1 < v2) ? Edge(v1, v2) : Edge(v2, v1));
                if (!edgeMap.contains(e))
                {
                    edgeMap[e] = (EdgeID)m_edge.size();
                    m_edge.push_back(e);
                }
                m_edgesInPrism[i][j] = edgeMap[e];
            }
        }
    }
}

bool PrismSetTopologyContainer::hasTriangles() const { return !d_triangle.getValue().empty(); }

Size PrismSetTopologyContainer::getNumberOfTriangles() const { return (Size)d_triangle.getValue().size(); }

void PrismSetTopologyContainer::createTriangleSetArray()
{
    if (!hasPrisms())
        createPrismSetArray();

    if (this->hasTriangles())
    {
        m_trianglesAroundVertex.clear();
        m_trianglesAroundEdge.clear();
        m_edgesInTriangle.clear();
        helper::WriteAccessor<Data<sofa::type::vector<Triangle>>> m_triangle = this->d_triangle;
        m_triangle.clear();

        clearTrianglesInPrism();
        clearPrismsAroundTriangle();
    }

    std::map<Triangle, TriangleID> triangleMap;
    helper::WriteAccessor<Data<sofa::type::vector<Triangle>>> m_triangle = this->d_triangle;
    const helper::ReadAccessor<Data<sofa::type::vector<Prism>>> m_prism = d_prism;

    for (size_t i = 0; i < m_prism.size(); ++i)
    {
        const Prism &p = m_prism[i];
        for (TriangleID j = 0; j < 2; ++j)
        {
            PointID v[3];
            for (int k = 0; k < 3; ++k)
                v[k] = p[trianglesOrientationInPrismArray[j][k]];

            // Sort to handle orientation-independent matching (boundary matching handles orientation)
            PointID v_sorted[3] = {v[0], v[1], v[2]};
            std::sort(v_sorted, v_sorted + 3);
            Triangle tr_sorted(v_sorted[0], v_sorted[1], v_sorted[2]);

            if (!triangleMap.contains(tr_sorted))
            {
                triangleMap[tr_sorted] = (TriangleID)m_triangle.size();
                m_triangle.push_back(Triangle(v[0], v[1], v[2]));
            }
        }
    }
}

void PrismSetTopologyContainer::createTrianglesInPrismArray()
{
    clearTrianglesInPrism();

    if (!hasTriangles())
        createTriangleSetArray();

    if (hasTrianglesInPrism())
        return;

    m_trianglesInPrism.resize(getNumberOfPrisms());
    const helper::ReadAccessor<Data<sofa::type::vector<Prism>>> m_prism = d_prism;
    for (size_t i = 0; i < m_prism.size(); ++i)
    {
        const Prism &p = m_prism[i];
        for (TriangleID j = 0; j < 2; ++j)
        {
            const TriangleID triangleIndex = getTriangleIndex(p[trianglesOrientationInPrismArray[j][0]], p[trianglesOrientationInPrismArray[j][1]], p[trianglesOrientationInPrismArray[j][2]]);
            if (triangleIndex != InvalidID)
                m_trianglesInPrism[i][j] = triangleIndex;
            else
                msg_error() << "Cannot find triangle " << j << " in prism " << i;
        }
    }
}

void PrismSetTopologyContainer::createQuadSetArray()
{
    if (!hasPrisms())
        createPrismSetArray();

    if (hasQuads())
    {
        QuadSetTopologyContainer::clear();
        clearQuadsInPrism();
        clearPrismsAroundQuad();
    }

    std::map<Quad, QuadID> quadMap;
    helper::WriteAccessor<Data<sofa::type::vector<Quad>>> m_quad = d_quad;
    const helper::ReadAccessor<Data<sofa::type::vector<Prism>>> m_prism = d_prism;

    for (size_t i = 0; i < m_prism.size(); ++i)
    {
        const Prism &p = m_prism[i];
        for (QuadID j = 0; j < 3; ++j)
        {
            PointID v[4];
            for (int k = 0; k < 4; ++k)
                v[k] = p[quadsOrientationInPrismArray[j][k]];

            PointID v_min = v[0];
            int min_idx = 0;
            for (int k = 1; k < 4; ++k) if (v[k] < v_min) { v_min = v[k]; min_idx = k; }
            
            // Representative for map
            PointID vr[4];
            for (int k = 0; k < 4; ++k) vr[k] = v[(min_idx + k) % 4];
            if (vr[1] > vr[3]) std::swap(vr[1], vr[3]);
            Quad qr(vr[0], vr[1], vr[2], vr[3]);

            if (!quadMap.contains(qr))
            {
                quadMap[qr] = (QuadID)m_quad.size();
                m_quad.push_back(Quad(v[0], v[1], v[2], v[3]));
            }
        }
    }
}

void PrismSetTopologyContainer::createQuadsInPrismArray()
{
    clearQuadsInPrism();
    if (!hasQuads()) createQuadSetArray();
    if (hasQuadsInPrism()) return;

    m_quadsInPrism.resize(getNumberOfPrisms());
    const helper::ReadAccessor<Data<sofa::type::vector<Prism>>> m_prism = d_prism;
    for (size_t i = 0; i < m_prism.size(); ++i)
    {
        const Prism &p = m_prism[i];
        for (QuadID j = 0; j < 3; ++j)
        {
            const QuadID quadIndex = getQuadIndex(p[quadsOrientationInPrismArray[j][0]], p[quadsOrientationInPrismArray[j][1]], p[quadsOrientationInPrismArray[j][2]], p[quadsOrientationInPrismArray[j][3]]);
            if (quadIndex != InvalidID)
                m_quadsInPrism[i][j] = quadIndex;
            else
                msg_error() << "Cannot find quad " << j << " in prism " << i;
        }
    }
}

void PrismSetTopologyContainer::createPrismsAroundVertexArray()
{
    clearPrismsAroundVertex();
    m_prismsAroundVertex.resize(getNbPoints());
    const helper::ReadAccessor<Data<sofa::type::vector<Prism>>> m_prism = d_prism;
    for (size_t i = 0; i < m_prism.size(); ++i)
        for (PointID j = 0; j < 6; ++j)
            m_prismsAroundVertex[m_prism[i][j]].push_back((PrismID)i);
}

void PrismSetTopologyContainer::createPrismsAroundEdgeArray()
{
    clearPrismsAroundEdge();
    if (!hasEdgesInPrism()) createEdgesInPrismArray();
    m_prismsAroundEdge.resize(getNumberOfEdges());
    for (size_t i = 0; i < getNumberOfPrisms(); ++i)
        for (EdgeID j = 0; j < 9; ++j)
            m_prismsAroundEdge[m_edgesInPrism[i][j]].push_back((PrismID)i);
}

void PrismSetTopologyContainer::createPrismsAroundTriangleArray()
{
    clearPrismsAroundTriangle();
    if (!hasTrianglesInPrism()) createTrianglesInPrismArray();
    m_prismsAroundTriangle.resize(getNumberOfTriangles());
    for (size_t i = 0; i < getNumberOfPrisms(); ++i)
        for (TriangleID j = 0; j < 2; ++j)
            m_prismsAroundTriangle[m_trianglesInPrism[i][j]].push_back((PrismID)i);
}

void PrismSetTopologyContainer::createPrismsAroundQuadArray()
{
    clearPrismsAroundQuad();
    if (!hasQuadsInPrism()) createQuadsInPrismArray();
    m_prismsAroundQuad.resize(getNumberOfQuads());
    for (size_t i = 0; i < getNumberOfPrisms(); ++i)
        for (QuadID j = 0; j < 3; ++j)
            m_prismsAroundQuad[m_quadsInPrism[i][j]].push_back((PrismID)i);
}

const sofa::type::vector<PrismSetTopologyContainer::Prism>& PrismSetTopologyContainer::getPrismArray()
{
    return d_prism.getValue();
}

const PrismSetTopologyContainer::Prism PrismSetTopologyContainer::getPrism(PrismID i)
{
    if ((size_t)i >= getNumberOfPrisms()) return Prism(InvalidID, InvalidID, InvalidID, InvalidID, InvalidID, InvalidID);
    return d_prism.getValue()[i];
}

PrismSetTopologyContainer::PrismID PrismSetTopologyContainer::getPrismIndex(PointID v1, PointID v2, PointID v3, PointID v4, PointID v5, PointID v6)
{
    if (!hasPrismsAroundVertex()) return InvalidID;
    auto set1 = getPrismsAroundVertex(v1);
    if (set1.empty()) return InvalidID;
    PointID v[] = {v2, v3, v4, v5, v6};
    std::sort(set1.begin(), set1.end());
    for (auto id : set1)
    {
        const Prism &p = getPrism(id);
        int match = 0;
        for (int i = 0; i < 5; ++i)
            for (int j = 0; j < 6; ++j)
                if (p[j] == v[i]) { match++; break; }
        if (match == 5) return id;
    }
    return InvalidID;
}

Size PrismSetTopologyContainer::getNumberOfPrisms() const
{
    const helper::ReadAccessor<Data<sofa::type::vector<Prism>>> m_prism = d_prism;
    return (Size)m_prism.size();
}

Size PrismSetTopologyContainer::getNumberOfElements() const
{
    return getNumberOfPrisms();
}

const sofa::type::vector<PrismSetTopologyContainer::EdgesInPrism>& PrismSetTopologyContainer::getEdgesInPrismArray() { return m_edgesInPrism; }
const sofa::type::vector<PrismSetTopologyContainer::TrianglesInPrism>& PrismSetTopologyContainer::getTrianglesInPrismArray() { return m_trianglesInPrism; }
const sofa::type::vector<PrismSetTopologyContainer::QuadsInPrism>& PrismSetTopologyContainer::getQuadsInPrismArray() { return m_quadsInPrism; }
const sofa::type::vector<PrismSetTopologyContainer::PrismsAroundVertex>& PrismSetTopologyContainer::getPrismsAroundVertexArray() { return m_prismsAroundVertex; }
const sofa::type::vector<PrismSetTopologyContainer::PrismsAroundEdge>& PrismSetTopologyContainer::getPrismsAroundEdgeArray() { return m_prismsAroundEdge; }
const sofa::type::vector<PrismSetTopologyContainer::PrismsAroundTriangle>& PrismSetTopologyContainer::getPrismsAroundTriangleArray() { return m_prismsAroundTriangle; }
const sofa::type::vector<PrismSetTopologyContainer::PrismsAroundQuad>& PrismSetTopologyContainer::getPrismsAroundQuadArray() { return m_prismsAroundQuad; }

const PrismSetTopologyContainer::PrismsAroundVertex& PrismSetTopologyContainer::getPrismsAroundVertex(PointID id) { return (id < m_prismsAroundVertex.size()) ? m_prismsAroundVertex[id] : InvalidSet; }
const PrismSetTopologyContainer::PrismsAroundEdge& PrismSetTopologyContainer::getPrismsAroundEdge(EdgeID id) { return (id < m_prismsAroundEdge.size()) ? m_prismsAroundEdge[id] : InvalidSet; }
const PrismSetTopologyContainer::PrismsAroundTriangle& PrismSetTopologyContainer::getPrismsAroundTriangle(TriangleID id) { return (id < m_prismsAroundTriangle.size()) ? m_prismsAroundTriangle[id] : InvalidSet; }
const PrismSetTopologyContainer::PrismsAroundQuad& PrismSetTopologyContainer::getPrismsAroundQuad(QuadID id) { return (id < m_prismsAroundQuad.size()) ? m_prismsAroundQuad[id] : InvalidSet; }
const PrismSetTopologyContainer::EdgesInPrism& PrismSetTopologyContainer::getEdgesInPrism(PrismID id) { return (id < m_edgesInPrism.size()) ? m_edgesInPrism[id] : InvalidEdgesInPrism; }
const PrismSetTopologyContainer::TrianglesInPrism& PrismSetTopologyContainer::getTrianglesInPrism(PrismID id) { return (id < m_trianglesInPrism.size()) ? m_trianglesInPrism[id] : InvalidTrianglesInPrism; }
const PrismSetTopologyContainer::QuadsInPrism& PrismSetTopologyContainer::getQuadsInPrism(PrismID id) { return (id < m_quadsInPrism.size()) ? m_quadsInPrism[id] : InvalidQuadsInPrism; }

int PrismSetTopologyContainer::getVertexIndexInPrism(const Prism &p, PointID vertexIndex) const
{
    for (int i = 0; i < 6; ++i) if (p[i] == vertexIndex) return i;
    return -1;
}

int PrismSetTopologyContainer::getEdgeIndexInPrism(const EdgesInPrism &p, EdgeID edgeIndex) const
{
    for (int i = 0; i < 9; ++i) if (p[i] == edgeIndex) return i;
    return -1;
}

int PrismSetTopologyContainer::getTriangleIndexInPrism(const TrianglesInPrism &p, TriangleID triangleIndex) const
{
    for (int i = 0; i < 2; ++i) if (p[i] == triangleIndex) return i;
    return -1;
}

int PrismSetTopologyContainer::getQuadIndexInPrism(const QuadsInPrism &p, QuadID quadIndex) const
{
    for (int i = 0; i < 3; ++i) if (p[i] == quadIndex) return i;
    return -1;
}

PrismSetTopologyContainer::Edge PrismSetTopologyContainer::getLocalEdgesInPrism(const EdgeID i) const
{
    assert(i < 9);
    return Edge(edgesInPrismArray[i][0], edgesInPrismArray[i][1]);
}

PrismSetTopologyContainer::Triangle PrismSetTopologyContainer::getLocalTrianglesInPrism(const TriangleID i) const
{
    assert(i < 2);
    return Triangle(trianglesOrientationInPrismArray[i][0], trianglesOrientationInPrismArray[i][1], trianglesOrientationInPrismArray[i][2]);
}

PrismSetTopologyContainer::Quad PrismSetTopologyContainer::getLocalQuadsInPrism(const QuadID i) const
{
    assert(i < 3);
    return Quad(quadsOrientationInPrismArray[i][0], quadsOrientationInPrismArray[i][1], quadsOrientationInPrismArray[i][2], quadsOrientationInPrismArray[i][3]);
}

bool PrismSetTopologyContainer::checkTopology() const
{
    if (!d_checkTopology.getValue()) return true;
    // Basic checks similar to Tetrahedron/Hexahedron could be added here
    return QuadSetTopologyContainer::checkTopology();
}

bool PrismSetTopologyContainer::checkConnexity()
{
    const size_t nbr = getNumberOfPrisms();
    if (nbr == 0) return false;
    return getConnectedElement(0).size() == nbr;
}

Size PrismSetTopologyContainer::getNumberOfConnectedComponent()
{
    const auto nbr = this->getNbPrisms();

    if (nbr == 0)
    {
        return 0;
    }

    VecPrismID elemAll = this->getConnectedElement(0);
    sofa::Size cpt = 1;

    while (elemAll.size() < nbr)
    {
        std::sort(elemAll.begin(), elemAll.end());
        PrismID other_id = (PrismID)elemAll.size();

        for (PrismID i = 0; i < (PrismID)elemAll.size(); ++i)
            if (elemAll[i] != i)
            {
                other_id = i;
                break;
            }

        VecPrismID elemTmp = this->getConnectedElement(other_id);
        cpt++;

        elemAll.insert(elemAll.begin(), elemTmp.begin(), elemTmp.end());
    }

    return cpt;
}

const PrismSetTopologyContainer::VecPrismID PrismSetTopologyContainer::getConnectedElement(PrismID elem)
{
    VecPrismID elemAll;
    if (!hasPrismsAroundVertex())
    {
        return elemAll;
    }

    VecPrismID elemOnFront, elemPreviousFront, elemNextFront;
    bool end = false;
    size_t cpt = 0;
    const size_t nbr = this->getNbPrisms();

    // init algo
    elemAll.push_back(elem);
    elemOnFront.push_back(elem);
    elemPreviousFront.clear();
    cpt++;

    while (!end && cpt < nbr)
    {
        // First Step - Create new region
        elemNextFront = this->getElementAroundElements(elemOnFront); // for each PrismID on the propagation front

        // Second Step - Avoid backward direction
        for (size_t i = 0; i < elemNextFront.size(); ++i)
        {
            bool find = false;
            PrismID id = elemNextFront[i];

            for (size_t j = 0; j < elemAll.size(); ++j)
                if (id == elemAll[j])
                {
                    find = true;
                    break;
                }

            if (!find)
            {
                elemAll.push_back(id);
                elemPreviousFront.push_back(id);
            }
        }

        // cpt for connexity
        cpt += elemPreviousFront.size();

        if (elemPreviousFront.empty())
        {
            end = true;
        }

        // iterate
        elemOnFront = elemPreviousFront;
        elemPreviousFront.clear();
    }

    return elemAll;
}

const PrismSetTopologyContainer::VecPrismID PrismSetTopologyContainer::getElementAroundElement(PrismID elem)
{
    VecPrismID elems;
    if (!hasPrismsAroundVertex()) return elems;
    const Prism &p = getPrism(elem);
    std::set<PrismID> uniqueElems;
    for (int i = 0; i < 6; ++i)
    {
        const auto &around = getPrismsAroundVertex(p[i]);
        for (auto id : around) if (id != elem) uniqueElems.insert(id);
    }
    elems.assign(uniqueElems.begin(), uniqueElems.end());
    return elems;
}

const PrismSetTopologyContainer::VecPrismID PrismSetTopologyContainer::getElementAroundElements(VecPrismID elems)
{
    std::set<PrismID> uniqueElems;
    for (auto e : elems)
    {
        auto around = getElementAroundElement(e);
        for (auto id : around) uniqueElems.insert(id);
    }
    // Remove input elems
    for (auto e : elems) uniqueElems.erase(e);
    VecPrismID result;
    for (auto id : uniqueElems) result.push_back(id);
    return result;
}

bool PrismSetTopologyContainer::hasPrisms() const { d_prism.updateIfDirty(); return !d_prism.getValue().empty(); }
bool PrismSetTopologyContainer::hasEdgesInPrism() const { return !m_edgesInPrism.empty(); }
bool PrismSetTopologyContainer::hasTrianglesInPrism() const { return !m_trianglesInPrism.empty(); }
bool PrismSetTopologyContainer::hasQuadsInPrism() const { return !m_quadsInPrism.empty(); }
bool PrismSetTopologyContainer::hasPrismsAroundVertex() const { return !m_prismsAroundVertex.empty(); }
bool PrismSetTopologyContainer::hasPrismsAroundEdge() const { return !m_prismsAroundEdge.empty(); }
bool PrismSetTopologyContainer::hasPrismsAroundTriangle() const { return !m_prismsAroundTriangle.empty(); }
bool PrismSetTopologyContainer::hasPrismsAroundQuad() const { return !m_prismsAroundQuad.empty(); }

void PrismSetTopologyContainer::clearPrisms() { helper::WriteAccessor<Data<sofa::type::vector<Prism>>> m_prism = d_prism; m_prism.clear(); }
void PrismSetTopologyContainer::clearEdgesInPrism() { m_edgesInPrism.clear(); }
void PrismSetTopologyContainer::clearTrianglesInPrism() { m_trianglesInPrism.clear(); }
void PrismSetTopologyContainer::clearQuadsInPrism() { m_quadsInPrism.clear(); }
void PrismSetTopologyContainer::clearPrismsAroundVertex() { m_prismsAroundVertex.clear(); }
void PrismSetTopologyContainer::clearPrismsAroundEdge() { m_prismsAroundEdge.clear(); }
void PrismSetTopologyContainer::clearPrismsAroundTriangle() { m_prismsAroundTriangle.clear(); }
void PrismSetTopologyContainer::clearPrismsAroundQuad() { m_prismsAroundQuad.clear(); }

void PrismSetTopologyContainer::clear()
{
    clearPrismsAroundVertex(); clearPrismsAroundEdge(); clearPrismsAroundTriangle(); clearPrismsAroundQuad();
    clearEdgesInPrism(); clearTrianglesInPrism(); clearQuadsInPrism(); clearPrisms();
    QuadSetTopologyContainer::clear();
}

PrismSetTopologyContainer::PrismsAroundVertex& PrismSetTopologyContainer::getPrismsAroundVertexForModification(const PointID i) { if (!hasPrismsAroundVertex()) createPrismsAroundVertexArray(); return m_prismsAroundVertex[i]; }
PrismSetTopologyContainer::PrismsAroundEdge& PrismSetTopologyContainer::getPrismsAroundEdgeForModification(const EdgeID i) { if (!hasPrismsAroundEdge()) createPrismsAroundEdgeArray(); return m_prismsAroundEdge[i]; }
PrismSetTopologyContainer::PrismsAroundTriangle& PrismSetTopologyContainer::getPrismsAroundTriangleForModification(const TriangleID i) { if (!hasPrismsAroundTriangle()) createPrismsAroundTriangleArray(); return m_prismsAroundTriangle[i]; }
PrismSetTopologyContainer::PrismsAroundQuad& PrismSetTopologyContainer::getPrismsAroundQuadForModification(const QuadID i) { if (!hasPrismsAroundQuad()) createPrismsAroundQuadArray(); return m_prismsAroundQuad[i]; }

void PrismSetTopologyContainer::setPrismTopologyToDirty() { m_prismTopologyDirty = true; }
void PrismSetTopologyContainer::cleanPrismTopologyFromDirty() { m_prismTopologyDirty = false; }

bool PrismSetTopologyContainer::linkTopologyHandlerToData(core::topology::TopologyHandler* topologyHandler, sofa::geometry::ElementType elementType)
{
    if (elementType == sofa::geometry::ElementType::PRISM) { d_prism.addOutput(topologyHandler); return true; }
    return QuadSetTopologyContainer::linkTopologyHandlerToData(topologyHandler, elementType);
}

bool PrismSetTopologyContainer::unlinkTopologyHandlerToData(core::topology::TopologyHandler* topologyHandler, sofa::geometry::ElementType elementType)
{
    if (elementType == sofa::geometry::ElementType::PRISM) { d_prism.delOutput(topologyHandler); return true; }
    return QuadSetTopologyContainer::unlinkTopologyHandlerToData(topologyHandler, elementType);
}

} // namespace sofa::component::topology::container::dynamic

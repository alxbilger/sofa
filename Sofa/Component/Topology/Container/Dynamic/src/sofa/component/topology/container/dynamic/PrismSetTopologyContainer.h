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

#include <sofa/component/topology/container/dynamic/config.h>
#include <sofa/component/topology/container/dynamic/QuadSetTopologyContainer.h>

namespace sofa::component::topology::container::dynamic
{

class PrismSetTopologyModifier;

/** a class that stores a set of prisms and provides access with adjacent quads, triangles, edges and vertices */
class SOFA_COMPONENT_TOPOLOGY_CONTAINER_DYNAMIC_API PrismSetTopologyContainer : public QuadSetTopologyContainer
{
    friend class PrismSetTopologyModifier;

public:
    SOFA_CLASS(PrismSetTopologyContainer, QuadSetTopologyContainer);

    typedef core::topology::BaseMeshTopology::PointID                     PointID;
    typedef core::topology::BaseMeshTopology::EdgeID                      EdgeID;
    typedef core::topology::BaseMeshTopology::TriangleID                  TriangleID;
    typedef core::topology::BaseMeshTopology::QuadID                      QuadID;
    typedef core::topology::BaseMeshTopology::PrismID                     PrismID;
    typedef core::topology::BaseMeshTopology::Edge                        Edge;
    typedef core::topology::BaseMeshTopology::Triangle                    Triangle;
    typedef core::topology::BaseMeshTopology::Quad                        Quad;
    typedef core::topology::BaseMeshTopology::Prism                       Prism;
    typedef core::topology::BaseMeshTopology::SeqPrisms                   SeqPrisms;
    typedef core::topology::BaseMeshTopology::PrismsAroundVertex          PrismsAroundVertex;
    typedef core::topology::BaseMeshTopology::PrismsAroundEdge            PrismsAroundEdge;
    typedef core::topology::BaseMeshTopology::PrismsAroundTriangle        PrismsAroundTriangle;
    typedef core::topology::BaseMeshTopology::PrismsAroundQuad            PrismsAroundQuad;
    typedef core::topology::BaseMeshTopology::EdgesInPrism                EdgesInPrism;
    typedef core::topology::BaseMeshTopology::TrianglesInPrism            TrianglesInPrism;
    typedef core::topology::BaseMeshTopology::QuadsInPrism                QuadsInPrism;

    typedef sofa::type::vector<PrismID> VecPrismID;

protected:
    PrismSetTopologyContainer();

    ~PrismSetTopologyContainer() override {}

public:
    void init() override;

    /// Procedural creation methods
    /// @{
    void clear() override;
    void addPrism(Index a, Index b, Index c, Index d, Index e, Index f) override;
    /// @}

    /// BaseMeshTopology API
    /// @{

    /** \brief Returns the prisms array. */
    const SeqPrisms& getPrisms() override
    {
        return getPrismArray();
    }

    /** \brief Returns the prism corresponding to the PrismID i.
     *
     * @param i ID of a prism.
     * @return The corresponding prism.
     */
    const Prism getPrism(PrismID i) override;

    /** \brief Returns the index of a prism given six vertex indices.
     *
     * @return the ID of the corresponding prism.
     * @return InvalidID if none
     */
    PrismID getPrismIndex(PointID v1, PointID v2, PointID v3, PointID v4, PointID v5, PointID v6) override;

    /** \brief Returns the 9 edges adjacent to a given prism.
     *
     * @param id ID of a prism.
     * @return EdgesInPrism list composing the input prism.
     */
    const EdgesInPrism& getEdgesInPrism(PrismID id) override;

    /** \brief Returns the 2 triangles adjacent to a given prism.
     *
     * @param id ID of a prism.
     * @return TrianglesInPrism list composing the input prism.
     */
    const TrianglesInPrism& getTrianglesInPrism(PrismID id) override;

    /** \brief Returns the 3 quads adjacent to a given prism.
     *
     * @param id ID of a prism.
     * @return QuadsInPrism list composing the input prism.
     */
    const QuadsInPrism& getQuadsInPrism(PrismID id) override;

    /** \brief Returns the set of prisms adjacent to a given vertex.
     *
     * @param id ID of a vertex.
     * @return PrismsAroundVertex list around the input vertex.
     */
    const PrismsAroundVertex& getPrismsAroundVertex(PointID id) override;

    /** \brief Returns the set of prisms adjacent to a given edge.
     *
     * @param id ID of an edge.
     * @return PrismsAroundEdge list around the input edge.
     */
    const PrismsAroundEdge& getPrismsAroundEdge(EdgeID id) override;

    /** \brief Returns the set of prisms adjacent to a given triangle.
     *
     * @param id ID of a triangle.
     * @return PrismsAroundTriangle list around the input triangle.
     */
    const PrismsAroundTriangle& getPrismsAroundTriangle(TriangleID id) override;

    /** \brief Returns the set of prisms adjacent to a given quad.
     *
     * @param id ID of a quad.
     * @return PrismsAroundQuad list around the input quad.
     */
    const PrismsAroundQuad& getPrismsAroundQuad(QuadID id) override;

    /** \brief Returns the position (either 0 to 5) of the vertex in the prism.
     *
     * @param p Prism.
     * @param vertexIndex ID of a vertex.
     * @return position (0-5), -1 if not found.
     */
    int getVertexIndexInPrism(const Prism &p, PointID vertexIndex) const override;

    /** \brief Returns the position (either 0 to 8) of the edge in the prism.
     *
     * @param p EdgesInPrism.
     * @param edgeIndex ID of an edge.
     * @return position (0-8), -1 if not found.
     */
    int getEdgeIndexInPrism(const EdgesInPrism &p, EdgeID edgeIndex) const override;

    /** \brief Returns the position (either 0 or 1) of the triangle in the prism.
     *
     * @param p TrianglesInPrism.
     * @param triangleIndex ID of a triangle.
     * @return position (0-1), -1 if not found.
     */
    int getTriangleIndexInPrism(const TrianglesInPrism &p, TriangleID triangleIndex) const override;

    /** \brief Returns the position (either 0 to 2) of the quad in the prism.
     *
     * @param p QuadsInPrism.
     * @param quadIndex ID of a quad.
     * @return position (0-2), -1 if not found.
     */
    int getQuadIndexInPrism(const QuadsInPrism &p, QuadID quadIndex) const override;

    /** \brief Returns local vertices indices forming the i-th edge of a prism.
     */
    Edge getLocalEdgesInPrism(const EdgeID i) const override;

    /** \brief Returns local vertices indices forming the i-th triangle of a prism.
     */
    Triangle getLocalTrianglesInPrism(const TriangleID i) const override;

    /** \brief Returns local vertices indices forming the i-th quad of a prism.
     */
    Quad getLocalQuadsInPrism(const QuadID i) const override;

    /// @}

    /// Dynamic Topology API
    /// @{

    void computeCrossElementBuffers() override;

    bool checkTopology() const override;

    bool checkConnexity() override;

    Size getNumberOfConnectedComponent() override;

    const VecPrismID getConnectedElement(PrismID elem) override;

    const VecPrismID getElementAroundElement(PrismID elem) override;

    const VecPrismID getElementAroundElements(VecPrismID elems) override;

    /// @}

    Size getNumberOfPrisms() const;

    Size getNumberOfElements() const override;

    const sofa::type::vector<Prism>& getPrismArray();

    const sofa::type::vector<EdgesInPrism>& getEdgesInPrismArray();

    const sofa::type::vector<TrianglesInPrism>& getTrianglesInPrismArray();

    const sofa::type::vector<QuadsInPrism>& getQuadsInPrismArray();

    const sofa::type::vector<PrismsAroundVertex>& getPrismsAroundVertexArray();

    const sofa::type::vector<PrismsAroundEdge>& getPrismsAroundEdgeArray();

    const sofa::type::vector<PrismsAroundTriangle>& getPrismsAroundTriangleArray();

    const sofa::type::vector<PrismsAroundQuad>& getPrismsAroundQuadArray();

    bool hasPrisms() const;
    bool hasEdgesInPrism() const;
    bool hasTriangles() const;
    bool hasTrianglesInPrism() const;
    bool hasQuadsInPrism() const;
    bool hasPrismsAroundVertex() const;
    bool hasPrismsAroundEdge() const;
    bool hasPrismsAroundTriangle() const;
    bool hasPrismsAroundQuad() const;

    Size getNumberOfTriangles() const;

    /** \brief Returns the type of the topology */
    sofa::geometry::ElementType getTopologyType() const override { return sofa::geometry::ElementType::PRISM; }

    bool linkTopologyHandlerToData(core::topology::TopologyHandler* topologyHandler, sofa::geometry::ElementType elementType) override;

    bool unlinkTopologyHandlerToData(core::topology::TopologyHandler* topologyHandler, sofa::geometry::ElementType elementType) override;

protected:
    void createEdgeSetArray() override;
    void createTriangleSetArray();
    void createQuadSetArray() override;

    virtual void createPrismSetArray();
    virtual void createEdgesInPrismArray();
    virtual void createTrianglesInPrismArray();
    virtual void createQuadsInPrismArray();
    virtual void createPrismsAroundVertexArray();
    virtual void createPrismsAroundEdgeArray();
    virtual void createPrismsAroundTriangleArray();
    virtual void createPrismsAroundQuadArray();

    void clearPrisms();
    void clearEdgesInPrism();
    void clearTrianglesInPrism();
    void clearQuadsInPrism();
    void clearPrismsAroundVertex();
    void clearPrismsAroundEdge();
    void clearPrismsAroundTriangle();
    void clearPrismsAroundQuad();

    virtual PrismsAroundVertex& getPrismsAroundVertexForModification(const PointID vertexIndex);
    virtual PrismsAroundEdge& getPrismsAroundEdgeForModification(const EdgeID edgeIndex);
    virtual PrismsAroundTriangle& getPrismsAroundTriangleForModification(const TriangleID triangleIndex);
    virtual PrismsAroundQuad& getPrismsAroundQuadForModification(const QuadID quadIndex);

    void setPrismTopologyToDirty();
    void cleanPrismTopologyFromDirty();
    const bool& isPrismTopologyDirty() { return m_prismTopologyDirty; }

public:
    Data<bool> d_createTriangleArray;
    Data<bool> d_createQuadArray;

    Data<sofa::type::vector<Triangle>> d_triangle;
    Data<sofa::type::vector<Prism>> d_prism;

protected:
    sofa::type::vector<EdgesInTriangle> m_edgesInTriangle;
    sofa::type::vector<TrianglesAroundVertex> m_trianglesAroundVertex;
    sofa::type::vector<TrianglesAroundEdge> m_trianglesAroundEdge;

    sofa::type::vector<EdgesInPrism> m_edgesInPrism;
    sofa::type::vector<TrianglesInPrism> m_trianglesInPrism;
    sofa::type::vector<QuadsInPrism> m_quadsInPrism;
    sofa::type::vector<PrismsAroundVertex> m_prismsAroundVertex;
    sofa::type::vector<PrismsAroundEdge> m_prismsAroundEdge;
    sofa::type::vector<PrismsAroundTriangle> m_prismsAroundTriangle;
    sofa::type::vector<PrismsAroundQuad> m_prismsAroundQuad;

    bool m_prismTopologyDirty = false;
};

} // namespace sofa::component::topology::container::dynamic

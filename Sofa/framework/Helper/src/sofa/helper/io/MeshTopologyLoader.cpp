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
#include <sofa/helper/io/MeshTopologyLoader.h>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/type/Vec.h>
#include <cstring>

#if defined(WIN32)
#define strcasecmp stricmp
#endif


MSG_REGISTER_CLASS(sofa::helper::io::MeshTopologyLoader, "MeshTopologyLoader")

namespace sofa
{

namespace helper
{

namespace io
{

using namespace sofa::type;

bool MeshTopologyLoader::addMeshtoTopology()
{
    if (m_mesh == nullptr)
        return false;

    setNbPoints((int)m_mesh->getVertices().size());

    const auto& vertices = m_mesh->getVertices();
    const auto& edges = m_mesh->getEdges();
    const auto& triangles = m_mesh->getTriangles();
    const auto& quads = m_mesh->getQuads();
    const auto& tetra = m_mesh->getTetrahedra();
    const auto& hexa = m_mesh->getHexahedra();

    for (const auto & vertice : vertices)
        addPoint(vertice[0], vertice[1], vertice[2]);

    for (auto edge : edges)
        addLine(edge[0], edge[1]);

    for (auto triangle : triangles)
        addTriangle(triangle[0], triangle[1], triangle[2]);

    for (auto quad : quads)
        addQuad(quad[0], quad[1], quad[2], quad[3]);

    for (auto i : tetra)
        addTetra(i[0], i[1], i[2], i[3]);

    for (const auto & i : hexa)
        addCube(i[0], i[1], i[2], i[3],
            i[4], i[5], i[6], i[7]);

    return true;
}

bool MeshTopologyLoader::loadObj(const char *filename)
{
    m_mesh = helper::io::Mesh::Create(filename);
    if (m_mesh ==nullptr)
        return false;

    setNbPoints((int)m_mesh->getVertices().size());
    for (auto & i : m_mesh->getVertices())
    {
        addPoint((SReal)i[0],
                (SReal)i[1],
                (SReal)i[2]);
    }

    const auto & facets = m_mesh->getFacets();
    std::set< std::pair<int,int> > edges;
    for (const auto & i : facets)
    {
        const auto& facet = i[0];
        if (facet.size()==2)
        {
            // Line
            if (facet[0]<facet[1])
                addLine(facet[0],facet[1]);
            else
                addLine(facet[1],facet[0]);
        }
        else if (facet.size()==4)
        {
            // Quad
            addQuad(facet[0],facet[1],facet[2],facet[3]);
        }
        else
        {
            // Triangularize
            for (size_t j=2; j<facet.size(); j++)
                addTriangle(facet[0],facet[j-1],facet[j]);
        }
#if 0
        // Add edges
        if (facet.size()>2)
        {
            for (size_t j=0; j<facet.size(); j++)
            {
                int i1 = facet[j];
                int i2 = facet[(j+1)%facet.size()];
                if (edges.count(std::make_pair(i1,i2))!=0)
                {
                }
                else if (edges.count(std::make_pair(i2,i1))==0)
                {
                    if (i1>i2)
                        addLine(i1,i2);
                    else
                        addLine(i2,i1);
                    edges.insert(std::make_pair(i1,i2));
                }
            }
        }
#endif
    }

    /// delete m_mesh;
    return true;
}

bool MeshTopologyLoader::loadGmsh(const char *filename)
{
    m_mesh = helper::io::Mesh::Create("gmsh", filename);      
    return addMeshtoTopology();
}

bool MeshTopologyLoader::loadMesh(std::ifstream &file)
{
    SOFA_UNUSED(file);
    return false;
}


bool MeshTopologyLoader::loadVtk(const char *filename)
{
    m_mesh = helper::io::Mesh::Create("vtu", filename);
    return addMeshtoTopology();
}

bool MeshTopologyLoader::load(const char *filename)
{
	std::string fname(filename);
	if (!sofa::helper::system::DataRepository.findFile(fname))
	{
		msg_error() << "Cannot find file: " << filename;
		return false;
	}

    bool fileLoaded = false;

	// check the extension of the filename
	if ((strlen(filename) > 4 && !strcmp(filename + strlen(filename) - 4, ".obj"))
		|| (strlen(filename) > 6 && !strcmp(filename + strlen(filename) - 6, ".trian")))
		fileLoaded = loadObj(fname.c_str());
	else if (strlen(filename) > 4 && !strcmp(filename + strlen(filename) - 4, ".vtk"))
		fileLoaded = loadVtk(fname.c_str());
	else if (strlen(filename) > 9 && !strcmp(filename + strlen(filename) - 9, ".vtk_swap"))
		fileLoaded = loadVtk(fname.c_str());
    else if (strlen(filename) > 4 && !strcmp(filename + strlen(filename) - 4, ".msh"))
        fileLoaded = loadGmsh(fname.c_str());
	else
	{
		std::ifstream file(filename);
		if (!file.good()) return false;
		msg_error() << "This file format: " << filename << " will not be supported anymore in sofa release 18.06.";
		fileLoaded = loadMesh(file);
		file.close();
	}
       
    if(fileLoaded)
    {
        // topology has been filled, so the Mesh is not needed anymore
        assert(m_mesh);

        delete m_mesh;
        m_mesh = nullptr;
    }
    else
    {
        msg_error() << "Unable to load mesh file '" << fname << "'" ;
    }
    return fileLoaded;
}

} // namespace io

} // namespace helper

} // namespace sofa


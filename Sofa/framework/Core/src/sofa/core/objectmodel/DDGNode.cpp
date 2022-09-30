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
#include <algorithm>
#include <iostream>
#include <cassert>
#include <sofa/core/objectmodel/DDGNode.h>
#include <sofa/helper/BackTrace.h>

#include <map>

namespace sofa::core::objectmodel
{

/// Constructor
DDGNode::DDGNode()
{
}

DDGNode::~DDGNode()
{
    for(auto it : inputs)
    {
        it->doDelOutput(this);
    }
    for(auto it : outputs)
    {
        it->doDelInput(this);
    }
}

void DDGNode::setDirtyValue()
{
    bool& dirtyValue = dirtyFlags.dirtyValue;
    if (!dirtyValue)
    {
        dirtyValue = true;
        setDirtyOutputs();
    }
}

void DDGNode::setDirtyOutputs()
{
    bool& dirtyOutputs = dirtyFlags.dirtyOutputs;
    if (!dirtyOutputs)
    {
        dirtyOutputs = true;
        for(DDGLinkIterator it=outputs.begin(), itend=outputs.end(); it != itend; ++it)
        {
            (*it)->setDirtyValue();
        }
    }
}

void DDGNode::cleanDirty()
{
    bool& dirtyValue = dirtyFlags.dirtyValue;
    if (dirtyValue)
    {
        dirtyValue = false;
        cleanDirtyOutputsOfInputs();
    }
}

void DDGNode::notifyEndEdit()
{
    for(auto it : outputs)
        it->notifyEndEdit();
}

void DDGNode::cleanDirtyOutputsOfInputs()
{
    for(auto it : inputs)
        it->dirtyFlags.dirtyOutputs = false;
}

void DDGNode::addInput(DDGNode* n)
{
    if(std::find(inputs.begin(), inputs.end(), n) != inputs.end())
    {
        assert(false && "trying to add a DDGNode that is already in the input set.");
        return;
    }
    doAddInput(n);
    n->doAddOutput(this);
    setDirtyValue();
}

void DDGNode::delInput(DDGNode* n)
{
    /// It is not allowed to remove an entry that is not in the set.
    assert(std::find(inputs.begin(), inputs.end(), n) != inputs.end());

    doDelInput(n);
    n->doDelOutput(this);
}

void DDGNode::addOutput(DDGNode* n)
{
    if(std::find(outputs.begin(), outputs.end(), n) != outputs.end())
    {
        assert(false && "trying to add a DDGNode that is already in the output set.");
        return;
    }

    doAddOutput(n);
    n->doAddInput(this);
    n->setDirtyValue();
}

void DDGNode::delOutput(DDGNode* n)
{
    /// It is not allowed to remove an entry that is not in the set.
    assert(std::find(outputs.begin(), outputs.end(), n) != outputs.end());

    doDelOutput(n);
    n->doDelInput(this);
}

const DDGNode::DDGLinkContainer& DDGNode::getInputs()
{
    return inputs;
}

const DDGNode::DDGLinkContainer& DDGNode::getOutputs()
{
    return outputs;
}

class UpdateCounter
{
public:
    static UpdateCounter& getInstance()
    {
        static UpdateCounter counter;
        return counter;
    }

    std::size_t count(const std::string& data)
    {
        return ++m_counter[data];
    }

    ~UpdateCounter()
    {
        std::vector<std::pair<std::string, std::size_t> > sortedCounter;
        for (const auto& [data, counter] : m_counter)
        {
            sortedCounter.emplace_back(data, counter);

        }
        std::sort(sortedCounter.begin(), sortedCounter.end(), [](const auto& e1, const auto& e2){ return e1.second < e2.second; });
        for (const auto& [data, counter] : sortedCounter)
        {
            std::cerr << data << ": " << counter << std::endl;
        }

    }

private:
    UpdateCounter() {}

    std::map<std::string, std::size_t> m_counter {};
};

void DDGNode::updateIfDirty() const
{
    // auto trace = helper::BackTrace::getTrace(3);
    // const auto count = UpdateCounter::getInstance().count(this->getIdString());
    // if (count >= 300000 && getIdString() == "RegularGridTopology::p0")
    // {
    //     std::cout << " " << std::endl;
    // }
    if (isDirty())
    {
        const_cast <DDGNode*> (this)->update();
    }
}

std::string DDGNode::getIdString() const
{
    return {};
}

void DDGNode::doAddInput(DDGNode* n)
{
    inputs.push_back(n);
}

void DDGNode::doDelInput(DDGNode* n)
{
    inputs.erase(std::remove(inputs.begin(), inputs.end(), n));
}

void DDGNode::doAddOutput(DDGNode* n)
{
    outputs.push_back(n);
}

void DDGNode::doDelOutput(DDGNode* n)
{
    outputs.erase(std::remove(outputs.begin(), outputs.end(), n));
}


} /// namespace sofa::core::objectmodel

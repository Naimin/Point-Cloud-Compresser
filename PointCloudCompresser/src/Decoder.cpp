#include "Decoder.h"
#include <stack>
#include <iostream>

using namespace CPC;

Decoder::Decoder()
{
}

Decoder::~Decoder()
{
}

Octree Decoder::decode(EncodedData& data)
{
    Octree octree(data.maxDepth, data.sceneBoundingBox);

    DepthFirstTransversal(data, octree);
    
    return octree;
}

void Decoder::DepthFirstTransversal(EncodedData & data, Octree & octree)
{
    auto& encodedData = data.encodedData;

    std::stack<DecoderTransversalData> states;
    
    unsigned char currentLevel = 0;
    Index currentIndex(0, 0, 0);
    // create the root node
    Node& root = octree.addNode(currentLevel, currentIndex, encodedData[0]);
    DecoderTransversalData trans(currentLevel, currentIndex, root);
    states.push(trans);

    for(size_t i = 1; i < data.encodedData.size(); ++i)
    {
        // get the parent state
        auto& parent = states.top();
        
        // process the child
        // Find the next child to process
        // Go backward since they are encode backward due to the depth-first transversal
        unsigned char childId = 7;
        Index childIndex(parent.index.index * 2);
        for (; childId >= 0; --childId)
        {
            // for each child id, check if they exist
            unsigned char childBit = 1 << childId;
            unsigned char exist = parent.node.children & childBit;
            if (exist)
            {
                childIndex.index += octree.getChildOffset(childId);
                break;
            }
        }

        // Mark child as processed
        parent.node.removeChild(childId);
        // if no more child, pop the parent
        if (parent.node.children == 0)
            states.pop();

        // Create the node
        if (parent.level + 1 < data.maxDepth)
        {
            Node& node = octree.addNode(parent.level + 1, childIndex, encodedData[i]);
            // if there is still more level to transverse, push the parent state
            if (parent.level + 2 < data.maxDepth)
            {
                DecoderTransversalData trans(parent.level + 1, childIndex, node);
                // add parent state
                states.push(trans);
            }
        }
        else
        {
            std::cout << "Shouldn't be here" << std::endl;
        }
    }
}

void CPC::DecoderTransversalData::processChild(unsigned char child)
{
    node.removeChild(child);
}

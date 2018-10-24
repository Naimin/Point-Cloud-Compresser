#include "Decoder.h"
#include <stack>

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

    Node& root = octree.addNode(currentLevel, currentIndex, encodedData[i]);
    DecoderTransversalData trans(currentLevel, currentIndex, root);
    states.push(trans);

    for(size_t i = 1; i < data.encodedData.size(); ++i)
    {
        // get the parent state
        auto& parent = states.top();

        // if no more child, pop the parent
        if (parent.node.children == 0)
            states.pop();
        
        // process the child

        // Find the next child to process
        unsigned char childId = 0;
        Index childIndex(parent.index * 2);
        for (; childId < 8; ++childId)
        {
            // for each child id, check if they exist
            unsigned char childBit = 1 << childId;
            unsigned char exist = child & childBit;
            if (exist)
            {
                childIndex += octree.getChildOffset(childId);
                break;
            }
        }

        // if there is still more level to transverse
        if (currentLevel + 1 < data.maxDepth)
        {
            Node& node = octree.addNode(currentLevel + 1, childIndex, encodedData[i]);
            DecoderTransversalData trans(currentLevel + 1, childIndex, node);

            // add parent state
            states.push(trans);
        }
        

    }
}

void CPC::DecoderTransversalData::processChild(unsigned char child)
{
    node.removeChild(child);
}

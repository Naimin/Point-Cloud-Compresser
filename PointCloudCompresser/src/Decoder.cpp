#include "Decoder.h"
#include <stack>
#include <iostream>
#include "MortonCode.h"

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
    
    for (size_t i = 0; i < data.encodedData.size(); )
    {
        // Each sub-root node need to be process
        unsigned char currentLevel = data.subOctreeDepth;
        unsigned int mortonCode;
        data.read(mortonCode);
        unsigned char rootChild = data.readNext();
        Index rootIndex = MortonCode::Decode(mortonCode);
        std::cout << "Sub root: " << rootIndex.index.x() << " , " << rootIndex.index.y() << " , " << rootIndex.index.z() << std::endl;
        
        // Need to recursively add the sub-root back to the main root
        for (unsigned char childId = 0; childId < 8; ++childId)
        {
            unsigned char childBit = 1 << childId;
            unsigned char exist = rootChild & childBit;
            if (exist)
            {
                octree.addNodeRecursive(currentLevel, rootIndex, childId);
            }
        }
        
        auto& root = octree.getLevels()[currentLevel][rootIndex];
        DecoderTransversalData trans(currentLevel, rootIndex, root);
        states.push(trans);

        // keep processing next byte until no more child/parent is un processed.
        // Then move to the next sub-root's octree
        while(!states.empty())
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
                Node& node = octree.addNode(parent.level + 1, childIndex, data.readNext());
                // if there is still more level to transverse, push the parent state
                if (parent.level + 2 < data.maxDepth)
                {
                    DecoderTransversalData trans(parent.level + 1, childIndex, node);
                    // add parent state
                    states.push(trans);
                }
            }
            else if(!states.empty())
            {
                std::cout << "Shouldn't be here" << std::endl;
            }
        }
        // remember to increament number of byte already read.
        i = data.currentSize;
    }
}

void CPC::DecoderTransversalData::processChild(unsigned char child)
{
    node.removeChild(child);
}

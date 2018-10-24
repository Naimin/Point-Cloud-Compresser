#include "Encoder.h"
#include <stack>

using namespace CPC;

bool EncodedData::isValid()
{
    return !encodedData.empty();
}

Encoder::Encoder()
{
}

Encoder::~Encoder()
{
}

EncodedData Encoder::encode(Octree & octree)
{
    EncodedData data;
    data.sceneBoundingBox = octree.getBoundingBox();
    data.maxDepth = octree.getMaxDepth();

    DepthFirstTransversal(octree, data);

    return data;
}

void Encoder::DepthFirstTransversal(Octree & octree, EncodedData & data)
{
    // since we know exactly how many node there is to write, we just allocate them
    data.encodedData.resize(octree.getNumOfAllNodes());
    auto& levels = octree.getLevels();
    size_t counter = 0; // track how many node is written

    std::stack<TransversalData> stack;
    
    TransversalData root(0, levels[0].begin()->first, levels[0].begin()->second);
    stack.push(root);
    
    while (!stack.empty())
    {
        TransversalData trans = stack.top();
        stack.pop();

        // Write into the data when evaluating a new node.
        unsigned char child = trans.node.children;
        data.encodedData[counter++] = child;

        // iterate over the child and push them into the stack
        for (unsigned char childId = 0; childId < 8; ++childId)
        {
            // for each child id, check if they exist
            unsigned char childBit = 1 << childId;
            unsigned char exist = child & childBit;
            if (exist)
            {
                Index childIndex(trans.index.index * 2);
                // apply the child offset to 2 x Parent index
                childIndex += octree.getChildOffset(childId);

                // only push node if there is actual child node
                if (trans.level + 1 < data.maxDepth)
                {
                    TransversalData transChild(trans.level + 1, childIndex, levels[trans.level + 1][childIndex]);
                    stack.push(transChild);
                }
            }
        }
    }
}

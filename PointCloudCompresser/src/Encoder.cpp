#include "Encoder.h"
#include <stack>
#include <tbb/parallel_for.h>
#include <iostream>
#include "MortonCode.h"

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

    auto& bestStats = computeBestSubOctreeLevel(octree);
    DepthFirstTransversal(octree, bestStats, data);

    return data;
}

void Encoder::DepthFirstTransversal(Octree & octree, BestStats& bestStats, EncodedData & data)
{
    // since we know exactly how many node there is to write, we just allocate them
    data.encodedData.resize(bestStats.size);
    auto& levels = octree.getLevels();

    std::stack<TransversalData> stack;
    
    // Encode from the subOctreeLevel and truncate the upper levels
    // For each node in the subOctreeLevel encode the index, then transverse the full sub-octree
    for (auto& itr : levels[bestStats.level])
    {
        // compute the Morton Code address of sub-octree root
        unsigned int mortonCode = MortonCode::Encode(itr->first);
        // Write the index address at the start of this sub-octree node.
        data.add(mortonCode);

        TransversalData root(bestStats.level, itr->first, itr->second);
        stack.push(root);

        while (!stack.empty())
        {
            TransversalData trans = stack.top();
            stack.pop();

            // Write into the data when evaluating a new node.
            unsigned char child = trans.node.children;
            data.add(child);

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
                    childIndex.index += octree.getChildOffset(childId);

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
}

BestStats CPC::Encoder::computeBestSubOctreeLevel(Octree & octree)
{ 
    BestStats best;

    auto& levels = octree.getLevels();
    size_t maxDepth = levels.size();
    tbb::parallel_for((size_t)0, maxDepth, [&](size_t level)
    {
        // Each sub octree root node need a address index (4 byte)
        size_t totalSize = levels[level].size() * sizeof(unsigned int);
        // Now compute the size of each of the children node using this sub octree level.
        for (unsigned char i = level; i < octree.getMaxDepth(); ++i)
        {
            totalSize = levels[i].size() * sizeof(unsigned char);
        }
        
        std::cout << "Level: " << level << " TotalSize: " << totalSize << std::endl;

        best.checkAndUpdate(totalSize, level);
    });

    std::cout << "Best Level: " << best.bestLevel << " TotalSize: " << best.bestSize << std::endl;
    return best;
}

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

EncodedData Encoder::encode(Octree & octree, unsigned char forceSubOctreeLevel)
{
    EncodedData data;
    data.sceneBoundingBox = octree.getBoundingBox();
    data.maxDepth = octree.getMaxDepth();

    BestStats best;
    // Compute the optimal sub octree depth if no force depth is specified.
    best = (forceSubOctreeLevel != (unsigned char)-1) ? BestStats(computeSubOctreeSize(octree, forceSubOctreeLevel), forceSubOctreeLevel) : computeBestSubOctreeLevel(octree);
    
    data.subOctreeDepth = best.level;
    DepthFirstTransversal(octree, best, data);

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
        unsigned int mortonCode = MortonCode::encode64(itr.first);
        // Write the index address at the start of this sub-octree node.
        data.add(mortonCode);
        unsigned char* chars = (unsigned char*)&mortonCode;
        std::cout << "Sub root: " << (int)chars[0] << " , " << (int)chars[1] << " , " << (int)chars[2] << " , " << (int)chars[3] << std::endl;

        //std::cout << "Sub root: " << itr.first.index.x() << " , " << itr.first.index.y() << " , " << itr.first.index.z() << std::endl;

        TransversalData root(bestStats.level, itr.first, itr.second);
        stack.push(root);

        while (!stack.empty())
        {
            TransversalData trans = stack.top();
            stack.pop();

            // Write into the data when evaluating a new node.
            unsigned char child = trans.node.children;
            data.add(child);
            std::cout << (int)child << std::endl;

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
    unsigned char maxDepth = (unsigned char)levels.size();
    //tbb::parallel_for((unsigned char)0, maxDepth, [&](unsigned char level)
    for(unsigned char level = 0; level < maxDepth; ++level)
    {
        size_t totalSize = computeSubOctreeSize(octree, level);
        std::cout << "Level: " << (int)level << " TotalSize: " << totalSize << std::endl;

        best.checkAndUpdate(totalSize, level);
    }//);

    std::cout << "Best Level: " << (int)best.level << " TotalSize: " << best.size << std::endl;
    return best;
}

size_t CPC::Encoder::computeSubOctreeSize(Octree& octree, unsigned char level)
{
    auto& levels = octree.getLevels();
    // Each sub octree root node need a address index (4 byte)
    size_t totalSize = levels[level].size() * sizeof(unsigned int);
    // Now compute the size of each of the children node using this sub octree level.
    for (unsigned char i = level; i < (unsigned char)octree.getMaxDepth(); ++i)
    {
        totalSize += levels[i].size() * sizeof(unsigned char);
    }
    return totalSize;
}
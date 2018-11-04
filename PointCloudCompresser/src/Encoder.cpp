#include "Encoder.h"
#include <stack>
#include <tbb/parallel_for.h>
#include <iostream>
#include "MortonCode.h"

using namespace CPC;

const int maxOffsetDist = 1024;

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
    Index currentIndex(0, 0, 0); // set the current Index to a large offset, hack to make it fail the first test
    
    // Encode from the subOctreeLevel and truncate the upper levels
    // For each node in the subOctreeLevel encode the index, then transverse the full sub-octree
    for (auto& itr : levels[bestStats.level])
    {
        Eigen::Vector3i offset((itr.first.cast<int>() - currentIndex.cast<int>()));
        std::cout << "offset: " << offset.x() << " , " << offset.y() << " , " << offset.z() << std::endl;
        // if the new offset is beyond the MAX_OFFSET distance, use full address instead of offset
        if (offset.x() < 0 || offset.x() > MAX_OFFSET || offset.y() < 0 || offset.y() > MAX_OFFSET || offset.z() < 0 || offset.z() > MAX_OFFSET)
        {
            // compute the Morton Code of sub-octree offset
            auto mortonCode = getEncodedFullAddress(itr.first); // set the left most bit, to signal full address
            // Write the offset index address at the start of this sub-octree node.
            data.add(mortonCode);
            currentIndex = itr.first;
#ifdef DEBUG_ENCODING
            unsigned char* chars = (unsigned char*)&mortonCode;
            std::cout << "Sub root: " << (int)chars[0] << " , " << (int)chars[1] << " , " << (int)chars[2] << " , " << (int)chars[3] << " , "
                                      << (int)chars[4] << " , " << (int)chars[5] << " , " << (int)chars[6] << " , " << (int)chars[7] << std::endl;
#endif
        }
        else
        {
            Index unsignedOffset((unsigned int)offset.x(), (unsigned int)offset.y(), (unsigned int)offset.z());
            auto mortonCode = getEncodedOffsetAddress(unsignedOffset);
            data.add(mortonCode);
            currentIndex += unsignedOffset;
#ifdef DEBUG_ENCODING
            std::cout << "Sub root Offset: " << (int)mortonCode << std::endl;
#endif
        }
        
        TransversalData root(bestStats.level, itr.first, itr.second);
        stack.push(root);

        while (!stack.empty())
        {
            TransversalData trans = stack.top();
            stack.pop();

            // Write into the data when evaluating a new node.
            unsigned char child = trans.node.children;
            data.add(child);
#ifdef DEBUG_ENCODING
            std::cout << (int)child << std::endl;
#endif
            // iterate over the child and push them into the stack
            for (unsigned char childId = 0; childId < 8; ++childId)
            {
                // for each child id, check if they exist
                unsigned char childBit = 1 << childId;
                unsigned char exist = child & childBit;
                if (exist)
                {
                    Index childIndex(trans.index * 2);
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

    size_t totalSize = 0;
    // compute the dynamic size of the variable length morton code
    Index currentIndex(0, 0, 0); // assume always start at (0,0,0)
    for (auto& itr : levels[level])
    {
        auto offset((currentIndex - itr.first).eval());
        if (offset.x() < 0 || offset.x() > MAX_OFFSET || offset.y() < 0 || offset.y() > MAX_OFFSET || offset.z() < 0 || offset.z() > MAX_OFFSET)
            totalSize += sizeof(unsigned long long);
        else
            totalSize += sizeof(unsigned char);
    }

    // Each sub octree root node need a address index (8 byte)
    // Now compute the size of each of the children node using this sub octree level.
    for (unsigned char i = level; i < (unsigned char)octree.getMaxDepth(); ++i)
    {
        totalSize += levels[i].size() * sizeof(unsigned char);
    }
    return totalSize;
}

unsigned long long CPC::Encoder::getEncodedFullAddress(const Index & index)
{
    return MortonCode::encode64(index) | 0x8000000000000000; // compute morton code then add a full address flag on left-most bit
}

unsigned char CPC::Encoder::getEncodedOffsetAddress(const Index & index)
{
    return MortonCode::encode8(index) & 0x7f; // compute morton code then unset the full address flag (left-most bit).
}

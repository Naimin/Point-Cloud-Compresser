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

    Index currentIndex(0, 0, 0);
    
    for (size_t i = 0; i < data.encodedData.size(); )
    {
        // Each sub-root node need to be process
        unsigned char currentLevel = data.subOctreeDepth;
        
        if (data.checkFullAddressFlag())
        {
            FullAddress mortonCode;
            data.read(mortonCode);
            currentIndex = decodedFullAddress(mortonCode);
#ifdef DEBUG_ENCODING
            unsigned char* chars = (unsigned char*)&mortonCode;
            std::cout << "Sub root: " << (int)chars[0] << " , " << (int)chars[1] << " , " << (int)chars[2] << " , " << (int)chars[3] << " , "
                                      << (int)chars[4] << " , " << (int)chars[5] << " , " << (int)chars[6] << " , " << (int)chars[7] << std::endl;
#endif
        }
        else
        {
            OffsetAddress mortonCode;
            data.read(mortonCode);
            auto offsets = decodedOffsetAddress(mortonCode);
#ifdef DEBUG_ENCODING
            std::cout << "offset: " << offsets.x() << " , " << offsets.y() << " , " << offsets.z() << std::endl;
#endif
            currentIndex.x() = (unsigned int)((int)currentIndex.x() + offsets.x());
            currentIndex.y() = (unsigned int)((int)currentIndex.y() + offsets.y());
            currentIndex.z() = (unsigned int)((int)currentIndex.z() + offsets.z());

#ifdef DEBUG_ENCODING
            unsigned char* chars = (unsigned char*)&mortonCode;
            std::cout << "Sub root Offset: " << (int)chars[0] << " , " << (int)chars[1] << " , " << (int)chars[2] << " , " << (int)chars[3] << std::endl;
#endif
        }
        unsigned char rootChild = data.readNext();
#ifdef DEBUG_ENCODING
        unsigned char* chars = (unsigned char*)&mortonCode;
        std::cout << "Current Sub root: " << (int)currentIndex.x() << " , " << (int)currentIndex.y() << " , " << (int)currentIndex.z() << std::endl;
        std::cout << (int)rootChild << std::endl;
#endif
        
        // Need to recursively add the sub-root back to the main root
        for (unsigned char childId = 0; childId < 8; ++childId)
        {
            unsigned char childBit = 1 << childId;
            unsigned char exist = rootChild & childBit;
            if (exist)
            {
                size_t dummy;
                octree.addNodeRecursive(currentLevel, currentIndex, childId, dummy);
            }
        }
        
        auto& root = octree.getLevels()[currentLevel][currentIndex];
        DecoderTransversalData trans(currentLevel, currentIndex, root);
        states.push(trans);

        // keep processing next byte until no more child/parent is un processed.
        // Then move to the next sub-root's octree
        while(!states.empty())
        {
            // get the parent state
            auto& parent = states.top();

            if (parent.node.children == 0) // assertion
            {
                states.pop();
                std::cout << "Shouldn't be here" << std::endl;
                return;
            }
            // process the child
            // Find the next child to process
            // Go backward since they are encode backward due to the depth-first transversal
            unsigned char childId = 7;
            Index childIndex(parent.index * 2);
            for (; childId >= 0; --childId)
            {
                // for each child id, check if they exist
                unsigned char childBit = 1 << childId;
                unsigned char exist = parent.node.children & childBit;
                if (exist)
                {
                    childIndex += octree.getChildOffset(childId);
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
#ifdef DEBUG_ENCODING
                std::cout << (int)node.children << std::endl;
#endif

                // if there is still more level to transverse, push the parent state
                if (parent.level + 2 < data.maxDepth)
                {
                    DecoderTransversalData trans(parent.level + 1, childIndex, node);
                    // add parent state
                    states.push(trans);
                }
            }
            else if(!states.empty()) // assertion
            {
                std::cout << "Shouldn't be here" << std::endl;
                return;
            }
        }
        // remember to increament number of byte already read.
        i = data.currentSize;
    }
}

Index CPC::Decoder::decodedFullAddress(const FullAddress & code)
{
    return MortonCode::decode64(code);
}

Eigen::Vector3i CPC::Decoder::decodedOffsetAddress(const OffsetAddress & code)
{
    auto index = MortonCode::decode32(code);
    Eigen::Vector3i result(index.x(), index.y(), index.z());
    if (index.x() > MAX_OFFSET)
    {
        result.x() = index.x() - 2 * MAX_OFFSET;
    }
    if (index.y() > MAX_OFFSET)
    {
        result.y() = index.y() - 2 * MAX_OFFSET;
    }
    if (index.z() > MAX_OFFSET)
    {
        result.z() = index.z() - 2 * MAX_OFFSET;
    }
    return result;
}

void CPC::DecoderTransversalData::processChild(unsigned char child)
{
    node.removeChild(child);
}

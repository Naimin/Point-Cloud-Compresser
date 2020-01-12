#include "Decoder.h"
#include <stack>
#include <iostream>
#include <map>
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
    auto subNodePos = decodeNodeHeaders(data);

    // Iterate through subNodePos to decode the subnode
    for (auto& subNode : subNodePos)
    {
        decodeNode(subNode.second, subNode.first, data, octree);
    }
}

std::map<Index, size_t> CPC::Decoder::decodeNodeHeaders(EncodedData& data)
{
    Index currentIndex(0, 0, 0);

    std::map<Index, size_t> subNodePos;

    // Decode all the subnode header and store their position in the subNodePos
    for (size_t i = 0; i < data.encodedData.size(); )
    {
        // Each sub-root node need to be process
        size_t pos = i;
        size_t nodeSize;
        decodeNodeHeader(pos, currentIndex, data, nodeSize);

        subNodePos.insert(std::make_pair(currentIndex, pos));
        // Advance by node header and node payload size
        i = pos + nodeSize;
    }

    return subNodePos;
}

void CPC::Decoder::decodeNodeHeader(size_t& pos, Index& index, EncodedData& data, size_t& nodeSize)
{
    // Decode the node index address
    if (data.checkFullAddressFlag(pos))
    {
        FullAddress mortonCode;
        data.read(pos, mortonCode);
        index = decodedFullAddress(mortonCode);
#ifdef DEBUG_ENCODING
        unsigned char* chars = (unsigned char*)&mortonCode;
        std::cout << "Full address root: " << (int)chars[0] << " , " << (int)chars[1] << " , " << (int)chars[2] << " , " << (int)chars[3] << " , "
            << (int)chars[4] << " , " << (int)chars[5] << " , " << (int)chars[6] << " , " << (int)chars[7] << std::endl;
#endif
    }
    else
    {
        OffsetAddress mortonCode;
        data.read(pos, mortonCode);
        auto offsets = decodedOffsetAddress(mortonCode);
#ifdef DEBUG_ENCODING
        std::cout << "offset: " << offsets.x() << " , " << offsets.y() << " , " << offsets.z() << std::endl;
#endif
        index.x() = (unsigned int)((int)index.x() + offsets.x());
        index.y() = (unsigned int)((int)index.y() + offsets.y());
        index.z() = (unsigned int)((int)index.z() + offsets.z());

#ifdef DEBUG_ENCODING
        unsigned char* chars = (unsigned char*)&mortonCode;
        std::cout << "Sub root Offset: " << (int)chars[0] << " , " << (int)chars[1] << " , " << (int)chars[2] << " , " << (int)chars[3] << std::endl;
#endif
    }
    // Read the total node size
    data.read(pos, nodeSize);
}

void CPC::Decoder::decodeNode(size_t& pos, const Index& index, EncodedData& data, Octree& octree)
{
    if (decodedNodes.find(index) != decodedNodes.end())
    {
#ifdef DEBUG_ENCODING
        std::cout << "Already decoded before." << std::endl;
#endif
        return;
    }
    decodedNodes.insert(index);

    // Each sub-root node need to be process
    unsigned char currentLevel = data.subOctreeDepth;

    unsigned char rootChild = data.readNext(pos);
#ifdef DEBUG_ENCODING
    std::cout << "Current Sub root: " << (int)index.x() << " , " << (int)index.y() << " , " << (int)index.z() << std::endl;
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
            octree.addNodeRecursive(currentLevel, index, childId, dummy);
        }
    }

    auto& root = octree.getLevels()[currentLevel][index];
    DecoderTransversalData trans(currentLevel, index, root);
    std::stack<DecoderTransversalData> states;
    states.push(trans);

    // keep processing next byte until no more child/parent is un processed.
    // Then move to the next sub-root's octree
    while (!states.empty())
    {
        // get the parent state
        auto& parent = states.top();

        if (parent.node.children == 0) // assertion
        {
            states.pop();
            std::cout << "Shouldn't be here 1" << std::endl;
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
            Node& node = octree.addNode(parent.level + 1, childIndex, data.readNext(pos));
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
        else if (!states.empty()) // assertion
        {
            std::cout << "Shouldn't be here 2" << std::endl;
            //return;
        }
    }

#ifdef DEBUG_ENCODING
  //  std::cout << "Node Size: " << nodeSize << std::endl;
#endif    
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

bool CPC::Decoder::intersect(const Eigen::Vector3f& point, EncodedData& data, Octree& octree, std::map<Index, size_t>& subNodePos, intersectionState& state)
{
    // Compute the leaf node address
    auto leafAddress = octree.computeLeafAddress(point);
    leafAddress = octree.computeParentAddress(leafAddress);
    // We already decoded this leaf node before
    if (octree.nodeExist(octree.getMaxDepth()-1, leafAddress))
    {
        state = ALREADY_EXIST;
        return true;
    }

    // If not already decoded try to find the subnode
    auto subNodeAddress = octree.computeParentAddress(octree.getMaxDepth()-1, data.subOctreeDepth, leafAddress);
    // If the subnode address don't exist this mean the child also don't exist
    auto subNodeItr = subNodePos.find(subNodeAddress);
    if (subNodeItr == subNodePos.end())
    {
        state = SUBNODE_NOT_FOUND;
        return false;
    }
        
    // Decode the subnode
    decodeNode(subNodeItr->second, subNodeItr->first, data, octree);

    // Check again if this leaf node get decoded just now.
    if (octree.nodeExist(octree.getMaxDepth()-1, leafAddress))
    {
        state = DECODE_FOUND;
        return true;
    }
    else
    {
        state = DECODE_NOT_FOUND;
        return false;
    }
}
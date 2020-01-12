#include "Octree.h"
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <iostream>

using namespace CPC;

CPC::Octree::Octree(unsigned int maxDepth, BoundingBox& bbox_) : levels(maxDepth), levelMutexs(maxDepth), bbox(bbox_)
{
    leafCellSize = computeLeafCellSize(maxDepth, bbox);
}

Octree::Octree(unsigned int maxDepth, PointCloud& pointCloud) : levels(maxDepth), levelMutexs(maxDepth)
{
    generate(maxDepth, pointCloud);
}

PointCloud Octree::generatePointCloud()
{
    auto& currentLevel = levels[levels.size() - 1];
    PointCloud pointCloud;
    pointCloud.resize(currentLevel.size() * 8); // maximum possible number of points, we will resize it back to actual size later

    tbb::atomic<size_t> counter = 0; // keep track of how many actual point there is

    // Hack: We should change the internal representation of each Octree Level as a vector instead of map.
    std::vector<std::pair< Index, Node>> vectorized(currentLevel.begin(), currentLevel.end());
    tbb::parallel_for((size_t)0, vectorized.size(), [&](const size_t i)
    //for(size_t i = 0; i < vectorized.size(); ++i)
    {
        auto& itr = vectorized[i];
        Index index = itr.first;
        Node node = itr.second;
        unsigned char child = node.children.load();
        
        // leafCellSize * 2 since it is the parent node size.
        float nodeX = bbox.min.x() + leafCellSize.x() * 2 * index.x();
        float nodeY = bbox.min.y() + leafCellSize.y() * 2 * index.y();
        float nodeZ = bbox.min.z() + leafCellSize.z() * 2 * index.z();
        Vector3f nodePos(nodeX, nodeY, nodeZ);
        
        for (unsigned char childId = 0; childId < 8; ++childId)
        {
            // for each child id, check if they exist
            unsigned char childBit = 1 << childId;
            unsigned char exist = child & childBit;
            if (exist)
            {
                switch (exist)
                {
                    case 1:
                        nodePos += Vector3f(0,0,0);
                        break;
                    case 2:
                        nodePos += Vector3f(leafCellSize.x(), 0, 0);
                        break;
                    case 4:
                        nodePos += Vector3f(0, leafCellSize.y(), 0);
                        break;
                    case 8:
                        nodePos += Vector3f(leafCellSize.x(), leafCellSize.y(), 0);
                        break;
                    case 16:
                        nodePos += Vector3f(0, 0, leafCellSize.z());
                        break;
                    case 32:
                        nodePos += Vector3f(leafCellSize.x(), 0, leafCellSize.z());
                        break;
                    case 64:
                        nodePos += Vector3f(0, leafCellSize.y(), leafCellSize.z());
                        break;
                    case 128:
                        nodePos += Vector3f(leafCellSize.x(), leafCellSize.y(), leafCellSize.z());
                        break;
                }
                pointCloud.positions[counter++] = nodePos;
            }
        }
    });

    pointCloud.resize(counter);
    pointCloud.shrink_to_fit();

    return pointCloud;
}

BoundingBox Octree::getBoundingBox() const
{
    return bbox;
}

Eigen::Vector3f Octree::getLeafCellSize() const
{
    return leafCellSize;
}

std::vector<std::map<Index, Node>>& Octree::getLevels()
{
    return levels;
}

unsigned int Octree::getMaxDepth() const
{
    return (unsigned int)levels.size();
}

size_t Octree::getNumOfAllNodes() const
{
    size_t numOfNodes = 0;
    for (auto& level : levels)
    {
        numOfNodes += level.size();
    }

    return numOfNodes;
}

Node& CPC::Octree::addNode(const unsigned int level, const Index & index, const unsigned char child)
{
    levels[level].insert(std::make_pair(index, Node(child)));
    return levels[level][index];
}

void Octree::generate(unsigned int maxDepth, PointCloud& pointCloud)
{
    bbox = computeBoundingBox(pointCloud);
    leafCellSize = computeLeafCellSize(maxDepth, bbox);

    tbb::atomic<size_t> bottomUpTransverseCounter = 0;

    // for each point add it into the octree
    tbb::parallel_for((size_t)0, pointCloud.positions.size(), [&](size_t index)
    //for(size_t index = 0; index < pointCloud.positions.size(); ++index)
    {
        Eigen::Vector3f& pos = pointCloud.positions[index];
        auto leafAddress = computeLeafAddress(pos);
        size_t transversalCounter = 0;
        addLeaf(maxDepth, leafAddress, transversalCounter);
        bottomUpTransverseCounter += transversalCounter;
    });

    std::cout << "BottomUp transversal: " << bottomUpTransverseCounter << std::endl;
    std::cout << "Normal transversal: " << pointCloud.positions.size() * maxDepth << std::endl;
}

BoundingBox Octree::computeBoundingBox(PointCloud & pointCloud)
{
    BoundingBox bbox;

    for (auto& point : pointCloud.positions)
    {
        bbox.expand(point);
    }

    return bbox;
}

Eigen::Vector3f Octree::computeLeafCellSize(const unsigned int maxDepth, const BoundingBox& bbox)
{
    Eigen::Vector3f volume = bbox.max - bbox.min;

    // half the volume once per depth level
    for (int depth = maxDepth; depth > 0; --depth)
    {
        volume /= 2.f;
    }

    return volume;
}

Index Octree::computeLeafAddress(const Eigen::Vector3f& point)
{
    Vector3f localPos = point - bbox.min;

    unsigned int x = (unsigned int)std::floor(localPos.x() / leafCellSize.x());
    unsigned int y = (unsigned int)std::floor(localPos.y() / leafCellSize.y());
    unsigned int z = (unsigned int)std::floor(localPos.z() / leafCellSize.z());

    Vector3f maxCheck = point - bbox.max;
    x = maxCheck.x() == 0 ? x - 1 : x;
    y = maxCheck.y() == 0 ? y - 1 : y;
    z = maxCheck.z() == 0 ? z - 1 : z;

    return Index(x,y,z);
}

Index Octree::computeParentAddress(const Index& index)
{
    return Index(index.x() / 2, index.y() / 2, index.z() / 2);
}

Index Octree::computeParentAddress(const unsigned int currentLevel, const unsigned int parentLevel, const Index& index)
{
    Index currentIndex = index;
    for (unsigned int i = currentLevel; i > parentLevel; --i)
    {
        currentIndex = computeParentAddress(currentIndex);
    }
    return currentIndex;
}

unsigned char Octree::computeOctreeChildIndex(const Index& index)
{
    //top    bottom
    //|0|1|  |4|5|
    //-----  -----
    //|2|3|  |6|7|

    unsigned char xOffset = index.x() % 2;
    unsigned char yOffset = index.y() % 2;
    unsigned char zOffset = index.z() % 2;

    unsigned char childIndex = 0;
    childIndex += xOffset;
    childIndex += 2 * yOffset;
    childIndex += 4 * zOffset;

    return childIndex;
}

bool Octree::addLeaf(const unsigned int maxDepth, const Index& index, size_t& transversalCounter)
{
    // get the parent address
    Index parent = computeParentAddress(index);
    unsigned char octreeChild = computeOctreeChildIndex(index);

    // check if the parent exist 
    if (!nodeExist(maxDepth - 1, parent))
        addNodeRecursive(maxDepth - 1, parent, octreeChild, transversalCounter);
    
    // Add this child into the the parent node
    addNodeChild(maxDepth - 1, parent, octreeChild);
    ++transversalCounter;

    return true;
}

bool Octree::nodeExist(const unsigned int level, const Index& index)
{
    if ((size_t)level > levels.size())
        return false;

    // lock before checking the current level
    tbb::mutex::scoped_lock lock(levelMutexs[level]);
    auto& currentLevel = levels[level];
    
    auto itr = currentLevel.find(index);
    return itr != currentLevel.end();
}

bool Octree::addNodeRecursive(const unsigned int level, const Index& index, const unsigned int childIndex, size_t& transversalCounter)
{
    if ((size_t)level >= levels.size())
        return false;

    auto localLevel = level;
    auto localIndex = index;
    unsigned int localChild = childIndex;

    while (!nodeExist(localLevel, localIndex) && localLevel > 0)
    {
        tbb::mutex::scoped_lock lock(levelMutexs[localLevel]);
        // Add and update the the child status
        addNodeChild(localLevel, localIndex, localChild);

        // move up 1 level
        --localLevel;
        // compute our child index
        localChild = computeOctreeChildIndex(localIndex);
        // find the parent index 
        localIndex = computeParentAddress(localIndex);
        ++transversalCounter;
    }

    // hit a node that already exist or root node, then we just need to update the node
    addNodeChild(localLevel, localIndex, localChild);
    ++transversalCounter;
    return true;
}

Vector3ui CPC::Octree::getChildOffset(unsigned char childId)
{
    // apply the child offset to 2 x Parent index
    Vector3ui result(0, 0, 0);
    unsigned char childBit = 1 << childId;
    switch (childBit)
    {
    case 1:
        break;
    case 2:
        result = Vector3ui(1, 0, 0);
        break;
    case 4:
        result = Vector3ui(0, 1, 0);
        break;
    case 8:
        result = Vector3ui(1, 1, 0);
        break;
    case 16:
        result = Vector3ui(0, 0, 1);
        break;
    case 32:
        result = Vector3ui(1, 0, 1);
        break;
    case 64:
        result = Vector3ui(0, 1, 1);
        break;
    case 128:
        result = Vector3ui(1, 1, 1);
        break;
    }

    return result;
}

void Octree::addNodeChild(const unsigned int level, const Index& parentIndex, const unsigned int childIndex)
{
    auto& currentLevel = levels[level];
    currentLevel[parentIndex].addChild(childIndex);
}

bool Node::addChild(unsigned char index)
{
    unsigned char childBit = 1 << index;
    children.fetch_or(childBit);
    return true;
}

void CPC::Node::removeChild(unsigned char index)
{
    unsigned char childBit = ~(1 << index);
    children.fetch_and(childBit);
}

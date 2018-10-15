#include "Octree.h"
#include <tbb/parallel_for.h>


using namespace PCC;

PCC::Octree::Octree(unsigned int maxDepth, PointCloud& pointCloud) : levels(maxDepth), levelMutexs(maxDepth)
{
    generate(maxDepth, pointCloud);
}

void PCC::Octree::generate(unsigned int maxDepth, PointCloud& pointCloud)
{
    const BoundingBox bbox = computeBoundingBox(pointCloud);

    const Eigen::Vector3f leafCellSize = computeLeafCellSize(maxDepth, bbox);

    // for each point add it into the octree
    tbb::parallel_for((size_t)0, pointCloud.positions.size(), [&](size_t index)
    {
        Eigen::Vector3f& pos = pointCloud.positions[index];
        auto leafAddress = computeLeafAddress(bbox, leafCellSize, pos);
        addLeaf(maxDepth, leafAddress);
    });
}

BoundingBox PCC::Octree::computeBoundingBox(PointCloud & pointCloud)
{
    BoundingBox bbox;

    for (auto& point : pointCloud.positions)
    {
        bbox.expand(point);
    }

    return bbox;
}

Eigen::Vector3f PCC::Octree::computeLeafCellSize(const unsigned int maxDepth, const BoundingBox& bbox)
{
    Eigen::Vector3f volume = bbox.max - bbox.min;

    // half the volume once per depth level
    for (int depth = maxDepth; depth > 0; --depth)
    {
        volume /= 2.f;
    }

    return volume;
}

Index PCC::Octree::computeLeafAddress(const BoundingBox& bbox, const Eigen::Vector3f& leafCellSize, const Eigen::Vector3f& point)
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

Index PCC::Octree::computeParentAddress(const Index index)
{
    return Index(index.index.x() / 2, index.index.y() / 2, index.index.z() / 2);
}

unsigned char PCC::Octree::computeOctreeChildIndex(const Index index)
{
    //top    bottom
    //|0|1|  |4|5|
    //-----  -----
    //|2|3|  |6|7|

    unsigned char xOffset = index.index.x() % 2;
    unsigned char yOffset = index.index.y() % 2;
    unsigned char zOffset = index.index.z() % 2;

    unsigned char childIndex = 0;
    childIndex += xOffset;
    childIndex += 2 * yOffset;
    childIndex += 4 * zOffset;

    return childIndex;
}

bool PCC::Octree::addLeaf(const unsigned int maxDepth, const Index index)
{
    // get the parent address
    Index parent = computeParentAddress(index);
    unsigned char octreeChild = computeOctreeChildIndex(index);

    // check if the parent exist 
    if (!nodeExist(maxDepth - 1, parent))
        addNode(maxDepth - 1, parent);
    
    // Add this child into the the parent node
    addNodeChild(maxDepth - 1, parent, octreeChild);

    return true;
}

bool PCC::Octree::nodeExist(const unsigned int level, const Index index)
{
    if ((size_t)level > levels.size())
        return false;

    // lock before checking the current level
    tbb::mutex::scoped_lock lock(levelMutexs[level]);
    auto& currentLevel = levels[level];
    
    auto itr = currentLevel.find(index);
    return itr != currentLevel.end();
}

bool PCC::Octree::addNode(const unsigned int level, const Index index)
{
    if ((size_t)level >= levels.size())
        return false;

    auto localLevel = level;
    auto localIndex = index;
    unsigned int localChild = computeOctreeChildIndex(localIndex);

    while (!nodeExist(localLevel, localIndex) && localLevel > 0)
    {
        tbb::mutex::scoped_lock lock(levelMutexs[localLevel]);
        // create the node
        levels[localLevel].insert(std::make_pair(localIndex, Node()));
        
        //addNode(localLevel, localIndex);
        // update the child status
        addNodeChild(localLevel, localIndex, localChild);

        // move up 1 level
        --localLevel;
        // find the parent index 
        localIndex = computeParentAddress(localIndex);
        // compute our child index
        localChild = computeOctreeChildIndex(localIndex);
    }
    // hit a node that already exist or root node, then we just need to update the node
    addNodeChild(localLevel, localIndex, localChild);
    return true;
}

void PCC::Octree::addNodeChild(const unsigned int level, const Index parentIndex, const unsigned int childIndex)
{
    auto& currentLevel = levels[level];
    currentLevel[parentIndex].addChild(childIndex);
}

bool PCC::Node::addChild(unsigned char index)
{
    unsigned char childBit = 1 << index;
    children.fetch_or(childBit);
    return true;
}
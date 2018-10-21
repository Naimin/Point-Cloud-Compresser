#pragma once
#include <tbb/atomic.h>
#include <tbb/mutex.h>
#include <vector>
#include <map>
#include <atomic>

#include "PointCloud.h"
#include "BoundingBox.h"

namespace PCC
{
    struct Index
    {
        Index(const Vector3ui& index_) : index(index_) {}
        Index(const unsigned int x, const unsigned int y, const unsigned int z) : index(x, y, z) {}
        bool operator < (const Index &b) const
        {
            if (index.x() != b.index.x()) {
                return index.x() < b.index.x();
            }
            if (index.y() != b.index.y()) {
                return index.y() < b.index.y();
            }
            return  index.z() < b.index.z();
        }

        Vector3ui index;
    };

    struct Node
    {
        Node() { children = 0; }
        Node(const Node& node) { children = node.children.load(); }
        bool addChild(unsigned char index);

        std::atomic<unsigned char> children; // one bit for each child
    };

    class Octree
    {
        public:
            Octree(unsigned int maxDepth, PointCloud& pointCloud);
            PointCloud getPointCloud(); // convert the octree back to point cloud 

        protected:
            void generate(unsigned int maxDepth, PointCloud& pointCloud);
            BoundingBox computeBoundingBox(PointCloud& pointCloud);
            Eigen::Vector3f computeLeafCellSize(const unsigned int maxDepth, const BoundingBox& bbox);
            Index computeLeafAddress(const BoundingBox& bbox, const Eigen::Vector3f& leafCellSize, const Eigen::Vector3f& point);
            Index computeParentAddress(const Index& index);
            unsigned char computeOctreeChildIndex(const Index& index);
            bool addLeaf(const unsigned int maxDepth, const Index& index);
            bool nodeExist(const unsigned int level, const Index& index);
            bool addNode(const unsigned int level, const Index& index, const unsigned int childIndex);
            void addNodeChild(const unsigned int level, const Index& parentIndex, const unsigned int childIndex);

            // a vector to store each level
            // each level is a map of node
            std::vector<std::map<Index, Node>> levels;
            std::vector<tbb::mutex> levelMutexs;
            BoundingBox bbox;
            Eigen::Vector3f leafCellSize;
    };
}
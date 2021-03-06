#pragma once
#include <tbb/atomic.h>
#include <tbb/mutex.h>
#include <vector>
#include <map>
#include <atomic>

#include "PointCloud.h"
#include "BoundingBox.h"
#include "Index.h"

namespace CPC
{
    struct Node
    {
        Node() { children = 0; }
        Node(const unsigned char child) : children(child) {}
        Node(const Node& node) { children = node.children.load(); }
        bool addChild(unsigned char index);
        void removeChild(unsigned char index);

        std::atomic<unsigned char> children; // one bit for each child
    };

    typedef std::map<Index, Node> Level;

    class Octree
    {
        public:
            Octree(unsigned int maxDepth, BoundingBox& bbox);
            Octree(unsigned int maxDepth, PointCloud& pointCloud);
            PointCloud generatePointCloud(); // convert the octree back to point cloud 
            
            BoundingBox getBoundingBox() const;
            Eigen::Vector3f getLeafCellSize() const;
            std::vector<Level>& getLevels();
            unsigned int getMaxDepth() const;
            size_t getNumOfAllNodes() const;
            Node& addNode(const unsigned int level, const Index& index, const unsigned char child);
            bool addNodeRecursive(const unsigned int level, const Index& index, const unsigned int childIndex, size_t& transversalCounter);
            static Vector3ui getChildOffset(unsigned char childId);

            Index computeLeafAddress(const Eigen::Vector3f& point);
            Index computeParentAddress(const unsigned int currentLevel, const unsigned int parentLevel, const Index& index);
            Index computeParentAddress(const Index& index);
            bool nodeExist(const unsigned int level, const Index& index);

            size_t bottomup;
            size_t topdown;

        protected:
            void generate(unsigned int maxDepth, PointCloud& pointCloud);
            BoundingBox computeBoundingBox(PointCloud& pointCloud);
            Eigen::Vector3f computeLeafCellSize(const unsigned int maxDepth, const BoundingBox& bbox);
            
            
            unsigned char computeOctreeChildIndex(const Index& index);
            bool addLeaf(const unsigned int maxDepth, const Index& index, size_t& transversalCounter);
            
            
            void addNodeChild(const unsigned int level, const Index& parentIndex, const unsigned int childIndex);

            // a vector to store each level
            // each level is a map of node
            std::vector<Level> levels;
            std::vector<tbb::mutex> levelMutexs;
            BoundingBox bbox;
            Eigen::Vector3f leafCellSize;
    };
}
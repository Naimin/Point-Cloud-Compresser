#pragma once
#include <tbb/atomic.h>
#include <tbb/mutex.h>
#include <vector>
#include <map>
#include <set>
#include <atomic>

#include "PointCloud.h"
#include "BoundingBox.h"
#include "Index.h"
#include "MortonCode.h"

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

    struct Level
    {
        Level(unsigned char level)
        {
            auto maxDimension = 2 ^ level;
            nodes.resize(3 * 2097152 );
        }

        void insert(const Index& index, Node& node)
        {
            auto address = MortonCode::encode(index);
            this->operator[](address).children = node.children.load();
            available.insert(address);
        }

        // add the overload operator []
        Node& operator[] (const FullAddress& x) {
            return nodes[x];
        }

        Node& operator[] (const Index& index)
        {
            auto address = MortonCode::encode(index);
            return this->operator[](address);
        }

        size_t size() const
        {
            return available.size();
        }

        std::vector<Node> nodes;
        std::set<FullAddress> available;
    };    
    //typedef std::map<Index, Node> Level;
    
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
            bool addNodeRecursive(const unsigned int level, const Index& index, const unsigned int childIndex);
            static Vector3ui getChildOffset(unsigned char childId);

        protected:
            void generate(unsigned int maxDepth, PointCloud& pointCloud);
            BoundingBox computeBoundingBox(PointCloud& pointCloud);
            Eigen::Vector3f computeLeafCellSize(const unsigned int maxDepth, const BoundingBox& bbox);
            Index computeLeafAddress(const BoundingBox& bbox, const Eigen::Vector3f& leafCellSize, const Eigen::Vector3f& point);
            Index computeParentAddress(const Index& index);
            unsigned char computeOctreeChildIndex(const Index& index);
            bool addLeaf(const unsigned int maxDepth, const Index& index);
            bool nodeExist(const unsigned int level, const Index& index);
            
            void addNodeChild(const unsigned int level, const Index& parentIndex, const unsigned int childIndex);

            // a vector to store each level
            // each level is a map of node
            std::vector<Level> levels;
            std::vector<tbb::mutex> levelMutexs;
            BoundingBox bbox;
            Eigen::Vector3f leafCellSize;
    };
}
#include "Decoder.h"

using namespace CPC;

Decoder::Decoder()
{
}

Decoder::~Decoder()
{
}

Octree Decoder::decode(EncodedData& data)
{
    Octree octree(data.maxDepth);
    
    DepthFirstTransversal(data, octree);
    
    return octree;
}

void Decoder::DepthFirstTransversal(EncodedData & data, Octree & octree)
{

}

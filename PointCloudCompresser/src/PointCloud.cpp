#include "PointCloud.h"

PCC::PointCloud::PointCloud(bool hasNormal_, bool hasColor_) : hasNormal(hasNormal_), hasColor(hasColor_)
{
}

void PCC::PointCloud::resize(size_t size)
{
    positions.resize(size);
    if (hasNormal)
        normals.resize(size);
    if (hasColor)
        colors.resize(size);
}

bool PCC::PointCloud::isValid()
{
    return !positions.empty();
}

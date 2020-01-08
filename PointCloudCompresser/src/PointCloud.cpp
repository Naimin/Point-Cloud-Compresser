#include "PointCloud.h"

CPC::PointCloud::PointCloud(bool hasNormal_, bool hasColor_, bool hasScalar_) : hasNormal(hasNormal_), hasColor(hasColor_), hasScalar(hasScalar_)
{
}

void CPC::PointCloud::resize(size_t size)
{
    positions.resize(size);
    if (hasNormal)
        normals.resize(size);
    if (hasColor)
        colors.resize(size);
    if (hasScalar)
        scalars.resize(size);
}

void CPC::PointCloud::shrink_to_fit()
{
    positions.shrink_to_fit();
    if (hasNormal)
        normals.shrink_to_fit();
    if (hasColor)
        colors.shrink_to_fit();
    if (hasScalar)
        scalars.shrink_to_fit();
}

bool CPC::PointCloud::isValid()
{
    return !positions.empty();
}

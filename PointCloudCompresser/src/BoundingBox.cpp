#include "BoundingBox.h"
#include <numeric>

using namespace CPC;

BoundingBox::BoundingBox() : min(Eigen::Vector3f(std::numeric_limits<float>::max(),
                                                      std::numeric_limits<float>::max(), 
                                                      std::numeric_limits<float>::max())),
                                  max(Eigen::Vector3f(std::numeric_limits<float>::lowest(), 
                                                      std::numeric_limits<float>::lowest(), 
                                                      std::numeric_limits<float>::lowest()))
{
}

void BoundingBox::expand(const Eigen::Vector3f & point)
{
    min.x() = min.x() < point.x() ? min.x() : point.x();
    min.y() = min.y() < point.y() ? min.y() : point.y();
    min.z() = min.z() < point.z() ? min.z() : point.z();
    max.x() = max.x() > point.x() ? max.x() : point.x();
    max.y() = max.y() > point.y() ? max.y() : point.y();
    max.z() = max.z() > point.z() ? max.z() : point.z();
}

bool BoundingBox::isInside(const Eigen::Vector3f& point)
{
    return (point.x() >= min.x() && point.x() <= max.x() &&
            point.y() >= min.y() && point.y() <= max.y() &&
            point.z() >= min.z() && point.z() <= max.z() );
}


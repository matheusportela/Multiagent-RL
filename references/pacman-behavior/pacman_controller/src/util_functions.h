#ifndef UTIL_FUNCTIONS_H
#define UTIL_FUNCTIONS_H

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"

namespace util
{
    geometry_msgs::Point createPoint(int x, int y, int z);
    geometry_msgs::Pose createPose(int x, int y);

    geometry_msgs::Pose sumPoses(geometry_msgs::Pose pose, geometry_msgs::Pose pose_diff);

    extern int MAX_DISTANCE;

    geometry_msgs::Pose actionToMovement(int action);
}

#endif // UTIL_FUNCTIONS_H
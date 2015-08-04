#ifndef UTIL_FUNCTIONS_H
#define UTIL_FUNCTIONS_H

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"

/**
 * Abstract file that implements util functions for the pacman game under the util namespace.
 * 
 * @author Tiago Pimentel Martins da Silva
 */
namespace util
{
    geometry_msgs::Point createPoint(int x, int y, int z);
    geometry_msgs::Pose createPose(int x, int y);

    geometry_msgs::Pose sumPoses(geometry_msgs::Pose pose, geometry_msgs::Pose pose_diff);

    extern int MAX_DISTANCE;
    extern int INFINITE;

    geometry_msgs::Pose actionToMovement(int action);
    geometry_msgs::Pose move(geometry_msgs::Pose agent_pose, int action);

    double getProbOfMeasurementGivenPosition(double pos_x, double pos_y, double measurement_x, double measurement_y, double standard_deviation);
}

#endif // UTIL_FUNCTIONS_H
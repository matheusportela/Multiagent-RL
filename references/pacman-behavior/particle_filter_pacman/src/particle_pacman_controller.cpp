#include "ros/ros.h"

#include "particle_filter_pacman/particle_filter.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "particle_filter");
    ros::NodeHandle n;
    ros::Rate loop_rate(60);

    ParticleFilter particle_filter;

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
#ifndef PACMAN_BEHAVIORS_H
#define PACMAN_BEHAVIORS_H

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "pacman_interface/PacmanAction.h"

pacman_interface::PacmanAction getEatAction(GameInfo game_info);

int main(int argc, char **argv);

#endif // PACMAN_BEHAVIORS_H
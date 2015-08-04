#include "behavior_keyboard_agent.h"

#include "ros/ros.h"
#include "pacman_interface/PacmanAction.h"
#include "std_msgs/String.h"

#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <string.h>

using namespace std;

void BehaviorKeyboardAgent::keypressCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
    string key = msg->data;
    boost::to_upper(key);
    ROS_WARN_STREAM_THROTTLE(2, "behavior: " << key);
    if( find( this->validKeys.begin(), this->validKeys.end(), key) != this->validKeys.end() )
        keyPressed = key;
}

BehaviorKeyboardAgent::BehaviorKeyboardAgent()
{
    keypressSubscriber = n_.subscribe<std_msgs::String>("/pacman_interface/keypress", 1000, boost::bind(&BehaviorKeyboardAgent::keypressCallback, this, _1));
    action_publisher_ = n_.advertise<pacman_interface::PacmanAction>("/pacman_interface/pacman_action", 1000);

    string validKeysArray[] = {"W", "E", "A", "S", "D"};
    this->validKeys.assign(validKeysArray, validKeysArray + (sizeof(validKeysArray)/sizeof(string)) );

    pacman_interface::PacmanAction actions;
    keyToBehavior["W"]  = STOP;
    keyToBehavior["E"]  = EAT;
    keyToBehavior["A"]  = EAT_BIG_FOOD;
    keyToBehavior["S"]  = RUN;
    keyToBehavior["D"]  = HUNT;

    this->keyPressed = "W";

    /*for(vector<string>::iterator it = this->validKeys.begin(); it != this->validKeys.end(); ++it) { 
        cout << *it << std::endl;
    }*/
    ROS_DEBUG("BehaviorKeyboardAgent initialized");
}

void BehaviorKeyboardAgent::sendAction(GameInfo game_info)
{
    pacman_interface::PacmanAction action;
    int behavior = keyToBehavior[this->keyPressed];
    ROS_WARN_STREAM_THROTTLE(2, "behavior: " << behavior);
    switch (behavior)
    {
        case STOP:
            action = getStopAction();
            break;
        case EAT:
            action = getEatAction(game_info);
            break;
        case EAT_BIG_FOOD:
            action = getEatBigFoodAction(game_info);
            break;
        case RUN:
            action = getRunAction(game_info);
            break;
        case HUNT:
            action = getHuntAction(game_info);
            break;
        default:
            action.action = action.STOP;
            break;
    }

    action_publisher_.publish(action);
}

string BehaviorKeyboardAgent::getAgentName()
{
    return "BehaviorKeyboardAgent";
}

pacman_interface::PacmanAction BehaviorKeyboardAgent::getHuntAction(GameInfo game_info) {
    pacman_interface::PacmanAction action;
    GameInfo game_info_ = game_info;

    geometry_msgs::Pose pacman_pose = game_info_.getPacmanPose();
    std::map< std::pair<int, int>, int > distances = game_info_.getDistances(pacman_pose.position.x, pacman_pose.position.y);

    int min_distance = util::MAX_DISTANCE;
    game_info.printMap();

    std::vector< geometry_msgs::Pose > ghosts_poses = game_info_.getGhostsPoses();
    std::vector< geometry_msgs::Pose >::reverse_iterator closest_ghost = ghosts_poses.rend();
        ROS_INFO_STREAM("Ghost size: " << ghosts_poses.size() << " num: " << game_info_.getNumberOfGhosts(););  
    for(std::vector< geometry_msgs::Pose >::reverse_iterator it = ghosts_poses.rbegin(); it != ghosts_poses.rend(); ++it) {
        /* std::cout << *it; ... */
        int distance = distances[std::make_pair(it->position.x, it->position.y)];
        ROS_INFO_STREAM("Ghost x: " << it->position.x << " y: " << it->position.y );    
        if(distance != 0 && distance < min_distance)
        {
            closest_ghost = it;
            min_distance = distance;
        }
    }

    if (closest_ghost == ghosts_poses.rend())
    {
        action.action = action.STOP;
        return action;
    }

    ROS_INFO_STREAM("x: " << closest_ghost->position.x << " y: " << closest_ghost->position.y );

    distances = game_info_.getDistances(closest_ghost->position.x, closest_ghost->position.y);
    std::vector< pacman_interface::PacmanAction > actions = game_info_.getLegalActions(pacman_pose.position.x, pacman_pose.position.y);
    std::vector< std::pair<int, int> > next_positions = game_info_.getLegalNextPositions(pacman_pose.position.x, pacman_pose.position.y);

    int action_iterator = 0;
    for(int i = next_positions.size() - 1; i != -1; i--)
    {
        ROS_INFO_STREAM("- distance " << (short) actions[i].action << " : " << distances[ next_positions[i] ] );
        if ( distances[ next_positions[i] ] < min_distance )
        {
            action_iterator = i;
            break;
        }
    }

    return actions[action_iterator];
}

pacman_interface::PacmanAction BehaviorKeyboardAgent::getRunAction(GameInfo game_info) {
    pacman_interface::PacmanAction action;
    GameInfo game_info_ = game_info;

    geometry_msgs::Pose pacman_pose = game_info_.getPacmanPose();
    std::map< std::pair<int, int>, int > distances = game_info_.getDistances(pacman_pose.position.x, pacman_pose.position.y);

    int min_distance = util::MAX_DISTANCE;

    std::vector< geometry_msgs::Pose > ghosts_poses = game_info_.getGhostsPoses();
    std::vector< geometry_msgs::Pose >::reverse_iterator closest_ghost;
    for(std::vector< geometry_msgs::Pose >::reverse_iterator it = ghosts_poses.rbegin(); it != ghosts_poses.rend(); ++it) {
        /* std::cout << *it; ... */
        int distance = distances[std::make_pair(it->position.x, it->position.y)];
        if(distance != 0 && distance < min_distance)
        {
            closest_ghost = it;
            min_distance = distance;
        }
    }

    game_info.printMap();
    ROS_INFO_STREAM("x: " << closest_ghost->position.x << " y: " << closest_ghost->position.y );

    distances = game_info_.getDistances(closest_ghost->position.x, closest_ghost->position.y);
    std::vector< pacman_interface::PacmanAction > actions = game_info_.getLegalActions(pacman_pose.position.x, pacman_pose.position.y);
    std::vector< std::pair<int, int> > next_positions = game_info_.getLegalNextPositions(pacman_pose.position.x, pacman_pose.position.y);

    int action_iterator = 0;
    for(int i = next_positions.size() - 1; i != -1; i--)
    {
        ROS_INFO_STREAM("- distance " << (short) actions[i].action << " : " << distances[ next_positions[i] ] );
        if ( distances[ next_positions[i] ] > min_distance )
        {
            action_iterator = i;
            break;
        }
    }

    return actions[action_iterator];
}

pacman_interface::PacmanAction BehaviorKeyboardAgent::getEatBigFoodAction(GameInfo game_info) {
    pacman_interface::PacmanAction action;
    GameInfo game_info_ = game_info;

    geometry_msgs::Pose pacman_pose = game_info_.getPacmanPose();
    std::map< std::pair<int, int>, int > distances = game_info_.getDistances(pacman_pose.position.x, pacman_pose.position.y);

    int width = game_info_.getWidth();
    int height = game_info_.getHeight();

    int min_distance = util::MAX_DISTANCE;
    int food_x = -1;
    int food_y = -1;
    for (int i = 0 ; i < width ; i++)
    {
        for (int j = 0 ; j < height ; j++)
        {
            if( game_info_.getMapElement(i, j) == GameInfo::BIG_FOOD)
            {
                int distance = distances[std::make_pair(i, j)];
                if(distance < min_distance)
                {
                    food_x = i;
                    food_y = j;
                    min_distance = distance;
                }
            }
        }
    }

    distances = game_info_.getDistances(food_x, food_y);
    std::vector< pacman_interface::PacmanAction > actions = game_info_.getLegalActions(pacman_pose.position.x, pacman_pose.position.y);
    std::vector< std::pair<int, int> > next_positions = game_info_.getLegalNextPositions(pacman_pose.position.x, pacman_pose.position.y);

    int action_iterator = 0;
    for(int i = next_positions.size() - 1; i != -1; i--)
    {
        if ( distances[ next_positions[i] ] < min_distance )
        {
            action_iterator = i;
            break;
        }
    }

    return actions[action_iterator];
}

pacman_interface::PacmanAction BehaviorKeyboardAgent::getEatAction(GameInfo game_info) {
    pacman_interface::PacmanAction action;
    GameInfo game_info_ = game_info;

    geometry_msgs::Pose pacman_pose = game_info_.getPacmanPose();
    std::map< std::pair<int, int>, int > distances = game_info_.getDistances(pacman_pose.position.x, pacman_pose.position.y);

    int width = game_info_.getWidth();
    int height = game_info_.getHeight();

    int min_distance = util::MAX_DISTANCE;
    int food_x = -1;
    int food_y = -1;
    for (int i = 0 ; i < width ; i++)
    {
        for (int j = 0 ; j < height ; j++)
        {
            if( game_info_.getMapElement(i, j) == GameInfo::FOOD)
            {
                int distance = distances[std::make_pair(i, j)];
                if(distance < min_distance)
                {
                    food_x = i;
                    food_y = j;
                    min_distance = distance;
                }
            }
        }
    }

    distances = game_info_.getDistances(food_x, food_y);
    std::vector< pacman_interface::PacmanAction > actions = game_info_.getLegalActions(pacman_pose.position.x, pacman_pose.position.y);
    std::vector< std::pair<int, int> > next_positions = game_info_.getLegalNextPositions(pacman_pose.position.x, pacman_pose.position.y);

    int action_iterator = 0;
    for(int i = next_positions.size() - 1; i != -1; i--)
    {
        if ( distances[ next_positions[i] ] < min_distance )
        {
            action_iterator = i;
            break;
        }
    }

    return actions[action_iterator];
}

pacman_interface::PacmanAction BehaviorKeyboardAgent::getStopAction() {
    pacman_interface::PacmanAction action;
    action.action = action.STOP;
    return action;
}
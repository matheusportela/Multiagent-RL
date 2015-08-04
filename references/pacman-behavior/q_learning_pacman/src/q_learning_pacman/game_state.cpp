#include "q_learning_pacman/game_state.h"

#include <sstream>

#include "pacman_msgs/PacmanMapInfo.h"

int GameState::MAX_DISTANCE = 1000000;

// TODO: check if set map_ to food instead of option is ok

GameState::GameState()
{
    ROS_DEBUG_STREAM("Initialize game state");
    ros::ServiceClient initInfoClient = n_.serviceClient<pacman_msgs::PacmanMapInfo>("/pacman/initialize_map_layout");
    pacman_msgs::PacmanMapInfo initInfo;

    ros::service::waitForService("/pacman/initialize_map_layout", -1);

    if (initInfoClient.call(initInfo))
    {
        std::vector<unsigned char> map_msg = initInfo.response.layout.map;
        int num_initialized_ghost = 0;

        width_ = initInfo.response.layout.width;
        height_ = initInfo.response.layout.height;

        num_ghosts_ = (int) initInfo.response.numGhosts;

        pacman_msgs::MapLayout map_layout;

        std::vector<float> pose_line (width_, 0);
        pacman_pose_map_ = std::vector< std::vector<float> > (height_, pose_line);

        for (int i = 0 ; i < num_ghosts_; i++)
        {
            ghosts_poses_map_.push_back( std::vector< std::vector<float> > (height_, pose_line));
        }

        for (int i = 0 ; i < height_ ; i++) {
            std::vector<MapElements> map_line;
            std::vector<float> foods_map_line;
            std::vector<float> big_foods_map_line;
            for (int j = 0 ; j < width_ ; j++) {
                    float has_food = 0.0;
                    float has_big_food = 0.0;

                    if (map_msg[i * width_ + j] == map_layout.EMPTY)
                        map_line.push_back(EMPTY);
                    else if (map_msg[i * width_ + j] == map_layout.FOOD)
                    {
                        map_line.push_back(FOOD);
                        has_food = 1.0;
                    }
                    else if (map_msg[i * width_ + j] == map_layout.BIG_FOOD)
                    {
                        map_line.push_back(BIG_FOOD);
                        has_big_food = 1.0;
                    }
                    else if (map_msg[i * width_ + j] == map_layout.WALL)
                        map_line.push_back(WALL);
                    else if (map_msg[i * width_ + j] == map_layout.GHOST)
                    {
                        ROS_DEBUG_STREAM("Ghsot in " << i << " and " << j);
                        if (num_initialized_ghost < num_ghosts_)
                        {
                            // deterministic variable
                            geometry_msgs::Pose new_pose;
                            new_pose.position.x = j;
                            new_pose.position.y = i;
                            ghosts_poses_.push_back(new_pose);
                            ghosts_spawn_poses_.push_back(new_pose);
                            // probabilistic variable
                            ghosts_poses_map_[num_initialized_ghost][i][j] = 1.0;

                            num_initialized_ghost++;
                        }
                        map_line.push_back(EMPTY);
                    }
                    else if (map_msg[i * width_ + j] == map_layout.PACMAN)
                    {
                        // deterministic variable
                        pacman_pose_.position.x = j;
                        pacman_pose_.position.y = i;

                        // probabilistic variable
                        pacman_pose_map_[i][j] = 1.0;

                        map_line.push_back(EMPTY);
                    }
                    else
                    {
                        map_line.push_back(ERROR);
                        std::cout << "Error reading map";
                    }

                    foods_map_line.push_back(has_food);
                    big_foods_map_line.push_back(has_big_food);
            }
            map_.push_back(map_line);
            foods_map_.push_back(foods_map_line);
            big_foods_map_.push_back(big_foods_map_line);
        }

        if (num_ghosts_ > num_initialized_ghost) {
            ghosts_poses_map_.erase(ghosts_poses_map_.begin() + num_initialized_ghost, ghosts_poses_map_.begin() + num_ghosts_);
            num_ghosts_ = num_initialized_ghost;
        }
        
        std::vector<float> probability_ghosts_white_line (num_ghosts_, 0);
        probability_ghosts_white_ = std::vector< std::vector<float> > (40, probability_ghosts_white_line);

        ROS_DEBUG_STREAM("Map width " << width_ << " height " << height_ << " num ghosts " << num_ghosts_);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to call service /pacman/initialize_map_layout");
    }

    ROS_DEBUG_STREAM("Initialize ghost size " << ghosts_poses_map_.size());
}

GameState::~GameState()
{
        ROS_DEBUG_STREAM("Game state being destroyed");

        for(std::vector< std::vector<MapElements> >::reverse_iterator it = map_.rbegin(); it != map_.rend(); ++it) {
            it->clear();
        }
        map_.clear();

        for(std::vector< std::vector<float> >::reverse_iterator it = pacman_pose_map_.rbegin(); 
                            it != pacman_pose_map_.rend(); ++it) {
            it->clear();
        }
        pacman_pose_map_.clear();

        for(std::vector< std::vector< std::vector<float> > >::reverse_iterator it = ghosts_poses_map_.rbegin();
                            it != ghosts_poses_map_.rend(); ++it) {
            for(std::vector< std::vector<float> >::reverse_iterator it_2 = it->rbegin();
                                it_2 != it->rend(); ++it_2) {
                it_2->clear();
            }
            it->clear();
        }
        ghosts_poses_map_.clear();

        for(std::vector< std::vector<float> >::reverse_iterator it = foods_map_.rbegin(); it != foods_map_.rend(); ++it) {
            it->clear();
        }
        foods_map_.clear();

        for(std::vector< std::vector<float> >::reverse_iterator it = big_foods_map_.rbegin(); it != big_foods_map_.rend(); ++it) {
            it->clear();
        }
        big_foods_map_.clear();

        ROS_DEBUG_STREAM("Game state destroyed");
}

void GameState::printDeterministicMap()
{
    geometry_msgs::Pose pacman_pose = pacman_pose_;
    std::vector< geometry_msgs::Pose > ghosts_poses = ghosts_poses_;

    for (int i = height_ -1 ; i > -1  ; i--) {
        std::ostringstream foo;
        for (int j = 0 ; j < width_ ; j++) {
                bool is_ghost = false;
                for(int ghost_counter = 0; ghost_counter < num_ghosts_ ; ghost_counter++)
                {
                    if (ghosts_poses[ghost_counter].position.x == j && ghosts_poses[ghost_counter].position.y == i)
                    {
                        foo << 'G';
                        is_ghost = true;
                        break;
                    }
                }
                if (is_ghost)
                    continue;
                else if (pacman_pose.position.x == j && pacman_pose.position.y == i)
                    foo << 'P';
                else if (map_[i][j] == FOOD)
                {
                    //foo << "\033[48;2;0;0;0m" << '.' << "\033[0m";
                    foo << "\033[48;5;46m" << '.' << "\033[0m";
                }
                else if (map_[i][j] == BIG_FOOD)
                    foo << 'B';
                else if (map_[i][j] == WALL)
                    foo << '#';
                else if(map_[i][j] == EMPTY)
                    foo << ' ';
                else
                    foo << 'E';
        }
        ROS_INFO_STREAM(foo.str());
    }
    ROS_INFO_STREAM("Sum: " << num_ghosts_ << std::endl);
}

void GameState::printMap()
{
    geometry_msgs::Pose pacman_pose = pacman_pose_;
    std::vector< geometry_msgs::Pose > ghosts_poses = ghosts_poses_;

    for (int i = height_ -1 ; i > -1  ; i--) {
        std::ostringstream foo;
        for (int j = 0 ; j < width_ ; j++) {
                bool is_ghost = false;
                for(int ghost_counter = 0; ghost_counter < num_ghosts_ ; ghost_counter++)
                {
                    if (ghosts_poses[ghost_counter].position.x == j && ghosts_poses[ghost_counter].position.y == i)
                    {
                        foo << 'G';
                        is_ghost = true;
                        break;
                    }
                }
                if (is_ghost)
                    continue;
                else if (pacman_pose.position.x == j && pacman_pose.position.y == i)
                    foo << 'P';
                else if (foods_map_[i][j] >= 0.3)
                {
                    //foo << "\033[48;2;0;0;0m" << '.' << "\033[0m";
                    if (foods_map_[i][j] >= 0.9)
                        foo << "\033[48;5;46m";
                    else if (foods_map_[i][j] >= 0.5)
                        foo << "\033[48;5;30m";
                    else if (foods_map_[i][j] >= 0.3)
                        foo << "\033[48;5;22m";
                    foo << '.' << "\033[0m";
                }
                else if (map_[i][j] == BIG_FOOD)
                    foo << 'B';
                else if (map_[i][j] == WALL)
                    foo << '#';
                else if(map_[i][j] == EMPTY)
                    foo << ' ';
                else
                    foo << 'E';
        }
        ROS_INFO_STREAM(foo.str());
    }
    ROS_INFO_STREAM("Sum: " << num_ghosts_ << std::endl);
}

void GameState::printFoodsMap()
{
    for (int i = height_ -1 ; i > -1  ; i--) {
        std::ostringstream foo;
        foo << std::fixed;
        foo << std::setprecision(0);
        //foo << std::setw(5) << std::setfill('0');
        for (int j = 1 ; j < width_ - 1 ; j++) {
            if( getMapElement(j, i) == WALL)
                foo << "###" << ' ';
            else
            {
                int chance = foods_map_[i][j]*100;

                if (chance >= 90)
                    foo << "\033[48;5;46m";
                else if (chance >= 50)
                    foo << "\033[48;5;30m";
                else if (chance >= 30)
                    foo << "\033[48;5;22m";
                else
                    foo << "\033[48;5;12m";

                foo << std::setw(3) << std::setfill('0') << chance;

                foo << "\033[0m" << ' ';
            }
        }
        ROS_INFO_STREAM(foo.str());
    }
}

void GameState::printBigFoodsMap()
{
    for (int i = height_ -1 ; i > -1  ; i--) {
        std::ostringstream foo;
        foo << std::fixed;
        foo << std::setprecision(0);
        //foo << std::setw(5) << std::setfill('0');
        for (int j = 1 ; j < width_ - 1 ; j++) {
            if( getMapElement(j, i) == WALL)
                foo << "###" << ' ';
            else
            {
                int chance = big_foods_map_[i][j]*100;

                if (chance >= 90)
                    foo << "\033[48;5;46m";
                else if (chance >= 50)
                    foo << "\033[48;5;30m";
                else if (chance >= 30)
                    foo << "\033[48;5;22m";
                else
                    foo << "\033[48;5;12m";

                foo << std::setw(3) << std::setfill('0') << chance;

                foo << "\033[0m" << ' ';
            }
        }
        ROS_INFO_STREAM(foo.str());
    }
}

void GameState::printWhiteGhostsProbabilities()
{
    for(std::vector< std::vector<float> >::reverse_iterator it = probability_ghosts_white_.rbegin(); it != probability_ghosts_white_.rend(); ++it)
    {
        std::ostringstream foo;
        foo << std::fixed;
        foo << std::setprecision(0);

        for(int ghost_index = 0; ghost_index < num_ghosts_ ; ++ghost_index)
        {
            int chance = (*it)[ghost_index] * 100;

            if (chance >= 70)
                foo << "\033[48;5;46m";
            else if (chance >= 40)
                foo << "\033[48;5;30m";
            else if (chance >= 20)
                foo << "\033[48;5;22m";
            else
                foo << "\033[48;5;12m";

            foo << std::setw(3) << std::setfill('0') << chance;
            foo << "\033[0m" << ' ';
        }
        ROS_INFO_STREAM(foo.str());
    }
}

void GameState::printPacmanOrGhostPose( bool is_pacman, int ghost_index)
{
    for (int i = height_ -1 ; i > -1  ; i--) {
        std::ostringstream foo;
        foo << std::fixed;
        foo << std::setprecision(0);
        //foo << std::setw(5) << std::setfill('0');
        for (int j = 1 ; j < width_ - 1 ; j++) {
            if( getMapElement(j, i) == WALL)
                foo << "###" << ' ';
            else
            {
                int chance = -1;

                if (is_pacman)
                    chance = pacman_pose_map_[i][j]*100;
                else
                    chance = ghosts_poses_map_[ghost_index][i][j]*100;

                if (chance >= 90)
                    foo << "\033[48;5;46m";
                else if (chance >= 50)
                    foo << "\033[48;5;30m";
                else if (chance >= 30)
                    foo << "\033[48;5;22m";
                else
                    foo << "\033[48;5;12m";

                foo << std::setw(3) << std::setfill('0') << chance;

                foo << "\033[0m" << ' ';
            }
        }
        ROS_INFO_STREAM(foo.str());
    }
}

GameState::MapElements GameState::getMapElement(int x, int y)
{
    return map_[y][x];
}

int GameState::getHeight()
{
    return height_;
}

int GameState::getWidth()
{
    return width_;
}

std::vector< pacman_msgs::PacmanAction > GameState::getLegalActions()
{
    return getLegalActions(pacman_pose_.position.x, pacman_pose_.position.y);
}

std::vector< pacman_msgs::PacmanAction > GameState::getLegalActions(int x, int y)
{
    std::vector< pacman_msgs::PacmanAction > legal_actions;

    if(map_[y][x] == WALL)
    {
        return legal_actions;
    }

    if(map_[y + 1][x] != WALL)
    {
        pacman_msgs::PacmanAction action;
        action.action = pacman_msgs::PacmanAction::NORTH;
        legal_actions.push_back(action);
    }
    if(map_[y - 1][x] != WALL)
    {
        pacman_msgs::PacmanAction action;
        action.action = pacman_msgs::PacmanAction::SOUTH;
        legal_actions.push_back(action);
    }
    if(map_[y][x + 1] != WALL)
    {
        pacman_msgs::PacmanAction action;
        action.action = pacman_msgs::PacmanAction::EAST;
        legal_actions.push_back(action);
    }
    if(map_[y][x - 1] != WALL)
    {
        pacman_msgs::PacmanAction action;
        action.action = pacman_msgs::PacmanAction::WEST;
        legal_actions.push_back(action);
    }

    return legal_actions;
}

std::vector< std::pair<int, int> > GameState::getLegalNextPositions(int x, int y)
{
    std::vector< std::pair<int, int> > legal_next_positions;

    if(map_[y][x] == WALL)
    {
        return legal_next_positions;
    }

    if(map_[y + 1][x] != WALL)
    {
        legal_next_positions.push_back(std::make_pair(x, y+1));
    }
    if(map_[y - 1][x] != WALL)
    {
        legal_next_positions.push_back(std::make_pair(x, y-1));
    }
    if(map_[y][x + 1] != WALL)
    {
        legal_next_positions.push_back(std::make_pair(x+1, y));
    }
    if(map_[y][x - 1] != WALL)
    {
        legal_next_positions.push_back(std::make_pair(x-1, y));
    }

    return legal_next_positions;
}

std::vector< std::pair< float, std::pair<int, int> > > GameState::getNextPositionsForActionWithProbabilities(int x, int y, pacman_msgs::PacmanAction action)
{
    std::vector< std::pair< float, std::pair<int, int> > > legal_next_positions_with_probabilities;

    if(map_[y][x] == WALL)
    {
        return legal_next_positions_with_probabilities;
    }

    float CHANCE_OF_ACTION_SUCCESS = 0.999;
    float chance_of_other_moves = (1 - CHANCE_OF_ACTION_SUCCESS )/4.0;

    float north_probability = ( action.action == pacman_msgs::PacmanAction::NORTH ) ? CHANCE_OF_ACTION_SUCCESS : chance_of_other_moves;
    float south_probability = ( action.action == pacman_msgs::PacmanAction::SOUTH ) ? CHANCE_OF_ACTION_SUCCESS : chance_of_other_moves;
    float west_probability = ( action.action == pacman_msgs::PacmanAction::WEST ) ? CHANCE_OF_ACTION_SUCCESS : chance_of_other_moves;
    float east_probability = ( action.action == pacman_msgs::PacmanAction::EAST ) ? CHANCE_OF_ACTION_SUCCESS : chance_of_other_moves;
    float stop_probability = ( action.action == pacman_msgs::PacmanAction::STOP ) ? CHANCE_OF_ACTION_SUCCESS : chance_of_other_moves;

    if(map_[y + 1][x] != WALL)
        legal_next_positions_with_probabilities.push_back(std::make_pair(north_probability, std::make_pair(x, y+1)));
    else
        stop_probability += north_probability;
    if(map_[y - 1][x] != WALL)
        legal_next_positions_with_probabilities.push_back(std::make_pair(south_probability, std::make_pair(x, y-1)));
    else
        stop_probability += south_probability;
    if(map_[y][x - 1] != WALL)
        legal_next_positions_with_probabilities.push_back(std::make_pair(west_probability, std::make_pair(x-1, y)));
    else
        stop_probability += west_probability;
    if(map_[y][x + 1] != WALL)
        legal_next_positions_with_probabilities.push_back(std::make_pair(east_probability, std::make_pair(x+1, y)));
    else
        stop_probability += east_probability;
    legal_next_positions_with_probabilities.push_back(std::make_pair(stop_probability, std::make_pair(x, y)));

    return legal_next_positions_with_probabilities;
}

int GameState::getNumberOfGhosts()
{
    return num_ghosts_;
}

std::vector< std::vector<float> > GameState::getPacmanPoseMap()
{
    return pacman_pose_map_;
}

void GameState::setPacmanPoseMap(std::vector< std::vector<float> > pacman_pose_map)
{
    pacman_pose_map_ = pacman_pose_map;
}

std::vector< std::vector<float> > GameState::getGhostPoseMap(int ghost_index)
{
    return ghosts_poses_map_[ghost_index];
}

std::vector< std::vector< std::vector<float> > > GameState::getGhostsPoseMaps()
{
    return ghosts_poses_map_;
}

std::vector< std::vector<float> > GameState::getFoodMap()
{
    return foods_map_;
}

std::vector< std::vector<float> > GameState::getBigFoodMap()
{
    return big_foods_map_;
}

void GameState::setGhostPoseMap(std::vector< std::vector<float> > ghost_pose_map, int ghost_index)
{
    ghosts_poses_map_[ghost_index] = ghost_pose_map;
}

geometry_msgs::Pose GameState::getPacmanPose()
{
    return pacman_pose_;
}

geometry_msgs::Pose GameState::getGhostPose(int ghost_index)
{
    return ghosts_poses_[ghost_index];
}

std::vector< geometry_msgs::Pose > GameState::getGhostsPoses()
{
    return ghosts_poses_;
}
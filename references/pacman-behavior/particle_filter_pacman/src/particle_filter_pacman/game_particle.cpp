#include "particle_filter_pacman/game_particle.h"

#include <sstream>

#include "particle_filter_pacman/util_constants.h"

#include "pacman_interface/PacmanAction.h"
#include "pacman_interface/AgentAction.h"
#include "pacman_interface/PacmanMapInfo.h"

bool GameParticle::particle_initialized = false;

GameParticle::GameParticle()
{
    if(!particle_initialized)
    {
        std::srand(std::time(NULL));
        particle_initialized = true;
    }

    ros::ServiceClient initInfoClient = n_.serviceClient<pacman_interface::PacmanMapInfo>("pacman_initialize_map_layout");
    pacman_interface::PacmanMapInfo initInfo;

    score_ = 0;

    ros::service::waitForService("pacman_initialize_map_layout", -1);

    if (initInfoClient.call(initInfo))
    {
        std::vector<unsigned char> map_msg = initInfo.response.layout.map;
        int num_initialized_ghost = 0;

        width_ = initInfo.response.layout.width;
        height_ = initInfo.response.layout.height;

        num_ghosts_ = (int) initInfo.response.numGhosts;

        white_ghosts_time_ = std::vector<int> (num_ghosts_, 0);

        pacman_interface::MapLayout map_layout;

        for (int i = 0 ; i < height_ ; i++) {
            std::vector<MapElements> map_line;
            for (int j = 0 ; j < width_ ; j++) {
                    if (map_msg[i * width_ + j] == map_layout.EMPTY)
                        map_line.push_back(EMPTY);
                    else if (map_msg[i * width_ + j] == map_layout.FOOD)
                        map_line.push_back(FOOD);
                    else if (map_msg[i * width_ + j] == map_layout.BIG_FOOD)
                        map_line.push_back(BIG_FOOD);
                    else if (map_msg[i * width_ + j] == map_layout.WALL)
                        map_line.push_back(WALL);
                    else if (map_msg[i * width_ + j] == map_layout.GHOST)
                    {
                        if (num_initialized_ghost < num_ghosts_)
                        {
                            geometry_msgs::Pose ghost_pose;
                            ghost_pose.position.x = j;
                            ghost_pose.position.y = i;
                            ghosts_poses_.push_back(ghost_pose);
                            spawn_ghosts_poses_.push_back(ghost_pose);

                            num_initialized_ghost++;
                        }
                        map_line.push_back(EMPTY);
                    }
                    else if (map_msg[i * width_ + j] == map_layout.PACMAN)
                    {
                        pacman_pose_.position.x = j;
                        pacman_pose_.position.y = i;
                        map_line.push_back(EMPTY);
                    }
                    else
                    {
                        map_line.push_back(ERROR);
                        ROS_ERROR_STREAM("Error reading map");
                    }
            }
            map_.push_back(map_line);
        }
    }
    else
    {
        ROS_ERROR("Failed to call service PacmanInitializationInfo");
    }

    ROS_DEBUG_STREAM("Particle initialized");
}

void GameParticle::printMap()
{
    for (int i = height_ -1 ; i > -1  ; i--) {
        std::ostringstream foo;
        for (int j = 0 ; j < width_ ; j++) {
                bool is_ghost = false;
                for(int ghost_counter = 0; ghost_counter < num_ghosts_ ; ghost_counter++)
                {
                    if (ghosts_poses_[ghost_counter].position.x == j && ghosts_poses_[ghost_counter].position.y == i)
                    {
                        foo << 'G';
                        is_ghost = true;
                        break;
                    }
                }
                if (is_ghost)
                    continue;
                else if (pacman_pose_.position.x == j && pacman_pose_.position.y == i)
                    foo << 'P';
                else if (map_[i][j] == FOOD)
                    foo << '.';
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

GameParticle::MapElements GameParticle::getMapElement(int x, int y)
{
    return map_[y][x];
}

int GameParticle::getHeight()
{
    return height_;
}

int GameParticle::getWidth()
{
    return width_;
}

std::vector< pacman_interface::PacmanAction > GameParticle::getLegalActions(int x, int y)
{
    std::vector< pacman_interface::PacmanAction > legal_actions;

    if(map_[y][x] == WALL)
    {
        return legal_actions;
    }

    if(map_[y + 1][x] != WALL)
    {
        pacman_interface::PacmanAction action;
        action.action = actions_.NORTH;
        legal_actions.push_back(action);
    }
    if(map_[y - 1][x] != WALL)
    {
        pacman_interface::PacmanAction action;
        action.action = actions_.SOUTH;
        legal_actions.push_back(action);
    }
    if(map_[y][x + 1] != WALL)
    {
        pacman_interface::PacmanAction action;
        action.action = actions_.EAST;
        legal_actions.push_back(action);
    }
    if(map_[y][x - 1] != WALL)
    {
        pacman_interface::PacmanAction action;
        action.action = actions_.WEST;
        legal_actions.push_back(action);
    }

    return legal_actions;
}

std::vector< std::pair<int, int> > GameParticle::getLegalNextPositions(int x, int y)
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

geometry_msgs::Pose GameParticle::getPacmanPose()
{
    return pacman_pose_;
}

geometry_msgs::Pose GameParticle::getGhostPose(int ghost_index)
{
    return ghosts_poses_[ghost_index];
}

std::vector< geometry_msgs::Pose > GameParticle::getGhostsPoses()
{
    return ghosts_poses_;
}

int GameParticle::getNumberOfGhosts()
{
    return num_ghosts_;
}

std::vector<int> GameParticle::getWhiteGhostsTime()
{
    return white_ghosts_time_;
}

std::vector< std::vector<GameParticle::MapElements> > GameParticle::getMap()
{
    return map_;
}

int GameParticle::getScore()
{
    return score_;
}

std::vector< std::pair< float, std::pair<int, int> > > GameParticle::getNextPositionsWithProbabilities(int x, int y, pacman_interface::PacmanAction action)
{
    std::vector< std::pair< float, std::pair<int, int> > > legal_next_positions_with_probabilities;

    if(map_[y][x] == WALL)
    {
        return legal_next_positions_with_probabilities;
    }

    float CHANCE_OF_ACTION_SUCCESS = util::CHANCE_OF_ACTION_SUCCESS;
    float chance_of_other_moves = (1 - CHANCE_OF_ACTION_SUCCESS )/4.0;

    float north_probability = ( action.action == actions_.NORTH ) ? CHANCE_OF_ACTION_SUCCESS : chance_of_other_moves;
    float south_probability = ( action.action == actions_.SOUTH ) ? CHANCE_OF_ACTION_SUCCESS : chance_of_other_moves;
    float west_probability = ( action.action == actions_.WEST ) ? CHANCE_OF_ACTION_SUCCESS : chance_of_other_moves;
    float east_probability = ( action.action == actions_.EAST ) ? CHANCE_OF_ACTION_SUCCESS : chance_of_other_moves;
    float stop_probability = ( action.action == actions_.STOP ) ? CHANCE_OF_ACTION_SUCCESS : chance_of_other_moves;

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

void GameParticle::movePacman(pacman_interface::PacmanAction action)
{
    int x = pacman_pose_.position.x;
    int y = pacman_pose_.position.y;

    std::vector< std::pair< float, std::pair<int, int> > > next_positions = getNextPositionsWithProbabilities(x, y, action);
    double random_variable = std::rand() / (double) RAND_MAX;
    double sum_probs = 0;

    for(std::vector< std::pair< float, std::pair<int, int> > >::reverse_iterator it = next_positions.rbegin(); it != next_positions.rend(); ++it)
    {
        sum_probs += it->first;
        if(sum_probs >= random_variable)
        {
            pacman_pose_.position.x = it->second.first;
            pacman_pose_.position.y = it->second.second;

            // if food, increase score
            if(map_[it->second.second][it->second.first] == FOOD)
            {
                score_ += 10;
            }
            // if big food, start white ghosts time
            if(map_[it->second.second][it->second.first] == BIG_FOOD)
            {
                for(std::vector<int>::reverse_iterator it = white_ghosts_time_.rbegin(); it != white_ghosts_time_.rend(); ++it)
                {
                    *it = 38;
                }
            }
            map_[it->second.second][it->second.first] = EMPTY;
            break;
        }
    }
}

void GameParticle::moveGhost(std::vector< geometry_msgs::Pose >::reverse_iterator it)
{
    int x = it->position.x;
    int y = it->position.y;

    std::vector< std::pair<int, int> > next_positions = getLegalNextPositions(x, y);
    int random_variable = std::rand() % next_positions.size();

    it->position.x = next_positions[random_variable].first;
    it->position.y = next_positions[random_variable].second;
}

void GameParticle::moveGhosts()
{
    std::vector< int >::reverse_iterator white_it = white_ghosts_time_.rbegin();
    std::vector< geometry_msgs::Pose >::reverse_iterator spawn_pose_it = spawn_ghosts_poses_.rbegin();

    for(std::vector< geometry_msgs::Pose >::reverse_iterator it = ghosts_poses_.rbegin(); it != ghosts_poses_.rend(); ++it, ++white_it, ++spawn_pose_it)
    {
        double random_number_of_moves = std::rand() / (double) RAND_MAX;

        if(*white_it) // if white, change probabilities of moves
        {
            if( random_number_of_moves > util::CHANCE_OF_WHITE_GHOST_STOP )
            {
                random_number_of_moves -= util::CHANCE_OF_WHITE_GHOST_STOP;
                moveGhost(it);

                // if eaten, go to initial position
                if( ( it->position.x == pacman_pose_.position.x ) && ( it->position.y == pacman_pose_.position.y ) )
                {
                    *it = *spawn_pose_it;
                    *white_it = 0;
                    score_ += 500;
                }

                if(random_number_of_moves > util::CHANCE_OF_WHITE_GHOST_ONE_MOVE)
                {   
                    moveGhost(it);

                    // if eaten, go to initial position
                    if( ( it->position.x == pacman_pose_.position.x ) && ( it->position.y == pacman_pose_.position.y ) )
                    {
                        *it = *spawn_pose_it;
                        *white_it = 0;
                        score_ += 500;
                    }
                }
            }
        }
        else // if not white, move normally
        {
            if( random_number_of_moves > util::CHANCE_OF_GHOST_STOP )
            {
                random_number_of_moves -= util::CHANCE_OF_GHOST_STOP;
                moveGhost(it);
                // if kileed, drop score
                if( ( it->position.x == pacman_pose_.position.x ) && ( it->position.y == pacman_pose_.position.y ) )
                {
                    score_ -= 1000;
                }

                if(random_number_of_moves > util::CHANCE_OF_GHOST_ONE_MOVE)
                {   
                    moveGhost(it);
                    // if kileed, drop score
                    if( ( it->position.x == pacman_pose_.position.x ) && ( it->position.y == pacman_pose_.position.y ) )
                    {
                        score_ -= 1000;
                    }
                }
            }
        }
    }
}

void GameParticle::checkIfDeadGhosts()
{
    std::vector< geometry_msgs::Pose >::reverse_iterator pose_it = ghosts_poses_.rbegin();
    std::vector< int >::reverse_iterator white_it = white_ghosts_time_.rbegin();
    std::vector< geometry_msgs::Pose >::reverse_iterator spawn_pose_it = spawn_ghosts_poses_.rbegin();
    for(; pose_it != ghosts_poses_.rend(); ++pose_it, ++white_it, ++spawn_pose_it)
    {
        if(*white_it && ( pose_it->position.x == pacman_pose_.position.x ) && ( pose_it->position.y == pacman_pose_.position.y ) )
        {
            *pose_it = *spawn_pose_it;
            *white_it = 0;
        }
    }
}

void GameParticle::move(pacman_interface::PacmanAction action)
{
    // count a step to white ghosts
    for(std::vector<int>::reverse_iterator it = white_ghosts_time_.rbegin(); it != white_ghosts_time_.rend(); ++it)
    {
        if(*it)
        {
            (*it)--;
        }
    }

    score_--;

    moveGhosts();
    movePacman(action);

    checkIfDeadGhosts();
}
#include "deterministic_q_learning/deterministic_game_state.h"

#include "pacman_abstract_classes/util_functions.h"


DeterministicGameState::DeterministicGameState()
{
    pacman_observer_service_ = n_.advertiseService<pacman_msgs::AgentPoseService::Request, pacman_msgs::AgentPoseService::Response>
                                ("/pacman/pacman_pose", boost::bind(&DeterministicGameState::observeAgent, this, _1, _2));
    ghost_distance_observer_service_ = n_.advertiseService<pacman_msgs::AgentPoseService::Request, pacman_msgs::AgentPoseService::Response>
                                ("/pacman/ghost_distance", boost::bind(&DeterministicGameState::observeAgent, this, _1, _2));

    precalculateAllDistances();
    ROS_DEBUG_STREAM("Bayesian game state initialized");
}

DeterministicGameState::~DeterministicGameState()
{
        pacman_observer_service_.shutdown();
        ghost_distance_observer_service_.shutdown();

        ROS_DEBUG_STREAM("Bayesian game state destroyed");
}

void DeterministicGameState::observePacman(int measurement_x, int measurement_y)
{
    pacman_pose_.position.x = measurement_x;
    pacman_pose_.position.y = measurement_y;

    map_[measurement_y][measurement_x] = EMPTY;
}

void DeterministicGameState::observeGhost(int measurement_x_dist, int measurement_y_dist, int ghost_index)
{
    ghosts_poses_[ghost_index].position.x = pacman_pose_.position.x + measurement_x_dist;
    ghosts_poses_[ghost_index].position.y = pacman_pose_.position.y + measurement_y_dist;
}

bool DeterministicGameState::observeAgent(pacman_msgs::AgentPoseService::Request &req, pacman_msgs::AgentPoseService::Response &res)
{
    int agent = (int) req.agent;
    geometry_msgs::Pose pose = (geometry_msgs::Pose) req.pose;
    int measurement_x = pose.position.x;
    int measurement_y = pose.position.y;

    is_finished_ = (bool) req.is_finished;

    if(agent == pacman_msgs::AgentPoseService::Request::PACMAN)
    {
        ROS_DEBUG_STREAM("Observe pacman");
        observePacman(measurement_x, measurement_y);
    }
    else
    {
        int ghost_index = agent - 1;
        ROS_DEBUG_STREAM("Observe ghost " << ghost_index);
        observeGhost(measurement_x, measurement_y, ghost_index);
    }

    //printDeterministicMap();

    res.observed = true;
    return true;
}

void DeterministicGameState::predictPacmanMove(pacman_msgs::PacmanAction action)
{
    ROS_DEBUG_STREAM("Predict pacman");

    // TODO: check if ok to predict stop when invalid
    geometry_msgs::Pose action_pose = util::actionToMovement(action.action);
    geometry_msgs::Pose new_pose = util::sumPoses(pacman_pose_, action_pose);

    if (map_[new_pose.position.y][new_pose.position.x] != WALL)
    {
        pacman_pose_ = new_pose;
    }
}

void DeterministicGameState::predictGhostMove(int ghost_index)
{
    ROS_DEBUG_STREAM("Predict ghost " << ghost_index);

    // TODO: add things here
}

void DeterministicGameState::predictGhostsMoves()
{
    for(int i = 0 ; i < num_ghosts_ ; ++i)
    {
        predictGhostMove(i);
    }
}

void DeterministicGameState::predictAgentsMoves(pacman_msgs::PacmanAction action)
{
    predictPacmanMove(action);
    predictGhostsMoves();
}

bool DeterministicGameState::isActionLegal(int action)
{
    geometry_msgs::Pose new_pose = util::move(pacman_pose_, action);

    if(map_[new_pose.position.y][new_pose.position.x] == WALL)
        return false;
    return true;
}

bool DeterministicGameState::isActionLegal(pacman_msgs::PacmanAction action)
{
    geometry_msgs::Pose new_pose = util::move(pacman_pose_, (int) action.action);

    if(map_[new_pose.position.y][new_pose.position.x] == WALL)
        return false;
    return true;
}

geometry_msgs::Pose DeterministicGameState::getNextPacmanPose(pacman_msgs::PacmanAction action)
{
    geometry_msgs::Pose new_pose = util::move(pacman_pose_, (int) action.action);
    int new_x = new_pose.position.x;
    int new_y = new_pose.position.y;

    if (map_[new_y][new_x] == WALL) {
        new_pose = pacman_pose_;
    }

    return new_pose;
}

bool DeterministicGameState::isFinished()
{
    return is_finished_;
}

// TODO: dividing by map width*height
float DeterministicGameState::getClosestFoodDistance(pacman_msgs::PacmanAction action)
{
    geometry_msgs::Pose new_pose = getPacmanPose();
    int new_x = new_pose.position.x;
    int new_y = new_pose.position.y;

    std::map< std::pair<int, int>, int > distances = getDistances(new_x, new_y);

    int min_dist = util::INFINITE;

    for (int i = 0 ; i < height_ ; i++) {
        for (int j = 0 ; j < width_ ; j++) {
            if(map_[i][j] == FOOD) {
                int dist = distances[std::make_pair(j, i)];
                if (dist < min_dist) {
                    min_dist = dist;
                }
            }
        }
    }

    return min_dist;// / ( (float) height_ * width_);
}

bool DeterministicGameState::eatsFood(pacman_msgs::PacmanAction action)
{
    geometry_msgs::Pose new_pose = getPacmanPose();
    int new_x = new_pose.position.x;
    int new_y = new_pose.position.y;

    if(map_[new_y][new_x] == FOOD)
        return true;
    return false;
}

int DeterministicGameState::getClosestGhostDistance(pacman_msgs::PacmanAction action)
{
    geometry_msgs::Pose new_pose = getPacmanPose();
    int new_x = new_pose.position.x;
    int new_y = new_pose.position.y;

    std::map< std::pair<int, int>, int > distances = getDistances(new_x, new_y);

    int min_dist = util::INFINITE;

    for(std::vector< geometry_msgs:: Pose >::reverse_iterator it = ghosts_poses_.rbegin();
                             it != ghosts_poses_.rend() ; ++it)
    {
        int dist = distances[std::make_pair(it->position.x, it->position.y)];

        if (dist < min_dist) {
            min_dist = dist;
        }
    }

    return min_dist;// / ( (float) height_ * width_);
}

bool DeterministicGameState::dies(pacman_msgs::PacmanAction action)
{
    geometry_msgs::Pose new_pose = getPacmanPose();
    int new_x = new_pose.position.x;
    int new_y = new_pose.position.y;

    for(std::vector< geometry_msgs:: Pose >::reverse_iterator it = ghosts_poses_.rbegin();
                             it != ghosts_poses_.rend() ; ++it)
    {
        if ( (new_x == it->position.x) && (new_y == it->position.y) )
            return true;
    }

    return false;
}

int DeterministicGameState::getNumberOfGhostsOneStepAway(pacman_msgs::PacmanAction action)
{
    geometry_msgs::Pose new_pose = getPacmanPose();
    int new_x = new_pose.position.x;
    int new_y = new_pose.position.y;

    int number_ghost = 0;

    for(std::vector< geometry_msgs:: Pose >::reverse_iterator it = ghosts_poses_.rbegin();
                             it != ghosts_poses_.rend() ; ++it)
    {
        int dist = abs(new_x - it->position.x) + abs(new_y - it->position.y);

        if (dist <= 1) {
            number_ghost++;
        }
    }

    return number_ghost;
}

int DeterministicGameState::getNumberOfGhostsNStepsAway(int n)
{
    int new_x = pacman_pose_.position.x;
    int new_y = pacman_pose_.position.y;

    int number_ghost = 0;

    for(std::vector< geometry_msgs:: Pose >::reverse_iterator it = ghosts_poses_.rbegin();
                             it != ghosts_poses_.rend() ; ++it)
    {
        int dist = abs(new_x - it->position.x) + abs(new_y - it->position.y);

        if (dist <= n) {
            number_ghost++;
        }
    }

    return number_ghost;
}

bool DeterministicGameState::hasGhostNStepsAway(int n)
{
    bool has_ghost = ( getNumberOfGhostsNStepsAway(n) > 0 );
    return has_ghost;
}

std::map< std::pair<int, int>, int > DeterministicGameState::calculateDistances(int x, int y)
{
    std::map< std::pair<int, int>, int > distances;

    distances[std::make_pair(x, y)] = 0;

    int current_distance = 0;
    bool done = false;

    std::vector< std::pair<int, int> > legal_positions = getLegalNextPositions(x, y);

    while(!done)
    {
        done = true;
        current_distance++;
        std::vector< std::pair<int, int> > next_legal_positions;

        for(std::vector< std::pair<int, int> >::reverse_iterator it = legal_positions.rbegin(); it != legal_positions.rend(); ++it)
        {
            if ( distances.find(*it) == distances.end() )
            {
                distances[*it] = current_distance;
                done = false;

                std::vector< std::pair<int, int> > temp_legal_positions = getLegalNextPositions(it->first, it->second);
                next_legal_positions.reserve(next_legal_positions.size() + temp_legal_positions.size());
                next_legal_positions.insert(next_legal_positions.end(), temp_legal_positions.begin(), temp_legal_positions.end());
            }
        }

        legal_positions = next_legal_positions;
    }
    
    return distances;
}

void DeterministicGameState::precalculateAllDistances()
{
    ROS_DEBUG_STREAM("Pre-calculating all distances");

    for (int i = 0 ; i < width_ ; i++)
    {
        for (int j = 0 ; j < height_ ; j++)
        {
            if( map_[j][i] != WALL )
            {
                precalculated_distances_[std::make_pair(i, j)] = calculateDistances(i, j);
            }
        }
    }

    ROS_DEBUG_STREAM("Pre-calculated all distances");
}

std::map< std::pair<int, int>, int > DeterministicGameState::getDistances(int x, int y)
{
    std::map< std::pair<int, int>, int > distances = precalculated_distances_[std::make_pair(x, y)];
    
    return distances;
}
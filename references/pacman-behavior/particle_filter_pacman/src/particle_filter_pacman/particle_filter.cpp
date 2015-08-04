#include "particle_filter_pacman/particle_filter.h"

#include "particle_filter_pacman/util_constants.h"
#include "particle_filter_pacman/util_functions.h"

#include <boost/bind.hpp>

ParticleFilter::ParticleFilter()
{
    GameParticle game_particle;
    game_particle.printMap();
    game_particles_ = std::vector< GameParticle > (util::NUMBER_OF_PARTICLES, game_particle);

    map_height_ = game_particle.getHeight();
    map_width_ = game_particle.getWidth();
    num_ghosts_ = game_particle.getNumberOfGhosts();
    score_ = 0;

    std::vector<bool> walls_line(map_height_, false);
    walls_ = std::vector< std::vector<bool> > (map_width_, walls_line);
    std::vector<GameParticle::MapElements> estimated_map_line(map_height_, GameParticle::EMPTY);
    estimated_map_ = std::vector< std::vector<GameParticle::MapElements> > (map_width_, estimated_map_line);
    for (int i = map_height_ -1 ; i > -1  ; i--)
    {
        for (int j = 0 ; j < map_width_ ; j++)
        {
            if( game_particle.getMapElement(j, i) == GameParticle::WALL)
            {
                walls_[i][j] = true;
                estimated_map_[i][j] = GameParticle::WALL;
            }
        }
    }

    precalculateAllDistances();

    ghost_distance_subscriber_ = n_.subscribe<pacman_interface::AgentPose>("/pacman_interface/ghost_distance", 20, boost::bind(&ParticleFilter::observeGhost, this, _1));
    pacman_pose_subscriber_ = n_.subscribe<geometry_msgs::Pose>("/pacman_interface/pacman_pose", 10, boost::bind(&ParticleFilter::observePacman, this, _1));

    is_observed_ = true;
}

void ParticleFilter::estimateMovement(pacman_interface::PacmanAction action)
{
    for(std::vector< GameParticle >::reverse_iterator it = game_particles_.rbegin(); it != game_particles_.rend(); ++it)
    {
        it->move(action);
    }
}

void ParticleFilter::sampleParticles(std::map< double, GameParticle > particles_map, double sum_prob_all_particles)
{
    // initialize and reserve memory for vector that willhold new particles
    std::vector< GameParticle > new_particles;
    new_particles.reserve(util::NUMBER_OF_PARTICLES);
    // precalculate random number multiplier
    double random_multiplier = sum_prob_all_particles / (double) RAND_MAX;

    // randomly sample a new particle group from <probabilities, particles> map
    for (int particlesCounter = util::NUMBER_OF_PARTICLES ; particlesCounter > 0 ; particlesCounter-- )
    {
        double random_number = std::rand() * random_multiplier;

        std::map< double, GameParticle >::iterator itlow;
        itlow = particles_map.lower_bound (random_number);

        new_particles.push_back(itlow->second);
    }

    // update particles
    game_particles_.clear();
    game_particles_ = new_particles;
}

void ParticleFilter::observePacman(const geometry_msgs::Pose::ConstPtr& msg)
{
    int measurement_x = msg->position.x;
    int measurement_y = msg->position.y;

    // map relating particles to probabilities
    std::map< double, GameParticle > particles_map;

    double sum_prob_all_particles = 0;

    // generate a map tying particles to their probability
    for(std::vector< GameParticle >::iterator it = game_particles_.begin(); it != game_particles_.end(); ++it)
    {
        geometry_msgs::Pose pose = it->getPacmanPose();
        double probability = util::getProbOfMeasurementGivenPosition(pose.position.x, pose.position.y, measurement_x, measurement_y, 1);

        sum_prob_all_particles += probability;
        particles_map.insert(std::pair<double,GameParticle>(sum_prob_all_particles, *it));
    }

    // if no particles have probability of existing (float) show error message
    if(sum_prob_all_particles == 0)
        ROS_ERROR_STREAM("Error, all particles have a zero probability of being correct for pacman");
    // else, sample new particles
    else
        sampleParticles(particles_map, sum_prob_all_particles);

    is_observed_ = true;
}

// TODO: merge all ghost position measurements to gain speed
void ParticleFilter::observeGhost(const pacman_interface::AgentPose::ConstPtr& msg)
{
    int ghost_index = msg->agent - 1;
    int measurement_x = msg->pose.position.x;
    int measurement_y = msg->pose.position.y;

    // map relating particles to probabilities
    std::map< double, GameParticle > particles_map;

    double sum_prob_all_particles = 0;

    // generate a map tying particles to their probability
    for(std::vector< GameParticle >::iterator it = game_particles_.begin(); it != game_particles_.end(); ++it)
    {
        geometry_msgs::Pose ghost_pose = it->getGhostPose(ghost_index);
        geometry_msgs::Pose pacman_pose = it->getPacmanPose();
        geometry_msgs::Pose distance;
        distance.position.x = ghost_pose.position.x - pacman_pose.position.x;
        distance.position.y = ghost_pose.position.y - pacman_pose.position.y;

        double probability = util::getProbOfMeasurementGivenPosition(distance.position.x, distance.position.y, measurement_x, measurement_y, 0.5);
        sum_prob_all_particles += probability;
        particles_map.insert(std::pair<double,GameParticle>(sum_prob_all_particles, *it));
    }

    // if no particles have probability of existing (float) show error message
    if(sum_prob_all_particles == 0)
        ROS_ERROR_STREAM("Error, all particles have a zero probability of being correct for ghost " << ghost_index);
    // else, sample new particles
    else
        sampleParticles(particles_map, sum_prob_all_particles);

//    is_observed_ = true;
    if(ghost_index==0)
    {
        ROS_INFO_STREAM("Observed ghost");
        //ROS_INFO_STREAM("x " << measurement_x << " y " << measurement_y);
        //printGhostParticles(ghost_index);
    }
}

void ParticleFilter::estimateMap()
{
    std::vector<float> probability_line(map_height_, 0);
    std::vector< std::vector<float> > pacman_probability_map(map_width_, probability_line);
    std::vector< std::vector< std::vector<float> > > ghosts_probability_map(num_ghosts_, pacman_probability_map);
    std::vector< std::vector<float> > food_probability_map(map_width_, probability_line);
    std::vector< std::vector<float> > big_food_probability_map(map_width_, probability_line);
    std::vector<double> white_ghosts_time(num_ghosts_, 0);
    double score = 0;

    double increase_amount = 1 / (double) game_particles_.size();

    ROS_INFO_STREAM("increase_amount " << increase_amount);

    for(std::vector< GameParticle >::reverse_iterator it = game_particles_.rbegin(); it != game_particles_.rend(); ++it)
    {
        geometry_msgs::Pose pose;

        score += it->getScore();
        
        pose = it->getPacmanPose();
        pacman_probability_map[pose.position.y][pose.position.x] += increase_amount;

        for(int i = 0 ; i < num_ghosts_ ; ++i)
        {
            pose = it->getGhostPose(i);
            ghosts_probability_map[i][pose.position.y][pose.position.x] += increase_amount;
        }

        std::vector<int> white_ghosts_time_particle = it->getWhiteGhostsTime();
        for(int i = white_ghosts_time.size() - 1 ; i > -1 ; i--)
        {
            white_ghosts_time[i] += white_ghosts_time_particle[i];
        }

        std::vector< std::vector<GameParticle::MapElements> > particle_map = it->getMap();
        for (int i = map_height_ -1 ; i > -1  ; i--)
        {
            for (int j = 1 ; j < map_width_ - 1 ; j++)
            {
                if( particle_map[i][j] == GameParticle::FOOD )
                    food_probability_map[i][j] += increase_amount;
                else if( particle_map[i][j] == GameParticle::BIG_FOOD )
                    big_food_probability_map[i][j] += increase_amount;
            }
        }
    }

    score = score * increase_amount;
    last_reward_ = score - score_;
    score_ = score;
    ROS_INFO_STREAM("score " << score_);
    ROS_INFO_STREAM("reward " << last_reward_);

    // round whith_ghosts_time number
    for(std::vector<double>::reverse_iterator it = white_ghosts_time.rbegin(); it != white_ghosts_time.rend(); ++it)
    {
        std::floor((*it * increase_amount ) + 0.5);
    ROS_INFO_STREAM("white_ghosts_time "  << *it);
    }


    double pacman_max = 0;
    std::vector< double > ghosts_max(num_ghosts_, 0);
    geometry_msgs::Pose pacman_pose;
    std::vector< geometry_msgs::Pose > ghosts_poses(num_ghosts_, pacman_pose);

    for (int i = map_height_ -1 ; i > -1  ; i--) {
        for (int j = 1 ; j < map_width_ - 1 ; j++) {
            if(pacman_max < pacman_probability_map[i][j])
            {
                pacman_pose.position.x = j;
                pacman_pose.position.y = i;
                pacman_max = pacman_probability_map[i][j];
            }
            for(int ghost_counter = 0; ghost_counter < num_ghosts_ ; ++ghost_counter)
            {
                if(ghosts_max[ghost_counter] < ghosts_probability_map[ghost_counter][i][j])
                {
                    ghosts_poses[ghost_counter].position.x = j;
                    ghosts_poses[ghost_counter].position.y = i;
                    ghosts_max[ghost_counter] = ghosts_probability_map[ghost_counter][i][j];
                }
            }
            if(food_probability_map[i][j] > util::PRINT_FOOD_MINIMUM)
                estimated_map_[i][j] = GameParticle::FOOD;
            else if(big_food_probability_map[i][j] > util::PRINT_FOOD_MINIMUM)
                estimated_map_[i][j] = GameParticle::BIG_FOOD;
            else
                estimated_map_[i][j] = GameParticle::EMPTY;
        }
    }

    estimated_ghosts_poses_.clear();

    estimated_pacman_pose_ = pacman_pose;
    estimated_ghosts_poses_ = ghosts_poses;
}

void ParticleFilter::printPacmanParticles()
{
    printPacmanOrGhostParticles(true, 0);
}

void ParticleFilter::printGhostParticles(int ghost_index)
{
    printPacmanOrGhostParticles(false, ghost_index);
}

void ParticleFilter::printPacmanOrGhostParticles(bool is_pacman, int ghost_index)
{
    GameParticle map = game_particles_[0];
    int height = map.getHeight();
    int width = map.getWidth();

    std::vector<float> probability_line(height, 0);
    std::vector< std::vector<float> > probability_map(width, probability_line);

    double increase_amount = 1 / (double) game_particles_.size();

    ROS_INFO_STREAM("increase_amount " << increase_amount);

    for(std::vector< GameParticle >::reverse_iterator it = game_particles_.rbegin(); it != game_particles_.rend(); ++it) {
        geometry_msgs::Pose pose;
        if(is_pacman)
            pose = it->getPacmanPose();
        else
            pose = it->getGhostPose(ghost_index);

        probability_map[pose.position.y][pose.position.x] += increase_amount;
    }

    for (int i = height -1 ; i > -1  ; i--) {
        std::ostringstream foo;
        foo << std::fixed;
        foo << std::setprecision(0);

        for (int j = 1 ; j < width - 1 ; j++) {
            if( map.getMapElement(j, i) == GameParticle::WALL)
                foo << "###" << ' ';
            else
            {
                int chance = probability_map[i][j]*100;

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

void ParticleFilter::printMostProbableMap()
{
    for (int i = map_height_ -1 ; i > -1  ; i--) {
        std::ostringstream foo;
        for (int j = 1 ; j < map_width_ - 1 ; j++) {
            if( walls_[i][j])
                foo << "#" << ' ';
            else
            {
                if(estimated_pacman_pose_.position.x == j && estimated_pacman_pose_.position.y == i)
                {
                    foo << "\033[48;5;90mP\033[0m" << ' ';
                    continue;
                }
                bool is_ghost = false;
                for(std::vector< geometry_msgs::Pose >::reverse_iterator it = estimated_ghosts_poses_.rbegin(); it != estimated_ghosts_poses_.rend(); ++it)
                    if(it->position.x == j && it->position.y == i)
                    {
                        foo << "\033[48;5;196mG\033[0m" << ' ';
                        is_ghost = true;
                        break;
                    }

                if(!is_ghost)
                    if(estimated_map_[i][j] == GameParticle::FOOD)
                        foo << "\033[48;5;40m.\033[0m" << ' ';
                    else if(estimated_map_[i][j] == GameParticle::BIG_FOOD)
                        foo << "\033[48;5;201mo\033[0m" << ' ';
                    else if(estimated_map_[i][j] == GameParticle::EMPTY)
                        foo << "\033[48;5;12m \033[0m" << ' ';
                    else
                        foo << "\033[48;5;0mE\033[0m" << ' ';
            }
        }
        ROS_INFO_STREAM(foo.str());
    }
}

bool ParticleFilter::hasNewObservation()
{
    return is_observed_;
}
void ParticleFilter::resetNewObservation()
{
    is_observed_ = false;
}

std::map< std::pair<int, int>, int > ParticleFilter::calculateDistances(int x, int y)
{
    std::map< std::pair<int, int>, int > distances;

    distances[std::make_pair(x, y)] = 0;

    int current_distance = 0;
    bool done = false;

    GameParticle map = game_particles_[0];
    std::vector< std::pair<int, int> > legal_positions = map.getLegalNextPositions(x, y);

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

                std::vector< std::pair<int, int> > temp_legal_positions = map.getLegalNextPositions(it->first, it->second);
                next_legal_positions.reserve(next_legal_positions.size() + temp_legal_positions.size());
                next_legal_positions.insert(next_legal_positions.end(), temp_legal_positions.begin(), temp_legal_positions.end());
            }
        }

        legal_positions = next_legal_positions;
    }
    
    return distances;
}

void ParticleFilter::precalculateAllDistances() {
    ROS_INFO_STREAM("Calculating all distances");

    for (int i = 0 ; i < map_width_ ; i++)
    {
        for (int j = 0 ; j < map_height_ ; j++)
        {
            if( !walls_[j][i] )
            {
                precalculated_distances_[std::make_pair(i, j)] = calculateDistances(i, j);
            }
        }
    }

    ROS_INFO_STREAM("Pre calculated all distances");
}

std::map< std::pair<int, int>, int > ParticleFilter::getDistances(int x, int y)
{
    std::map< std::pair<int, int>, int > distances = precalculated_distances_[std::make_pair(x, y)];
    
    return distances;
}

geometry_msgs::Pose ParticleFilter::getEstimatedPacmanPose()
{
    return estimated_pacman_pose_;
}

std::vector< geometry_msgs::Pose > ParticleFilter::getEstimatedGhostsPoses()
{
    return estimated_ghosts_poses_;
}

double ParticleFilter::getEstimatedScore()
{
    return score_;
}

double ParticleFilter::getEstimatedReward()
{
    return last_reward_;
}

int ParticleFilter::getMapWidth()
{
    return map_width_;
}

int ParticleFilter::getMapHeight()
{
    return map_height_;
}

std::vector< std::vector<GameParticle::MapElements> > ParticleFilter::getEstimatedMap()
{
    return estimated_map_;
}

std::vector< pacman_interface::PacmanAction > ParticleFilter::getLegalActions(int x, int y)
{
    return game_particles_[0].getLegalActions(x, y);
}

std::vector< std::pair<int, int> > ParticleFilter::getLegalNextPositions(int x, int y)
{
    return game_particles_[0].getLegalNextPositions(x, y);
}
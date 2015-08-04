#include <stdio.h>

#include "ros/ros.h"

#include "pacman_controller/game_info.h"


void printDistances(GameInfo gameInfo, std::map< std::pair<int, int>, int > distances)
{
    int width = gameInfo.getWidth();
    int height = gameInfo.getHeight();

    for (int i = 0 ; i < height ; i++) {
        std::ostringstream foo;
        for (int j = 0 ; j < width ; j++) {
            std::pair<int, int> position = std::make_pair(j, i);

            std::map< std::pair<int, int>, int >::iterator it = distances.find(position);

            if ( it == distances.end() )
            {
                foo << "## ";
            }
            else
            {
                if(it->second < 10)
                {
                    foo << 0;
                    foo << it->second;
                }
                else
                {
                    foo << it->second;
                }
                foo << ' ';
            }
        }
        ROS_INFO_STREAM(foo.str());
    }
}

std::map< std::pair<int, int>, int > findDistances(GameInfo gameInfo, int x, int y)
{
    int width = gameInfo.getWidth();
    int height = gameInfo.getHeight();

    std::map< std::pair<int, int>, int > distances;

    distances[std::make_pair(x, y)] = 0;

    int current_distance = 0;
    bool done = false;

    std::vector< pacman_interface::PacmanAction > legal_actions =  gameInfo.getLegalActions(x, y);
    std::vector< std::pair<int, int> > legal_positions = gameInfo.getLegalNextPositions(x, y);

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

                std::vector< std::pair<int, int> > temp_legal_positions = gameInfo.getLegalNextPositions(it->first, it->second);
                next_legal_positions.reserve(next_legal_positions.size() + temp_legal_positions.size());
                next_legal_positions.insert(next_legal_positions.end(), temp_legal_positions.begin(), temp_legal_positions.end());
            }
        }

        legal_positions = next_legal_positions;
    }
    
    return distances;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pacman_controller");
    ros::NodeHandle n;

    GameInfo game_info;
    game_info.printMap();

    int width = game_info.getWidth();
    int height = game_info.getHeight();

    for (int i = 0 ; i < width ; i++)
    {
        for (int j = 0 ; j < height ; j++)
        {
            if( game_info.getMapElement(i, j) != GameInfo::WALL)
            {
                findDistances(game_info, i, j);
            }
        }
    }

    ROS_INFO_STREAM("Calculations are over");
}
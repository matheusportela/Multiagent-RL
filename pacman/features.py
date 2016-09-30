#  -*- coding: utf-8 -*-
##    @package features.py
#      @author Matheus Portela & Guilherme N. Ramos (gnramos@unb.br)
#
#


class Feature(object):
    """Defines a representation of the current status of a variable in the
    simulated environment.
    """
    def __call__(self, state, action):
        raise NotImplementedError('Feature must implement __call__')


# O Q-learning com aproximação de funções requer que características sejam
# extraídas a partir do estado estimado do sistema. Portanto, duas
# características foram implementadas no sistema:
# - Distância para cada agente, utilizando grafo de acessibilidade para lidar
# com obstáculos no ambiente;
# - Indicador se Pac-Man capturou a cápsula e, portanto, é capaz de capturar o
# fantasma.

class EnemyDistanceFeature(Feature):
    """Defines the distance to an enemy."""
    def __init__(self, enemy_id):
        self.enemy_id = enemy_id

    def __call__(self, state, action):
        my_position = state.get_position()
        enemy_position = state.get_agent_position(self.enemy_id)
        distance = state.calculate_distance(my_position, enemy_position)

        return 1.0 if distance == 0.0 else 1.0/distance


class FoodDistanceFeature(Feature):
    def __call__(self, state, action):
        distance = state.get_food_distance()

        return 1.0 if distance == 0.0 else 1.0/distance


class FragileAgentFeature(Feature):
    """Indicates the probability that the Pac-Man is able to capture the ghosts
    or not.
    """
    def __init__(self, agent_id):
        self.agent_id = agent_id

    def __call__(self, state, action):
        return state.get_fragile_agent(self.agent_id)

#!/usr/bin/env python

class Feature(object):
    def __call__(self, state, action):
        raise NotImplementedError, 'Feature must implement __call__'


class EnemyDistanceFeature(Feature):
    def __init__(self, enemy_id):
        self.enemy_id = enemy_id

    def __call__(self, state, action):
        my_position = state.get_position()
        enemy_position = state.get_agent_position(self.enemy_id)
        distance = state.calculate_distance(my_position, enemy_position)

        if distance == 0.0:
            distance = 1.0

        return (1.0/distance)


class FoodDistanceFeature(Feature):
    def __call__(self, state, action):
        distance = state.get_food_distance()

        if distance == 0.0:
            distance = 1.0

        return (1.0/distance)


class FragileAgentFeature(Feature):
    def __init__(self, agent_id):
        self.agent_id = agent_id

    def __call__(self, state, action):
        return state.get_fragile_agent(self.agent_id)
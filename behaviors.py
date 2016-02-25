#!/usr/bin/env python

import random

class Behavior(object):
    def __str__(self):
        return self.__class__.__name__

    def __call__(self, state, legal_actions):
        raise NotImplementedError, 'Behavior must implement __call__'


class RandomBehavior(Behavior):
    def __call__(self, state, legal_actions):
        if legal_actions == []:
            return 'Stop'
        else:
            return random.choice(legal_actions)


class EatBehavior(Behavior):
    def __call__(self, state, legal_actions):
        agent_position = state.get_position()
        agent_map = state.get_map()
        food_map = state.food_map
        food_prob_threshold = food_map.max() / 2.0
        best_action = None
        min_dist = None

        random.shuffle(legal_actions)

        for action in legal_actions:
            diff = agent_map.action_to_pos[action]
            new_position = (agent_position[0] + diff[0], agent_position[1] + diff[1])

            for x in range(food_map.width):
                for y in range(food_map.height):
                    new_distance = state.calculate_distance(new_position, (y, x))

                    if (best_action == None) or (food_map[y][x] > food_prob_threshold and new_distance < min_dist):
                        min_dist = new_distance
                        best_action = action

        return best_action


class FleeBehavior(Behavior):
    def __call__(self, state, legal_actions):
        agent_position = state.get_position()
        enemy_position = state.get_agent_position(state.get_closest_enemy(state))
        agent_map = state.get_map()

        best_action = None
        max_distance = None

        random.shuffle(legal_actions)

        for action in legal_actions:
            diff = agent_map.action_to_pos[action]
            new_position = (agent_position[0] + diff[0], agent_position[1] + diff[1])
            new_distance = state.calculate_distance(new_position, enemy_position)

            if (best_action == None) or (agent_map._is_valid_position(new_position) and
                new_distance > max_distance):
                best_action = action
                max_distance = new_distance

        return best_action


class SeekBehavior(Behavior):
    def __call__(self, state, legal_actions):
        agent_position = state.get_position()
        enemy_position = state.get_agent_position(state.get_closest_enemy(state))
        agent_map = state.get_map()

        best_action = None
        min_distance = None

        random.shuffle(legal_actions)

        for action in legal_actions:
            diff = agent_map.action_to_pos[action]
            new_position = (agent_position[0] + diff[0], agent_position[1] + diff[1])
            new_distance = state.calculate_distance(new_position, enemy_position)

            if (best_action == None) or (agent_map._is_valid_position(new_position) and
                new_distance < min_distance):
                best_action = action
                min_distance = new_distance

        return best_action


class PursueBehavior(Behavior):
    def __init__(self, n=2):
        self.n = n
        self.enemy_previous_position = None

    def _estimate_enemy_future_position(self, current_position, agent_map):
        enemy_position = current_position

        if not self.enemy_previous_position:
            enemy_diff = (0, 0)
        else:
            enemy_diff = (enemy_position[0] - self.enemy_previous_position[0], enemy_position[1] - self.enemy_previous_position[1])
        self.enemy_previous_position = enemy_position
        simulated_steps = 0

        while agent_map._is_valid_position(enemy_position) and simulated_steps < self.n:
            enemy_position = (enemy_position[0] + enemy_diff[0], enemy_position[1] + enemy_diff[1])
            simulated_steps += 1

        return enemy_position

    def __call__(self, state, legal_actions):
        agent_map = state.get_map()
        agent_position = state.get_position()
        enemy_position = self._estimate_enemy_future_position(
            state.get_agent_position(state.get_closest_enemy(state)), agent_map)

        best_action = None
        min_distance = None

        random.shuffle(legal_actions)

        for action in legal_actions:
            diff = agent_map.action_to_pos[action]
            new_position = (agent_position[0] + diff[0], agent_position[1] + diff[1])
            new_distance = state.calculate_distance(new_position, enemy_position)

            if (best_action == None) or (agent_map._is_valid_position(new_position) and
                new_distance < min_distance):
                best_action = action
                min_distance = new_distance

        return best_action
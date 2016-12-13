import random

from multiagentrl import core
from multiagentrl import exploration
from multiagentrl import learning


class RandomAgent(core.BaseControllerAgent):
    """Agent that randomly selects an action."""
    def __init__(self, agent_id, ally_ids, enemy_ids):
        super(RandomAgent, self).__init__(agent_id)

    def start_game(self):
        pass

    def finish_game(self):
        pass

    def learn(self, state, action, reward):
        pass

    def act(self, state, legal_actions, explore):
        if legal_actions:
            return random.choice(legal_actions)


class LearningAgent(core.BaseControllerAgent):
    def __init__(self, agent_id, ally_ids, enemy_ids):
        super(LearningAgent, self).__init__(agent_id)
        self.game_number = 1
        self.game_step = 1
        self.exploration_rate = 0.1
        self.learning = learning.QLearning(
            learning_rate=0.3,
            discount_factor=0.95,
            actions=[0, 1])
        self.exploration = exploration.EGreedy(
            exploration_rate=self.exploration_rate)

    def get_policy(self):
        return self.learning.q_values

    def set_policy(self, weights):
        self.learning.q_values = weights

    def start_game(self):
        self.game_step = 1

    def finish_game(self):
        self.game_number += 1

    def learn(self, state, action, reward):
        self.learning.learn(state, action, reward)

    def act(self, state, legal_actions, explore):
        action = self.learning.act(state)

        if explore:
            action = self.exploration.explore(action, legal_actions)

        self.game_step += 1

        print self.learning

        return action

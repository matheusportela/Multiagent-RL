#  -*- coding: utf-8 -*-
##    @package agents.py
#      @author Matheus Portela & Guilherme N. Ramos (gnramos@unb.br)
#
# Defines the agents.


import math
import random

from berkeley.game import Agent as BerkeleyGameAgent, Directions

import behaviors
import features
import learning

from communication import (ZMQMessengerBase,
                           # AckMessage, ActionMessage, BehaviorCountMessage,
                           RequestInitializationMessage,
                           # RequestBehaviorCountMessage, PolicyMessage,
                           RequestGameStartMessage, RequestRegisterMessage,
                           # RequestPolicyMessage,
                           StateMessage)


# Default settings
DEFAULT_NOISE = 0

# Global variable
NOISE = 0

GHOST_ACTIONS = [Directions.NORTH, Directions.SOUTH, Directions.EAST,
                 Directions.WEST]
PACMAN_ACTIONS = GHOST_ACTIONS + [Directions.STOP]
PACMAN_INDEX = 0

###############################################################################
#                                AdapterAgents                                #
###############################################################################


class AdapterAgent(object, BerkeleyGameAgent):
    """Communicating client for game adapter."""
    def __init__(self, agent_id, client):
        BerkeleyGameAgent.__init__(self, agent_id)

        self.agent_id = agent_id

        if not isinstance(client, ZMQMessengerBase):
            raise ValueError('Invalid client')

        self.client = client

        self.previous_action = Directions.STOP

        self.test_mode = False

    def __noise_error__(self):
        return random.randrange(-NOISE, NOISE + 1)

    def calculate_reward(self, current_score):
        raise NotImplementedError('Communicating agent must calculate score')

    def communicate(self, msg):
        self.client.send(msg)
        return self.client.receive()

    def create_state_message(self, state):
        agent_positions = {}

        agent_positions[PACMAN_INDEX] = state.getPacmanPosition()[::-1]

        for id_, pos in enumerate(state.getGhostPositions()):
            pos_y = pos[::-1][0] + self.__noise_error__()
            pos_x = pos[::-1][1] + self.__noise_error__()
            agent_positions[id_ + 1] = (pos_y, pos_x)

        food_positions = []
        for x, row in enumerate(state.getFood()):
            for y, is_food in enumerate(row):
                if is_food:
                    food_positions.append((y, x))

        fragile_agents = {}
        for id_, s in enumerate(state.data.agentStates):
            fragile_agents[id_] = 1.0 if s.scaredTimer > 0 else 0.0

        wall_positions = []
        for x, row in enumerate(state.getWalls()):
            for y, is_wall in enumerate(row):
                if is_wall:
                    wall_positions.append((y, x))

        reward = self.calculate_reward(state.getScore())
        self.previous_score = state.getScore()

        msg = StateMessage(agent_id=self.agent_id,
                           agent_positions=agent_positions,
                           food_positions=food_positions,
                           fragile_agents=fragile_agents,
                           wall_positions=wall_positions,
                           legal_actions=state.getLegalActions(self.agent_id),
                           reward=reward,
                           executed_action=self.previous_action,
                           test_mode=self.test_mode)

        return msg

    def enable_learn_mode(self):
        self.test_mode = False

    def enable_test_mode(self):
        self.test_mode = True

    def getAction(self, state):
        """Returns an action from Directions."""
        msg = self.create_state_message(state)
        reply_msg = self.communicate(msg)

        self.previous_action = reply_msg.action

        if reply_msg.action not in state.getLegalActions(self.agent_id):
            self.invalid_action = True
            return self.act_when_invalid(state)
        else:
            self.invalid_action = False
            return reply_msg.action

    def start_game(self, layout):
        self.previous_score = 0
        self.previous_action = Directions.STOP
        msg = RequestGameStartMessage(agent_id=self.agent_id,
                                      map_width=layout.width,
                                      map_height=layout.height)
        self.communicate(msg)

    def update(self, state):
        msg = self.create_state_message(state)
        self.communicate(msg)


class PacmanAdapterAgent(AdapterAgent):
    def __init__(self, client):
        super(PacmanAdapterAgent, self).__init__(agent_id=PACMAN_INDEX,
                                                 client=client)

    ## @todo is this ever used?
    # def act_when_invalid(self, state):
    #     return Directions.STOP

    def calculate_reward(self, current_score):
        return current_score - self.previous_score


class GhostAdapterAgent(AdapterAgent):
    def __init__(self, agent_id, client):
        super(GhostAdapterAgent, self).__init__(agent_id, client)

        self.previous_action = Directions.NORTH
        # self.actions = GHOST_ACTIONS

    ## @todo is this ever used?
    # def act_when_invalid(self, state):
    #     return random.choice(state.getLegalActions(self.agent_id))

    def calculate_reward(self, current_score):
        return self.previous_score - current_score

###############################################################################
#                                                                             #
###############################################################################

###############################################################################
#                              ControllerAgents                               #
###############################################################################


class ControllerAgent(object):
    """Autonomous agent for game controller."""
    def __init__(self, agent_id):
        self.agent_id = agent_id

    def choose_action(self, state, action, reward, legal_actions, explore):
        """Select an action to be executed by the agent.

        Args:
            state: Current game state.
            action: Last executed action.
            reward: Reward for the previous action.
            legal_actions: List of currently allowed actions.
            explore: Boolean whether agent is allowed to explore.

        Returns:
            A Direction for the agent to follow (NORTH, SOUTH, EAST, WEST or
            STOP).
        """
        raise NotImplementedError('Agent must implement choose_action.')


class PacmanAgent(ControllerAgent):
    def __init__(self, agent_id, ally_ids, enemy_ids):
        super(PacmanAgent, self).__init__(agent_id)
        self.actions = PACMAN_ACTIONS


class GhostAgent(ControllerAgent):
    def __init__(self, agent_id, ally_ids, enemy_ids):
        super(GhostAgent, self).__init__(agent_id)
        self.actions = GHOST_ACTIONS


class RandomPacmanAgent(PacmanAgent):
    """Agent that randomly selects an action."""
    def choose_action(self, state, action, reward, legal_actions, explore):
        if len(legal_actions) > 0:
            return random.choice(legal_actions)
  
 class RandomPacmanAgentTwo(PacmanAgent):
    """Agent that after choosing a random direction will follow that direction until it reaches a wall or have more than
       three possible moves. In these case, continue to follow the previous directions have twice the chance of 
       heppening then the other possible movements"""
    def choose_action(self, state, action, reward, legal_actions, explore):

        if action == 'Stop' or action not in legal_actions:
            legal_actions.remove('Stop')
            if len(legal_actions) > 0:
                return random.choice(legal_actions)
        else:
            if len(legal_actions) > 3 :
                if len(legal_actions)==4:
                    number=random.choice([1,2,3,4])
                else:
                    number=random.choice([1,2,3,4,5])
                if number==1 or number==2:
                    return action
                else:
                    aux=3
                    legal_actions.remove(action)
                    for possible_action in legal_actions:
                        if number==aux:
                            return possible_action
                        else:
                            aux+=1
                    else:
                        return random.choice(legal_actions)
            else:
                return action

class RandomGhostAgent(GhostAgent):
    """Agent that randomly selects an action."""
    def choose_action(self, state, action, reward, legal_actions, explore):
        if len(legal_actions) > 0:
            return random.choice(legal_actions)


class EaterPacmanAgent(PacmanAgent):
    def __init__(self, agent_id, ally_ids, enemy_ids):
        super(EaterPacmanAgent, self).__init__(agent_id, ally_ids, enemy_ids)
        self.eat_behavior = behaviors.EatBehavior()

    def choose_action(self, state, action, reward, legal_actions, test):
        suggested_action = self.eat_behavior(state, legal_actions)

        if suggested_action in legal_actions:
            return suggested_action
        elif legal_actions == []:
            return Directions.STOP
        else:
            return random.choice(legal_actions)


class BehaviorLearningPacmanAgent(PacmanAgent):
    def __init__(self, agent_id, ally_ids, enemy_ids):
        super(BehaviorLearningPacmanAgent, self).__init__(agent_id, ally_ids,
                                                          enemy_ids)
        self.features = [features.FoodDistanceFeature()]
        for enemy_id in enemy_ids:
            self.features.append(features.EnemyDistanceFeature(enemy_id))
        for id_ in [agent_id] + ally_ids + enemy_ids:
            self.features.append(features.FragileAgentFeature(id_))

        self.behaviors = [behaviors.EatBehavior(),
                          behaviors.FleeBehavior(),
                          behaviors.SeekBehavior(),
                          behaviors.PursueBehavior()]

        self.K = 1.0  # Learning rate
        self.exploration_rate = 0.1

        QLearning = learning.QLearningWithApproximation
        self.learning = QLearning(learning_rate=0.1, discount_factor=0.9,
                                  actions=self.behaviors,
                                  features=self.features,
                                  exploration_rate=self.exploration_rate)
        self.previous_behavior = self.behaviors[0]
        self.behavior_count = {}
        self.reset_behavior_count()

        self.test_mode = False

    def reset_behavior_count(self):
        for behavior in self.behaviors:
            self.behavior_count[str(behavior)] = 0

    def get_policy(self):
        return self.learning.get_weights()

    def set_policy(self, weights):
        self.learning.set_weights(weights)

    def choose_action(self, state, action, reward, legal_actions, test):
        if test:
            self.enable_test_mode()
        else:
            self.enable_learn_mode()

        if not self.test_mode:
            self.learning.learning_rate = self.K/(self.K + state.iteration)
            self.learning.learn(state, self.previous_behavior, reward)

        behavior = self.learning.act(state)
        self.previous_behavior = behavior
        suggested_action = behavior(state, legal_actions)

        self.behavior_count[str(behavior)] += 1

        if suggested_action in legal_actions:
            return suggested_action
        elif legal_actions == []:
            return Directions.STOP
        else:
            return random.choice(legal_actions)

    def enable_learn_mode(self):
        self.test_mode = False
        self.learning.exploration_rate = self.exploration_rate

    def enable_test_mode(self):
        self.test_mode = True
        self.learning.exploration_rate = 0


class BehaviorLearningGhostAgent(GhostAgent):
    def __init__(self, agent_id, ally_ids, enemy_ids):
        super(BehaviorLearningGhostAgent, self).__init__(agent_id, ally_ids,
                                                         enemy_ids)
        self.features = [features.FoodDistanceFeature()]
        for enemy_id in enemy_ids:
            self.features.append(features.EnemyDistanceFeature(enemy_id))
        for id_ in [agent_id] + ally_ids + enemy_ids:
            self.features.append(features.FragileAgentFeature(id_))

        self.behaviors = [behaviors.FleeBehavior(),
                          behaviors.SeekBehavior(),
                          behaviors.PursueBehavior()]

        self.K = 1.0  # Learning rate
        self.exploration_rate = 0.1
        QLearning = learning.QLearningWithApproximation
        self.learning = QLearning(learning_rate=0.1, discount_factor=0.9,
                                  actions=self.behaviors,
                                  features=self.features,
                                  exploration_rate=self.exploration_rate)
        self.previous_behavior = self.behaviors[0]
        self.behavior_count = {}
        self.reset_behavior_count()

        self.test_mode = False

    def reset_behavior_count(self):
        for behavior in self.behaviors:
            self.behavior_count[str(behavior)] = 0

    def get_policy(self):
        return self.learning.get_weights()

    def set_policy(self, weights):
        self.learning.set_weights(weights)

    def choose_action(self, state, action, reward, legal_actions, test):
        if test:
            self.enable_test_mode()
        else:
            self.enable_learn_mode()

        if not self.test_mode:
            self.learning.learning_rate = self.K/(self.K + state.iteration)
            self.learning.learn(state, self.previous_behavior, reward)

        behavior = self.learning.act(state)
        self.previous_behavior = behavior
        suggested_action = behavior(state, legal_actions)

        self.behavior_count[str(behavior)] += 1

        if suggested_action in legal_actions:
            return suggested_action
        elif legal_actions == []:
            return Directions.STOP
        else:
            return random.choice(legal_actions)

    def enable_learn_mode(self):
        self.test_mode = False
        self.learning.exploration_rate = self.exploration_rate

    def enable_test_mode(self):
        self.test_mode = True
        self.learning.exploration_rate = 0

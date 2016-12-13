#!/usr/bin/env python

from __future__ import absolute_import

import argparse
import logging
import math
import os
import pickle

import gym

from . import agents
from multiagentrl import core
from multiagentrl import messages


# Logging configuration
logger = logging.getLogger(__name__)


class CartPoleExperiment(core.BaseExperiment):
    def __init__(self, learn_games, test_games, graphics, agent_algorithm,
                 output_file, policy_file):
        super(CartPoleExperiment, self).__init__(
            learn_games=learn_games,
            test_games=test_games)

        logger.info('Instantiating adapter')

        self.simulator = gym.make('CartPole-v0')
        self.agent = CartPole(agent_algorithm)
        self.agents = [self.agent]
        self.results = {
            'learn_scores': [],
            'test_scores': []
        }
        self.graphics = graphics
        self.output_file = output_file
        self.policy_file = policy_file

    def start(self):
        logger.info('Starting')
        self._load_agent_policy()

    def _load_agent_policy(self):
        if self.policy_file and os.path.isfile(self.policy_file):
            logger.debug('Loading policies from "{}"'.format(self.policy_file))
            self.agent.policy = pickle.load(open(self.policy_file))

    def execute_game(self):
        logger.info('Executing game')

        done = False
        score = 0
        observation = self.simulator.reset()

        # Send first state before start learning
        self.agent.state = self._estimate_state(observation)
        self.agent.send_state()

        while not done:
            # Receive an action for the current state
            action = self.agent.receive_action()

            # Simulate one step
            observation, reward, done, info = self.simulator.step(action)

            # Show simulation state
            if self.graphics:
                self.simulator.render()

            if done:
                reward = -100

            # Update state to learn from the received reward
            self.agent.state = self._estimate_state(observation)
            self.agent.send_state()

            # Get reward when executing the action and reaching the new state
            self.agent.reward = reward
            self.agent.send_reward()

            score += 1

        if self.is_learn_game:
            self.results['learn_scores'].append(score)
        else:
            self.results['test_scores'].append(score)

    def _estimate_state(self, observation):
        (x, x_dot, theta, theta_dot) = observation

        if (x < -2.4 or x > 2.4 or
                theta < -self._to_radians(12) or theta > self._to_radians(12)):
            return -1

        if x < -0.8:
            state = 0
        elif x < 0.8:
            state = 1
        else:
            state = 2

        if x_dot < -0.5:
            state += 0
        elif x_dot < 0.5:
            state += 3
        else:
            state += 6

        if theta < -self._to_radians(6):
            state += 0
        elif theta < -self._to_radians(1):
            state += 9
        elif theta < self._to_radians(0):
            state += 18
        elif theta < self._to_radians(1):
            state += 27
        elif theta < self._to_radians(6):
            state += 36
        else:
            state += 45

        if theta_dot < -self._to_radians(50):
            state += 0
        elif theta_dot < self._to_radians(50):
            state += 54
        else:
            state += 108

        return state

    def _to_radians(self, degrees):
        return degrees*math.pi/180.0

    def stop(self):
        logger.info('Stopping')
        logger.info('Results: {}'.format(self.results))
        self._save_results()
        self._save_agent_policy()

    def _save_results(self):
        if self.output_file:
            logger.info('Saving results to "{}"'.format(self.output_file))
            pickle.dump(self.results, open(self.output_file, 'wb'))

    def _save_agent_policy(self):
        if self.policy_file:
            logger.info('Saving policy to "{}"'.format(self.policy_file))
            pickle.dump(self.agent.policy, open(self.policy_file, 'wb'))


class CartPole(core.BaseAdapterAgent):
    def __init__(self, agent_algorithm, policy=None):
        super(CartPole, self).__init__()
        self.agent_id = 0
        self.agent_type = 'cart-pole'
        self.map_width = None
        self.map_height = None
        self.actions = [0, 1]
        self.action = None
        self.state = None
        self.reward = 0
        self.is_learning = True
        self.policy = None
        self.game_number = 0
        self.experiment_number = 0
        self._set_agent_class(agent_algorithm)

    def _set_agent_class(self, agent_algorithm):
        if agent_algorithm == 'random':
            self.agent_class = agents.RandomAgent
        elif agent_algorithm == 'ai':
            self.agent_class = agents.LearningAgent
        else:
            raise ValueError('Windy agent must be random or ai')

    def start_experiment(self):
        logger.info('Starting experiment')
        logger.info('Agent class "{}"'.format(self.agent_class.__name__))
        message = messages.StartExperimentMessage(
            agent_id=self.agent_id,
            agent_team=self.agent_type,
            agent_class=self.agent_class,
            map_width=self.map_width,
            map_height=self.map_height)
        self.communicate(message)
        self.experiment_number += 1

    def finish_experiment(self):
        logger.info('Finishing experiment')
        message = messages.FinishExperimentMessage(agent_id=self.agent_id)
        self.communicate(message)

    def start_game(self):
        logger.info('Starting game #{}'.format(self.game_number + 1))
        self._send_start_game_message()
        self._send_policy()
        self.game_number += 1

    def _send_start_game_message(self):
        message = messages.StartGameMessage(agent_id=self.agent_id)
        self.communicate(message)

    def _send_policy(self):
        if self.policy and self.game_number == 0:
            logger.debug('Loading policy'.format(self.agent_id))
            message = messages.PolicyMessage(
                agent_id=self.agent_id, policy=self.policy)
            self.communicate(message)

    def finish_game(self):
        logger.info('Finishing game')
        self._send_finish_game_message()
        self._update_policy()

    def _send_finish_game_message(self):
        message = messages.FinishGameMessage(agent_id=self.agent_id)
        self.communicate(message)

    def _update_policy(self):
        message = messages.RequestPolicyMessage(agent_id=self.agent_id)
        policy_message = self.communicate(message)
        self.policy = policy_message.policy

    def send_state(self):
        logger.debug('Sending state')
        message = messages.StateMessage(
            agent_id=self.agent_id,
            state=self.state,
            legal_actions=self.actions,
            explore=self.is_learning)
        logger.debug('State: {}'.format(self.state))
        return self.communicate(message)

    def receive_action(self):
        logger.debug('Receiving action')
        action_message = self.send_state()
        self.action = action_message.action
        logger.debug('Action: {}'.format(self.action))
        return self.action

    def send_reward(self):
        if self.is_learning:
            logger.debug('Sending reward')
            message = messages.RewardMessage(
                agent_id=self.agent_id, state=self.state,
                action=self.action, reward=self.reward)
            logger.debug('Reward: {}'.format(self.reward))
            self.communicate(message)


def build_parser():
    parser = argparse.ArgumentParser(
        description='Run cart-pole scenario with OpenAI Gym.')
    parser.add_argument(
        '-l', '--learn-games', dest='learn_games', type=int, default=1,
        help='number of games to learn from')
    parser.add_argument(
        '-t', '--test-games', dest='test_games', type=int, default=1,
        help='number of games to test learned policy')
    parser.add_argument(
        '-g', '--graphics', dest='graphics', default=False,
        action='store_true', help='display GUI')
    parser.add_argument(
        '-a', '--agent', dest='agent_algorithm', type=str,
        choices=['random', 'ai'],
        default='random', help='select Reinforcement Learning agent')
    parser.add_argument(
        '-o', '--output', dest='output_file', type=str,
        help='output file to save simulation results (e.g. "cart_pole.res")')
    parser.add_argument(
        '-p', '--policy', dest='policy_file', type=str,
        help='file to load and save agent policies (e.g. "cart_pole.pol")')
    return parser


def build_adapter_with_args(args):
    return CartPoleExperiment(
        learn_games=args.learn_games,
        test_games=args.test_games,
        graphics=args.graphics,
        agent_algorithm=args.agent_algorithm,
        output_file=args.output_file,
        policy_file=args.policy_file)

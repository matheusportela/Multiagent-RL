#!/usr/bin/env python
#  -*- coding: utf-8 -*-

"""Communicate controller and Berkeley Pac-man simulator."""

from __future__ import absolute_import
from __future__ import print_function

import argparse
import logging
import os
import pickle
import random

from .berkeley.game import Agent as BerkeleyGameAgent, Directions
from .berkeley.graphicsDisplay import PacmanGraphics as BerkeleyGraphics
from .berkeley.layout import getLayout as get_berkeley_layout
from .berkeley.pacman import runGames as run_berkeley_games
from .berkeley.textDisplay import NullGraphics as BerkeleyNullGraphics

from . import agents
from multiagentrl import communication
from multiagentrl import core
from multiagentrl import messages
from . import state


# Logging configuration
logger = logging.getLogger(__name__)


class BerkeleyAdapter(core.BaseExperiment):
    def __init__(self, pacman_agent, ghost_agent, num_ghosts, noise, max_steps,
                 policy_file, layout_map, learn_games, test_games, output_file,
                 graphics):

        super(BerkeleyAdapter, self).__init__()

        logger.info('Instantiating adapter')

        # Layout
        LAYOUT_PATH = 'experiments/pacman/layouts'
        file_name = str(num_ghosts) + 'Ghosts'
        layout_file = '/'.join([LAYOUT_PATH, layout_map + file_name])
        self.layout = get_berkeley_layout(layout_file)
        if not self.layout:
            raise ValueError('Missing layout file "{}"'.format(layout_file))
        logger.info('Loading layout file "{}"'.format(layout_file))

        # Pac-Man
        self.pacman = BerkeleyAdapterAgent(agent_id=0, agent_type='pacman',
                                           agent_algorithm=pacman_agent)

        # Ghosts
        self.num_ghosts = int(num_ghosts)
        if not (0 <= self.num_ghosts <= 4):
            raise ValueError('Must 0-4 ghost(s).')

        self.ghosts = []
        for i in range(num_ghosts):
            self.ghosts.append(BerkeleyAdapterAgent(
                                   agent_id=i+1, agent_type='ghost',
                                   agent_algorithm=ghost_agent))

        self.agents = [self.pacman] + self.ghosts

        # Noise
        BerkeleyAdapterAgent.noise = noise

        # Policies
        self.policy_file = str(policy_file) if policy_file else None

        # Runs
        self.max_steps = max_steps

        self.learn_games = int(learn_games)
        assert self.learn_games >= 0

        self.test_games = int(test_games)
        assert self.test_games >= 0

        # Output
        if output_file:
            self.output_file = str(output_file)
        else:
            self.output_file = '{}_{}_{}_{}.res'.format(pacman_agent,
                                                        layout_map,
                                                        num_ghosts,
                                                        ghost_agent)

        # Graphical interface
        if graphics:
            self.display = BerkeleyGraphics()
        else:
            self.display = BerkeleyNullGraphics()

        logger.info('Ready')

    def start(self):
        logger.info('Starting')

        self.results = {
            'learn_scores': [],
            'test_scores': [],
        }

        # Load policies from file
        self._load_policies_from_file(self.policy_file)

        # Initialize agents
        for agent in self.agents:
            agent.policy = self.policies.get(agent.agent_id, None)
            agent.layout = self.layout

    def _load_policies_from_file(self, filename):
        self.policies = {}

        if filename and os.path.isfile(filename):
            logger.info('Loading policies from {}'.format(filename))
            with open(filename) as f:
                self.policies = pickle.loads(f.read())

    def execute_game(self):
        logger.info('Executing game #{}'.format(self.game_number + 1))

        score = self._run_game()

        if self.is_learn_game:
            self.results['learn_scores'].append(score)
        else:
            self.results['test_scores'].append(score)

        for agent in self.agents:
            agent.results['scores'].append(score)

    def _run_game(self):
        num_berkeley_games = 1
        record_berkeley_games = False
        simulated_game = run_berkeley_games(
            self.layout, self.pacman, self.ghosts, self.display,
            num_berkeley_games, record_berkeley_games,
            maxSteps=self.max_steps)[0]

        # Do this so as agents can receive the last reward
        for agent in self.agents:
            agent.getAction(simulated_game.state)

        # Return game final score
        return simulated_game.state.getScore()

    def stop(self):
        logger.info('Stopping')

        if self.policy_file:
            self._save_policies()

        logger.info('Learn scores: {}'.format(self.results['learn_scores']))
        logger.info('Test scores: {}'.format(self.results['test_scores']))

        self._write_to_file(self.output_file, self.results)

    def _save_policies(self):
        for agent in self.agents:
            if agent.policy:
                self.policies[agent.agent_id] = agent.policy

        self._write_to_file(self.policy_file, self.policies)

    def _write_to_file(self, filename, content):
        logger.info('Saving results to "{}"'.format(filename))
        with open(filename, 'wb') as f:
            pickle.dump(content, f)


class BerkeleyAdapterAgent(core.BaseAdapterAgent, BerkeleyGameAgent):
    pacman_index = None
    noise = 0

    def __init__(self, agent_id, agent_type, agent_algorithm='random', *args,
                 **kwargs):
        core.BaseAdapterAgent.__init__(self, *args, **kwargs)
        BerkeleyGameAgent.__init__(self, agent_id)

        self.agent_type = agent_type
        if self.agent_type == 'pacman':
            BerkeleyAdapterAgent.pacman_index = agent_id

        self.agent_algorithm = agent_algorithm
        self.agent_class = None
        self.policy = None
        self.pacman_game_state = None
        self.game_number = 0
        self.game_state = None
        self.is_first_step = True
        self.agent_ids = [self.agent_id]
        self.ally_ids = []
        self.enemy_ids = []
        self.layout = None
        self.results = {
            'scores': [],
        }

    # BerkeleyGameAgent required methods

    @property
    def agent_id(self):
        return self.index  # from BerkeleyGameAgent

    def getAction(self, pacman_game_state):
        """Returns a legal action (from Directions)."""
        self.pacman_game_state = pacman_game_state

        if self.is_first_step:
            self.is_first_step = False
            self.previous_score = self.pacman_game_state.getScore()
        else:
            self.send_reward()

        action = self.receive_action()
        self.previous_action = action
        return action

    # BaseAdapterAgent required methods

    def start_experiment(self):
        logger.info('#{} Starting experiment'.format(self.agent_id))
        self._send_start_experiment_message()

    def _send_start_experiment_message(self):
        logger.info('#{} Registering {}/{}'.format(
            self.agent_id, self.agent_type, self.agent_algorithm))

        if self.agent_type == 'pacman':
            self._register_pacman()
        elif self.agent_type == 'ghost':
            self._register_ghost()
        else:
            raise ValueError('Agent type must be either "pacman" or "ghost"')

        message = messages.StartExperimentMessage(
            agent_id=self.agent_id,
            agent_team=self.agent_type,
            agent_class=self.agent_class,
            map_width=self.layout.width,
            map_height=self.layout.height)
        self.communicate(message)

    def finish_experiment(self):
        logger.info('#{} Finishing experiment'.format(self.agent_id))
        self._save_policy()
        self._send_finish_experiment_message()

    def _save_policy(self):
        message = messages.RequestPolicyMessage(agent_id=self.agent_id)
        policy_message = self.communicate(message)
        self.policy = policy_message.policy

    def _send_finish_experiment_message(self):
        message = messages.FinishExperimentMessage(agent_id=self.agent_id)
        self.communicate(message)

    def start_game(self):
        logger.info('#{} Starting game #{}'.format(
            self.agent_id, self.game_number + 1))
        self._send_start_game_message()
        self._reset_game_data()
        self._load_policy()
        self.is_first_step = True

    def _send_start_game_message(self):
        message = messages.StartGameMessage(agent_id=self.agent_id)
        response = self.communicate(message)
        self.ally_ids = response.ally_ids
        self.enemy_ids = response.enemy_ids
        self.agent_ids += self.ally_ids + self.enemy_ids

    def _reset_game_data(self):
        self.previous_score = 0
        self.previous_action = Directions.NORTH
        self.reward = 0

    def _load_policy(self):
        if self.policy and self.game_number == 0:
            logger.info('#{} Loading policy'.format(self.agent_id))
            message = messages.PolicyMessage(
                agent_id=self.agent_id, policy=self.policy)
            self.communicate(message)

    def _register_pacman(self):
        if self.agent_algorithm == 'random':
            self.agent_class = agents.RandomPacmanAgent
        elif self.agent_algorithm == 'random2':
            self.agent_class = agents.RandomPacmanAgentTwo
        elif self.agent_algorithm == 'ai':
            self.agent_class = agents.BehaviorLearningPacmanAgent
        elif self.agent_algorithm == 'eater':
            self.agent_class = agents.EaterPacmanAgent
        elif self.agent_algorithm == 'qlearning':
            self.agent_class = agents.QLearningPacmanAgent
        elif self.agent_algorithm == 'sarsa':
            self.agent_class = agents.SARSALearningPacmanAgent
        else:
            raise ValueError('Pac-Man agent must be "ai", "random", "random2",'
                             ' "eater", "qlearning" or "sarsa".')

    def _register_ghost(self):
        if self.agent_algorithm == 'random':
            self.agent_class = agents.RandomGhostAgent
        elif self.agent_algorithm == 'ai':
            self.agent_class = agents.BehaviorLearningGhostAgent
        else:
            raise ValueError('Ghost agent must be ai or random.')

    def finish_game(self):
        logger.info('#{} Finishing game'.format(self.agent_id))
        self._update_game_number()
        self._send_finish_game_message()

    def _update_game_number(self):
        self.game_number += 1

    def _send_finish_game_message(self):
        message = messages.FinishGameMessage(agent_id=self.agent_id)
        self.communicate(message)

    def send_state(self):
        logger.debug('#{} Sending state'.format(self.agent_id))
        message = self._create_state_message()
        return self.communicate(message)

    def _create_state_message(self):
        agent_positions = {}

        pacman_position = self.pacman_game_state.getPacmanPosition()[::-1]
        pos_y = pacman_position[0] + self._generate_measurement_noise()
        pos_x = pacman_position[1] + self._generate_measurement_noise()
        agent_positions[BerkeleyAdapterAgent.pacman_index] = (pos_y, pos_x)

        for id_, pos in enumerate(self.pacman_game_state.getGhostPositions()):
            pos_y = pos[::-1][0] + self._generate_measurement_noise()
            pos_x = pos[::-1][1] + self._generate_measurement_noise()
            agent_positions[id_ + 1] = (pos_y, pos_x)

        food_positions = []
        for x, row in enumerate(self.pacman_game_state.getFood()):
            for y, is_food in enumerate(row):
                if is_food:
                    food_positions.append((y, x))

        fragile_agents = {}
        for id_, s in enumerate(self.pacman_game_state.data.agentStates):
            fragile_agents[id_] = 1.0 if s.scaredTimer > 0 else 0.0

        wall_positions = []
        for x, row in enumerate(self.pacman_game_state.getWalls()):
            for y, is_wall in enumerate(row):
                if is_wall:
                    wall_positions.append((y, x))

        if self.game_number == 0:
            self.game_state = state.GameState(
                agent_id=self.agent_id,
                width=self.layout.width,
                height=self.layout.height,
                ally_ids=self.ally_ids,
                enemy_ids=self.enemy_ids,
                walls=wall_positions,
                eater=(self.agent_type == 'pacman'),
                iteration=self.game_number)
        self.game_state.set_food_positions(food_positions)

        for id_, pos in agent_positions.items():
            self.game_state.observe_agent(id_, pos)

        for id_, status in fragile_agents.items():
            self.game_state.observe_fragile_agent(id_, status)

        message = messages.StateMessage(
            agent_id=self.agent_id,
            state=self.game_state,
            legal_actions=self.pacman_game_state.getLegalActions(
                self.agent_id),
            explore=self.is_learning)
        return message

    def _generate_measurement_noise(self):
        return random.randrange(
            -BerkeleyAdapterAgent.noise, BerkeleyAdapterAgent.noise + 1)

    def receive_action(self):
        logger.debug('#{} Receiving action'.format(self.agent_id))
        action_message = self.send_state()
        self.game_state.predict_agent(self.agent_id, action_message.action)
        return action_message.action

    def send_reward(self):
        if self.is_learning:
            logger.debug('#{} Sending reward'.format(self.agent_id))
            self.reward = self._calculate_reward(
                self.pacman_game_state.getScore())
            self.previous_score = self.pacman_game_state.getScore()

            message = messages.RewardMessage(
                agent_id=self.agent_id, state=self.game_state,
                action=self.previous_action, reward=self.reward)
            self.communicate(message)

    def _calculate_reward(self, current_score):
        return current_score - self.previous_score


def build_parser():
    parser = argparse.ArgumentParser(
        description='Run Pac-Man simulator adapter system.')
    parser.add_argument('-g', '--graphics', dest='graphics', default=False,
                        action='store_true',
                        help='display graphical user interface')
    parser.add_argument('-o', '--output', dest='output_file', type=str,
                        help='results output file')

    group = parser.add_argument_group('Experiment Setup')
    group.add_argument('-t', '--test-games', dest='test_games', type=int,
                       default=15,
                       help='number of games to test learned policy')
    group.add_argument('-l', '--learn-games', dest='learn_games', type=int,
                       default=100,
                       help='number of games to learn from')
    group.add_argument('--ghost-agent', dest='ghost_agent', type=str,
                       choices=['random', 'ai'], default='ai',
                       help='select ghost agent')
    group.add_argument('--pacman-agent', dest='pacman_agent', type=str,
                       choices=['random', 'random2', 'ai', 'eater',
                                'qlearning', 'sarsa'],
                       default='random',
                       help='select Pac-Man agent')
    group.add_argument('--layout', dest='layout', type=str,
                       default='classic', choices=['classic', 'medium'],
                       help='Game layout')
    group.add_argument('--steps', dest='steps', type=int,
                       default=None,
                       help='maximum number of steps per game')
    group.add_argument('--noise', dest='noise', type=int,
                       default=0,
                       help='introduce noise in position measurements')
    group.add_argument('--num-ghosts', dest='num_ghosts',
                       type=int, choices=range(0, 5),
                       default=3,
                       help='number of ghosts in game')
    group.add_argument('--policy-file', dest='policy_file',
                       type=lambda s: unicode(s, 'utf8'),
                       help='load and save Pac-Man policy from the given file')

    return parser


def build_adapter_with_args(args):
    return BerkeleyAdapter(
        pacman_agent=args.pacman_agent,
        ghost_agent=args.ghost_agent,
        num_ghosts=args.num_ghosts,
        noise=args.noise,
        max_steps=args.steps,
        policy_file=args.policy_file,
        layout_map=args.layout,
        learn_games=args.learn_games,
        test_games=args.test_games,
        output_file=args.output_file,
        graphics=args.graphics)


if __name__ == '__main__':
    parser = build_parser()
    args = parser.parse_args()
    adapter = build_adapter_with_args(args)

    try:
        adapter.run()
    except KeyboardInterrupt:
        print('\n\nInterrupted execution\n')

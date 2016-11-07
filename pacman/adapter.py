#!/usr/bin/env python
#  -*- coding: utf-8 -*-
"""
Adapts communication between controller and the Berkeley Pac-man simulator.
"""


import argparse
import os
import pickle
import random

from berkeley.game import Agent as BerkeleyGameAgent, Directions
from berkeley.graphicsDisplay import PacmanGraphics as BerkeleyGraphics
from berkeley.layout import getLayout as get_berkeley_layout
from berkeley.pacman import runGames as run_berkeley_games
from berkeley.textDisplay import NullGraphics as BerkeleyNullGraphics

import agents
import messages
from state import GameState

# @todo properly include communication module from parent folder
import sys
sys.path.insert(0, '..')
import communication
import core

__author__ = "Matheus Portela and Guilherme N. Ramos"
__credits__ = ["Matheus Portela", "Guilherme N. Ramos", "Renato Nobre",
               "Pedro Saman"]
__maintainer__ = "Guilherme N. Ramos"
__email__ = "gnramos@unb.br"


# Default settings (CLI parsing)
DEFAULT_GHOST_AGENT = 'ai'
DEFAULT_LAYOUT = 'classic'
DEFAULT_NUMBER_OF_GHOSTS = 3
DEFAULT_NUMBER_OF_LEARNING_GAMES = 100
DEFAULT_NUMBER_OF_TEST_GAMES = 15
DEFAULT_PACMAN_AGENT = 'random'
DEFAULT_NOISE = 0

# Pac-Man game configuration
NUMBER_OF_BERKELEY_GAMES = 1
RECORD_BERKELEY_GAMES = False


def log(msg):
    print '[  Adapter ] {}'.format(msg)


class BerkeleyAdapter(core.BaseExperiment):
    def __init__(self,
                 pacman_agent=DEFAULT_PACMAN_AGENT,
                 ghost_agent=DEFAULT_GHOST_AGENT,
                 num_ghosts=DEFAULT_NUMBER_OF_GHOSTS,
                 noise=DEFAULT_NOISE,
                 policy_file=None,
                 layout_map=DEFAULT_LAYOUT,
                 learn_games=DEFAULT_NUMBER_OF_LEARNING_GAMES,
                 test_games=DEFAULT_NUMBER_OF_TEST_GAMES,
                 output_file=None,
                 graphics=False,
                 context=None,
                 endpoint=None,
                 address=None,
                 port=None):

        super(BerkeleyAdapter, self).__init__()

        log('Instantiating adapter')

        # Layout
        LAYOUT_PATH = 'pacman/layouts'
        file_name = str(num_ghosts) + 'Ghosts'
        layout_file = '/'.join([LAYOUT_PATH, layout_map + file_name])
        self.layout = get_berkeley_layout(layout_file)
        if not self.layout:
            raise ValueError('Missing layout file "{}"'.format(layout_file))
        log('Loading layout file "{}"'.format(layout_file))

        # Pac-Man
        self.pacman = BerkeleyAdapterAgent(agent_id=0, agent_type='pacman',
                                           agent_algorithm=pacman_agent)

        # Ghosts
        self.num_ghosts = int(num_ghosts)
        if not (1 <= self.num_ghosts <= 4):
            raise ValueError('Must 1-4 ghost(s).')

        self.ghosts = []
        for i in xrange(num_ghosts):
            self.ghosts.append(BerkeleyAdapterAgent(
                                   agent_id=i+1, agent_type='ghost',
                                   agent_algorithm=ghost_agent))

        self.agents = [self.pacman] + self.ghosts

        # Policies
        self.policy_file = str(policy_file) if policy_file else None

        # Runs
        self.learn_games = int(learn_games)
        assert self.learn_games > 0

        self.test_games = int(test_games)
        assert self.test_games > 0

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

        log('Ready')

    def start(self):
        log('Starting')

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
            log('Loading policies from {}.'.format(filename))
            with open(filename) as f:
                self.policies = pickle.loads(f.read())

    def execute_game(self):
        log('Executing game')

        score = self._run_game()

        if self.is_learn_game:
            self.results['learn_scores'].append(score)
        else:
            self.results['test_scores'].append(score)

        for agent in self.agents:
            agent.results['scores'].append(score)

    def _run_game(self):
        simulated_game = run_berkeley_games(self.layout, self.pacman,
                                            self.ghosts, self.display,
                                            NUMBER_OF_BERKELEY_GAMES,
                                            RECORD_BERKELEY_GAMES)[0]

        # Do this so as agents can receive the last reward
        for agent in self.agents:
            agent.getAction(simulated_game.state)

        # Return game final score
        return simulated_game.state.getScore()

    def stop(self):
        log('Stopping')

        if self.policy_file:
            self._save_policies()

        log('Learn scores: {}'.format(self.results['learn_scores']))
        log('Test scores: {}'.format(self.results['test_scores']))

        self._write_to_file(self.output_file, self.results)

    def _save_policies(self):
        for agent in self.agents:
            if agent.policy:
                self.policies[agent.agent_id] = agent.policy

        self._write_to_file(self.policy_file, self.policies)

    def _write_to_file(self, filename, content):
        log('Saving results to "{}"'.format(filename))
        with open(filename, 'w') as f:
            f.write(pickle.dumps(content))


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
        action = self.receive_action()
        self.previous_action = action
        return action

    # BaseAdapterAgent required methods

    def start_experiment(self):
        log('#{} Starting experiment'.format(self.agent_id))
        self._send_start_experiment_message()

    def _send_start_experiment_message(self):
        log('#{} Registering {}/{}'.format(
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
        log('#{} Finishing experiment'.format(self.agent_id))
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
        log('#{} Starting game'.format(self.agent_id))
        self._send_start_game_message()
        self._reset_game_data()
        self._load_policy()

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
            log('#{} Loading policy'.format(self.agent_id))
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
        else:
            raise ValueError('Pac-Man agent must be ai, random, random2 or '
                             'eater.')

    def _register_ghost(self):
        if self.agent_algorithm == 'random':
            self.agent_class = agents.RandomGhostAgent
        elif self.agent_algorithm == 'ai':
            self.agent_class = agents.BehaviorLearningGhostAgent
        else:
            raise ValueError('Ghost agent must be ai or random.')

    def finish_game(self):
        log('#{} Finishing game'.format(self.agent_id))
        self._update_game_number()
        self._send_finish_game_message()

    def _update_game_number(self):
        self.game_number += 1

    def _send_finish_game_message(self):
        message = messages.FinishGameMessage(agent_id=self.agent_id)
        self.communicate(message)

    def send_state(self):
        log('#{} Sending state'.format(self.agent_id))
        message = self._create_state_message()
        return self.communicate(message)

    def _create_state_message(self):
        agent_positions = {}

        agent_positions[BerkeleyAdapterAgent.pacman_index] = (
            self.pacman_game_state.getPacmanPosition()[::-1])

        for id_, pos in enumerate(self.pacman_game_state.getGhostPositions()):
            pos_y = pos[::-1][0] + self._noise_error()
            pos_x = pos[::-1][1] + self._noise_error()
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

        self.reward = self._calculate_reward(self.pacman_game_state.getScore())
        self.previous_score = self.pacman_game_state.getScore()

        if self.game_number == 0:
            self.game_state = GameState(agent_id=self.agent_id,
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
            executed_action=self.previous_action,
            reward=self.reward,
            legal_actions=self.pacman_game_state.getLegalActions(
                self.agent_id),
            test_mode=(not self.is_exploring))
        return message

    def _noise_error(self):
        return random.randrange(-BerkeleyAdapterAgent.noise,
                                BerkeleyAdapterAgent.noise + 1)

    def receive_action(self):
        log('#{} Receiving action'.format(self.agent_id))
        action_message = self.send_state()
        self.game_state.predict_agent(self.agent_id, action_message.action)
        return action_message.action

    def send_reward(self):
        log('#{} Sending reward'.format(self.agent_id))
        message = messages.RewardMessage(
            agent_id=agent_id, action=self.last_action, reward=self.reward)
        self.communicate(message)

    def _calculate_reward(self, current_score):
        return current_score - self.previous_score


def build_adapter(context=None, endpoint=None,
                  address=communication.DEFAULT_CLIENT_ADDRESS,
                  port=communication.DEFAULT_TCP_PORT,
                  **kwargs):
    if context and endpoint:
        log('Connecting with inproc communication')
        adapter = BerkeleyAdapter(context=context, endpoint=endpoint, **kwargs)
    else:
        log('Connecting with TCP communication (address {}, port {})'.format(
            address, port))
        adapter = BerkeleyAdapter(address=address, port=port, **kwargs)

    return adapter


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Run Pac-Man simulator adapter system.')
    parser.add_argument('-g', '--graphics', dest='graphics', default=False,
                        action='store_true',
                        help='display graphical user interface')
    parser.add_argument('-o', '--output', dest='output_file', type=str,
                        help='results output file')

    group = parser.add_argument_group('Experiment Setup')
    group.add_argument('--ghost-agent', dest='ghost_agent', type=str,
                       choices=['random', 'ai'], default=DEFAULT_GHOST_AGENT,
                       help='select ghost agent')
    group.add_argument('-l', '--learn-games', dest='learn_games', type=int,
                       default=DEFAULT_NUMBER_OF_LEARNING_GAMES,
                       help='number of games to learn from')
    group.add_argument('--layout', dest='layout', type=str,
                       default=DEFAULT_LAYOUT, choices=['classic', 'medium'],
                       help='Game layout')
    group.add_argument('--noise', dest='noise', type=int,
                       default=DEFAULT_NOISE,
                       help='introduce noise in position measurements')
    group.add_argument('--num-ghosts', dest='num_ghosts',
                       type=int, choices=range(1, 5),
                       default=DEFAULT_NUMBER_OF_GHOSTS,
                       help='number of ghosts in game')
    group.add_argument('--pacman-agent', dest='pacman_agent', type=str,
                       choices=['random', 'random2', 'ai', 'eater'],
                       default=DEFAULT_PACMAN_AGENT,
                       help='select Pac-Man agent')
    group.add_argument('--policy-file', dest='policy_file',
                       type=lambda s: unicode(s, 'utf8'),
                       help='load and save Pac-Man policy from the given file')
    group.add_argument('-t', '--test-games', dest='test_games', type=int,
                       default=DEFAULT_NUMBER_OF_TEST_GAMES,
                       help='number of games to test learned policy')

    group = parser.add_argument_group('Communication')
    group.add_argument('--addr', dest='address', type=str,
                       default=communication.DEFAULT_CLIENT_ADDRESS,
                       help='Client address to connect to adapter (TCP '
                            'connection)')
    group.add_argument('--port', dest='port', type=int,
                       default=communication.DEFAULT_TCP_PORT,
                       help='Port to connect to controller (TCP connection)')

    args, unknown = parser.parse_known_args()

    adapter = build_adapter(
        address=communication.DEFAULT_CLIENT_ADDRESS,
        port=communication.DEFAULT_TCP_PORT,
        pacman_agent=args.pacman_agent,
        ghost_agent=args.ghost_agent,
        num_ghosts=args.num_ghosts,
        noise=args.noise,
        policy_file=args.policy_file,
        layout_map=args.layout,
        learn_games=args.learn_games,
        test_games=args.test_games,
        output_file=args.output_file,
        graphics=args.graphics)

    try:
        adapter.run()
    except KeyboardInterrupt:
        print '\n\nInterrupted execution\n'

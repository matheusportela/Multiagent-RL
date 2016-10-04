# -*- coding: utf-8 -*-
# @package pac_experiment.py
# @author Guilherme N. Ramos (gnramos@unb.br)
#
# Implements experiments with the Berkeley Pac-Man Game.

import pickle

from pac_utils import Experiment

# Pac-Man game configuration
PACMAN_INDEX = 0  # As established in pacman.berkeley.game


class PacmanExperiment(Experiment):
    """Defines a basic Pac-Man experiment."""
    def __init__(self, layout, display, pacman_agent, ghost_agent):
        super(PacmanExperiment, self).__init__()

        self.layout = layout
        self.display = display

        # Agents ##############################################################
        self.pacman = pacman_agent(PACMAN_INDEX)
        self.ghosts = [ghost_agent(x + 1)
                       for x in range(self.layout.numGhosts)]

        self.all_agents = {}
        for agent in [self.pacman] + self.ghosts:
            self.all_agents[agent.index] = agent

    def setup(self):
        self.results = []

        for agent in self.all_agents.values():
            agent.setup()

    def execute_step(self):
        self.logger.log('Simulating game...')

        # runGames returns a berkeley.game.Game class, not a
        # berkeley.pacman.GameState (the latter is stored in Game.state).
        from berkeley.pacman import runGames
        games = runGames(self.layout, self.pacman, self.ghosts, self.display,
                         numGames=1, record=False)

        self.results.append(games[0].state.getScore())

    def cleanup(self):
        for agent in self.all_agents.values():
            agent.cleanup()


class LearningPacmanExperiment(PacmanExperiment):
    def __init__(self, layout, display, pacman_agent, ghost_agent,
                 policy_file=None):
        super(LearningPacmanExperiment, self).__init__(layout, display,
                                                       pacman_agent,
                                                       ghost_agent)

        from pac_agents import LearningInterface
        self.learning_agents = {}
        for index, agent in self.all_agents.items():
            if isinstance(agent, LearningInterface):
                self.learning_agents[index] = agent

        from pac_agents import BehaviorInterface
        self.behavior_agents = {}
        for index, agent in self.all_agents.items():
            if isinstance(agent, BehaviorInterface):
                self.learning_agents[index] = agent

        # Policies
        self.policy_file = policy_file

    def run(self, learn_steps, test_steps):
        self.setup()

        for x in range(learn_steps):
            self.logger.log('Learning #{} (of {})'.format(x + 1, learn_steps))
            self.execute_step('learn')

        for agent in self.learning_agents.values():
            agent.enable_test_mode()

        for x in range(test_steps):
            self.logger.log('Testing #{} (of {})'.format(x + 1, test_steps))
            self.execute_step('test')

        self.cleanup()

    def setup(self):
        PacmanExperiment.setup(self)

        # Prepare to store results ############################################
        self.results = {'scores': {'learn': [], 'test': []}}
        if self.behavior_agents:
            self.results['behavior'] = {i: [] for i in self.behavior_agents}

        # Load policies #######################################################
        if self.policy_file:
            self.logger.log(
                'Loading policies from {}.'.format(self.policy_file))

            policies = {}
            with open(self.policy_file) as f:
                policies = pickle.loads(f.read())
            for index, policy in policies.items():
                if index not in self.learning_agents:
                    raise ValueError(
                        'Policy file does not match experiment configuration')
                self.agents[index].load(policy)

    def execute_step(self, exp_type):
        self.logger.log('Simulating game...')

        # runGames returns a berkeley.game.Game class, not a
        # berkeley.pacman.GameState (the latter is stored in Game.state).
        from berkeley.pacman import runGames
        games = runGames(self.layout, self.pacman, self.ghosts, self.display,
                         numGames=1, record=False)

        for agent in self.learning_agents.values():
            agent.update(games[0])

        # Keep track of games #################################################
        self.results['scores'][exp_type].append(games[0].state.getScore())

        for index, agent in self.behavior_agents.items():
            behavior, count = agent.get_behavior_count().items()
            self.results['behavior'][index][behavior].append(count)

    def cleanup(self):
        # Store policies ######################################################
        if self.policy_file:
            self.logger.log(
                'Writing policies into {}.'.format(self.policy_file))
            policies = {}
            for index, agent in self.learning_agents.items():
                policies[index] = agent.get_policy()
            with open(self.policy_file, 'w') as f:
                f.write(pickle.dumps(policies))

        # Cleanup agents ######################################################
        PacmanExperiment.cleanup(self)

def file_choices(choices, fname):
    from os.path import splitext
    ext = splitext(fname)[1][1:]
    if ext not in choices:
        parser.error('file doesn\'t end with one of {}'.format(choices))
    return unicode(fname, 'utf8')


if __name__ == '__main__':
    from argparse import ArgumentParser

    parser = ArgumentParser(description='Run a Pac-Man experiment.',
                            epilog='There is always only one Pac-man, and the '
                            'number of ghosts is defined by the layout file.')

    group = parser.add_argument_group('Simulator Setup')
    group.add_argument('-l', '--layout', dest='layout', type=str,
                       default='medium2Ghosts', help='game layout file')
    parser.add_argument('-q', '--quiet', dest='quiet',
                        action='store_true', default=False,
                        help='generate minimal output and no graphics')

    group = parser.add_argument_group('Agent Setup')
    group.add_argument('-p', '--pacman', dest='pacman', type=str,
                       default='RandomAgent',
                       help='the Pac-Man agent to use')
    group.add_argument('-g', '--ghost', dest='ghost', type=str,
                       default='RandomAgent',
                       help='the Ghost agent to use')

    group = parser.add_argument_group('Q-learning Setup')
    group.add_argument('--policy-file', dest='policy_file',
                       type=lambda s: file_choices(['pol'], s),
                       help='file for loading/storing policies.')

    args, unknown = parser.parse_known_args()

    # Simulator Setup #########################################################
    from berkeley.layout import getLayout
    layout = getLayout(args.layout)
    if not layout:
        raise ValueError('Layout {} missing.'.format(args.layout))

    if args.quiet:
        from berkeley.textDisplay import NullGraphics
        display = NullGraphics()
    else:
        from berkeley.graphicsDisplay import PacmanGraphics
        display = PacmanGraphics(zoom=1.0, frameTime=0.1)  # Berkeley defaults

    # Agents Setup ############################################################
    from berkeley.pacman import loadAgent
    # Agents must inherit berkeley.game.Agent and be defined inside a file
    # ending in 'gents.py' for this to work.
    pacman = loadAgent(args.pacman, display)
    ghost = loadAgent(args.ghost, display)

    # Experiment Setup ########################################################
    if 'ql' in [args.pacman, args.ghost]:
        experiment = LearningPacmanExperiment(layout, display, pacman, ghost,
                                              args.policy_file)
    else:
        experiment = PacmanExperiment(layout, display, pacman, ghost)

    experiment.run(2)

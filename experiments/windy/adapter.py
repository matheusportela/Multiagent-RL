import argparse
import logging
import pickle

import agents
from multiagentrl import core
from multiagentrl import messages
from simulator import WindyWorldSimulator


# Logging configuration
logging.getLogger().setLevel(logging.INFO)
logger = logging.getLogger(__name__)


class WindyExperiment(core.BaseExperiment):
    def __init__(self, learn_games, test_games, sleep, agent_algorithm,
                 output_file):
        super(WindyExperiment, self).__init__(
            learn_games=learn_games,
            test_games=test_games)

        logger.info('Instantiating adapter')

        self.simulator = WindyWorldSimulator(sleep=sleep)
        self.agent = WindyAgent(agent_algorithm)
        self.agents = [self.agent]
        self.results = {
            'learn_scores': [],
            'test_scores': []
        }
        self.output_file = output_file

    def start(self):
        logger.info('Starting')
        self.agent.map_width = self.simulator.cols
        self.agent.map_height = self.simulator.rows
        self.agent.actions = range(len(self.simulator.actions))

    def execute_game(self):
        logger.info('Executing game')
        self.simulator.start()

        # Send first state before start learning
        self.agent.state = self.simulator.get_state()
        self.agent.send_state()

        while not self.simulator.is_finished():
            # Receive an action for the current state
            action = self.agent.receive_action()

            # Simulate one step
            self.simulator.step(action)

            # Show simulation state
            logger.info('Simulator:\n{}'.format(self.simulator))

            # Update state to learn from the received reward
            self.agent.state = self.simulator.get_state()
            self.agent.send_state()

            # Get reward when executing the action and reaching the new state
            self.agent.reward = self.simulator.get_reward()
            self.agent.send_reward()

        if self.is_learn_game:
            self.results['learn_scores'].append(self.simulator.score)
        else:
            self.results['test_scores'].append(self.simulator.score)

    def stop(self):
        logger.info('Stopping')
        print self.results
        self._save_results()

    def _save_results(self):
        if self.output_file:
            logger.info('Saving results to "{}"'.format(self.output_file))
            pickle.dump(self.results, open(self.output_file, 'w'))


class WindyAgent(core.BaseAdapterAgent):
    def __init__(self, agent_algorithm):
        super(WindyAgent, self).__init__()
        self.agent_id = 0
        self.agent_type = 'windy'
        self.map_width = None
        self.map_height = None
        self.actions = None
        self.action = None
        self.state = None
        self.reward = 0
        self.is_learning = True
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

    def finish_experiment(self):
        logger.info('Finishing experiment')
        message = messages.FinishExperimentMessage(agent_id=self.agent_id)
        self.communicate(message)

    def start_game(self):
        logger.info('Starting game')
        message = messages.StartGameMessage(agent_id=self.agent_id)
        self.communicate(message)

    def finish_game(self):
        logger.info('Finishing game')
        message = messages.FinishGameMessage(agent_id=self.agent_id)
        self.communicate(message)

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
    parser = argparse.ArgumentParser(description='Run windy world system.')
    parser.add_argument(
        '-l', '--learn-games', dest='learn_games', type=int, default=1,
        help='number of games to learn from')
    parser.add_argument(
        '-t', '--test-games', dest='test_games', type=int, default=1,
        help='number of games to test learned policy')
    parser.add_argument(
        '-s', '--sleep', dest='sleep', type=float, default=0.1,
        help='seconds to sleep between game steps')
    parser.add_argument(
        '-a', '--agent', dest='agent', type=str, choices=['random', 'ai'],
        default='random', help='select Reinforcement Learning agent')
    parser.add_argument(
        '-o', '--output', dest='output', type=str,
        help='output file to save simulation results (e.g. "windy.res")')
    return parser


def build_adapter_with_args(args):
    return WindyExperiment(
        learn_games=args.learn_games,
        test_games=args.test_games,
        sleep=args.sleep,
        agent_algorithm=args.agent,
        output_file=args.output)

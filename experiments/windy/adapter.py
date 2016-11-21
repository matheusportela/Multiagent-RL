import argparse
import logging

import agents
from multiagentrl import core
from multiagentrl import messages
from simulator import WindyWorldSimulator


# Logging configuration
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('Windy')


class WindyExperiment(core.BaseExperiment):
    def __init__(self, learn_games, test_games, sleep, agent_algorithm):
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

    def start(self):
        logger.info('Starting')
        self.agent.map_width = self.simulator.cols
        self.agent.map_height = self.simulator.rows
        self.agent.actions = range(len(self.simulator.actions))

    def execute_game(self):
        logger.info('Executing game')
        self.simulator.start()

        while not self.simulator.is_finished():
            self.agent.state = self.simulator.get_state()
            self.agent.send_state()
            action = self.agent.receive_action()
            self.simulator.step(action)
            print self.simulator
            self.agent.reward = self.simulator.get_reward()
            self.agent.send_reward()

        if self.is_learn_game:
            self.results['learn_scores'].append(self.simulator.score)
        else:
            self.results['test_scores'].append(self.simulator.score)

    def stop(self):
        logger.info('Stopping')
        print self.results


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
        logger.info('Sending state')
        message = messages.StateMessage(
            agent_id=self.agent_id,
            state=self.state,
            legal_actions=self.actions,
            explore=self.is_learning)
        return self.communicate(message)

    def receive_action(self):
        logger.info('Receiving action')
        action_message = self.send_state()
        self.action = action_message.action
        return self.action

    def send_reward(self):
        if self.is_learning:
            logger.info('Sending reward')
            message = messages.RewardMessage(
                agent_id=self.agent_id, state=self.state,
                action=self.action, reward=self.reward)
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
        '-s', '--sleep', dest='sleep', type=int, default=0.1,
        help='seconds to sleep between game steps')
    parser.add_argument(
        '-a', '--agent', dest='agent', type=str, choices=['random', 'ai'],
        default='random', help='select Reinforcement Learning agent')
    return parser


def build_adapter_with_args(args):
    return WindyExperiment(
        learn_games=args.learn_games,
        test_games=args.test_games,
        sleep=args.sleep,
        agent_algorithm=args.agent)

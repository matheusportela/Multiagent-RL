#!/usr/bin/env python

from __future__ import division
import pymas
import random
import time

class Cell(object):
    def __init__(self):
        self.objects = []

    def __str__(self):
        if len(self.objects) == 0:
            return '.'
        return str(self.objects[-1])

    def add(self, object):
        self.objects.append(object)

    def remove(self, object):
        self.objects = [o for o in self.objects if id(o) != id(object)]


class EmptyCell(Cell):
    def __init__(self):
        super(EmptyCell, self).__init__()


class Map(object):
    empty = None

    def __init__(self, width=1, height=1):
        self.width = width
        self.height = height
        self.clear()

    def __str__(self):
        string = []

        for cell_line in self.cells:
            for cell in cell_line:
                string.append(str(cell))
                string.append(' ')
            string.append('\n')

        return ''.join([str(line) for line in string])

    def add_object(self, object, x, y):
        self.cells[x][y].add(object)

    def clear(self):
        self.cells = [[EmptyCell() for _ in range(self.width)] for _ in
                       range(self.height)]


class QLearner(object):
    """Q-learning algorithm implementation.

    Q-learning is a model free reinforcement learning algorithm that tries and
    learning state values and chooses actions that maximize the expected
    discounted reward for the current state.

    Instance variables:
    current_state -- State in which the algorithm currently is.
    q_values -- Matrix that stores the value for a (state, action) pair.
    learning_rate -- Value in [0, 1] interval that determines how much of the
        new information overrides the previous value. Deterministic scenarios
        may have optimal results with learning rate of 1, which means the new
        information completely replaces the old one.
    discount_factor -- Value in [0, 1) interval that determines the importance
        of future rewards. 0 makes the agent myopic and greedy, trying to
        achieve higher rewards in the next step. Closer to 1 makes the agent
        maximize long-term rewards. Although values of 1 and higher are possible,
        it may make the expected discounted reward infinite or divergent.
    """

    def __init__(self, initial_state=0, num_states=0, num_actions=0,
        learning_rate=1, discount_factor=1):
        """Constructor.

        Parameters:
        initial_state -- State where the algorithm begins.
        num_states -- Number of states to be represented.
        num_actions -- Number of actions to be represented.
        """
        self.current_state = initial_state
        self.q_values = QValues(num_states=num_states, num_actions=num_actions)
        self.learning_rate = learning_rate
        self.discount_factor = discount_factor

    def update_state(self, new_state):
        """Update Q Learning current state.

        Parameters:
        new_state -- State to which the learning algorithm is going.
        """
        self.current_state = new_state

    def learn(self, state, action, reward):
        """Learn by updating the (state, action) reward.

        Learn by applying the reward received when transitioning from the
        current state to the new one by executing an action.

        Parameters:
        state -- Agent state after executing the action.
        action -- Executed action.
        reward -- Reward received after executing the action.
        """
        old_value = self.q_values.get(self.current_state, action)
        next_expected_value = self.q_values.get_max_value(state)
        new_value = old_value + self.learning_rate*(reward + self.discount_factor*next_expected_value - old_value)
        self.q_values.set(self.current_state, action, new_value)

        self.update_state(state)

    def act(self, state):
        """Select an action for the given state.

        Parameters:
        state -- Agent state to select an action.
        """
        return self.q_values.get_max_action(state)

    def __str__(self):
        return ('Q-learning\n' + str(self.q_values))


class QValues(object):
    """Container for Q values.

    Instance variables:
    num_states -- Number of states that will be stored.
    num_actions -- Number of actions that will be stored.
    q_values -- Container for Q values.
    """

    def __init__(self, num_states=0, num_actions=0):
        self.num_states = num_states
        self.num_actions = num_actions
        self.q_values = [[0 for _ in xrange(num_actions)] for _ in xrange(num_states)]

    def get(self, state, action):
        """Get stored Q value for a (state, action) pair.

        Parameters:
        state -- State index.
        action -- Action index.
        """
        return self.q_values[state][action]

    def set(self, state, action, q_value):
        """Set Q value for a (state, action) pair.

        Parameters:
        state -- State index.
        action -- Action index.
        q_value -- Q value to be stored.
        """
        self.q_values[state][action] = q_value

    def get_max_value(self, state):
        """Returns the maximum Q value possible for the given state.

        Parameters:
        state -- State from which to find the maximum Q value possible.
        """
        return max(self.q_values[state])

    def get_max_action(self, state):
        """Returns the action index for which the Q value is maximum for the
        given state.

        Parameters:
        state -- State from which to find the action.
        """
        max_value = self.get_max_value(state)
        actions = [action for action, value in enumerate(self.q_values[state]) if value == max_value]
        return random.choice(actions)

    def __str__(self):
        output = ['\t%d' % action for action in range(self.num_actions)]
        output.append('\n')
        for state, values in enumerate(self.q_values):
            output.append('%d' % state)
            for value in values:
                output.append('\t%1.1f' % value)
            output.append('\n')
        return ''.join(output)


class EGreedyExplorer(object):
    """e-greedy exploration algorithm.

    Selects the suggested action or another random action with the given
    exploration_frequency.
    """
    def __init__(self, num_actions=1, exploration_frequency=0.0):
        self.actions = range(num_actions)
        self.exploration_frequency = exploration_frequency

    def select_action(self, suggested_action):
        if random.random() < self.exploration_frequency:
            return random.choice(self.actions)
        else:
            return suggested_action


class LearningAgent(pymas.Agent):
    def __init__(self):
        super(LearningAgent, self).__init__()
        self.predator_x = 0
        self.predator_y = 0
        self.prey_x = 0
        self.prey_y = 0
        self.rows = SimulatorAgent.map_rows
        self.cols = SimulatorAgent.map_cols
        self.learner = QLearner(initial_state=0, num_states=(self.rows*self.cols)**2,
            num_actions=4, learning_rate=0.9, discount_factor=0.9)
        self.explorer = EGreedyExplorer(num_actions=4, exploration_frequency=0.1)
        self.last_action = 0

    def on_run(self):
        state = self.calculate_state()
        suggested_action = self.learner.act(state)
        action = self.explorer.select_action(suggested_action)
        self.last_action = action
        self.send_message(pymas.Message(receiver=SimulatorAgent.simulator_id, data=action))

    def on_receive_message(self, message):
        if (message.sender == SimulatorAgent.simulator_id):
            if message.data == 'stop':
                self.stop()
            else:
                self.predator_x = message.data['predator_x']
                self.predator_y = message.data['predator_y']
                self.prey_x = message.data['prey_x']
                self.prey_y = message.data['prey_y']
                reward = message.data['reward']
                state = self.calculate_state()
                self.learner.learn(state, self.last_action, reward)

    def calculate_state(self):
        return (self.predator_x*self.cols + self.predator_y
            + self.rows*self.cols*(self.prey_x*self.cols + self.prey_y))


class PredatorAgent(LearningAgent):
    def __init__(self):
        super(PredatorAgent, self).__init__()

    def __str__(self):
        return 'P'

    def on_stop(self):
        print 'Predator learned Q-values'
        print str(self.learner)


class PreyAgent(LearningAgent):
    def __init__(self):
        super(PreyAgent, self).__init__()

    def __str__(self):
        return 'p'

    def on_stop(self):
        print 'Prey learned Q-values'
        print str(self.learner)


class SimulatorAgent(pymas.Agent):
    simulator_id = 0
    map_rows = 7
    map_cols = 7
    num_episodes = 1000

    def __init__(self):
        super(SimulatorAgent, self).__init__()
        self.map = Map(SimulatorAgent.map_rows, SimulatorAgent.map_cols)
        self.agents = []
        self.prey_agent = None
        self.predator_agents = []
        self.current_episode = 0
        SimulatorAgent.simulator_id = self.id

    def register_agent(self, agent):
        agent.x = random.randrange(self.map.width)
        agent.y = random.randrange(self.map.height)
        self.agents.append(agent)

        if type(agent) == PreyAgent:
            self.prey_agent = agent
        elif type(agent) == PredatorAgent:
            self.predator_agents.append(agent)

    def prepate_new_episode(self):
        for agent in self.agents:
            agent.x = random.randrange(0, SimulatorAgent.map_rows)
            agent.y = random.randrange(0, SimulatorAgent.map_cols)

    def on_start(self):
        self.prepate_new_episode()

    def on_run(self):
        self.map.clear()
        for agent in self.agents:
            self.map.add_object(agent, agent.x, agent.y)

        if self.current_episode == SimulatorAgent.num_episodes - 1:
            print str(self.map)
            time.sleep(0.5)

        for predator in self.predator_agents:
            if (predator.x, predator.y) == (self.prey_agent.x, self.prey_agent.y):
                self.send_message(pymas.Message(receiver=predator.id,
                    data={
                        'predator_x': predator.x,
                        'predator_y': predator.y,
                        'prey_x': self.prey_agent.x,
                        'prey_y': self.prey_agent.y,
                        'reward': 100,
                    }))
                self.send_message(pymas.Message(receiver=self.prey_agent.id,
                    data={
                        'predator_x': predator.x,
                        'predator_y': predator.y,
                        'prey_x': self.prey_agent.x,
                        'prey_y': self.prey_agent.y,
                        'reward': -100,
                    }))
            else:
                self.send_message(pymas.Message(receiver=predator.id,
                    data={
                        'predator_x': predator.x,
                        'predator_y': predator.y,
                        'prey_x': self.prey_agent.x,
                        'prey_y': self.prey_agent.y,
                        'reward': -1,
                    }))
                self.send_message(pymas.Message(receiver=self.prey_agent.id,
                    data={
                        'predator_x': predator.x,
                        'predator_y': predator.y,
                        'prey_x': self.prey_agent.x,
                        'prey_y': self.prey_agent.y,
                        'reward': 1,
                    }))

        for predator in self.predator_agents:
            if (predator.x, predator.y) == (self.prey_agent.x, self.prey_agent.y):
                print 'Predator %d capturated prey at %d, %d' % (predator.id,
                    predator.x, predator.y)

                self.current_episode += 1

                if self.current_episode == SimulatorAgent.num_episodes:
                    self.finish_simulation()
                else:
                    self.prepate_new_episode()

    def finish_simulation(self):
        for agent in self.agents:
            self.send_message(pymas.Message(receiver=agent.id, data='stop'))

        self.stop()

    def on_receive_message(self, message):
        action = message.data
        agent = None

        for a in self.agents:
            if a.id == message.sender:
                agent = a

        if not agent:
            return

        if action == 0:
            agent.x += 1
        elif action == 1:
            agent.x -= 1
        elif action == 2:
            agent.y += 1
        elif action == 3:
            agent.y -= 1

        if agent.x >= self.map.width:
            agent.x = self.map.width - 1
        elif agent.x < 0:
            agent.x = 0

        if agent.y >= self.map.height:
            agent.y = self.map.height - 1
        elif agent.y < 0:
            agent.y = 0


if __name__ == '__main__':
    system = pymas.System()
    simulator = system.add_agent(SimulatorAgent)
    simulator.register_agent(system.add_agent(PreyAgent))
    simulator.register_agent(system.add_agent(PredatorAgent))
    system.run(sleep=0)
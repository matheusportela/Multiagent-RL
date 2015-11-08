from simulator import pacman as pacman_simulator
from simulator import layout as simulator_layout
from simulator import textDisplay
from simulator import graphicsDisplay
from simulator import game

import communication as comm
import messages
import pickle
import random
import argparse
import agents


class CommunicatingAgent(game.Agent):
    def __init__(self, agent_id):
        super(CommunicatingAgent, self).__init__()
        self.agent_id = agent_id
        self.client = comm.Client()
        self.previous_score = 0
        self.previous_action = 'Stop'
        self.invalid_action = False
        self.actions = []
        self.init = True
        self.explore = True

    def enable_explore(self):
        self.explore = True

    def disable_explore(self):
        self.explore = False

    def calculate_reward(self, current_score):
        raise NotImplementedError, 'Communicating agent must calculate score'

    def create_state_message(self, state):
        agent_positions = {}
        agent_positions[0] = state.getPacmanPosition()[::-1]
        for id_, g in enumerate(state.getGhostPositions()):
            agent_positions[id_ + 1] = g[::-1]

        food_positions = []

        for x, k in enumerate(state.getFood()):
            for y, l in enumerate(k):
                if l:
                    food_positions.append((y, x))

        wall_positions = []

        for x, k in enumerate(state.getWalls()):
            for y, l in enumerate(k):
                if l:
                    wall_positions.append((y, x))

        reward = self.calculate_reward(state.getScore())
        self.previous_score = state.getScore()

        message = messages.StateMessage(
            agent_id=self.agent_id,
            agent_positions=agent_positions,
            food_positions=food_positions,
            wall_positions=wall_positions,
            legal_actions=state.getLegalActions(self.agent_id),
            reward=reward,
            executed_action=self.previous_action,
            explore=self.explore)

        return message

    def create_save_message(self, filename):
        message = messages.SaveMessage(
            agent_id=self.agent_id,
            filename=filename)

        return message

    def create_load_message(self, filename):
        message = messages.LoadMessage(
            agent_id=self.agent_id,
            filename=filename)

        return message

    def init_game(self):
        self.previous_score = 0
        self.previous_action = 'Stop'
        self.send_message(messages.InitMessage(agent_id=self.agent_id))
        self.receive_message()

    def register_agent(self, agent_team, agent_class):
        message = messages.RegisterMessage(
            agent_id=self.agent_id,
            agent_team=agent_team,
            agent_class=agent_class)
        self.send_message(message)
        self.receive_message()

    def send_message(self, message):
        self.client.send(pickle.dumps(message))

    def receive_message(self):
        return pickle.loads(self.client.recv())

    def act_when_invalid(self, state):
        raise NotImplementedError

    def getAction(self, state):
        message = self.create_state_message(state)
        self.send_message(message)

        message = self.receive_message()
        while message.agent_id != self.agent_id:
            message = self.receive_message()

        self.previous_action = message.action

        if message.action not in state.getLegalActions(self.agent_id):
            self.invalid_action = True
            return self.act_when_invalid(state)
        else:
            self.invalid_action = False
            return message.action


class CommunicatingPacmanAgent(CommunicatingAgent):
    def __init__(self):
        super(CommunicatingPacmanAgent, self).__init__(0)
        self.actions = ['North', 'South', 'East', 'West', 'Stop']

    def act_when_invalid(self, state):
        return 'Stop'

    def calculate_reward(self, current_score):
        return current_score - self.previous_score


class CommunicatingGhostAgent(CommunicatingAgent):
    def __init__(self, agent_id):
        super(CommunicatingGhostAgent, self).__init__(agent_id)
        self.previous_action = 'North'
        self.actions = ['North', 'South', 'East', 'West']

    def act_when_invalid(self, state):
        return random.choice(state.getLegalActions(self.agent_id))

    def calculate_reward(self, current_score):
        return self.previous_score - current_score


def create_layout(layout_file):
    layout = simulator_layout.getLayout(layout_file)

    if layout == None:
        raise Exception("The layout " + layout_file + " cannot be found")

    return layout

def create_pacman(agent_class):
    agent = CommunicatingPacmanAgent()
    agent.register_agent('pacman', agent_class)
    print 'Created Pacman\tID: %d\tClass: %s' % (agent.agent_id, agent_class.__name__)
    return agent

def create_ghosts(num_ghosts, agent_class):
    agents = []

    for i in range(num_ghosts):
        agent = CommunicatingGhostAgent(i+1)
        agent.register_agent('ghost', agent_class)
        print 'Created ghost\tID: %d\tClass: %s' % (agent.agent_id, agent_class.__name__)
        agents.append(agent)

    return agents

def create_display(display_type='None', zoom=1.0, frameTime=0.1):
    if display_type == 'Text':
        display = textDisplay.PacmanGraphics()
    elif display_type == 'Graphic':
        display = graphicsDisplay.PacmanGraphics(zoom, frameTime=frameTime)
    elif display_type == 'None':
        display = textDisplay.NullGraphics()
    else:
        raise ValueError, 'Display type must be either Text, Graphic, or None'

    return display

def load_policy(filename):
    print 'Loading policy from', filename
    pacman = create_pacman()
    msg = pacman.create_load_message(filename)
    pacman.send_message(msg)
    pacman.receive_message()

def save_policy(filename):
    print 'Saving policy to', filename
    pacman = create_pacman()
    msg = pacman.create_save_message(filename)
    pacman.send_message(msg)
    pacman.receive_message()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Run Pacman simulations.')
    parser.add_argument('-l', '--learn-num', dest='learn', type=int, default=100,
                       help='number of games to learn from')
    parser.add_argument('-t', '--test-num', dest='test', type=int, default=100,
                       help='number of games to test learned policy')
    parser.add_argument('-p', '--policy-file', dest='policy_filename', type=str,
                        help='load and save Pacman policy from the given file')
    parser.add_argument('-g', '--graphics', dest='graphics', action='store_true',
                        help='display graphical user interface')
    parser.add_argument('--no-graphics', dest='graphics', action='store_false',
                        help='do not display graphical user interface')
    parser.set_defaults(graphics=False)

    args = parser.parse_args()

    layout_file = 'mediumClassic'
    num_ghosts = 2
    # layout_file = 'oneGhostMediumClassic'
    # num_ghosts = 1
    # layout_file = 'ghostlessMediumClassic'
    # num_ghosts = 0
    learn_games = args.learn
    test_games = args.test
    pacman_policy_filename = args.policy_filename
    record = False
    # pacman_class = agents.BehaviorLearningPacmanAgent
    # ghost_class = agents.RandomGhostAgent
    pacman_class = agents.RandomPacmanAgent
    ghost_class = agents.BehaviorLearningGhostAgent

    if args.graphics:
        display_type = 'Graphic'
    else:
        display_type = 'None'

    layout = create_layout(layout_file)
    display = create_display(display_type=display_type)

    learn_scores = []
    test_scores = []
    log_behavior_count = []

    if pacman_policy_filename:
        load_policy(pacman_policy_filename)

    pacman = create_pacman(pacman_class)
    ghosts = create_ghosts(num_ghosts, ghost_class)

    for i in range(learn_games + test_games):
        print '\nGame #%d' % (i+1)

        pacman.init_game()
        for ghost in ghosts:
            ghost.init_game()

        if i >= learn_games:
            pacman.disable_explore()

            for ghost in ghosts:
                ghost.disable_explore()

        games = pacman_simulator.runGames(layout, pacman, ghosts, display, 1, record)

        # Do this so as agents can receive the last reward
        pacman.send_message(pacman.create_state_message(games[0].state))
        pacman.receive_message()

        for ghost in ghosts:
            ghost.send_message(ghost.create_state_message(games[0].state))
            ghost.receive_message()

        # Log behavior count
        if pacman_class == agents.BehaviorLearningPacmanAgent:
            msg = messages.RequestBehaviorCountMessage(agent_id=pacman.agent_id)
            pacman.send_message(msg)
            behavior_count_msg = pacman.receive_message()

        if ghost_class == agents.BehaviorLearningGhostAgent:
            for ghost in ghosts:
                msg = messages.RequestBehaviorCountMessage(agent_id=ghost.agent_id)
                ghost.send_message(msg)
                behavior_count_msg = ghost.receive_message()

        print behavior_count_msg.count
        log_behavior_count.append(behavior_count_msg.count)

        # Log score
        if i >= learn_games:
            test_scores.append(games[0].state.getScore())
        else:
            learn_scores.append(games[0].state.getScore())

    if pacman_policy_filename:
        save_policy(pacman_policy_filename)

    print learn_scores
    print test_scores

    with open('results/learn_scores.txt', 'w') as output:
        for score in learn_scores:
            output.write(str(score) + '\n')

    with open('results/test_scores.txt', 'w') as output:
        for score in test_scores:
            output.write(str(score) + '\n')

    with open('results/behavior_count.txt', 'w') as output:
        names = [name for name in log_behavior_count[0]]
        output.write(','.join(names) + '\n')
        output.write('\n'.join([','.join([str(behavior_count[name]) for name in names]) for behavior_count in log_behavior_count]))
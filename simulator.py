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


class CommunicatingAgent(game.Agent):
    def __init__(self, index):
        super(CommunicatingAgent, self).__init__()
        self.index = index
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

    def create_message(self, state):
        pacman_position = state.getPacmanPosition()[::-1]
        ghost_positions = [g[::-1] for g in state.getGhostPositions()]

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

        if self.invalid_action:
            # Punish for invalid actions
            reward = -100
        else:
            reward = state.getScore() - self.previous_score
        self.previous_score = state.getScore()

        message = messages.StateMessage(
            index=self.index,
            pacman_position=pacman_position,
            ghost_positions=ghost_positions,
            food_positions=food_positions,
            wall_positions=wall_positions,
            legal_actions=state.getLegalActions(self.index),
            reward=reward,
            executed_action=self.previous_action,
            explore=self.explore)

        return message

    def create_save_message(self, filename):
        message = messages.SaveMessage(
            index=self.index,
            filename=filename)

        return message

    def create_load_message(self, filename):
        message = messages.LoadMessage(
            index=self.index,
            filename=filename)

        return message

    def send_message(self, message):
        self.client.send(pickle.dumps(message))

    def receive_message(self):
        return pickle.loads(self.client.recv())

    def act_when_invalid(self, state):
        raise NotImplementedError

    def getAction(self, state):
        if self.init and self.index == 0:
            self.init = False
            message = messages.InitMessage(msg_type=messages.INIT)
            self.send_message(message)
            self.receive_message()

        message = self.create_message(state)
        self.send_message(message)

        message = self.receive_message()
        while message.index != self.index:
            message = self.receive_message()

        self.previous_action = message.action

        if message.action not in state.getLegalActions(self.index):
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


class CommunicatingGhostAgent(CommunicatingAgent):
    def __init__(self, index):
        super(CommunicatingGhostAgent, self).__init__(index)
        self.previous_action = 'North'
        self.actions = ['North', 'South', 'East', 'West']

    def act_when_invalid(self, state):
        return random.choice(state.getLegalActions(self.index))


def create_layout(layout_file):
    layout = simulator_layout.getLayout(layout_file)

    if layout == None:
        raise Exception("The layout " + layout_file + " cannot be found")

    return layout

def create_pacman():
    return CommunicatingPacmanAgent()

def create_ghosts(num_ghosts):
    return [CommunicatingGhostAgent(i+1) for i in range(num_ghosts)]

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
    parser.add_argument('-l', dest='learn', type=int, default=100,
                       help='number of games to learn from')
    parser.add_argument('-t', dest='test', type=int, default=100,
                       help='number of games to test learned policy')
    parser.add_argument('-p', dest='policy_filename', type=str,
                        help='load and save Pacman policy from the given file')

    args = parser.parse_args()

    # layout_file = 'mediumClassic'
    # layout_file = 'ghostlessMediumClassic'
    layout_file = 'oneGhostMediumClassic'
    num_ghosts = 1
    learn_games = args.learn
    test_games = args.test
    pacman_policy_filename = args.policy_filename
    record = False
    display_type = 'None'

    layout = create_layout(layout_file)
    display = create_display(display_type=display_type)

    learn_scores = []
    test_scores = []
    log_behavior_count = []

    if pacman_policy_filename:
        load_policy(pacman_policy_filename)

    for i in range(learn_games + test_games):
        print '\nGame #%d' % (i+1)

        pacman = create_pacman()
        ghosts = create_ghosts(num_ghosts)

        if i >= learn_games:
            pacman.enable_explore()

            for ghost in ghosts:
                ghost.enable_explore()

        games = pacman_simulator.runGames(layout, pacman, ghosts, display, 1, record)
        # games = pacman_simulator.runGames(layout, pacman, ghosts, create_display(display_type='Graphic'), 1, record)

        # Do this so as Pacman can receive the last reward
        msg = pacman.create_message(games[0].state)
        pacman.send_message(msg)
        pacman.receive_message()

        # Log behavior count
        msg = messages.RequestBehaviorCountMessage(index=pacman.index)
        pacman.send_message(msg)
        behavior_count_msg = pacman.receive_message()
        print behavior_count_msg.count
        log_behavior_count.append(behavior_count_msg.count)

        if i >= learn_games:
            test_scores.append(games[0].state.getScore())
        else:
            learn_scores.append(games[0].state.getScore())

    if pacman_policy_filename:
        save_policy(pacman_policy_filename)

    print learn_scores
    print test_scores

    with open('learn_scores.txt', 'w') as output:
        for score in learn_scores:
            output.write(str(score) + '\n')

    with open('test_scores.txt', 'w') as output:
        for score in test_scores:
            output.write(str(score) + '\n')

    with open('behavior_count.txt', 'w') as output:
        names = [name for name in log_behavior_count[0]]
        output.write(','.join(names) + '\n')
        output.write('\n'.join([','.join([str(behavior_count[name]) for name in names]) for behavior_count in log_behavior_count]))
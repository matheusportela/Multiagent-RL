from simulator import pacman as pacman_simulator
from simulator import layout as simulator_layout
from simulator import textDisplay
from simulator import graphicsDisplay
from simulator import game

import communication as comm
import messages
import pickle


class CommunicatingAgent(game.Agent):
    def __init__(self, index):
        super(CommunicatingAgent, self).__init__()
        self.index = index
        self.client = comm.Client()
        self.previous_score = 0
        self.previous_action = 'Stop'
        self.invalid_action = False

    def create_message(self, state):
        food_positions = []

        for x, k in enumerate(state.getFood()):
            for y, l in enumerate(k):
                if l:
                    food_positions.append((x, y))

        if self.invalid_action:
            # Punish for invalid actions
            reward = -100
        else:
            reward = state.getScore() - self.previous_score
        self.previous_score = state.getScore()

        message = messages.StateMessage(
            msg_type = messages.STATE,
            index=self.index,
            pacman_position=state.getPacmanPosition(),
            ghost_positions=state.getGhostPositions(),
            food_positions=food_positions,
            legal_actions=['North', 'South', 'East', 'West', 'Stop'],
            reward=reward,
            executed_action=self.previous_action)

        return message

    def send_message(self, message):
        self.client.send(pickle.dumps(message))

    def receive_message(self):
        return pickle.loads(self.client.recv())

    def getAction(self, state):
        message = self.create_message(state)
        self.send_message(message)

        message = self.receive_message()
        while message.index != self.index:
            message = self.receive_message()

        self.previous_action = message.action

        if message.action not in state.getLegalActions(self.index):
            self.invalid_action = True
            return 'Stop'
        else:
            self.invalid_action = False
            return message.action


class CommunicatingPacmanAgent(CommunicatingAgent):
    def __init__(self):
        super(CommunicatingPacmanAgent, self).__init__(0)


def create_layout(layout_file):
    layout = simulator_layout.getLayout(layout_file)

    if layout == None:
        raise Exception("The layout " + layout_file + " cannot be found")

    return layout

def create_pacman():
    return CommunicatingPacmanAgent()

def create_ghosts(num_ghosts):
    return [CommunicatingAgent(i+1) for i in range(num_ghosts)]

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

if __name__ == '__main__':
    # layout_file = 'mediumClassic'
    layout_file = 'ghostlessMediumClassic'
    # num_ghosts = 4
    num_ghosts = 0
    num_games = 1000
    record = False
    display_type = 'None'

    layout = create_layout(layout_file)
    pacman = create_pacman()
    ghosts = create_ghosts(num_ghosts)
    display = create_display(display_type=display_type)

    scores = []

    for i in range(num_games):
        print '\nGame #%d' % (i+1)
        games = pacman_simulator.runGames(layout, pacman, ghosts, display, 1, record)
        # games = pacman_simulator.runGames(layout, pacman, ghosts, create_display(display_type='Graphic'), 1, record)

        # Do this so as Pacman can receive the last reward
        pacman.getAction(games[0].state)

        scores.append(games[0].state.getScore())

    print scores

    import matplotlib.pylab as plt

    plt.scatter(range(len(scores)), scores)
    plt.show()
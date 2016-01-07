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
import os

NOISE = 0

class CommunicatingAgent(game.Agent):
    def __init__(self, agent_id, port):
        super(CommunicatingAgent, self).__init__()
        self.agent_id = agent_id
        self.client = comm.Client(port=port)
        self.previous_score = 0
        self.previous_action = 'Stop'
        self.invalid_action = False
        self.actions = []
        self.init = True
        self.test_mode = False

    def enable_test_mode(self):
        self.test_mode = True

    def enable_learn_mode(self):
        self.test_mode = False

    def calculate_reward(self, current_score):
        raise NotImplementedError, 'Communicating agent must calculate score'

    def _introduce_position_error(self, pos, min_, max_):
        ex = random.choice(range(min_, max_ + 1))
        ey = random.choice(range(min_, max_ + 1))

        return (pos[0] + ex, pos[1] + ey)

    def create_state_message(self, state):
        agent_positions = {}
        agent_positions[0] = state.getPacmanPosition()[::-1]
        for id_, pos in enumerate(state.getGhostPositions()):
            if NOISE == 0:
                agent_positions[id_ + 1] = pos[::-1]
            else:
                agent_positions[id_ + 1] = self._introduce_position_error(pos[::-1], -NOISE, NOISE)

        food_positions = []

        for x, k in enumerate(state.getFood()):
            for y, l in enumerate(k):
                if l:
                    food_positions.append((y, x))

        fragile_agents = {}
        for id_, s in enumerate(state.data.agentStates):
            if s.scaredTimer > 0:
                fragile_agents[id_] = 1.0
            else:
                fragile_agents[id_] = 0.0

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
            fragile_agents=fragile_agents,
            wall_positions=wall_positions,
            legal_actions=state.getLegalActions(self.agent_id),
            reward=reward,
            executed_action=self.previous_action,
            test_mode=self.test_mode)

        return message

    def init_agent(self):
        self.send_message(messages.InitMessage(agent_id=self.agent_id))
        self.receive_message()

    def start_game(self, map_width, map_height):
        self.previous_score = 0
        self.previous_action = 'Stop'
        self.send_message(messages.StartMessage(
            agent_id=self.agent_id,
            map_width=map_width,
            map_height=map_height))
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
    def __init__(self, port):
        super(CommunicatingPacmanAgent, self).__init__(0, port)
        self.actions = ['North', 'South', 'East', 'West', 'Stop']

    def act_when_invalid(self, state):
        return 'Stop'

    def calculate_reward(self, current_score):
        return current_score - self.previous_score


class CommunicatingGhostAgent(CommunicatingAgent):
    def __init__(self, agent_id, port):
        super(CommunicatingGhostAgent, self).__init__(agent_id, port)
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

def create_pacman(agent_class, port):
    agent = CommunicatingPacmanAgent(port=port)
    agent.register_agent('pacman', agent_class)
    print 'Created Pacman\tID: %d\tClass: %s' % (agent.agent_id, agent_class.__name__)
    return agent

def create_ghosts(num_ghosts, agent_class, port):
    agents = []

    for i in range(num_ghosts):
        agent = CommunicatingGhostAgent(i+1, port=port)
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

def save_results(filename, results):
    with open(filename, 'w') as f:
        f.write(pickle.dumps(results))

def main():
    parser = argparse.ArgumentParser(description='Run Pacman adapter system.')
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
    parser.add_argument('-e', '--experiment', dest='experiment_number', type=int,
                        default=3, help='select experiment from 1 to 6')
    parser.add_argument('--pacman-agent', dest='pacman_agent', type=str,
                        default='random', help='select pacman agent: random, ai, or eater')
    parser.add_argument('--ghost-agent', dest='ghost_agent', type=str,
                        default='ai', help='select ghost agent: random or ai')
    parser.add_argument('-o', '--output', dest='output_filename', type=str,
                        default='results.txt', help='results output file')
    parser.add_argument('--noise', dest='noise', type=int, default=0,
                        help='introduce noise in position measurements')
    parser.add_argument('--port', dest='port', type=int, default=5555,
                        help='TCP port to connect to controller')
    parser.set_defaults(graphics=False)

    args = parser.parse_args()

    if args.experiment_number == 1:
        layout_file = 'simulator/layouts/classic1Ghost'
        num_ghosts = 1
    elif args.experiment_number == 2:
        layout_file = 'simulator/layouts/classic2Ghosts'
        num_ghosts = 2
    elif args.experiment_number == 3:
        layout_file = 'simulator/layouts/classic3Ghosts'
        num_ghosts = 3
    elif args.experiment_number == 4:
        layout_file = 'simulator/layouts/classic4Ghosts'
        num_ghosts = 4
    elif args.experiment_number == 5:
        layout_file = 'simulator/layouts/medium1Ghosts'
        num_ghosts = 1
    elif args.experiment_number == 6:
        layout_file = 'simulator/layouts/medium2Ghosts'
        num_ghosts = 2
    else:
        raise ValueError, 'Experiment number must be between 1 and 6'

    global NOISE
    NOISE = args.noise

    learn_games = args.learn
    test_games = args.test
    policy_filename = args.policy_filename
    results_output_filename = args.output_filename
    policies = {}
    record = False

    if args.pacman_agent == 'random':
        pacman_class = agents.RandomPacmanAgent
    elif args.pacman_agent == 'ai':
        pacman_class = agents.BehaviorLearningPacmanAgent
    elif args.pacman_agent == 'eater':
        pacman_class = agents.EaterPacmanAgent
    else:
        raise ValueError, 'Pacman agent must be random, ai, or eater'

    if args.ghost_agent == 'random':
        ghost_class = agents.RandomGhostAgent
    elif args.ghost_agent == 'ai':
        ghost_class = agents.BehaviorLearningGhostAgent
    else:
        raise ValueError, 'Ghost agent must be random or ai'

    if args.graphics:
        display_type = 'Graphic'
    else:
        display_type = 'None'

    layout = create_layout(layout_file)
    map_width = layout.width
    map_height = layout.height

    display = create_display(display_type=display_type)

    results = {
        'learn_scores': [],
        'test_scores': [],
        'behavior_count': {}
    }

    pacman = create_pacman(pacman_class, args.port)
    ghosts = create_ghosts(num_ghosts, ghost_class, args.port)

    if pacman_class == agents.BehaviorLearningPacmanAgent:
        results['behavior_count'][pacman.agent_id] = {}

    if ghost_class == agents.BehaviorLearningGhostAgent:
        for ghost in ghosts:
            results['behavior_count'][ghost.agent_id] = {}

    # Load policies from file
    if policy_filename and os.path.isfile(policy_filename):
        print 'Loading policies from file'
        with open(policy_filename) as f:
            policies = pickle.loads(f.read())

    # Initialize agents
    pacman.init_agent()
    for ghost in ghosts:
        ghost.init_agent()

    for i in range(learn_games + test_games):
        print '\nGame #%d' % (i+1)

        # Start new game
        pacman.start_game(map_width, map_height)
        for ghost in ghosts:
            ghost.start_game(map_width, map_height)

        # Load policies to agents
        if policy_filename and os.path.isfile(policy_filename):
            print 'Loading policies to agents'
            if pacman.agent_id in policies:
                print 'Loading Pacman policy'
                pacman.send_message(messages.PolicyMessage(
                    agent_id=pacman.agent_id,
                    policy=policies[pacman.agent_id]))
                pacman.receive_message()


            for ghost in ghosts:
                if ghost.agent_id in policies:
                    print 'Loading ghost %d policy' % ghost.agent_id
                    ghost.send_message(messages.PolicyMessage(
                        agent_id=ghost.agent_id,
                        policy=policies[ghost.agent_id]))
                    ghost.receive_message()

        if i >= learn_games:
            pacman.enable_test_mode()

            for ghost in ghosts:
                ghost.enable_test_mode()

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
            print 'Pacman behavior count:', behavior_count_msg.count
            for behavior, count in behavior_count_msg.count.items():
                if behavior not in results['behavior_count'][pacman.agent_id]:
                    results['behavior_count'][pacman.agent_id][behavior] = []
                results['behavior_count'][pacman.agent_id][behavior].append(count)

        if ghost_class == agents.BehaviorLearningGhostAgent:
            for ghost in ghosts:
                msg = messages.RequestBehaviorCountMessage(agent_id=ghost.agent_id)
                ghost.send_message(msg)
                behavior_count_msg = ghost.receive_message()
                print 'Ghost', ghost.agent_id, 'behavior count:', behavior_count_msg.count
                for behavior, count in behavior_count_msg.count.items():
                    if behavior not in results['behavior_count'][ghost.agent_id]:
                        results['behavior_count'][ghost.agent_id][behavior] = []
                    results['behavior_count'][ghost.agent_id][behavior].append(count)

        # Log score
        if i >= learn_games:
            results['test_scores'].append(games[0].state.getScore())
        else:
            results['learn_scores'].append(games[0].state.getScore())

    # Save policies
    if policy_filename:
        if pacman_class == agents.BehaviorLearningPacmanAgent:
            pacman.send_message(messages.RequestPolicyMessage(pacman.agent_id))
            msg = pacman.receive_message()
            policies[pacman.agent_id] = msg.policy

        if ghost_class == agents.BehaviorLearningGhostAgent:
            for ghost in ghosts:
                ghost.send_message(messages.RequestPolicyMessage(ghost.agent_id))
                msg = ghost.receive_message()
                policies[ghost.agent_id] = msg.policy

        with open(policy_filename, 'w') as f:
            f.write(pickle.dumps(policies))

    # Save results
    print 'Learn scores:', results['learn_scores']
    print 'Test scores:', results['test_scores']

    save_results(results_output_filename, results)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print '\n\nInterrupted execution\n'
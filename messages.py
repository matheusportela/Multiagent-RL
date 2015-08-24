class State(object):
    def __init__(self, index=None, pacman_position=None, ghost_positions=None,
        legal_actions=None):
        self.index = index
        self.pacman_position = pacman_position
        self.ghost_positions = ghost_positions
        self.legal_actions = legal_actions
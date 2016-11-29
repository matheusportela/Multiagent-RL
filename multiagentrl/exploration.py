#  -*- coding: utf-8 -*-

"""Collection of exploration algorithms.

This module contains classes implementing exploration algorithms. These classes
are agnostic to their usage, containing no information specific to environments
where they will be applied to.

In order to achieve that, all classes must inherit from
BaseExplorationAlgorithm and implement its virtual methods.
"""

import random


class BaseExplorationAlgorithm(object):
    def explore(self, selected_action, legal_actions):
        raise NotImplementedError


class EGreedy(BaseExplorationAlgorithm):
    def __init__(self, exploration_rate):
        super(EGreedy, self).__init__()
        self.exploration_rate = exploration_rate

    def explore(self, selected_action, legal_actions):
        p = random.random()

        if p < self.exploration_rate:
            return random.choice(legal_actions)
        else:
            return selected_action

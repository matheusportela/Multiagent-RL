"""Represent and operate on probability distributions."""

from __future__ import division
import numpy as np
import matplotlib.pylab as plt


class Grid(object):
    """Probability distribution grid.

    Every cell contains a value in the interval [0, 1] indicating a probability.
    The grid is indexed according to the values min, max, and step.

    For instance, the following syntax is valid:

    g = Grid(min=0, max=3, step=0.5)
    g[0] = g[0.5] = g[1] = g[1.5] = g[2] = g[2.5] = g[3] = 1
    g.normalize()

    Attributes:
    min -- Minimum grid index.
    max -- Maximum grid index.
    step -- Step between grid indices.
    num_cells -- Number of grid cells.
    indices -- List of valid grid indices.
    cells -- Grid probability values.
    """
    def __init__(self, min=0, max=1, step=1):
        self.min = min
        self.max = max
        self.step = step
        self.num_cells = int(np.ceil((max - min)/step)) + 1
        self.indices = [i*step + min for i in range(self.num_cells)]
        self.cells = np.array([1 for _ in range(self.num_cells)])
        self.normalize()

    def _check_index(self, i):
        """Check whether the index is valid for the grid dimensions."""
        if i not in self.indices:
            raise ValueError, 'Grid index must be in the interval [{}, {}] (given index: {})'.format(self.min, self.max, i)

    def __getitem__(self, i):
        """Retrieve a grid value."""
        self._check_index(i)
        index = (i - self.min)/self.step
        return self.cells[index]

    def __setitem__(self, i, value):
        """Set a grid value."""
        self._check_index(i)
        index = (i - self.min)/self.step
        self.cells[index] = value

    def __iter__(self):
        for value in self.cells:
            yield value

    def __str__(self):
        result = []
        result.append('[')

        for value in self.cells:
            result.append(str(value))

        result.append(']')
        return ' '.join(result)

    def __len__(self):
        return self.num_cells

    def __mul__(self, other):
        return (self.cells * other.cells)

    def normalize(self):
        """Normalize to guarantee all values sum up to 1."""
        self.cells = self.cells/self.cells.sum()

    def apply(self, dist):
        """Apply a probability distribution to the cells."""
        for i in self.indices:
            self[i] *= dist.calculate(i)

        self.normalize()

    def plot(self):
        """Plot grid values."""
        plt.bar(self.indices, self.cells)
        plt.show()
#!/usr/bin/env python

from __future__ import division
import unittest
import pybayes

class TestGrid(unittest.TestCase):
    def test_getitem(self):
        grid = pybayes.Grid(min=1, max=10, step=1)
        self.assertEqual(len(grid), 10)
        self.assertEqual(grid[1], 0.1)
        self.assertEqual(grid[10], 0.1)
        self.assertRaises(ValueError, grid.__getitem__, 0)
        self.assertRaises(ValueError, grid.__getitem__, 11)

        grid = pybayes.Grid(min=0.1, max=10, step=0.1)
        self.assertEqual(len(grid), 100)
        self.assertEqual(grid[0.1], 0.01)
        self.assertEqual(grid[10], 0.01)
        self.assertRaises(ValueError, grid.__getitem__, 0)
        self.assertRaises(ValueError, grid.__getitem__, 10.1)

    def test_iter(self):
        grid = pybayes.Grid(min=1, max=10, step=1)

        for value in grid:
            self.assertEqual(value, 0.1)

    def test_normalize(self):
        g = pybayes.Grid(min=0, max=3.5, step=0.5)
        g[0] = g[0.5] = g[1] = g[1.5] = g[2] = g[2.5] = g[3] = g[3.5] = 2
        g.normalize()

        for value in g:
            self.assertEqual(value, 0.125)


if __name__ == '__main__':
    unittest.main()
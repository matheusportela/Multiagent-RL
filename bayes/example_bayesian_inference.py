#!/usr/bin/env python

from __future__ import division
import scipy.stats as stats
import pybayes

if __name__ == '__main__':
    grid = pybayes.Grid(min=-10, max=10, step=0.25)
    data = stats.norm(loc=3, scale=10).rvs(size=1000)

    for d in data:
        dist = pybayes.NormalDistribution(mean=d, var=30)
        grid.apply(dist)

    grid.plot()
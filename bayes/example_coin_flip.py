#!/usr/bin/env python

from __future__ import division
import scipy.stats as stats
import pybayes

if __name__ == '__main__':
    grid = pybayes.Grid(min=0, max=1, step=0.025)
    num_flips = 100
    head_probability = 0.1
    data = stats.binom.rvs(1, head_probability, size=num_flips)

    for d in data:
        dist = pybayes.NormalDistribution(mean=d, var=1)
        grid.apply(dist)

    print 'Number of flips:', num_flips
    print 'True probability:', head_probability
    print 'Estimated probability:', grid.max_index()
    grid.plot()
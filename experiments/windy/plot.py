from __future__ import division
from __future__ import print_function

import argparse
import pickle
import matplotlib.pylab as plt
import numpy as np

COLOR_TABLE = {
    'r': '#c0392b',
    'g': '#16a085',
    'b': '#2980b9',
    'y': '#f39c12',
    'p': '#8e44ad',
    'w': '#ecf0f1',
    'k': '#2c3e50',
}

COLOR_LIST = ['r', 'g', 'b', 'y', 'p', 'w', 'k']


def load_results(filename):
    with open(filename, 'rb') as f:
        results = pickle.load(f)

    return results


def calculate_regression_coefficients(data, degree=4):
    return np.polyfit(range(len(data)), data, degree)


def calculate_regression_y(x, coeff):
    return np.sum([x**(len(coeff) - i - 1) *
                   coeff[i] for i in range(len(coeff))])


def plot_scores(learn_scores, test_scores):
    fig = plt.figure()
    ax = fig.add_subplot(111)
    plt.xlabel(u'Game number')
    plt.ylabel(u'Final score')
    plt.title(u'Windy game scores')
    data = learn_scores + test_scores
    coeff = calculate_regression_coefficients(data, degree=1)
    regression = [calculate_regression_y(x, coeff) for x in range(len(data))]

    print('Regression coefficients:', coeff)

    ax.scatter(range(len(learn_scores)), learn_scores,
               c=COLOR_TABLE['r'], marker='o')
    ax.scatter(range(len(learn_scores), len(learn_scores) + len(test_scores)),
               test_scores, c=COLOR_TABLE['g'], marker='D')
    ax.plot(regression, c=COLOR_TABLE['b'], linewidth=2.0)


def plot(args):
    results = load_results(args.input)
    plot_scores(results['learn_scores'], results['test_scores'])
    plt.show()


def build_parser():
    parser = argparse.ArgumentParser(
        description='Plot windy simulation results.')
    parser.add_argument(
        'input', type=str, help='results file (e.g. "windy.res")')
    return parser


if __name__ == '__main__':
    parser = build_parser()
    args = parser.parse_args()
    plot(args)

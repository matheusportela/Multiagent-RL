from __future__ import division
import csv
import pickle
import matplotlib.pylab as plt
import numpy as np

def load_results(filename):
    with open(filename) as f:
        results = pickle.loads(f.read())
    return results

def calculate_regression_coefficients(data, degree=5):
    return np.polyfit(range(len(data)), data, degree)

def calculate_regression_y(x, coeff):
    return np.sum([x**(len(coeff) - i - 1) * coeff[i] for i in range(len(coeff))])

def plot_scores(learn_scores, test_scores):
    fig = plt.figure()
    ax = fig.add_subplot(111)
    plt.xlabel('Number of games')
    plt.ylabel('Game final score')
    plt.title('Game scores over time')
    data = learn_scores + test_scores
    coeff = calculate_regression_coefficients(data, degree=5)
    regression = [calculate_regression_y(x, coeff) for x in range(len(data))]

    print 'Regression coefficients:', coeff

    ax.scatter(range(len(learn_scores)), learn_scores, c='b')
    ax.scatter(range(len(learn_scores), len(learn_scores) + len(test_scores)), test_scores, c='g')
    ax.plot(regression, c='r')

def plot_behavior_count(agent_id, behavior_count):
    fig = plt.figure()
    ax = fig.add_subplot(111)
    plt.xlabel('Number of games')
    plt.ylabel('Probability of selecting a behavior')
    plt.title('Probability of agent %d selecting the behavior' % agent_id)
    plt.ylim([0, 1])
    colors = ['r', 'g', 'b']

    data = np.array([b for b in behavior_count.values()])
    prob = data/np.sum(data, axis=0)

    for i, behavior in enumerate(behavior_count):
        coeff = calculate_regression_coefficients(prob[i], degree=5)
        regression = [calculate_regression_y(x, coeff) for x in range(len(prob[i]))]
        ax.plot(regression, label=behavior, c=colors[i], linewidth=2.0)
        ax.scatter(range(len(prob[i])), prob[i], c=colors[i], alpha=0.10)

    ax.legend()

if __name__ == '__main__':
    results = load_results('results.txt')

    plot_scores(results['learn_scores'], results['test_scores'])
    for agent_id, behavior_count in results['behavior_count'].items():
        plot_behavior_count(agent_id, behavior_count)
    plt.show()